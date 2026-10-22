"""树莓派 <-> STM32 的串口指令通道（USART3，115200，每条指令一行文本）。

树莓派 -> STM32:  PING | HOME | P2 | YAW? | GET | SET 名字 数值 | CAL [距离mm [速度]]
                  F <mm> [速度] | S <mm> [速度] | R <度> | A <id> <度> | U <id> | !(急停，单独一个字符)
    F: 前进(负数后退)  S: 横移(正数向左，负数向右)  R: 原地转(正数逆时针；绕车中心转，按累计目标航向转)
    R 0: 把现在的车头方向记为要保持的方向
STM32 -> 树莓派:  DONE [t=用时ms e=车头误差0.01度] | ERR <原因> | PONG | READY | YAW100 <角度x100>（空闲时每0.1秒一次）
                  P 名字=数值（GET 的回复）| CAL LEFT/RIGHT ...（CAL 的结果）
同一时间只发一条，等到 DONE/ERR 再发下一条。
"""
import queue
import re
import threading
import time


def parse_done(reply):
    """'DONE t=1372 e=-6' -> {'t': 1.372(秒), 'err': -0.06(度)}；没有这两项就返回空字典。"""
    out = {}
    m = re.search(r't=(\d+)', reply or '')
    if m:
        out['t'] = int(m.group(1)) / 1000.0
    m = re.search(r'e=(-?\d+)', reply or '')
    if m:
        out['err'] = int(m.group(1)) / 100.0
    return out


class Stm32Link:
    def __init__(self, port, baud=115200):
        self.ok = False
        self.err = ''
        self.value = None          # 最近一次 YAW100（和原 GyroReader 接口兼容）
        self.t = 0.0
        self.on_info = None        # 回调：收到“不是回复”的信息行(P …、CAL …、SERVO …)时调用
        self._resp = queue.Queue()
        self._info = queue.Queue()
        self._lock = threading.Lock()
        try:
            import serial
            self.s = serial.Serial(port, baud, timeout=0.2)
            self.ok = True
            threading.Thread(target=self._run, daemon=True).start()
        except Exception as e:
            self.err = str(e)

    def _run(self):
        buf = b''
        while True:
            try:
                buf += self.s.read(64)
            except Exception:
                time.sleep(0.2)
                continue
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', 'ignore').strip()
                if not text:
                    continue
                parts = text.split()
                if parts[0] == 'YAW100' and len(parts) == 2:
                    try:
                        self.value = int(parts[1])
                        self.t = time.monotonic()
                    except ValueError:
                        pass
                elif text.startswith(('DONE', 'PONG', 'ERR')):
                    self._resp.put(text)
                else:
                    self._info.put(text)
                    if self.on_info is not None:
                        try:
                            self.on_info(text)
                        except Exception:
                            pass

    def fresh(self, max_age=1.0):
        return self.value is not None and time.monotonic() - self.t <= max_age

    def request(self, text, timeout, collect=False):
        """发一行，等 DONE / PONG / ERR。返回 (是否成功, 回复文本)；collect=True 时多返回一个信息行列表。"""
        with self._lock:
            for q in (self._resp, self._info):      # 丢掉旧的回复
                while not q.empty():
                    try:
                        q.get_nowait()
                    except queue.Empty:
                        break
            self.s.write((text + '\n').encode('ascii'))
            end = time.monotonic() + timeout
            ok, reply = False, f'超时({timeout:.0f}秒没收到 DONE)'
            while time.monotonic() < end:
                try:
                    line = self._resp.get(timeout=0.1)
                except queue.Empty:
                    continue
                ok, reply = line.startswith(('DONE', 'PONG')), line
                break
            info = []
            if collect:
                time.sleep(0.05)
                while not self._info.empty():
                    try:
                        info.append(self._info.get_nowait())
                    except queue.Empty:
                        break
        return (ok, reply, info) if collect else (ok, reply)

    def abort(self):
        """紧急停车：直接写一个 '!'，不等回复(正在跑的那条指令会回 ERR ABORT)。"""
        try:
            self.s.write(b'!')
        except Exception:
            pass

    def ping(self):
        return self.request('PING', 2.0)

    def home(self):
        return self.request('HOME', 2.0)      # 把“现在的车头方向”记为要保持的方向

    def second_move(self, rel=None, log=None):
        """第二站移动：按配置里的 second_relative_moves 逐条发 S(左移)、F(前进)、R(顺时针转 cw_deg)。
        不给 rel 就发 STM32 里写死的 P2(兼容旧版 STM32 程序)。"""
        if rel is None:
            return self.request('P2', 60.0)
        steps = []
        if abs(rel.get('left_mm', 0)) >= 1:
            steps.append(('S', rel['left_mm']))
        if abs(rel.get('forward_mm', 0)) >= 1:
            steps.append(('F', rel['forward_mm']))
        if abs(rel.get('cw_deg', 0)) >= 0.5:
            steps.append(('R', -rel['cw_deg']))
        reply = 'DONE'
        for c, v in steps:
            ok, reply = self.move(c, v)
            if log:
                log(f'    {c} {v:+.0f} -> {reply}')
            if not ok:
                return False, f'{c} {v} 失败：{reply}'
        return True, reply

    def move(self, cmd, val, speed=None):
        val = int(round(val))
        timeout = 25.0 if cmd == 'R' else 15.0 + abs(val) / 100.0
        text = f'{cmd} {val}' + (f' {int(speed)}' if speed else '')
        return self.request(text, timeout)

    # ---------- 参数：SET / GET / CAL ----------
    def set_param(self, name, value):
        return self.request(f'SET {name} {value:.6g}', 2.0)

    def get_params(self):
        ok, reply, info = self.request('GET', 4.0, collect=True)
        if not ok:
            return None, reply
        out = {}
        for line in info:
            m = re.match(r'P (\w+)=(-?[\d.]+)', line)
            if m:
                out[m.group(1)] = float(m.group(2))
        return out, reply

    def sync_params(self, params, log=None):
        """把配置里的 stm32_params 全部发给 STM32(STM32 每次断电重启后都会回到编译进去的默认值)。返回失败的列表。"""
        bad = []
        for k, v in (params or {}).items():
            if k.startswith('_'):
                continue
            ok, reply = self.set_param(k, float(v))
            if not ok:
                bad.append((k, reply))
                if log:
                    log(f'  SET {k} {v} 失败：{reply}')
        return bad

    def cal(self, dist=800, speed=None, timeout=60.0):
        """横移校准：车往左走 dist 再往右走回来(不纠偏)，返回 ({'SSCAL':..,'SSCDL':..,'SSCAR':..,'SSCDR':..}, 回复)。"""
        text = f'CAL {int(dist)}' + (f' {int(speed)}' if speed else '')
        ok, reply, info = self.request(text, timeout, collect=True)
        if not ok:
            return None, reply
        out = {}
        for line in info:
            m = re.match(r'CAL (LEFT|RIGHT) CA=(-?\d+) CD=(-?\d+)', line)
            if m:
                side = 'L' if m.group(1) == 'LEFT' else 'R'
                out['SSCA' + side] = int(m.group(2)) / 1e7
                out['SSCD' + side] = int(m.group(3)) / 1e7
            elif line.startswith('CAL FAIL'):
                return None, line
        return (out or None), (reply if out else '没有收到 CAL 结果')


class FakeLink:
    """桌面模拟用：不接串口，指令只打印。"""
    ok = True
    err = ''
    value = None
    on_info = None

    def __init__(self, delay=0.02):
        self.delay = delay
        self.log = []
        self.params = {}

    def fresh(self, max_age=1.0):
        return False

    def request(self, text, timeout, collect=False):
        self.log.append(text)
        print(f'  [模拟STM32] 收到 {text} -> DONE', flush=True)
        time.sleep(self.delay)
        return (True, 'DONE', []) if collect else (True, 'DONE')

    def abort(self):
        print('  [模拟STM32] 收到 ! (急停)', flush=True)

    def ping(self):
        return self.request('PING', 2.0)

    def home(self):
        return self.request('HOME', 2.0)

    def second_move(self, rel=None, log=None):
        if rel is None:
            return self.request('P2', 60.0)
        for c, v in (('S', rel.get('left_mm', 0)), ('F', rel.get('forward_mm', 0)), ('R', -rel.get('cw_deg', 0))):
            if abs(v) >= 0.5:
                self.move(c, v)
        return True, 'DONE'

    def move(self, cmd, val, speed=None):
        return self.request(f'{cmd} {int(round(val))}' + (f' {int(speed)}' if speed else ''), 10.0)

    def set_param(self, name, value):
        self.params[name] = value
        return self.request(f'SET {name} {value:.6g}', 2.0)

    def get_params(self):
        base = dict(FPPM=12.84, BPPM=12.78, LPPM=13.593, RPPM=13.741, SSACC=200.0, SCACC=300.0, DRL=-0.037, DRR=-0.033)
        base.update(self.params)
        return base, 'DONE'

    def sync_params(self, params, log=None):
        for k, v in (params or {}).items():
            if not k.startswith('_'):
                self.set_param(k, float(v))
        return []

    def cal(self, dist=800, speed=None, timeout=60.0):
        self.request(f'CAL {dist}' + (f' {speed}' if speed else ''), timeout)
        return dict(SSCAL=-0.0134, SSCDL=-0.006, SSCAR=0.0128, SSCDR=0.0041), 'DONE'
