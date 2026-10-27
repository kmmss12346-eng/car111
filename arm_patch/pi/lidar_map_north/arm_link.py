"""树莓派 -> STM32 的机械臂指令封装。

link 就是 stm32_link.Stm32Link(或 FakeLink 之类有 request(text, timeout, collect=True) 的对象)。
每条指令发一行，等 DONE / ERR；STM32 的指令说明见 STM32 工程里的 arm.h。
"""
import re
import time


class ArmError(Exception):
    pass


class ArmAbort(ArmError):
    """收到急停(STM32 回 ERR ABORT)。"""


# 每类指令最多等多久(秒)。机械臂整套动作(夹取/放置)比较长
_TIMEOUTS = (
    ('APZ', 30.0), ('GRAB', 60.0), ('PICK', 60.0), ('PLACE', 70.0), ('TAKE', 40.0), ('DROP', 30.0), ('OBS', 40.0), ('STOW', 40.0),
    ('LIFT', 25.0), ('AP', 15.0), ('APN', 2.0), ('AD', 10.0), ('AF', 10.0), ('A?', 4.0), ('CLAW', 5.0), ('TT', 5.0),
    ('SCR', 2.0), ('SCMD', 2.0), ('QR', 2.0), ('SET', 2.0), ('GET', 4.0),
    ('PARK', 40.0), ('STBY', 40.0), ('ZONE', 3.0), ('ZONE?', 2.0), ('TRAY', 40.0), ('PRE', 20.0),
)


# 这些指令会动 ID1/ID2 但不回报角度，执行后缓存的角度就过期了
_MOVING = ('OBS', 'GRAB', 'PICK', 'PLACE', 'TAKE', 'DROP', 'STOW', 'PARK', 'STBY', 'TRAY', 'PRE')


def _timeout_for(text):
    head = text.split(' ', 1)[0]
    for k, t in _TIMEOUTS:
        if head == k:
            return t
    return 15.0


_ANG = re.compile(r'ANG (\d) (-?\d+(?:\.\d+)?)')
_LIFT = re.compile(r'LIFT (\d) (-?\d+(?:\.\d+)?)')
_QR = re.compile(r'QR (\d{3}\+\d{3}\+\d{3}\+\d{3})')
_PARAM = re.compile(r'P (\w+)=(-?\d+(?:\.\d+)?)')
_BOOT = re.compile(r'BOOT=(\w+)')
_ZONE = re.compile(r'ZONE (\d) (\d)')
_SLIP = re.compile(r'LIFT(SLIP|PUSH) want=(-?\d+(?:\.\d+)?) got=(-?\d+(?:\.\d+)?)')


def lift_slip_text(line):
    """(10-21)STM32 升降走完用编码器核对，没到要去的高度(顶住、丢步)时回 LIFTSLIP want=… got=…：翻成给人看的话。不是这行返回 None。"""
    m = _SLIP.match(line)
    if not m:
        return None
    want, got = float(m.group(2)), float(m.group(3))
    # (10-25)放地上的高度 ZPLC 偏低最常见的原因：量的时候物料是用手塞进爪子的，和比赛时从车上转盘夹的位置不一样
    zplc = '；放地上的 ZPLC 用 python3 arm_calib.py --from ZPLC，输入 take 和比赛时一样从车上转盘夹物料再量' if want <= 30 else ''
    if m.group(1) == 'PUSH':                     # (10-21)差一点没到：爪子顶到下面(原料盘/地面/转盘槽底)，驱动器一直往下顶，爪子合不上
        if got > want:
            return (f'★ 升降要降到 {want:.1f}mm，到 {got:.1f}mm 就下不去了(爪子顶到下面了？顶着爪子合不上)：STM32 已叫它别再往下顶。'
                    f'哪个高度设的 {want:.0f}(ZGRAB / ZPLC / ZDROP)调高一两毫米(set 名字 {got + 1:.0f}){zplc}')
        return f'★ 升降要升到 {want:.1f}mm，到 {got:.1f}mm 就上不去了(顶到最高点？)：STM32 已叫它别再往上顶'
    if 'too far' in line:
        return (f'★ 升降核对：要到 {want:.1f}mm，编码器算出 {got:.1f}mm，差太多(升降没走/被卡住了？)：没改，这一步不往下做(ERR LIFT)。'
                '车停好以后看一下升降，放到最低点，输入 arm LIFT ZERO')
    if 'blocked' in line:                        # (10-25)走的时候一路盯着编码器，在离要去的 15mm 以外就顶住了：高度可信，只是差太多
        if got > want:
            return (f'★ 升降要降到 {want:.1f}mm，在 {got:.1f}mm 就被挡住了(差 {got - want:.0f}mm，太多)：STM32 已马上叫它停住、不再往下顶，'
                    f'高度按编码器记成 {got:.1f}，这一步不往下做(ERR LIFT，不松/不夹爪子)。看看爪子、物料碰到了什么{zplc}')
        return (f'★ 升降要升到 {want:.1f}mm，在 {got:.1f}mm 就上不去了(差 {want - got:.0f}mm)：STM32 已叫它停住、按编码器记了高度，'
                '这一步不往下做(ERR LIFT)。看看是不是卡住了')
    hint = ''
    if got < want and want >= 60:
        hint = f'。多半是 {want:.0f}mm 比升降能到的最高还高：哪个参数设的 {want:.0f}(ZHI / STBY / LFMAX …)改到 {got - 2:.0f} 以下'
    elif got > want and want <= 30:
        hint = (f'。多半是 {want:.0f}mm 太低了：在 {got:.1f}mm 就碰到底了(物料/爪子碰到地面、槽底，或者到了升降最低点)：'
                f'哪个参数设的 {want:.0f}(ZPLC / ZGRAB / ZDROP …)改到 {got + 1:.0f} 以上{zplc}')
    return f'★ 升降没到 {want:.1f}mm，实际在 {got:.1f}mm(顶住了：STM32 已叫它停住、不再使劲，高度按编码器改过来){hint}'


_CLAW_WHY = {'O': '张开', 'OR': '原料盘上方张开', 'OT': '车上转盘上方张开', 'C': '夹紧', 'STBY': '待机姿态', 'US': '直接给的脉宽'}
_KEEP_WHY = {'OBS': '摆到观察姿态(OBS … O)', 'STBY': '回待机姿态(PARK/STBY)'}


def claw_line_text(line, cmd=''):
    """(10-21 用户"放 1、2 号环的时候爪子莫名其妙松开")STM32 每次动夹爪回 CLAWSET <微秒> <哪一步>，夹着物料没让张开回 CLAWKEEP <哪一步>，
    重启时会发 READY / LIFTBOOT：翻成给人看的话(哪条指令里动的)。不是这几种返回 None。"""
    p = line.split()
    if not p:
        return None
    where = f'  ← {cmd}' if cmd else ''
    if p[0] == 'CLAWSET' and len(p) >= 2:
        what = _CLAW_WHY.get(p[2], p[2]) if len(p) >= 3 else ''
        return f'      (夹爪 → {p[1]}µs {what}{where})'
    if p[0] == 'CLAWKEEP':
        what = _KEEP_WHY.get(p[1], p[1]) if len(p) >= 2 else ''
        return (f'  ★ STM32 记着爪子里夹着物料：{what}本来要张开爪子，没让它张开(夹着物料时只有放下那一步能张开){where}。'
                '确实要拿出来：arm CLAW O')
    if p[0] == 'LIFTSYNC':
        return f'  升降高度按编码器认准了：{line.strip()}{where}'
    if p[0] == 'READY' or p[0] == 'LIFTBOOT':
        return (f'  ★★ STM32 刚才重启了({line.strip()}){where}：夹爪、手臂会回开机/待机的样子，参数回到程序里写的默认值。'
                '多半是电池电压被拉低(舵机、升降一起使劲)或者 STM32 的电源线/复位线松了')
    return None


def unknown_cmd(reply):
    """STM32 不认识这条指令(旧程序回 ERR CMD)。"""
    return 'ERR CMD' in (reply or '')


# (10-23)串口偶尔一次没回话、总线舵机偶尔一次读写失败：这几条都是"转到/升到/张到某个绝对位置"，重发一次没坏处(只多等一下)。
#   以前一次没回话，这个物料就放回车上不放了(少 15 分)。STM32 明确说了原因的错(升降位置不知道、夹着东西、打滑、参数不对…)不重发
_RETRY_VERBS = ('LIFT', 'AP', 'APZ', 'CLAW', 'TT', 'STOW', 'OBS', 'SIDE')
_NO_RETRY = ('ERR NOZERO', 'ERR LIFT', 'ERR HOLD', 'ERR RANGE', 'ERR ARG', 'ERR CMD', 'ERR NOCAL', 'ERR NOENC', 'ERR ENCMOVE',
             'ERR STALL', 'ERR NOSTBY', 'ERR NAME', 'ERR FLASH', 'ERR GYRO', 'ABORT')
_NUM = re.compile(r'^-?\d+(?:\.\d+)?$')


def retry_ok(text, reply):
    """这条指令失败了，能不能原样再发一次。LIFT 只重发"升到多少毫米"(LIFT ZERO / HOME / CAL / SYNC 不重发)。"""
    parts = str(text).split()
    if not parts or parts[0] not in _RETRY_VERBS:
        return False
    if parts[0] == 'LIFT' and (len(parts) != 2 or not _NUM.match(parts[1])):
        return False
    return not any(k in (reply or '') for k in _NO_RETRY)


class ArmLink:
    def __init__(self, link, log=print):
        self.link = link
        self.log = log
        self.angle = {1: None, 2: None}          # 最近一次读到的 ID1、ID2 角度(度)
        self._screen_last = {}
        self._params = None
        self.lift_boot = None                    # 开机时升降高度是怎么认出来的(LIFT? 回复里的 BOOT=…；旧程序没有)
        self._n_req = 0                          # 发过几条(第一条可能收到 STM32 上电时留在串口里的 READY/LIFTBOOT，不算中途重启)
        self.claw_last = None                    # (10-21)上一条指令里夹爪最后一次动作(CLAWSET 的 C/O/OR/OT…，没动过是 None)
        try:                                     # 扫码器插在树莓派 USB 口上时从这里读(见 qr_usb.py)；没有就只问 STM32
            import qr_usb
            self.usb_qr = qr_usb.reader(log)
        except Exception:
            self.usb_qr = None

    # ------------------------------------------------------------ 基础
    def request(self, text, timeout=None):
        """发一条，返回 (ok, 回复, 信息行列表)。收到急停抛 ArmAbort。"""
        if text.split(' ', 1)[0] in _MOVING:
            self.angle = {1: None, 2: None}
        self.claw_last = None                       # (10-21)这条指令里夹爪最后一次动作(CLAWSET 的 C/O/OR/OT…)：出错时看爪子里有没有物料
        out = self.link.request(text, timeout or _timeout_for(text), collect=True)
        ok, reply, info = out
        reply = reply or ''
        if 'ABORT' in reply:
            raise ArmAbort(f'{text} -> {reply}')
        for line in info:
            m = _ANG.match(line)
            if m:
                self.angle[int(m.group(1))] = float(m.group(2))
            if line.startswith('CLAWSET '):
                parts_ = line.split()
                self.claw_last = parts_[2] if len(parts_) >= 3 else None
            slip = lift_slip_text(line)
            if slip:
                self.log('  ' + slip)
            cl = claw_line_text(line, text)
            if cl and not (self._n_req == 0 and line.startswith(('READY', 'LIFTBOOT'))):
                self.log(cl)
        self._n_req += 1
        return ok, reply, info

    def do(self, text, timeout=None):
        """发一条，必须成功，否则抛 ArmError(带原因，如 ERR NOCAL / ERR NOZERO)。
        (10-23)转到/升到绝对位置的指令(retry_ok)偶尔一次没回话：歇一下原样再发一次。"""
        ok, reply, info = self.request(text, timeout)
        if not ok and retry_ok(text, reply):
            self.log(f'    ({text} 没成：{reply}；串口/舵机偶尔一次没回话，再发一次)')
            time.sleep(0.2)
            ok, reply, info = self.request(text, timeout)
        if not ok:
            raise ArmError(f'{text} 失败：{reply}')
        return reply, info

    # ------------------------------------------------------------ 二维码 / 串口屏
    def qr(self):
        """读到的任务码，没有返回 None。先看插在树莓派 USB 口上的扫码器，再问 STM32。"""
        if getattr(self, "usb_qr", None) is not None:
            c = self.usb_qr.get()
            if c:
                return c
        ok, reply, info = self.request('QR?', 2.0)
        for line in info:
            m = _QR.match(line)
            if m:
                return m.group(1)
        return None

    def qr_clear(self, keep_recent_s=0.0):
        """清掉存的任务码(树莓派 USB 扫码器的和 STM32 的)。返回 STM32 是否回了 DONE。
        keep_recent_s > 0：USB 扫码器在这么多秒以内刚读到的码留着(开跑前手拿码板扫一下就用它)。"""
        u = getattr(self, "usb_qr", None)
        if u is not None:
            import time as _t
            if not (keep_recent_s and u.get() and _t.monotonic() - float(getattr(u, 't', 0.0) or 0.0) <= float(keep_recent_s)):
                u.clear()
        ok, _reply, _info = self.request('QR CLR', 2.0)
        return bool(ok)

    def screen(self, obj, text):
        """往串口屏的文本控件写字(只能 ASCII，且不超过 STM32 一行 47 字符的限制)。同样的内容不重复发。失败只记日志不抛错。"""
        text = str(text).encode('ascii', 'replace').decode('ascii').replace('"', "'").replace('!', '.')   # '!' = STM32 的急停
        cmd = f'SCR {obj} {text}'[:47]
        if self._screen_last.get(obj) == cmd:
            return
        try:
            ok, reply, _ = self.request(cmd, 2.0)
            if ok:
                self._screen_last[obj] = cmd
            else:
                self.log(f'    (串口屏 {obj} 写入失败：{reply})')
        except ArmAbort:
            raise
        except Exception as ex:
            self.log(f'    (串口屏 {obj} 写入出错：{ex!r})')

    def screen_reset(self):
        """屏被清过(ZONE LOCK、STM32 重启)：忘掉"已经写过什么"，下次每个控件都重新写。"""
        self._screen_last = {}

    def screen_cmd(self, raw):
        try:
            self.request(f'SCMD {raw}'[:47], 2.0)
        except ArmAbort:
            raise
        except Exception:
            pass

    # ------------------------------------------------------------ 升降 / 夹爪 / 转盘
    def lift_zero(self):
        self.do('LIFT ZERO')

    def lift(self, mm):
        self.do(f'LIFT {mm:.1f}')

    def lift_state(self):
        """(是否已回零, 当前毫米)。新程序的回复后面还带 BOOT=ENC|NOCAL|NOENC|POS(开机怎么认的高度)，记在 self.lift_boot。"""
        ok, reply, info = self.request('LIFT?', 3.0)
        for line in info:
            m = _LIFT.match(line)
            if m:
                b = _BOOT.search(line)
                self.lift_boot = b.group(1) if b else None
                return bool(int(m.group(1))), float(m.group(2))
        return False, 0.0

    def park(self):
        """PARK：收臂(像 STOW)，再把升降停到开机高度 60mm(60 只在 STM32 里定义一处)。
        返回 (True, 回复) 成功；(False, 回复) STM32 认识但没做成(比如 ERR NOZERO 升降位置不知道)；
        (None, 回复) 旧程序不认识 PARK(调用的地方改用 STOW + LIFT 60)。"""
        ok, reply, _ = self.request('PARK')
        if not ok and unknown_cmd(reply):
            return None, reply
        return bool(ok), reply

    def zone(self):
        """ZONE?：屏上选的启停区。返回 (区 0/1/2(0 = 还没选), 按过 START 没有 0/1)；旧程序不认识返回 None。"""
        ok, reply, info = self.request('ZONE?', 2.0)
        if not ok:
            return None
        for line in info:
            m = _ZONE.match(line.strip())
            if m:
                return int(m.group(1)), int(m.group(2))
        return None

    def zone_cmd(self, arg):
        """ZONE ASK / ZONE 1 / ZONE 2 / ZONE MSG 文字(≤20 个 ASCII) / ZONE LOCK。返回是否成功(旧程序不认识：False)。"""
        arg = str(arg).strip()
        if arg.upper().startswith('MSG'):
            txt = arg[3:].strip().encode('ascii', 'replace').decode('ascii').replace('"', "'")[:20]
            arg = 'MSG ' + txt
        else:
            arg = arg.upper()
        ok, reply, _ = self.request(f'ZONE {arg}'[:47], 3.0)
        if not ok and not unknown_cmd(reply):
            self.log(f'    (ZONE {arg} 失败：{reply})')
        return bool(ok)

    def claw(self, open_, where=None):
        """夹爪张开/夹紧。(10-20 用户)张开分三个角度：where='RAW' 原料盘上方(参数 CLWOR)、'TRAY' 车上转盘上方(CLWOT)、
        其他(None)= CLWO。STM32 是旧程序(不认 CLAW OR / CLAW OT)：照旧 CLAW O。"""
        if not open_:
            self.do('CLAW C')
            return
        cmd = {'RAW': 'CLAW OR', 'TRAY': 'CLAW OT'}.get(str(where or '').upper())
        if cmd and getattr(self, '_claw3', True):
            ok, reply, _ = self.request(cmd)
            if not ok and retry_ok(cmd, reply):
                self.log(f'    ({cmd} 没成：{reply}；再发一次)')       # (10-23)和 do() 一样重发一次
                time.sleep(0.2)
                ok, reply, _ = self.request(cmd)
            if ok:
                return
            if not ('ERR ARG' in reply or unknown_cmd(reply)):
                raise ArmError(f'{cmd} 失败：{reply}')
            self._claw3 = False
            self.log('    (STM32 是旧程序，不认 CLAW OR / CLAW OT：夹爪照旧按 CLWO 张开。刷新的 arm.c 以后原料盘、车上转盘上方张开各用各的角度)')
        self.do('CLAW O')

    def tt(self, slot):
        self.do(f'TT {int(slot)}')

    # ------------------------------------------------------------ 夹取 / 放置分步指令
    def obs(self, kind, open_claw=False):
        """手臂摆到观察姿态。kind='RAW' 原料盘 / 'RING' 地上圆环。"""
        self.do(f'OBS {kind}' + (' O' if open_claw else ''))

    def grab_here(self, slot):
        self.do(f'GRAB {int(slot)} H')

    def pick_here(self, slot):
        self.do(f'PICK {int(slot)} H')

    def take(self, slot):
        self.do(f'TAKE {int(slot)}')

    def drop(self, stack=False):
        self.do('DROP S' if stack else 'DROP')

    def stow(self):
        self.do('STOW')

    # ------------------------------------------------------------ ID1 / ID2
    def read_angles(self, tries=6):
        """读 ID1、ID2 现在的角度。总线舵机偶尔一次没回话：STM32 回 ERR READ，或者回了 OK 却没带 ANG 行(第二次读没回话)。
        这两种都歇一下再读，最多读 tries 次；还读不到就报错。"""
        for i in (1, 2):
            for k in range(tries):
                self.angle[i] = None
                ok, reply, _info = self.request(f'A? {i}')
                if ok and self.angle[i] is not None:
                    break
                if (ok or 'READ' in (reply or '')) and k < tries - 1:
                    time.sleep(0.05 + 0.05 * k)
                    continue
                if k == 0 and not (unknown_cmd(reply) or 'ERR ARG' in (reply or '')):
                    time.sleep(0.2)                     # (10-23)串口偶尔一次没回话(超时)：只读角度，再问一次没坏处
                    continue
                raise ArmError(f'A? {i} 失败：{reply if not ok else "回了 OK 但没有角度"}(读了 {k + 1} 次)')
        return self.angle[1], self.angle[2]

    def apz(self, a1, a2, z):
        """(10-14)转到 (a1, a2) 并停在高度 z：新 STM32 程序(APZ)边升边转、边转边降；旧程序(ERR CMD)改成 AP + LIFT。"""
        if not getattr(self, '_no_apz', False):
            self.angle[1] = None
            self.angle[2] = None
            cmd = f'APZ {a1:.3f} {a2:.3f} {float(z):.1f}'
            ok, reply, info = self.request(cmd)
            if not ok and retry_ok(cmd, reply) and not unknown_cmd(reply):
                self.log(f'    ({cmd} 没成：{reply}；串口/舵机偶尔一次没回话，再发一次)')   # (10-23)和 do() 一样重发一次
                time.sleep(0.2)
                ok, reply, info = self.request(cmd)
            if ok:
                return
            if 'CMD' not in str(reply):
                raise ArmError(f'APZ 失败：{reply}')
            self._no_apz = True
        self.ap(a1, a2)
        self.lift(z)

    def apn(self, a1, a2, ms=None):
        """(10-16)跟随用：让 ID1、ID2 往 (a1, a2) 转，不等转完(STM32 先回报现在的角度再发目标)。ms = 大约多少毫秒转到。
        返回发之前读到的 (ID1, ID2)(读不到的是 None)；旧程序不认 APN 返回 False(以后不再试)。"""
        if getattr(self, '_no_apn', False):
            return False
        self.angle[1] = None
        self.angle[2] = None
        ok, reply, _info = self.request(f'APN {a1:.3f} {a2:.3f}' + (f' {max(1.0, float(ms)):.0f}' if ms else ''))
        if not ok:
            if unknown_cmd(reply):
                self._no_apn = True
                return False
            raise ArmError(f'APN 失败：{reply}')
        return self.angle[1], self.angle[2]

    def ap(self, a1, a2):
        """ID1、ID2 一起转到指定角度，回读实际角度(没回读到就是 None = 不知道，不能当成"没动")。"""
        self.angle[1] = None
        self.angle[2] = None
        self.do(f'AP {a1:.3f} {a2:.3f}')

    # ------------------------------------------------------------ 参数
    def params(self, refresh=False):
        """STM32 现在的全部参数(GET)，缓存。取不到返回 {}。"""
        if self._params is None or refresh:
            ok, reply, info = self.request('GET', 4.0)
            d = {}
            for line in info:
                m = _PARAM.match(line)
                if m:
                    d[m.group(1)] = float(m.group(2))
            if not ok or not d:                 # (10-23)这次没取到不记下来(以前记成 {}，一次没回话整轮都当没有参数、不夹不放)：
                return self._params or {}       #   有以前取到的就还用它，下次再问
            self._params = d
        return self._params

    def set_param(self, name, value):
        ok, reply, _ = self.request(f'SET {name} {value:.6g}', 2.0)
        if ok and self._params is not None:
            self._params[name] = float(value)
        return ok, reply


class ArmActuators:
    """给 visual_servo 用：手臂(ID2/ID1)微调 + 底盘(S/F)位移。"""

    def __init__(self, arm, link, fine_speed_rpm=60, log=print):
        self.arm = arm
        self.link = link
        self.fine_speed = int(fine_speed_rpm)
        self.log = log
        self.disp = {'S': 0.0, 'F': 0.0}      # 到这个停车点以来底盘累计动了多少毫米(S 向左为正，F 向前为正)
        # (10-27 实车 ID1 齿轮间隙)id1_backlash > 0：arm_move 的每一下 ID1 最后都从 id1_from 那一边到位(+1 = 从大往小转到位)，
        #   要往那一边转就先多转 id1_backlash 度再转回来。mission_hooks 的 1、3 号环微调时设，平时 0
        self.id1_backlash = 0.0
        self.id1_from = 1.0
        # (10-27 实车"调了这么久机械臂位置基本没有变")id1_comp > 0：每一下 ID1 往转的方向多给这么多度。STM32 的 AP 离目标还差 ATOL(0.3°)
        #   以内就算到了、停下(还差不到 ATOL 的干脆不转)，零点几度的小修正等于没转；多给一点，停下的地方才是要的地方
        self.id1_comp = 0.0

    def reset_disp(self):
        self.disp = {'S': 0.0, 'F': 0.0}

    def arm_move(self, d_id2, d_id1):
        """ID2、ID1 相对现在的角度再转 d_id2、d_id1 度(一起转)。返回实际转了多少 (ID2, ID1)，用读回来的角度算。"""
        if self.arm.angle[1] is None or self.arm.angle[2] is None:
            self.arm.read_angles()
        a1_0, a2_0 = self.arm.angle[1], self.arm.angle[2]
        if a1_0 is None or a2_0 is None:
            raise ArmError('读不到 ID1/ID2 的角度(A? 没有回 ANG)，检查总线舵机')
        bl = float(getattr(self, 'id1_backlash', 0.0) or 0.0)
        side = 1.0 if float(getattr(self, 'id1_from', 1.0)) >= 0.0 else -1.0
        comp = float(getattr(self, 'id1_comp', 0.0) or 0.0)
        last = (1.0 if d_id1 > 0 else -1.0) if abs(float(d_id1)) > 1e-6 else 0.0   # 最后一下 ID1 往哪边转
        if bl > 0.0 and float(d_id1) * side > 1e-6:
            # 这一下要往 id1_from 那一边转：只转过去，舵机先走完齿轮间隙手臂才动、最后还停在间隙另一边。先多转 bl 再转回来
            self.arm.ap(a1_0 + d_id1 + side * bl, a2_0 + d_id2)
            last = -side
        self.arm.ap(a1_0 + d_id1 + last * comp, a2_0 + d_id2)
        a1_1, a2_1 = self.arm.angle[1], self.arm.angle[2]
        if a1_1 is None or a2_1 is None:
            return None                                   # 没读回来，按指令的量算
        return (a2_1 - a2_0, a1_1 - a1_0)

    def vel(self, s_mmps, f_mmps):
        """(10-16)底盘按速度一直走：横移 s_mmps、前进 f_mmps(毫米/秒，左/前为正)，发了马上返回(STM32 回 DONE)，
        400ms 没发新的就自己停。返回 True；STM32 是旧程序(不认 V)返回 False(以后不再试)。位移不记进 disp：用 vel_read 算。"""
        if getattr(self, '_no_vel', False):
            return False
        ok, reply, _info = self.link.request(f'V {float(s_mmps):.1f} {float(f_mmps):.1f}', 1.0, collect=True)
        reply = reply or ''
        if 'ABORT' in reply:
            raise ArmAbort(f'V -> {reply}')
        if not ok:
            if 'CMD' in reply or 'ARG' in reply:
                self._no_vel = True
                return False
            raise ArmError(f'底盘 V 失败：{reply}')
        return True

    def vel_read(self):
        """V 一共走了多少 (横移, 前进) 毫米(STM32 按发给电机的速度算)；读不到返回 None。"""
        ok, reply, info = self.link.request('V?', 1.0, collect=True)
        for line in info or []:
            m = re.match(r'VD (-?\d+) (-?\d+)', line)
            if m:
                return int(m.group(1)) / 10.0, int(m.group(2)) / 10.0
        return None

    def chassis_move(self, s_mm, f_mm, speed=None):
        """底盘横移 s_mm(向左为正)、前进 f_mm(向前为正)，一条一条发，等 DONE。speed = 这次的速度(转/分)；None = 小距离用 fine_speed。"""
        for cmd, v in (('S', s_mm), ('F', f_mm)):
            v = int(round(v))
            if v == 0:
                continue
            sp = int(speed) if speed else (self.fine_speed if abs(v) <= 60 else None)   # 小距离慢一点准一点；大距离用默认速度
            ok, reply = self.link.move(cmd, v, sp)
            if 'ABORT' in (reply or ''):
                raise ArmAbort(f'{cmd} {v} -> {reply}')
            if not ok:
                raise ArmError(f'底盘 {cmd} {v} 失败：{reply}')
            self.disp[cmd] += v
