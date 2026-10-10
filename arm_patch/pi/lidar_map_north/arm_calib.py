"""机械臂一次性标定向导：夹爪(PA2)、转盘(PB3)、ID1/ID2 两个舵机的各个姿态、升降的各个高度，一次调完。

在树莓派终端运行(先在 map_merge_live 里输入 q 退出，它占着串口)：

    cd ~/lidar_map_north
    python3 arm_calib.py

程序一步一步带着调。每一步先把机械臂摆到这一步的起始状态(要转舵机之前会先把升降抬到 ZHI，转的时候不会刮到东西)，
然后你输入下面的东西一点点调，满意了直接回车，就保存并进入下一步：

    +5  -5        这一步要调的东西加/减(舵机 = 度，升降 = 毫米，夹爪/转盘 = 微秒)
    350           直接到 350(ID2 的角度本来就是负数：2 -862 是直接转到 -862 度；
                  正负 200 以内带符号的数才算加减)
    1 +5   2 -3   调 ID1 / ID2 舵机(摆姿态的那几步要同时调两个舵机)
    z -10         调升降(任何一步都能用)
    o   c         夹爪张开 / 夹紧
    u             两个舵机松开，用手直接摆；摆好以后回车 = 读出角度并保存
    r             重新读一下两个舵机现在的角度
    v             打开/关掉摄像头画面(vlive.py 的窗口：看物料、圆环认得怎么样)。原料盘上方姿态、ZGRAB、ZOBRAW、
                  圆环上方姿态、ZPLC、ZOBRNG 这几步会自动打开(要在树莓派桌面的终端里运行向导)
    回车(或 ok)   保存当前值，下一步
    s             跳过这一步(保持原来的值)
    b             回上一步
    q             退出(已经保存的都保留；这一步调过还没按回车的不保存，会先提醒一次)
    h             再看一遍这个说明

保存 = 发给 STM32(SET) + 写进配置文件的 stm32_params(备份原文件为 .json.bak)。
以后每次启动 map_merge_live 都会自动把这些值发给 STM32，不用重新烧录。

    python3 arm_calib.py --from ZGRAB    从某一步开始(只重调其中几项时用)
    python3 arm_calib.py --config xxx.json --port /dev/serial0

需要 STM32 里是带 CLWO/TT1/AEXT 参数的新程序(2026-10-08 以后的 arm.c)。
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time
import unicodedata
from pathlib import Path

ROOT = Path(__file__).resolve().parent

# 夹爪、转盘脉宽的限位(和 main.c 里 CLAW_MIN_US/CLAW_MAX_US、TURNTABLE_MIN_US/TURNTABLE_MAX_US 一样)
CLAW_RANGE = (1700, 2910)
TT_RANGE = (500, 2608)
MATERIAL_MM = 60.0            # 物料高度：码垛高度 ZSTK 默认 = ZPLC + 这个

TIMEOUT = dict(LIFT=25.0, AP=15.0, AF=10.0, CLAW=5.0, TT=5.0, U=3.0, SET=3.0, GET=5.0, PING=2.0)
TIMEOUT['A?'] = 3.0
TIMEOUT['LIFT?'] = 3.0

HELP = __doc__.split('然后你输入下面的东西', 1)[1].split('保存 = ', 1)[0]


class CalError(Exception):
    pass


class Quit(Exception):
    pass


# ---------------------------------------------------------------- 每一步
# kind：choice 选一个数 / claw 夹爪脉宽 / speed 夹爪速度 / tt 转盘脉宽 / lift 升降高度 / pose 两个舵机 / servo 一个舵机
# setup：这一步开始前把机械臂摆成什么样(见 Wizard._setup)
STEPS = [
    dict(key='AEXT', kind='choice', title='哪个舵机管前后伸缩', setup='high',
         text='输入 1 = ID1 管伸缩(ID2 管旋转)；输入 2 = ID2 管伸缩(ID1 管旋转)。\n'
              '不确定就先输入 w1 或 w2：那个舵机会来回动 5 度，看它是伸缩还是旋转。'),
    dict(key='CLWO', kind='claw', title='夹爪张开', setup='high',
         text='调到爪子张开得足够大、能从上面套住物料，又不会碰到旁边的东西。'),
    dict(key='CLWC', kind='claw', title='夹爪夹紧', setup='high',
         text='拿一个物料放在两个爪子中间，调到刚好夹稳(用手拽不下来)。\n'
              '不要比需要的更紧：夹得太紧，夹爪舵机会一直使劲顶着，很快发烫。'),
    dict(key='CLSPD', kind='speed', title='夹爪夹紧(夹取)的速度 CLSPD', setup='high',
         text='夹爪合上时转得多快(脉宽每秒变多少微秒)：越小越慢，0 = 一下子合上。\n'
              '直接输入数字(例如 800)，或者 +100 / -100，夹爪会张开再合上一次给你看。原来的做法是 0(一下子夹紧)。'),
    dict(key='CLSPO', kind='speed', title='夹爪张开(放下)的速度 CLSPO', setup='high',
         text='夹爪张开时转得多快：越小越慢，0 = 一下子张开。\n'
              '直接输入数字(例如 800)，或者 +100 / -100，夹爪会合上再张开一次给你看。'),
    dict(key='ZHI', kind='lift', title='搬运高度 ZHI', setup='zhi',
         text='手臂转动、搬运时用的高度：在这个高度，手臂怎么转、伸都碰不到车上的东西。一般就用最高点。'),
    dict(key=('A1D', 'A2R'), kind='pose', title='转盘上方的姿态', setup='turntable',
         text='把爪子移到转盘 1 号位的正上方(爪子中心对准放物料的格子)。\n'
              '用 1 +5 / 2 -5 调，或者输入 u 松开舵机用手摆，摆好回车。'),
    dict(key='TT1', kind='tt', slot=1, title='转盘 1 号位', setup='turntable',
         text='微调转盘，让 1 号格子的中心正好在爪子下面。觉得高处看不准，可以先 z -40 降低一点再看。'),
    dict(key='TT2', kind='tt', slot=2, title='转盘 2 号位', setup='turntable',
         text='微调转盘，让 2 号格子的中心正好在爪子下面。'),
    dict(key='TT3', kind='tt', slot=3, title='转盘 3 号位', setup='turntable',
         text='微调转盘，让 3 号格子的中心正好在爪子下面。'),
    dict(key='ZDROP', kind='lift', title='往转盘里放 / 从转盘里取的高度 ZDROP', setup='turntable_down',
         text='先输入 c 让爪子夹住一个物料，再用 -10 / -5 / -1 慢慢往下降，\n'
              '降到物料底部离转盘格子大约 2~5mm(松开就落进去)。从转盘里取物料时也用这个高度，爪子要能夹住格子里的物料。'),
    dict(key=('A1G', 'A2E'), kind='pose', title='原料盘上方的姿态', setup='raw',
         text='车按比赛时的位置停在原料盘前面，把爪子移到原料盘中间那个物料的正上方。\n'
              '之后摄像头还会自动微调，差几毫米没关系。'),
    dict(key='ZGRAB', kind='lift', title='在原料盘上夹物料的高度 ZGRAB', setup='raw_down',
         text='爪子张开，用 -10 / -5 / -1 慢慢往下降，降到张开的爪子正好套住物料、夹下去能夹稳的位置。\n'
              '可以输入 c 试夹一下，再 o 张开。爪子不能碰到盘子。'),
    dict(key='ZOBRAW', kind='lift', title='摄像头看原料盘的高度 ZOBRAW', setup='raw_obs', optional=True,
         text='摄像头对准原料盘时手臂停的高度，要能从画面里看到整个物料。\n'
              '已经用 vcal 标定过摄像头的话不要改它，输入 s 跳过(改了要重新 vcal)。'),
    dict(key=('A1P', 'A2P'), kind='pose', title='地上圆环上方的姿态', setup='ring',
         text='车按比赛时的位置停在圆环旁边，把爪子移到圆环中心的正上方。'),
    dict(key='ZPLC', kind='lift', title='往地上放物料的高度 ZPLC', setup='ring_down',
         text='先输入 c 夹住一个物料，用 -10 / -5 / -1 慢慢往下降，降到物料底部刚好碰到地面(圆环里)。'),
    dict(key='ZSTK', kind='lift', title='码垛高度 ZSTK', setup='ring_down',
         text='在已经放好的物料上面再放一个时的高度，一般 = ZPLC + 物料高度(60mm)。\n'
              '可以在圆环里放一个物料，夹着第二个物料往下降到刚好叠上去。'),
    dict(key='ZOBRNG', kind='lift', title='摄像头看圆环的高度 ZOBRNG', setup='ring_obs', optional=True,
         text='摄像头对准地上圆环时手臂停的高度。已经用 vcal 标定过的话输入 s 跳过。'),
    dict(key='A1H', kind='servo', sid=1, title='收起待命的姿态(ID1)', setup='stow',
         text='车跑起来时手臂待命的位置：爪子收在车身范围内，不会碰到任何东西。\n'
              '这一步只调 ID1(ID2 用转盘上方那一步的角度)。'),
]
STEP_KEYS = [s['key'] if isinstance(s['key'], str) else s['key'][0] for s in STEPS]

# 这几步自动打开摄像头画面(vlive.py)：raw = 看物料，ring = 看圆环
CAM_STEPS = {'A1G': 'raw', 'ZGRAB': 'raw', 'ZOBRAW': 'raw', 'A1P': 'ring', 'ZPLC': 'ring', 'ZOBRNG': 'ring'}
VIEW_LOG = '/tmp/vlive_wizard.log'


def has_display():
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


def _num(text):
    try:
        return float(text)
    except ValueError:
        return None


def config_from_run_live(folder=ROOT):
    """run_live.sh 里写了 --config 的话，用它(和 map_merge_live 用同一个配置文件)。"""
    sh = Path(folder) / 'run_live.sh'
    try:
        m = re.search(r'--config[ =]+["\']?([^\s"\']+)', sh.read_text(encoding='utf-8', errors='ignore'))
    except OSError:
        return None
    if not m:
        return None
    p = Path(m.group(1)).expanduser()
    return p if p.is_absolute() else Path(folder) / p


class SerialLink:
    """一行一条指令，等 DONE / ERR / PONG，中间的信息行收集起来(YAW100 丢掉)。"""

    def __init__(self, port):
        import serial
        self.err = serial.SerialException
        self.s = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(0.2)
        self.s.reset_input_buffer()

    def request(self, text, timeout=5.0):
        self.s.reset_input_buffer()
        self.s.write((text + '\n').encode('ascii'))
        end = time.time() + timeout
        buf = b''
        info = []
        while time.time() < end:
            try:
                buf += self.s.read(256)
            except self.err:
                time.sleep(0.05)
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                t = line.decode('ascii', 'ignore').strip()
                if not t or t.startswith('YAW100'):
                    continue
                if t.startswith(('DONE', 'ERR', 'PONG')):
                    return t, info
                info.append(t)
        return None, info

    def abort(self):
        try:
            self.s.write(b'!')
        except Exception:
            pass


class Wizard:
    def __init__(self, link, cfg_path, inp=input, out=print, view=False, popen=None):
        self.link = link
        self.view = view                 # 摄像头那几步自动打开 vlive.py 的窗口
        self._popen = popen or subprocess.Popen
        self._viewer = None              # vlive.py 进程
        self._view_mode = None
        self._view_told = False
        self.cfg_path = Path(cfg_path) if cfg_path else None
        self.inp = inp
        self.out = out
        self.P = {}                      # STM32 里现在的参数
        self.spd = 0.0                   # 夹爪速度那两步正在试的值
        self.saved = {}                  # 这次保存过的
        self.claw = None                 # 夹爪现在的脉宽(只知道发过去的值)
        self.tt = None                   # 转盘现在的脉宽
        self.released = False            # 舵机松开了(用手摆)
        self.z_cmd = None                # 最后一次让升降去的高度(一样就不重复发)

    # ------------------------------------------------------------ 和 STM32 说话
    def do(self, cmd, quiet=False):
        head = cmd.split(' ', 1)[0]
        rep, info = self.link.request(cmd, TIMEOUT.get(head, 10.0))
        for line in info:
            if line.startswith('SERVO') or line.startswith('LIFTBOOT') or 'STUCK' in line:
                self.out('  ' + line)          # 舵机卡住/没到位这类提示要让人看到
        if rep is None:
            raise CalError(f'{cmd}：STM32 没回复(串口线？STM32 在跑吗？)')
        if not rep.startswith(('DONE', 'PONG')):
            raise CalError(f'{cmd}：STM32 回 {rep}')
        if not quiet:
            self.out(f'  > {cmd}')
        return info

    def angle(self, sid):
        for line in self.do(f'A? {sid}', quiet=True):
            m = re.match(r'ANG (\d) (-?\d+(?:\.\d+)?)', line)
            if m and int(m.group(1)) == sid:
                return float(m.group(2))
        raise CalError(f'读不到 ID{sid} 的角度')

    def lift_now(self):
        for line in self.do('LIFT?', quiet=True):
            m = re.match(r'LIFT (\d) (-?\d+(?:\.\d+)?)', line)
            if m:
                return float(m.group(2)) if m.group(1) == '1' else None
        return None

    def lift_to(self, mm):
        mm = max(0.0, min(self.P.get('LFMAX', 100.0), mm))
        if self.z_cmd is None or abs(self.z_cmd - mm) > 0.05:
            self.z_cmd = None                  # 发的过程中出错(急停)就不知道在哪了
            self.do(f'LIFT {mm:g}')
            self.z_cmd = mm
        return mm

    def servo_to(self, sid, deg):
        self.released = False
        self.do(f'AF {sid} {deg:g}')
        return self.angle(sid)

    def pose(self, a1, a2):
        self.released = False
        self.do(f'AP {a1:g} {a2:g}')

    def claw_to(self, us):
        us = int(round(max(CLAW_RANGE[0], min(CLAW_RANGE[1], us))))
        if us != self.claw:
            self.do(f'CLAW {us}')
            self.claw = us
        return us

    def tt_to(self, us):
        us = int(round(max(TT_RANGE[0], min(TT_RANGE[1], us))))
        if us != self.tt:
            self.do(f'TT {us}')
            self.tt = us
        return us

    def release(self):
        self.do('U 1')
        self.do('U 2')
        self.released = True

    # ------------------------------------------------------------ 保存
    def save(self, values):
        for name, v in values.items():
            self.do(f'SET {name} {v:g}', quiet=True)
            self.P[name] = v
            self.saved[name] = v
        if self.cfg_path is not None:
            cfg = json.loads(self.cfg_path.read_text(encoding='utf-8'))
            bak = self.cfg_path.with_suffix('.json.bak')
            if not bak.exists():
                bak.write_text(self.cfg_path.read_text(encoding='utf-8'), encoding='utf-8')
            sp = dict(cfg.get('stm32_params') or {})
            sp.update(values)
            cfg['stm32_params'] = sp
            self.cfg_path.write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding='utf-8')
        self.out('  已保存：' + '  '.join(f'{k}={v:g}' for k, v in values.items()))

    # ------------------------------------------------------------ 开始前检查
    def check(self):
        self.do('PING', quiet=True)
        P = {}
        for line in self.do('GET', quiet=True):
            m = re.match(r'P (\w+)=(-?\d+(?:\.\d+)?)', line)
            if m:
                P[m.group(1)] = float(m.group(2))
        need = ['CLWO', 'CLWC', 'TT1', 'TT2', 'TT3', 'AEXT', 'A1G', 'A2E', 'ZHI', 'ZGRAB']
        miss = [k for k in need if k not in P]
        if miss:
            raise CalError('STM32 里还是旧程序(没有 ' + ' '.join(miss) + ' 参数)：先把新的 arm.c/main.c 编译烧录进去。')
        self.P = P
        if self.cfg_path is not None:
            sp = json.loads(self.cfg_path.read_text(encoding='utf-8')).get('stm32_params') or {}
            for k, v in sp.items():                # 配置里存的值 STM32 刚上电时还没收到：以配置为准
                if k in P and abs(P[k] - float(v)) > 1e-6:
                    try:
                        self.do(f'SET {k} {float(v):g}', quiet=True)
                        P[k] = float(v)
                    except CalError:
                        pass
        z = self.lift_now()
        if z is None:
            raise CalError('升降现在位置不明(LIFT? 回 0)：先在 map_merge_live 里看 arm LIFT ENC?，或者重新上电让它自动回到 60mm。')
        self.out(f'STM32 连接正常。升降现在 {z:g}mm，最高 {P.get("LFMAX", 100):g}mm。')
        self.out('夹爪 CLWO={CLWO:g}/CLWC={CLWC:g}  转盘 TT1={TT1:g} TT2={TT2:g} TT3={TT3:g}  伸缩舵机 AEXT={AEXT:g}'.format(**P))

    # ------------------------------------------------------------ 每一步开始前摆好
    def _setup(self, st):
        P = self.P
        kind = st['setup']
        if kind in ('high', 'zhi'):
            self.lift_to(P['ZHI'])
            return
        self.lift_to(P['ZHI'])                     # 转舵机之前先抬高
        if kind in ('turntable', 'turntable_down'):
            self.pose(P['A1D'], P['A2R'])
            slot = st.get('slot', 1)
            self.tt_to(P[f'TT{slot}'])
        elif kind in ('raw', 'raw_down', 'raw_obs'):
            self.claw_to(P['CLWO'])
            self.pose(P['A1G'], P['A2E'])
        elif kind in ('ring', 'ring_down', 'ring_obs'):
            self.pose(P['A1P'], P['A2P'])
        elif kind == 'stow':
            self.pose(P['A1H'], P['A2R'])
        if kind in ('raw_obs', 'ring_obs'):
            self.lift_to(P['ZOBRAW' if kind == 'raw_obs' else 'ZOBRNG'])

    # ------------------------------------------------------------ 一步
    def _show(self, st):
        P = self.P
        k = st['kind']
        if k == 'pose':
            n1, n2 = st['key']
            return f'ID1={self.cur[1]:g}°  ID2={self.cur[2]:g}°   (原来 {n1}={P[n1]:g} {n2}={P[n2]:g})'
        if k == 'servo':
            return f'ID{st["sid"]}={self.cur[st["sid"]]:g}°   (原来 {st["key"]}={P[st["key"]]:g})'
        if k == 'claw':
            return f'夹爪 {self.claw}us   (原来 {st["key"]}={P[st["key"]]:g})'
        if k == 'speed':
            return f'{st["key"]}={self.spd:g} 微秒/秒(0 = 一下子到)   (原来 {P.get(st["key"], 0):g})'
        if k == 'tt':
            return f'转盘 {self.tt}us   (原来 {st["key"]}={P[st["key"]]:g})'
        if k == 'lift':
            extra = ''
            if st['key'] == 'ZSTK':
                extra = f'，建议 ZPLC+{MATERIAL_MM:g}={P["ZPLC"] + MATERIAL_MM:g}'
            return f'升降 {self.z:g}mm   (原来 {st["key"]}={P[st["key"]]:g}{extra})'
        return f'AEXT={P["AEXT"]:g}(2 = ID2 管伸缩，1 = ID1 管伸缩)'

    def _pending(self, st):
        """这一步调过、但还没按回车保存的值(文字)；没改过返回 None。"""
        P, k = self.P, st['kind']
        try:
            if k == 'lift' and self.z is not None and abs(self.z - P[st['key']]) > 0.05:
                return f'{st["key"]}={self.z:g}'
            if k == 'claw' and self.claw is not None and abs(self.claw - P[st['key']]) > 0.5:
                return f'{st["key"]}={self.claw:g}'
            if k == 'speed' and abs(self.spd - P.get(st['key'], 0.0)) > 0.5:
                return f'{st["key"]}={self.spd:g}'
            if k == 'tt' and self.tt is not None and abs(self.tt - P[st['key']]) > 0.5:
                return f'{st["key"]}={self.tt:g}'
            if k == 'pose':
                n1, n2 = st['key']
                if any(self.cur[i] is not None and abs(self.cur[i] - P[n]) > 0.2 for i, n in ((1, n1), (2, n2))):
                    return f'{n1}={self.cur[1]:g} {n2}={self.cur[2]:g}'
            if k == 'servo':
                a = self.cur[st['sid']]
                if a is not None and abs(a - P[st['key']]) > 0.2:
                    return f'{st["key"]}={a:g}'
        except (KeyError, TypeError):
            return None
        return None

    # ------------------------------------------------------------ 摄像头画面(另开一个 vlive.py 进程，向导只用串口，不冲突)
    def _view_open(self, mode):
        if self._viewer is not None and self._viewer.poll() is None and self._view_mode == mode:
            return
        self._view_close()
        args = [sys.executable, str(ROOT / 'vlive.py')] + (['--ring'] if mode == 'ring' else ['1'])
        try:
            with open(VIEW_LOG, 'w') as logf:              # 子进程自己拿着这个文件，这边可以关
                self._viewer = self._popen(args, stdout=logf, stderr=subprocess.STDOUT, cwd=str(ROOT))
        except Exception as e:
            self.out(f'  (打不开摄像头画面：{e})')
            self._viewer = None
            return
        self._view_mode, self._view_told = mode, False
        self.out('  摄像头画面已打开(另一个窗口' + ('，看圆环' if mode == 'ring' else '，看物料，窗口里按 1~6 换颜色') +
                 ')；输入 v 关掉/再打开')

    def _view_close(self):
        v = self._viewer
        self._viewer, self._view_mode = None, None
        if v is not None and v.poll() is None:
            try:
                v.terminate()
                v.wait(timeout=3)
            except Exception:
                try:
                    v.kill()
                except Exception:
                    pass

    def _view_check(self):
        """vlive 窗口自己退出了(摄像头被占、不在桌面终端里…)：说一次原因。"""
        v = self._viewer
        if v is None or self._view_told or v.poll() is None:
            return
        self._view_told = True
        why = ''
        try:
            lines = [l.strip() for l in open(VIEW_LOG, encoding='utf-8', errors='ignore') if l.strip()]
            why = lines[-1] if lines else ''
        except Exception:
            pass
        self.out(f'  (摄像头画面关了{("：" + why) if why else ""}。输入 v 再打开)')

    def _read_servos(self):
        self.cur = {1: self.angle(1), 2: self.angle(2)}

    def step(self, i):
        """返回 +1 下一步，-1 上一步。"""
        st = STEPS[i]
        P = self.P
        k = st['kind']
        self.out('')
        self.out(f'==== 第 {i + 1}/{len(STEPS)} 步：{st["title"]} ====')
        self.out(st['text'])
        self._setup(st)
        mode = CAM_STEPS.get(st['key'] if isinstance(st['key'], str) else st['key'][0])
        if self.view and mode:
            self._view_open(mode)
        elif self._viewer is not None:
            self._view_close()
        self._step_mode = mode
        self.z = self.lift_now()
        if self.z is None:
            self.z = P['ZHI']
        self.cur = {1: None, 2: None}
        self._q_warned = False
        if k in ('pose', 'servo'):
            self._read_servos()
        if k == 'claw':
            self.claw_to(P[st['key']])
        if k == 'speed':
            if st['key'] not in P:
                self.out(f'  STM32 里还没有 {st["key"]} 这个参数(是旧程序)：先烧录新的 arm.c，这一步跳过。')
                return +1
            self.spd = float(P[st['key']])
        if k == 'tt':
            self.tt_to(P[st['key']])
        if k == 'lift' and st['key'] in ('ZHI', 'ZOBRAW', 'ZOBRNG'):
            self.z = self.lift_to(P[st['key']])
        while True:
            self._view_check()
            self.out('  现在：' + self._show(st))
            try:
                raw = self.inp('  调整(回车=保存，h=说明)> ')
            except EOFError:
                raise Quit()
            line = unicodedata.normalize('NFKC', raw).replace('　', ' ').strip().lower()
            try:
                r = self._handle(st, line)
            except CalError as e:
                self.out(f'  !! {e}')
                continue
            if r is not None:
                return r

    def _handle(self, st, line):
        P = self.P
        k = st['kind']
        if line in ('', 'ok', 'y'):
            return self._accept(st)
        if line == 'q':
            pend = self._pending(st)
            if pend and not self._q_warned:
                self._q_warned = True
                self.out(f'  ★ 这一步改成了 {pend}，还没保存：按回车 = 保存(再输入 q 退出)；再输入 q = 不保存直接退出')
                return None
            raise Quit()
        if line == 's':
            self.out(f'  跳过，保持原来的值。')
            return +1
        if line == 'b':
            return -1
        if line in ('h', '?', 'help'):
            self.out(HELP)
            return None
        if line == 'o':
            self.claw_to(P['CLWO'] if st['key'] != 'CLWO' else self.claw)
            return None
        if line == 'c':
            self.claw_to(P['CLWC'] if st['key'] != 'CLWC' else self.claw)
            return None
        if line == 'u':
            self.release()
            self.out('  两个舵机松开了，用手摆好以后直接回车(会读出角度)。想先看看读数就输入 r。')
            return None
        if line == 'v':
            if self._viewer is not None and self._viewer.poll() is None:
                self._view_close()
                self.out('  摄像头画面已关')
            else:
                self._view_open(getattr(self, '_step_mode', None) or 'raw')
            return None
        if line == 'r':
            self._read_servos()
            self.out(f'  ID1={self.cur[1]:g}°  ID2={self.cur[2]:g}°')
            return None
        if k == 'choice':
            if line in ('w1', 'w2'):
                sid = int(line[1])
                a = self.angle(sid)
                self.servo_to(sid, a + 5)
                self.servo_to(sid, a)
                return None
            if line in ('1', '2'):
                self.save({'AEXT': float(line)})
                return +1
            raise CalError('这一步输入 1 或 2(哪个舵机管伸缩)，或者 w1 / w2 让它动一下看看')
        if k == 'speed':
            v = _num(line)
            if v is None:
                raise CalError('这一步输入速度数字(例如 800)，或者 +100 / -100')
            want = self.spd + v if line[:1] in '+-' else v
            want = max(0.0, min(20000.0, want))
            self.do(f'SET {st["key"]} {want:g}', quiet=True)         # 先试，回车才存进配置
            self.spd = want
            a, b = (P['CLWO'], P['CLWC']) if st['key'] == 'CLSPD' else (P['CLWC'], P['CLWO'])
            self.claw_to(a)
            self.claw_to(b)                                           # 这一下就是按新速度转的
            return None
        parts = line.split()
        axis, val = (parts[0], parts[1]) if len(parts) == 2 else (None, parts[0] if parts else '')
        if axis is None:
            axis = {'claw': 'c', 'tt': 't', 'lift': 'z', 'servo': str(st.get('sid', 1))}.get(k)
            if axis is None:
                raise CalError('这一步要说清楚调哪个：1 +5 (ID1) 或 2 -5 (ID2)，升降用 z -10')
        v = _num(val)
        if v is None or axis not in ('1', '2', 'z', 'c', 't'):
            raise CalError(f'看不懂 "{line}"。输入 h 看说明')
        rel = val[:1] == '+' or (val[:1] == '-' and abs(v) <= 200)   # ID2 的 -862 这种是直接转到这个角度
        if axis in ('1', '2'):
            sid = int(axis)
            base = self.angle(sid)
            want = base + v if rel else v
            got = self.servo_to(sid, want)
            self.cur[sid] = got
            if abs(got - want) > 1.0:
                self.out(f'  注意：ID{sid} 停在 {got:g}°，没到 {want:g}°(到限位了，或者被挡住了)')
        elif axis == 'z':
            self.z = self.lift_to(self.z + v if rel else v)
        elif axis == 'c':
            want = (self.claw or P['CLWO']) + v if rel else v
            got = self.claw_to(want)
            if abs(got - want) > 0.5:
                self.out(f'  夹爪到限位了({CLAW_RANGE[0]}~{CLAW_RANGE[1]}us，main.c 里 CLAW_MIN_US / CLAW_MAX_US)')
        elif axis == 't':
            want = (self.tt or P['TT1']) + v if rel else v
            got = self.tt_to(want)
            if abs(got - want) > 0.5:
                self.out(f'  转盘到限位了({TT_RANGE[0]}~{TT_RANGE[1]}us，main.c 里 TURNTABLE_MIN_US / TURNTABLE_MAX_US)')
        return None

    def _accept(self, st):
        k = st['kind']
        if k == 'choice':
            self.save({'AEXT': self.P['AEXT']})
            return +1
        if k == 'claw':
            self.save({st['key']: float(self.claw)})
            self.claw_to(self.P['CLWO'])           # 张开，免得一直夹着
        elif k == 'speed':
            self.save({st['key']: float(self.spd)})
            self.claw_to(self.P['CLWO'])
        elif k == 'tt':
            self.save({st['key']: float(self.tt)})
        elif k == 'lift':
            self.save({st['key']: round(self.z, 1)})
        elif k == 'pose':
            self._read_servos()
            n1, n2 = st['key']
            self.save({n1: self.cur[1], n2: self.cur[2]})
        elif k == 'servo':
            self._read_servos()
            self.save({st['key']: self.cur[st['sid']]})
        if self.released:
            self._read_servos()
            self.pose(self.cur[1], self.cur[2])     # 用手摆的：让舵机重新出力，停在摆好的位置
        if st['setup'].endswith('_down'):
            self.claw_to(self.P['CLWO'])           # 往下降的那几步：松开(物料留在转盘/盘子/地上)，再抬起来
            self.lift_to(self.P['ZHI'])
        return +1

    # ------------------------------------------------------------ 整个流程
    def run(self, start=0):
        self.check()
        self.out(HELP)
        i = start
        try:
            while 0 <= i < len(STEPS):
                i += self.step(i)
                if i < 0:
                    i = 0
        except Quit:
            self.out('退出。')
        finally:
            self._view_close()
        try:
            self.lift_to(self.P['ZHI'])
            self.pose(self.P['A1H'], self.P['A2R'])
        except CalError as e:
            self.out(f'  !! 收臂失败：{e}')
        if self.saved:
            self.out('')
            self.out('这次保存的参数：' + '  '.join(f'{k}={v:g}' for k, v in self.saved.items()))
            if self.cfg_path is not None:
                self.out(f'已写进 {self.cfg_path}(原文件备份为 .json.bak)，以后 map_merge_live 启动会自动发给 STM32。')
            self.out('回到正常使用：bash run_live.sh')
        if i >= len(STEPS) and self.P.get('ARMOK', 0) < 0.5:
            try:
                ans = self.inp('全部标定完了。打开 ARMOK(允许自动夹放流程 GRAB/TAKE/PLACE 等)？输入 y 打开，其他键不打开> ')
            except EOFError:
                ans = ''
            if ans.strip().lower() == 'y':
                self.save({'ARMOK': 1.0})
        return self.saved


def main(argv=None):
    ap = argparse.ArgumentParser(description='机械臂一次性标定向导(先退出 map_merge_live)')
    ap.add_argument('--port', default='/dev/serial0')
    ap.add_argument('--config', default=None, help='默认：run_live.sh 里的 --config，没有就用 map_config_start2_roi.json')
    ap.add_argument('--from', dest='start', default=None, help='从哪一步开始，例如 --from ZGRAB')
    ap.add_argument('--no-view', action='store_true', help='摄像头那几步不自动打开画面')
    args = ap.parse_args(argv)

    cfg = Path(args.config) if args.config else (config_from_run_live() or ROOT / 'map_config_start2_roi.json')
    if not cfg.exists():
        print(f'找不到配置文件 {cfg}：参数只发给 STM32，不存(重新上电会丢)。可以用 --config 指定 map_merge_live 用的配置文件。')
        cfg = None
    else:
        print(f'参数会存进：{cfg}')
    start = 0
    if args.start:
        key = args.start.upper()
        if key not in STEP_KEYS:
            print('--from 可以用：' + ' '.join(STEP_KEYS))
            return 1
        start = STEP_KEYS.index(key)
    try:
        link = SerialLink(args.port)
    except Exception as e:
        print(f'打不开串口 {args.port}：{e}\n是不是 map_merge_live 还开着？先在它里面输入 q 退出再运行。')
        return 1
    view = not args.no_view and has_display() and (ROOT / 'vlive.py').exists()
    if not args.no_view and not has_display():
        print('(不是在树莓派桌面的终端里运行：摄像头那几步不会自动打开画面)')
    wiz = Wizard(link, cfg, view=view)
    try:
        wiz.run(start)
    except CalError as e:
        print(f'停止：{e}')
        return 1
    except KeyboardInterrupt:
        link.abort()
        print('\n按了 Ctrl+C：已给 STM32 发急停。已经保存的参数都保留。')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
