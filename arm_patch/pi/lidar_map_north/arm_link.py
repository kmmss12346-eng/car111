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
    ('GRAB', 60.0), ('PICK', 60.0), ('PLACE', 70.0), ('TAKE', 40.0), ('DROP', 30.0), ('OBS', 40.0), ('STOW', 40.0),
    ('LIFT', 25.0), ('AP', 15.0), ('AD', 10.0), ('AF', 10.0), ('A?', 4.0), ('CLAW', 5.0), ('TT', 5.0),
    ('SCR', 2.0), ('SCMD', 2.0), ('QR', 2.0), ('SET', 2.0), ('GET', 4.0),
)


# 这些指令会动 ID1/ID2 但不回报角度，执行后缓存的角度就过期了
_MOVING = ('OBS', 'GRAB', 'PICK', 'PLACE', 'TAKE', 'DROP', 'STOW')


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


class ArmLink:
    def __init__(self, link, log=print):
        self.link = link
        self.log = log
        self.angle = {1: None, 2: None}          # 最近一次读到的 ID1、ID2 角度(度)
        self._screen_last = {}
        self._params = None

    # ------------------------------------------------------------ 基础
    def request(self, text, timeout=None):
        """发一条，返回 (ok, 回复, 信息行列表)。收到急停抛 ArmAbort。"""
        if text.split(' ', 1)[0] in _MOVING:
            self.angle = {1: None, 2: None}
        out = self.link.request(text, timeout or _timeout_for(text), collect=True)
        ok, reply, info = out
        reply = reply or ''
        if 'ABORT' in reply:
            raise ArmAbort(f'{text} -> {reply}')
        for line in info:
            m = _ANG.match(line)
            if m:
                self.angle[int(m.group(1))] = float(m.group(2))
        return ok, reply, info

    def do(self, text, timeout=None):
        """发一条，必须成功，否则抛 ArmError(带原因，如 ERR NOCAL / ERR NOZERO)。"""
        ok, reply, info = self.request(text, timeout)
        if not ok:
            raise ArmError(f'{text} 失败：{reply}')
        return reply, info

    # ------------------------------------------------------------ 二维码 / 串口屏
    def qr(self):
        """STM32 读到的任务码，没有返回 None。"""
        ok, reply, info = self.request('QR?', 2.0)
        for line in info:
            m = _QR.match(line)
            if m:
                return m.group(1)
        return None

    def qr_clear(self):
        self.request('QR CLR', 2.0)

    def screen(self, obj, text):
        """往串口屏的文本控件写字(只能 ASCII，且不超过 STM32 一行 47 字符的限制)。同样的内容不重复发。失败只记日志不抛错。"""
        text = str(text).encode('ascii', 'replace').decode('ascii').replace('"', "'")
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
        """(是否已回零, 当前毫米)。"""
        ok, reply, info = self.request('LIFT?', 3.0)
        for line in info:
            m = _LIFT.match(line)
            if m:
                return bool(int(m.group(1))), float(m.group(2))
        return False, 0.0

    def claw(self, open_):
        self.do('CLAW O' if open_ else 'CLAW C')

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
    def read_angles(self, tries=4):
        """读 ID1、ID2 现在的角度。总线舵机偶尔一次没回话(STM32 回 ERR READ)：歇一下再读，最多读 tries 次。"""
        for i in (1, 2):
            for k in range(tries):
                ok, reply, _info = self.request(f'A? {i}')
                if ok:
                    break
                if 'READ' not in (reply or '') or k == tries - 1:
                    raise ArmError(f'A? {i} 失败：{reply}')
                time.sleep(0.05 + 0.05 * k)
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
            self._params = d if ok else {}
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

    def reset_disp(self):
        self.disp = {'S': 0.0, 'F': 0.0}

    def arm_move(self, d_id2, d_id1):
        """ID2、ID1 相对现在的角度再转 d_id2、d_id1 度(一起转)。返回实际转了多少 (ID2, ID1)，用读回来的角度算。"""
        if self.arm.angle[1] is None or self.arm.angle[2] is None:
            self.arm.read_angles()
        a1_0, a2_0 = self.arm.angle[1], self.arm.angle[2]
        if a1_0 is None or a2_0 is None:
            raise ArmError('读不到 ID1/ID2 的角度(A? 没有回 ANG)，检查总线舵机')
        self.arm.ap(a1_0 + d_id1, a2_0 + d_id2)
        a1_1, a2_1 = self.arm.angle[1], self.arm.angle[2]
        if a1_1 is None or a2_1 is None:
            return None                                   # 没读回来，按指令的量算
        return (a2_1 - a2_0, a1_1 - a1_0)

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
