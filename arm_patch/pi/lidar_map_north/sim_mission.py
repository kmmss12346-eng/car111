"""整场模拟：假 STM32 + 假摄像头 + 一个带几何关系的物理世界，把 mission_hooks 从头到尾跑一遍。

  python3 sim_mission.py              按"默认速度参数"跑一次完整的比赛流程，打印过程、结果和用时估计
  python3 sim_mission.py fast         同上，用"提速参数"(升降 400 转/分、ID1/ID2 270°/秒等)，看提速后能省多少时间
  python3 sim_mission.py 1            只做第一批(路线 QR→RAW→ROUGH→TEMP→START)；可以和 fast 一起用
  python3 sim_mission.py 1 LFRPM=300 ASPD=200 CLWAIT=250    用您自己的速度参数估算(名字和 STM32 的参数名一样)
  python3 -m unittest sim_mission     作为测试：正常流程(多组随机)、读不到码、没标定、急停、物料缺失、时间不够、机械臂出错……

世界(都在"工位坐标系"里，单位毫米)：
  径向 rad = 爪子往工位方向伸出去的方向；切向 tan = 车头方向(向前为正)。
  爪子位置 = [k2*(ID2 角度-基准) - 底盘横移 + 停车误差径向,  k1*(ID1 角度-基准) + 底盘前进 + 停车误差切向]
  目标(物料/圆环)位置固定。摄像头看到的偏差像素 = 比例 * 随机旋转/镜像 @ (目标 - 爪子) + 噪声。
所以程序事先并不知道摄像头朝向、ID1/ID2 每度多少毫米；和真车一样要靠探测。

用时估计：按 STM32 的速度参数(LFRPM、ASPD、CLWAIT……)和 arm.c 里每个动作的实际步骤累加，
不含路线行驶时间；是估计，真车要以实测为准。
"""
import math
import sys
import unittest

import numpy as np

from mission_hooks import MissionHooks, Abort
from task_plan import parse_code, role_of

DUR = dict(GET=0.3, SCR=0.02, SET=0.02, QR=0.02, SCMD=0.02, LIFT=0.0, A=0.03)
MEASURE_S = 0.35
CLAW_CLOSE_S = 0.15                     # 发出"合上"到夹爪真的夹住要多久
FRAME_S = 0.07                          # 跟踪原料盘上的物料时，认一帧要多久


def _frames_dt(n):
    """拍 n 帧、认 n 次要多久(秒)。没给 n = 默认的 5 帧(MEASURE_S)。"""
    return MEASURE_S if not n else 0.05 + 0.06 * int(n)

DEFAULT_PARAMS = dict(ARMOK=1.0, ZHI=0.0, ZGRAB=100.0, ZDROP=60.0, ZPLC=100.0, ZSTK=40.0, ZOBRAW=80.0, ZOBRNG=0.0,
                      A1G=500.0, A1D=430.0, A1H=410.0, A1P=470.0, A2E=-800.0, A2R=-1100.0, A2P=-780.0,
                      LFPPM=80.0, LFRPM=150.0, LFSPR=3200.0, LFMRG=200.0, ASPD=90.0, ASPDF=40.0, ATOLC=1.0, ATOL=0.3,
                      PARA=1.0, CLWAIT=400.0, TTWAIT=800.0, FPPM=12.9, LPPM=13.7)
FAST_PARAMS = dict(LFRPM=400.0, ASPD=270.0, ASPDF=120.0, ATOLC=2.0, CLWAIT=250.0, TTWAIT=600.0, LFMRG=100.0)


class SimLink:
    """stm32_link.Stm32Link 里被用到的那几个方法。"""

    def __init__(self, world):
        self.w = world

    def request(self, text, timeout, collect=False):
        ok, reply, info = self.w.handle(text)
        return (ok, reply, info) if collect else (ok, reply)

    def move(self, cmd, val, speed=None):
        return self.w.move(cmd, val, speed)

    def abort(self):
        self.w.abort_flag = True


class SimWorld:
    def __init__(self, seed=0, code='156+123+516+231', armok=True, qr_present=True, noise_px=0.6, stop_err_mm=8.0,
                 missing_batch1=(), fail_cmd=None, abort_at=None, params=None, cam_deg=None, f_gain=1.0, park=None,
                 park_side=None, plate_stop_s=None, plate_move_s=3.0, plate_off_mm=0.0, raw_scale=None, tilt=None, ap_bias=None,
                 qr_window=None, qr_latch_moving=False, qr_stale=None, park_cmd=True, zone_fw=None, lift_boot='ENC'):
        self.rng = np.random.default_rng(seed)
        self.lay = np.random.default_rng([int(seed), 7])   # 每次到停车点的停车误差、原料盘上物料的位置：单独的随机数(程序多拍一张、多动一下也不改变场地)
        self.t = 0.0
        self.code = code
        self.qr_present = qr_present
        self.noise_px = noise_px
        self.stop_err_mm = stop_err_mm
        self.params = dict(DEFAULT_PARAMS)
        self.params.update(params or {})
        self.params['ARMOK'] = 1.0 if armok else 0.0
        self.lift_known = True                # 和 STM32 一样：开机就认为升降在离最低点 60mm
        self.lift_mm = 60.0
        self.a1, self.a2 = self.params['A1H'], self.params['A2R']
        self.a1_ref = self.a2_ref = None
        self.claw_open = True
        self.tt_slot = 1
        self.tt_ready = 0.0
        self.screen = {}
        self.requests = []
        self.abort_flag = False
        self.abort_at = abort_at              # 例如 ('GRAB', 2) = 第 2 次 GRAB 时被急停
        self.fail_cmd = fail_cmd              # 例如 ('GRAB', 1) = 第 1 次 GRAB 时 STM32 回 ERR TIMEOUT
        self.count = {}
        self.time_by_verb = {}
        # 世界
        self.zone = None
        self.tray = {1: None, 2: None, 3: None}
        self.held = None
        self.raw_items = []
        self.rings = {}
        self.placed = []                      # 每次放置：(区, 环, 颜色, 误差mm, 是否码垛)
        self.air = 0
        self.collisions = 0
        self.loose = 0                        # 夹着物料在不该松手的地方张开了爪子(物料掉在半路上)
        self.off_s = self.off_f = 0.0
        self.stop_err = np.zeros(2)
        self.missing_batch1 = set(missing_batch1)
        # 摄像头和机构(程序不知道)
        th = self.rng.uniform(0, 2 * math.pi)
        refl = self.rng.choice([-1.0, 1.0])
        dev = (math.degrees(th) % 90.0) - 45.0                 # 正好斜 45° 时"从左到右"说不清(真车上摄像头是正的)：避开斜着的那一段
        if abs(dev) < 12.0:
            th += math.radians(24.0 if dev >= 0 else -24.0)
        if cam_deg is not None:                                # 摄像头装得正：圆环那一排在画面里差不多是横的
            th, refl = math.radians(float(cam_deg)), 1.0
        c, s = math.cos(th), math.sin(th)
        self.Rcam = np.array([[c, -s], [s, c]]) @ np.diag([1.0, refl])
        self.k2 = 0.175 * self.rng.choice([-1.0, 1.0])        # ID2 每度动多少毫米(径向)
        self.k1 = 2.6 * self.rng.choice([-1.0, 1.0])          # ID1 每度动多少毫米(切向)
        self.scale = dict(RAW=4.36, RING=1.46, PICK=1.46 * 1.3)   # RING：现场画面里圆环间距 150mm ≈ 219 像素；PICK：物料顶面(比地面近，像素/毫米大)
        self.f_gain = float(f_gain)                            # 底盘前后实际走的距离 = 指令 × 这个(测"走得不准")
        self.park = dict(park or {})                           # 工位 -> 车停的位置沿圆环那一排偏了多少毫米(测"车没停在 2 号环")
        self.park_side = dict(park_side or {})                 # 工位 -> 车离圆环那一排远/近了多少毫米(测"手臂伸缩够不着")
        self.moves = []                                        # 底盘每条指令 (S/F, 毫米, 当时在哪个工位)
        self.tilt = dict(tilt or {})                           # 工位 -> 车身和圆环那一排差多少度(车是手摆的)：车头、底盘前后、手臂、摄像头一起转了这么多
        self.ap_bias = tuple(ap_bias) if ap_bias else None     # 大幅 AP(从转盘那边转回来)总是多转 (ID1, ID2) 度(齿轮间隙、舵机过冲)
        if raw_scale:
            self.scale['RAW'] = float(raw_scale)               # 真车上原料区画面约 1.97 像素/毫米(看得比较大一片)
        # 原料盘转一会儿停一会儿(停 plate_stop_s 秒、转 plate_move_s 秒，每次转 120°，三个物料轮流停到爪子附近；None = 一直停着)：
        # 物料在半径 150mm 的圆上，停下时离爪子最近的那个在爪子外面 plate_off_mm(车停得偏一点)
        self.plate_stop = float(plate_stop_s) if plate_stop_s else None
        self.plate_move = float(plate_move_s)
        self.plate_t0 = 0.0
        self.plate_r = 150.0
        self.plate_c = np.array([self.plate_r + float(plate_off_mm), 0.0])
        self.homed = 0                                         # 收到几次 HOME(把现在的车头方向记为要保持的方向)
        # 二维码：扫码器只有车停在 QR 点前后挪 qr_window=(下限, 上限) mm 以内时才看得到码(None = 一直看得到)；
        # STM32 读到一次就一直存着(qr_code)，QR CLR 才清掉；qr_latch_moving = 新程序：车在走的时候读到的也存下来；
        # qr_stale = 开机前(上一轮/调试时)就存着的旧码
        self.qr_window = tuple(qr_window) if qr_window else None
        self.qr_latch_moving = bool(qr_latch_moving)
        self.qr_code = qr_stale
        self.at = None                                         # 现在停在哪个停车点(角色)
        self.park_cmd = bool(park_cmd)                         # STM32 认识 PARK(新程序)
        self.parked = 0                                        # 收到几次 PARK
        self.zone_fw = zone_fw                                 # 新程序：ZONE? 回的 (区, START 按过没有)；None = 旧程序不认识
        self.lift_boot = lift_boot                             # 新程序：LIFT? 后面带的 BOOT=…(默认 ENC)；None = 旧程序不带
        self.move_speeds = []                                  # 底盘每条指令的速度(转/分；None = 默认)，和 moves 一一对应
        self.ring_tan = {'ROUGH': {1: 150.0, 2: 0.0, 3: -150.0}, 'TEMP': {1: -150.0, 2: 0.0, 3: 150.0}}
        self.link = SimLink(self)
        self.vision = SimVision(self)

    # ------------------------------------------------------------------ 时间
    def advance(self, s):
        self.t += s

    def now(self):
        return self.t

    # ------------------------------------------------------------------ 动作耗时(照着 arm.c 的步骤)
    def _lift_to(self, z):
        P = self.params
        d = abs(z - self.lift_mm)
        if d >= 0.05:
            self.advance(d * P['LFPPM'] / (P['LFRPM'] / 60.0 * P['LFSPR']) + P['LFMRG'] / 1000.0)
        self.lift_mm = z

    @staticmethod
    def _servo_t(d, speed):
        return abs(d) / speed + 0.2 + 0.03

    def _servos_to(self, a1, a2, speed, fine=False):
        P = self.params
        t1, t2 = self._servo_t(a1 - self.a1, speed), self._servo_t(a2 - self.a2, speed)
        self.advance(max(t1, t2) if P['PARA'] > 0.5 else t1 + t2)
        if fine:                                           # AP：到位误差 0.3°，实际重复精度更好
            self.a1 = a1 + self.rng.normal(0, 0.03)
            self.a2 = a2 + self.rng.normal(0, 0.08)
        else:                                              # 大动作：到位误差 ATOLC
            tol = P['ATOLC']
            self.a1 = a1 + self.rng.uniform(-0.8 * tol, 0.8 * tol)
            self.a2 = a2 + self.rng.uniform(-0.8 * tol, 0.8 * tol)

    def _open_claw(self):
        """张开爪子。夹着物料时：在转盘上方(ZDROP、转盘姿态)张开 = 放回转盘；在工位里降到 ZPLC / ZSTK 张开 = 放在地上(圆环里)；
        在别处张开 = 物料掉在半路上。"""
        P = self.params
        if self.held is not None:
            at_tray = (abs(self.lift_mm - P['ZDROP']) < 1.0 and abs(self.a1 - P['A1D']) < 3.0 and abs(self.a2 - P['A2R']) < 3.0)
            on_ground = self.zone in ('ROUGH', 'TEMP') and min(abs(self.lift_mm - P['ZPLC']), abs(self.lift_mm - P['ZSTK'])) < 1.0
            if at_tray:
                if self.tray[self.tt_slot] is not None:
                    self.collisions += 1
                self.tray[self.tt_slot] = self.held
                self.held = None
            elif on_ground:
                self._place_held(abs(self.lift_mm - P['ZSTK']) < abs(self.lift_mm - P['ZPLC']))
            else:
                self.loose += 1
                self.held = None
        self.claw_open = True

    def _place_held(self, stack):
        """手里的物料放在现在爪子下面的地上(工位里)：记下落在哪个圆环、偏多少；放在两个环中间、平放压到别的物料、码垛下面是空的都算碰倒。"""
        if self.held is None:
            self.air += 1
            return
        c = self.claw()
        err, k = min((np.linalg.norm(self.ring_center(k) - c), k) for k in (1, 2, 3))
        existing = self.rings.setdefault((self.zone, k), [])
        if err > 12.0:
            self.collisions += 1                       # 放在两个圆环之间，倒了
        else:
            if existing and not stack:
                self.collisions += 1                   # 想平放，下面已经有物料
            if stack and not existing:
                self.collisions += 1                   # 想码垛，下面是空的
            existing.append(dict(color=self.held, pos=c.copy(), err=float(err)))
            self.placed.append((self.zone, k, self.held, float(err), stack))
        self.held = None

    def _tt_go(self, slot):
        if slot != self.tt_slot:
            self.tt_slot = slot
            self.tt_ready = self.t + self.params['TTWAIT'] / 1000.0

    def _tt_wait(self):
        if self.t < self.tt_ready:
            self.advance(self.tt_ready - self.t)

    def _claw_wait(self):
        self.advance(self.params['CLWAIT'] / 1000.0)

    # ------------------------------------------------------------------ 到达一个工位(由测试代码调用，相当于路线把车开到了停车点)
    def arrive(self, role, batch=1):
        self.zone = role if role in ('RAW', 'ROUGH', 'TEMP') else None
        self.at = role
        self.off_s = self.off_f = 0.0
        self.stop_err = np.clip(self.lay.normal(0, self.stop_err_mm / 2.0, 2), -self.stop_err_mm, self.stop_err_mm)
        self.stop_err[1] += float(self.park.get(role, 0.0))
        self.stop_err[0] += float(self.park_side.get(role, 0.0))
        if role == 'RAW':
            # 转盘转到、停稳在爪子够得着的区域里的物料(视野只有 147x110mm，所以只会在 ±30mm 左右)
            try:
                colors = [it.color for it in parse_code(self.code).items(batch)]
            except Exception:
                colors = [1, 2, 3]
            self.raw_items = []
            if self.plate_stop:
                self.plate_t0 = self.t - self.lay.uniform(0, self.plate_stop + self.plate_move)   # 到的时候转盘在一个周期里的随便哪个时刻
                shift = int(self.lay.integers(0, 3))
                for i, c in enumerate(colors):
                    if not (batch == 1 and c in self.missing_batch1):
                        self.raw_items.append(dict(color=c, r=self.plate_r + self.lay.normal(0, 1.0),
                                                   off=-((i + shift) % 3) * 2 * math.pi / 3 + self.lay.normal(0, 0.02)))
                colors = []
            for c in colors:
                if batch == 1 and c in self.missing_batch1:
                    continue
                for _ in range(200):                           # 离停车点 22mm 以内(原料区只动手臂：车要停得让物料在手臂够得着的地方)；两个物料不会叠在一起
                    pos = self.lay.uniform(-22, 22, 2)
                    if np.linalg.norm(pos) <= 22.0 and all(np.linalg.norm(pos - it['pos']) >= 20.0 for it in self.raw_items):
                        break
                self.raw_items.append(dict(color=c, pos=pos))
        if role in ('ROUGH', 'TEMP'):
            for k in (1, 2, 3):
                self.rings.setdefault((role, k), [])

    # ------------------------------------------------------------------ 几何
    def plate_phase(self, t=None):
        """(原料盘转过的角度, 现在是不是在转)。"""
        if not self.plate_stop:
            return 0.0, False
        cyc = self.plate_stop + self.plate_move
        tau = (self.t if t is None else t) - self.plate_t0
        k = math.floor(tau / cyc)
        r = tau - k * cyc
        step = 2 * math.pi / 3
        if r < self.plate_stop:
            return k * step, False
        return k * step + step * (r - self.plate_stop) / self.plate_move, True

    def raw_pos(self, it, t=None):
        """原料盘上物料现在的位置(转盘转起来时随时间变)。"""
        if 'r' not in it:
            return it['pos']
        th = self.plate_phase(t)[0] + it['off']
        return np.array([self.plate_c[0] - it['r'] * math.cos(th), self.plate_c[1] + it['r'] * math.sin(th)])

    def _rot(self, zone=None):
        """车身相对圆环那一排转了多少(tilt)：车上的方向(手臂伸缩、底盘前后) -> 工位坐标系。"""
        t = math.radians(float(self.tilt.get(zone or self.zone, 0.0)))
        c, s = math.cos(t), math.sin(t)
        return np.array([[c, -s], [s, c]])

    def _cam(self, zone=None):
        """工位坐标系里的偏差 -> 画面里的方向(摄像头装在车上，跟着车身一起转)。"""
        return self.Rcam @ self._rot(zone).T

    def claw(self):
        """爪子在工位坐标系里的位置(rad, tan)。车身斜了(tilt)：手臂和底盘的动作都按车身的方向走。"""
        if self.a1_ref is None:
            return np.array([0.0, 0.0])
        d = np.array([self.k2 * (self.a2 - self.a2_ref) - self.off_s, self.k1 * (self.a1 - self.a1_ref) + self.off_f])
        return self.stop_err + self._rot() @ d

    def ring_center(self, k):
        return np.array([0.0, self.ring_tan[self.zone][k]])

    def ring_order(self, zone):
        """这个假摄像头里，zone 的 1、2、3 号环在画面里怎么排('lr' / 'rl')：和 mission_hooks._survey 一样的定方向办法。"""
        t = self.ring_tan[zone]
        d = self._cam(zone) @ np.array([0.0, t[3] - t[1]])     # 画面里从 1 号到 3 号的方向
        a = d / np.linalg.norm(d)
        if abs(a[0]) >= abs(a[1]):
            a = a if a[0] > 0 else -a
        else:
            a = a if a[1] > 0 else -a
        return 'lr' if float(a @ d) > 0 else 'rl'

    def pixel_error(self, target, kind, dt=MEASURE_S, n=None):
        """目标在画面里离爪子点多少像素。n = 这次测量拍几帧：noise_px 是默认 5 帧取平均的噪声，帧少噪声大(按根号算)。"""
        e = np.asarray(target, float) - self.claw()
        sd = self.noise_px * (math.sqrt(5.0 / max(1, int(n))) if n else 1.0)
        p = self.scale[kind] * (self._cam() @ e) + self.rng.normal(0, sd, 2)
        self.advance(dt)
        if abs(p[0]) > 280 or abs(p[1]) > 200:
            return None                                   # 出了画面
        return (float(p[0]), float(p[1]))

    # ------------------------------------------------------------------ 底盘
    def move(self, cmd, val, speed=None):
        self.advance(1.0 + abs(val) / 150.0)
        self.moves.append((cmd, val, self.zone))
        self.move_speeds.append(speed)
        f_before = self.off_f
        if cmd == 'S':
            self.off_s += val * (1 + self.rng.normal(0, 0.03)) + self.rng.normal(0, 1.0)
        elif cmd == 'F':
            self.off_f += val * self.f_gain * (1 + self.rng.normal(0, 0.03)) + self.rng.normal(0, 1.0)
        if self.qr_latch_moving and self._qr_visible(min(f_before, self.off_f), max(f_before, self.off_f)):
            self.qr_code = self.code                           # 新程序：走的时候扫到的码也存下来
        return True, 'DONE t=1000 e=10'

    def _qr_visible(self, f0=None, f1=None):
        """扫码器(车停在 QR 点前后挪了 f0..f1 mm)看不看得到码。"""
        if not self.qr_present or self.at not in (None, 'QR'):          # 还没到过任何停车点(终端测试)：当作在 QR 点
            return False
        if self.qr_window is None:
            return True
        f0 = self.off_f if f0 is None else f0
        f1 = self.off_f if f1 is None else f1
        return f1 >= self.qr_window[0] and f0 <= self.qr_window[1]

    # ------------------------------------------------------------------ 假 STM32
    def handle(self, text):
        self.requests.append(text)
        t_before = self.t
        try:
            return self._handle(text)
        finally:
            v = text.split(' ', 1)[0] if text else ''
            self.time_by_verb[v] = self.time_by_verb.get(v, 0.0) + (self.t - t_before)

    def _handle(self, text):
        parts = text.split()
        verb = parts[0] if parts else ''
        info = []
        self.advance(DUR.get(verb.rstrip('?'), 0.0) if verb != 'A?' else DUR['A'])
        n = self.count[verb] = self.count.get(verb, 0) + 1
        if self.abort_at and verb == self.abort_at[0] and n == self.abort_at[1]:
            return False, 'ERR ABORT', info
        if self.fail_cmd and verb == self.fail_cmd[0] and n == self.fail_cmd[1]:
            return False, 'ERR TIMEOUT', info
        P = self.params

        def ang(i):
            info.append(f'ANG {i} {(self.a1 if i == 1 else self.a2):.4f}')

        if verb == 'PING':
            return True, 'PONG', info
        if verb == 'HOME':
            self.homed += 1                                      # 现在的车头方向记为要保持的方向
            return True, 'DONE', info
        if verb == 'GET':
            info += [f'P {k}={v:.4f}' for k, v in P.items()]
            return True, 'DONE', info
        if verb == 'SET' and len(parts) == 3:
            P[parts[1]] = float(parts[2])
            return True, 'DONE', info
        if verb == 'LIFT?':
            info.append(f'LIFT {int(self.lift_known)} {self.lift_mm:.4f}' + (f' BOOT={self.lift_boot}' if self.lift_boot else ''))
            return True, 'DONE', info
        if verb == 'ZONE?' or verb == 'ZONE':
            if self.zone_fw is None:
                return False, 'ERR CMD', info
            if verb == 'ZONE?':
                info.append(f'ZONE {int(self.zone_fw[0])} {int(self.zone_fw[1])}')
            elif len(parts) >= 2 and parts[1] in ('1', '2'):
                self.zone_fw = (int(parts[1]), 0)
            return True, 'DONE', info
        if verb == 'PARK':
            if not self.park_cmd:
                return False, 'ERR CMD', info
            if not self.lift_known:
                return False, 'ERR NOZERO', info
            self.parked += 1
            if P['ARMOK'] >= 0.5:
                self._lift_to(P['ZHI'])
                self._servos_to(P['A1H'], P['A2R'], P['ASPD'])
            self._lift_to(60.0)
            return True, 'DONE', info
        if verb == 'LIFT' and len(parts) == 2:
            if parts[1] == 'ZERO':
                self.lift_known, self.lift_mm = True, 0.0
                return True, 'DONE', info
            if not self.lift_known:
                return False, 'ERR NOZERO', info
            self._lift_to(float(parts[1]))
            return True, 'DONE', info
        if verb == 'QR?':
            if self.qr_code is None and self._qr_visible():
                self.qr_code = self.code                       # 车停着、扫码器对着码：读到，存下来
            info.append(f'QR {self.qr_code}' if self.qr_code else 'QR NONE')
            return True, 'DONE', info
        if verb == 'QR':
            if len(parts) == 2 and parts[1] == 'CLR':
                self.qr_code = None
            return True, 'DONE', info
        if verb in ('SCR', 'SCMD'):
            if verb == 'SCR' and len(parts) >= 3:
                self.screen[parts[1]] = text.split(' ', 2)[2]
            return True, 'DONE', info
        if verb == 'CLAW':
            if parts[1] == 'O':
                self._open_claw()
            else:
                self.claw_open = False
            self._claw_wait()
            return True, 'DONE', info
        if verb == 'TT':
            self._tt_go(int(parts[1]))
            self._tt_wait()
            return True, 'DONE', info
        if verb == 'A?':
            ang(int(parts[1]))
            return True, 'DONE', info
        if verb in ('AD', 'AF'):
            i, v = int(parts[1]), float(parts[2])
            cur = self.a1 if i == 1 else self.a2
            new = cur + v if verb == 'AD' else v
            self.advance(self._servo_t(new - cur, P['ASPD'] if abs(new - cur) > 8 else P['ASPDF']))
            if i == 1:
                self.a1 = new
            else:
                self.a2 = new
            ang(i)
            return True, 'DONE', info
        if verb == 'AP':
            a1_, a2_ = float(parts[1]), float(parts[2])
            spd = P['ASPDF'] * 2.0
            big = abs(a1_ - self.a1) > 10.0 or abs(a2_ - self.a2) > 10.0
            if big:
                spd = max(spd, P['ASPD'])                       # 角度变化大：用大动作速度(和 arm.c 一致)
            b1, b2 = self.ap_bias if (big and self.ap_bias) else (0.0, 0.0)
            self._servos_to(a1_ + b1, a2_ + b2, spd, fine=True)
            ang(1)
            ang(2)
            return True, 'DONE', info

        # ---- 下面是夹放流程：要求已标定、已回零
        if verb in ('OBS', 'GRAB', 'PICK', 'TAKE', 'DROP', 'STOW'):
            if P['ARMOK'] < 0.5:
                return False, 'ERR NOCAL', info
            if not self.lift_known:
                return False, 'ERR NOZERO', info
        if verb == 'OBS':
            ring = parts[1] == 'RING'
            self.a1_ref = P['A1P'] if ring else P['A1G']
            self.a2_ref = P['A2P'] if ring else P['A2E']
            if len(parts) == 3:
                self._open_claw()
            self._lift_to(P['ZHI'])
            self._servos_to(self.a1_ref, self.a2_ref, P['ASPD'])
            self._lift_to(P['ZOBRNG'] if ring else P['ZOBRAW'])
            return True, 'DONE', info
        if verb in ('GRAB', 'PICK'):
            slot = int(parts[1])
            if len(parts) != 3 or parts[2] != 'H':
                return False, 'ERR ARG', info
            self._open_claw()                                      # arm.c 下降前先张开爪子
            self._tt_go(slot)
            self._lift_to(P['ZGRAB'] if verb == 'GRAB' else P['ZPLC'])
            self.advance(CLAW_CLOSE_S)                             # 夹爪合上的那一刻(原料盘在转的话物料又走了一点)
            c = self.claw()
            got = None
            if verb == 'GRAB' and self.zone == 'RAW':
                best = min(self.raw_items, key=lambda it: np.linalg.norm(self.raw_pos(it) - c), default=None)
                if best is not None and np.linalg.norm(self.raw_pos(best) - c) <= 8.0 and self.claw_open:
                    got = best['color']
                    self.raw_items.remove(best)
            elif verb == 'PICK' and self.zone in ('ROUGH', 'TEMP'):
                cands = [(k, st[-1]) for (z, k), st in self.rings.items() if z == self.zone and st]
                best = min(cands, key=lambda ks: np.linalg.norm(ks[1]['pos'] - c), default=None)
                if best is not None and np.linalg.norm(best[1]['pos'] - c) <= 8.0 and self.claw_open:
                    got = best[1]['color']
                    self.rings[(self.zone, best[0])].pop()
            if got is None:
                self.air += 1
            else:
                if self.tray[slot] is not None:
                    self.collisions += 1
                self.tray[slot] = got
            self.advance(max(0.0, self.params['CLWAIT'] / 1000.0 - CLAW_CLOSE_S))
            self._lift_to(P['ZHI'])
            self._servos_to(P['A1D'], P['A2R'], P['ASPD'])
            self._lift_to(P['ZDROP'])
            self._tt_wait()
            self._claw_wait()
            self._lift_to(P['ZHI'])
            self.claw_open = True
            return True, 'DONE', info
        if verb == 'TAKE':
            slot = int(parts[1])
            self._open_claw()                                      # arm.c 一开始就张开爪子
            self.held = self.tray[slot]
            self.tray[slot] = None
            self._tt_go(slot)
            self._lift_to(P['ZHI'])
            self._servos_to(P['A1D'], P['A2R'], P['ASPD'])
            self._tt_wait()
            self._lift_to(P['ZDROP'])
            self._claw_wait()
            self._lift_to(P['ZHI'])
            self.claw_open = False
            return True, 'DONE', info
        if verb == 'DROP':
            stack = len(parts) == 2 and parts[1] == 'S'
            self._lift_to(P['ZSTK'] if stack else P['ZPLC'])
            if self.held is None:
                self.air += 1
            elif self.zone in ('ROUGH', 'TEMP'):
                self._place_held(stack)
            self.held = None
            self.claw_open = True
            self._claw_wait()
            self._lift_to(P['ZHI'])
            self.advance(self._servo_t(P['A2R'] - self.a2, P['ASPD']))
            self.a2 = P['A2R']
            return True, 'DONE', info
        if verb == 'STOW':
            self._lift_to(P['ZHI'])
            self._servos_to(P['A1H'], P['A2R'], P['ASPD'])
            return True, 'DONE', info
        return False, 'ERR CMD', info


class SimVision:
    def __init__(self, world):
        self.w = world
        self.cfg = {'claw_px': {'RAW': [320.0, 240.0], 'RING': [320.0, 240.0]}}

    def claw(self, kind):
        cp = self.cfg['claw_px']
        v = cp.get(kind) or cp['RING']
        return (float(v[0]), float(v[1]))

    def has_pick(self):
        return self.cfg['claw_px'].get('PICK') is not None

    def set_pick(self, uv, r=None):
        if uv is None:
            self.cfg['claw_px'].pop('PICK', None)
        else:
            self.cfg['claw_px']['PICK'] = [float(uv[0]), float(uv[1])]

    TRUE_PICK = (338.0, 214.0)          # 物料在爪子正下方时，它顶面圆心在画面里的位置(顶面比圆环高，有视差，不是 TRUE_CLAW)
    last_pick_r = None
    hide_pick = ()                      # 测试用：这些颜色的物料在圆环上认不到

    def pick_px(self, color_id, n=None):
        """圆环上(最上面一层)这个颜色的物料顶面圆心的像素位置。"""
        w = self.w
        self.last_pick_r = None
        tops = [st[-1] for (z, k), st in w.rings.items() if z == w.zone and st and st[-1]['color'] == int(color_id)]
        if w.zone not in ('ROUGH', 'TEMP') or not tops or int(color_id) in self.hide_pick:
            w.advance(_frames_dt(n))
            return None
        c = w.claw()
        it = min(tops, key=lambda i: np.linalg.norm(i['pos'] - c))
        e = w.pixel_error(it['pos'], 'PICK', _frames_dt(n), n)
        if e is None:
            return None
        self.last_pick_r = 25.0 * w.scale['PICK']
        return (self.TRUE_PICK[0] + e[0], self.TRUE_PICK[1] + e[1])

    def pick_error(self, color_id, n=None):
        if not self.has_pick():
            return None
        p = self.pick_px(color_id, n)
        if p is None:
            return None
        c = self.claw('PICK')
        return (p[0] - c[0], p[1] - c[1])

    def material_px(self, color_id, n=None):
        e = self.material_error(color_id, n)
        if e is None:
            return None
        c = self.claw('RAW')
        return (c[0] + float(e[0]), c[1] + float(e[1]))

    def ring_px(self, n=None, expect=None, any_size=False, max_px=None):
        if expect is not None:                            # 和真的 Vision 一样：离 expect 最近的那个(不是离爪子最近的)
            rings = self.ring_list(n)
            if not rings:
                return None
            best = min(rings, key=lambda r: (r[0] - expect[0]) ** 2 + (r[1] - expect[1]) ** 2)
            if max_px and math.hypot(best[0] - expect[0], best[1] - expect[1]) > max_px:
                return None
            self.last_ring_rmax = best[2]
            return (best[0], best[1])
        e = self.ring_error(n, max_px=max_px)
        self.last_ring_rmax = None if e is None else 48.25 * self.scale('RING')
        if e is None:
            return None
        c = self.claw('RING')
        return (c[0] + float(e[0]), c[1] + float(e[1]))

    def material_detector_obj(self):
        return None

    def open(self):
        return self

    def close(self):
        pass

    def scale(self, kind):
        return self.w.scale[kind]

    def bounds(self, kind, inset=8.0):
        return ((-280.0 + inset, -200.0 + inset), (280.0 - inset, 200.0 - inset))

    TRUE_CLAW = (320.0, 240.0)          # 真的爪子点；配置里的 claw_px 不对，对准就会偏(vclaw 测的就是它)

    def _off(self, kind, e):
        if e is None:
            return None
        c = self.claw(kind)
        return (float(e[0]) + self.TRUE_CLAW[0] - c[0], float(e[1]) + self.TRUE_CLAW[1] - c[1])

    def material_error(self, color_id, n=None):
        w = self.w
        items = [it for it in w.raw_items if it['color'] == int(color_id)]
        if not items:
            w.advance(MEASURE_S)
            return None
        c = w.claw()
        it = min(items, key=lambda i: np.linalg.norm(w.raw_pos(i) - c))
        dt = MEASURE_S if not n else 0.05 + 0.07 * int(n)          # 一帧大约 0.07 秒(n 帧取平均)
        return self._off('RAW', w.pixel_error(w.raw_pos(it), 'RAW', dt))

    def material_stream(self, color_id):
        """和真的 Vision.material_stream 一样：每认完一帧 yield (这帧拍下的时间, 像素位置或 None, 画面变了的比例或 None)；认一帧要 FRAME_S。"""
        w = self.w
        t_prev = None
        while True:
            t_cap, p = w.t, None
            moved = None
            # color_id=None：只看画面动没动
            if t_prev is not None:
                moved = 0.03 if (w.plate_phase(t_cap)[1] or w.plate_phase(t_prev)[1]) else abs(float(w.rng.normal(0, 0.0005)))
            t_prev = t_cap
            items = [it for it in w.raw_items if it['color'] == int(color_id)] if (w.zone == 'RAW' and color_id is not None) else []
            if items:
                c = w.claw()
                it = min(items, key=lambda i: np.linalg.norm(w.raw_pos(i) - c))
                e = w.scale['RAW'] * (w._cam() @ (w.raw_pos(it) - c)) + w.rng.normal(0, w.noise_px, 2)
                if abs(e[0]) <= 300 and abs(e[1]) <= 220:
                    p = (self.TRUE_CLAW[0] + float(e[0]), self.TRUE_CLAW[1] + float(e[1]))
            w.advance(FRAME_S)
            yield t_cap, p, moved

    def material_errors(self, colors):
        """同一帧里认这几个颜色(和真的 Vision.material_errors 一样)。"""
        w = self.w
        out = {}
        c = w.claw()
        for col in colors:
            items = [it for it in w.raw_items if it['color'] == int(col)]
            q = None
            if items:
                it = min(items, key=lambda i: np.linalg.norm(w.raw_pos(i) - c))
                e = w.scale['RAW'] * (w._cam() @ (w.raw_pos(it) - c)) + w.rng.normal(0, w.noise_px, 2)
                if abs(e[0]) <= 300 and abs(e[1]) <= 220:
                    q = self._off('RAW', (float(e[0]), float(e[1])))
            out[int(col)] = q
        w.advance(0.05 + 0.07 * len(colors))
        return out

    def wait_still(self, color_id, timeout_s=10.0, **kw):
        """和真的 Vision.wait_still 一样：要在画面里检测到才算(不在画面里 = 看不到)。"""
        w = self.w
        w.advance(0.8)
        items = [it for it in w.raw_items if it['color'] == int(color_id)]
        if not items:
            return False, None
        c = w.claw()
        it = min(items, key=lambda i: np.linalg.norm(w.raw_pos(i) - c))
        e = w.scale['RAW'] * (w._cam() @ (w.raw_pos(it) - c))
        if abs(e[0]) > 280 or abs(e[1]) > 200:
            return False, None
        return not w.plate_phase()[1], (0.0, 0.0)

    def ring_error(self, n=None, max_px=None):
        w = self.w
        if w.zone not in ('ROUGH', 'TEMP'):
            return None
        c = w.claw()
        k = min((1, 2, 3), key=lambda k: np.linalg.norm(w.ring_center(k) - c))
        e = self._off('RING', w.pixel_error(w.ring_center(k), 'RING', _frames_dt(n), n))
        if e is not None and max_px and math.hypot(e[0] + self.claw('RING')[0] - self.TRUE_CLAW[0],
                                                     e[1] + self.claw('RING')[1] - self.TRUE_CLAW[1]) > max_px:
            return None                                       # 离爪子点太远：不是要对的那个环
        return e

    def ring_list(self, n=None):
        """画面里看得到的所有圆环 [(u, v, 最外圈半径)]，按 u 排。"""
        w = self.w
        w.advance(_frames_dt(n))
        if w.zone not in ('ROUGH', 'TEMP'):
            return []
        c = w.claw()
        out = []
        sd = w.noise_px * (math.sqrt(5.0 / max(1, int(n))) if n else 1.0)
        for k in (1, 2, 3):
            e = w.scale['RING'] * (w._cam() @ (w.ring_center(k) - c)) + w.rng.normal(0, sd, 2)
            if abs(e[0]) > 280 or abs(e[1]) > 200:
                continue
            out.append((self.TRUE_CLAW[0] + float(e[0]), self.TRUE_CLAW[1] + float(e[1]), 48.25 * w.scale['RING']))
        out.sort(key=lambda r: r[0])
        return out


# ======================================================================================================
def run_mission(world, hooks, stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'RAW', 'ROUGH', 'TEMP', 'START1'), log=lambda m: None, nav=None):
    """按停车点顺序把 hooks.task 跑一遍。返回实际去了哪些停车点。
    nav：模拟新的导航(dict)，不给 = 和以前一样只调 task。给了：一开始 prepare(nav['prepare'] 默认 True)、start_clock；
      开到每个停车点按 leg_s 秒；到了先 set_home_eta(home_s[角色])；开往下一个停车点之前问 go_home_now，回 True 就直接回家。
      例：nav=dict(leg_s=10, home_s={'QR': 25, 'RAW': 30, 'ROUGH': 22, 'TEMP': 25})"""
    visits = {}
    stops = list(stops)
    done = []
    home_s = (nav or {}).get('home_s') or {}
    leg = float((nav or {}).get('leg_s', 10.0))
    if nav is not None:
        if nav.get('prepare', True):
            hooks.prepare(world.link, log)
        hooks.start_clock()
    i = 0
    while i < len(stops):
        stop = stops[i]
        role = role_of(stop)
        if nav is not None:
            prev = role_of(done[-1]) if done else None
            world.advance(float(home_s.get(prev, 25.0)) if (role == 'START' and prev) else leg)   # 开过来
            hooks.set_home_eta(0.0 if role == 'START' else float(home_s.get(role, 25.0)))
        visits[role] = visits.get(role, 0) + 1
        world.arrive(role, visits[role])
        hooks.task(stop, world.link, log)
        done.append(stop)
        if nav is not None and i + 1 < len(stops) and role_of(stops[i + 1]) != 'START':
            nxt = stops[i + 1]
            if hooks.go_home_now(nxt, leg, float(home_s.get(role_of(nxt), 25.0))):
                stops = stops[:i + 1] + [s for s in stops[i + 1:] if role_of(s) == 'START'][-1:]
        i += 1
    return done


class FakeCtx:
    """导航的 ctx：aborted() 和 obstacles()(地图上的障碍物 [(x, y, 半径)])。"""

    def __init__(self, obstacles=None, abort=None):
        self._obs = list(obstacles or [])
        self._abort = abort or (lambda: False)

    def aborted(self):
        return bool(self._abort())

    def obstacles(self):
        return list(self._obs)


QR_STOPS = {'QR': [2100, 1200, 90], 'RAW': [1200, 2100, 180], 'ROUGH': [1200, 340, 0], 'TEMP': [340, 1200, -90],
            'START1': [2250, 2250, 180], 'START2': [2250, 150, 90]}     # 用户配置里的停车点(10-06)


LEGACY = dict(raw_any_order=False, raw_wheels_first=False, wheels_first=False)
LIVE_DEFAULTS = dict(raw_any_order=True, raw_wheels_first=True, wheels_first=True)


def make(world, cfg_extra=None, log=lambda m: None, raw=None):
    """raw：配置文件里 mission_cfg 以外的东西(stops、car_length_mm…)，QR 前后挪着找码要用 stops。"""
    cfg = dict(time_limit_s=1e9, servo_cal_file=None, vision_cal_file='',      # 测试里不限时、不读写文件
               ring_order={z: world.ring_order(z) for z in ('ROUGH', 'TEMP')})  # 假摄像头装的方向是随机的
    # 以前的测试是按"按任务码顺序抓、先动手臂"写的：这里固定成那样；测比赛默认(不按顺序、先动车轮)的用 LIVE_DEFAULTS
    cfg.update(LEGACY)
    cfg.update(cfg_extra or {})
    rc = dict(raw or {})
    rc['mission_cfg'] = cfg
    return MissionHooks(rc, log=log, vision=world.vision, now=world.now, sleep=world.advance)


def rings_summary(world, zone):
    return {k: [it['color'] for it in world.rings.get((zone, k), [])] for k in (1, 2, 3)}


def time_report(world):
    rows = sorted(world.time_by_verb.items(), key=lambda kv: -kv[1])
    return '  '.join(f'{k}={v:.0f}s' for k, v in rows if v >= 0.5)


class MissionSimTests(unittest.TestCase):
    CODE = '156+123+516+231'

    def test_full_mission_many_seeds(self):
        errs, times = [], []
        for seed in range(12):
            w = SimWorld(seed=seed, code=self.CODE)
            h = make(w)
            run_mission(w, h)
            self.assertEqual(w.air, 0, f'seed {seed}: 夹空了 {w.air} 次')
            self.assertEqual(w.loose, 0, f"seed {seed}: 物料掉在半路上 {w.loose} 次")
            self.assertEqual(w.collisions, 0, f'seed {seed}: 碰倒/叠错 {w.collisions} 次')
            self.assertEqual((h.stats.grab_ok, h.stats.grab_total, h.stats.place_ok, h.stats.place_total), (6, 6, 12, 12), f'seed {seed}')
            self.assertTrue(all(v == [] for v in rings_summary(w, 'ROUGH').values()), f'seed {seed}: {rings_summary(w, "ROUGH")}')
            self.assertEqual(rings_summary(w, 'TEMP'), {1: [1, 1], 2: [5, 5], 3: [6, 6]}, f'seed {seed}')
            self.assertEqual(w.tray, {1: None, 2: None, 3: None})
            errs += [e for (_, _, _, e, st) in w.placed]
            times.append(w.t)
        errs = np.array(errs)
        print(f'\n  [整场 12 组随机] 放置真实误差：平均 {errs.mean():.2f}mm / 95% {np.percentile(errs, 95):.2f}mm / 最大 {errs.max():.2f}mm；'
              f'操作用时估计(不含路线行驶) 平均 {np.mean(times):.0f}s')
        self.assertLess(errs.mean(), 1.5)
        self.assertLess(errs.max(), 4.0)

    def test_fast_params_are_faster(self):
        w1 = SimWorld(seed=1, code=self.CODE)
        run_mission(w1, make(w1))
        w2 = SimWorld(seed=1, code=self.CODE, params=FAST_PARAMS)
        run_mission(w2, make(w2))
        print(f'\n  [用时估计] 默认速度参数 {w1.t:.0f}s → 提速参数 {w2.t:.0f}s (不含路线行驶)')
        self.assertLess(w2.t, w1.t * 0.8)
        self.assertEqual(w2.air + w2.collisions, 0)

    def test_screen_content(self):
        w = SimWorld(seed=1, code=self.CODE)
        h = make(w)
        run_mission(w, h)
        s = w.screen
        self.assertEqual((s['t0'], s['t7']), ('156+123+', '516+231'))     # 屏上的任务码带上两组之间的 +
        self.assertEqual(s['t2'], 'GRAB 6')                                # 回家以后只显示做成的个数
        self.assertEqual(s['t3'], 'PLACE 12')
        self.assertEqual(s['t1'], 'DONE')
        self.assertEqual(s['t5'], 'B1 RED1 BLK2 LBL3')
        self.assertEqual(s['t6'], 'B2 BLK2 RED3 LBL1')
        self.assertTrue(s['t4'].startswith('T='))
        self.assertTrue(all(all(ord(ch) < 128 for ch in v) for v in s.values()))

    def test_chassis_returns_to_stop_point(self):
        w = SimWorld(seed=2, code=self.CODE)
        h = make(w)
        run_mission(w, h)
        self.assertLess(abs(h.act.disp['F']), 1.0)
        self.assertLess(abs(h.act.disp['S']), 1.0)

    def test_pickback_orders_all_work(self):
        for mode in ('code', 'reverse', 'near'):
            w = SimWorld(seed=3, code=self.CODE)
            h = make(w, dict(pickback_order=mode))
            run_mission(w, h)
            self.assertEqual((w.air, w.collisions), (0, 0), mode)
            self.assertEqual(rings_summary(w, 'TEMP'), {1: [1, 1], 2: [5, 5], 3: [6, 6]}, mode)

    def test_no_qr_still_drives_on(self):
        w = SimWorld(seed=3, code=self.CODE, qr_present=False)
        h = make(w)
        run_mission(w, h)
        self.assertEqual(w.air, 0)
        self.assertEqual([r for r in w.requests if r.split()[0] in ('OBS', 'GRAB', 'PICK', 'TAKE', 'DROP')], [])
        self.assertEqual(w.screen.get('t4', '')[:2], 'T=')
        self.assertEqual(h.stats.grab_total, 0)

    def test_bad_qr_code(self):
        w = SimWorld(seed=4, code='756+123+516+231')
        h = make(w)
        run_mission(w, h)
        self.assertEqual(h.stats.grab_total, 0)

    def test_not_calibrated_disables_arm(self):
        w = SimWorld(seed=5, code=self.CODE, armok=False)
        h = make(w)
        run_mission(w, h)
        self.assertIsNotNone(h.disabled)
        self.assertEqual([r for r in w.requests if r.split()[0] in ('OBS', 'GRAB', 'PICK', 'TAKE', 'DROP', 'STOW', 'AP')], [])

    def test_abort_stops_route(self):
        w = SimWorld(seed=6, code=self.CODE, abort_at=('GRAB', 2))
        h = make(w)
        with self.assertRaises(Abort):
            run_mission(w, h)

    def test_abort_via_ctx_between_commands(self):
        w = SimWorld(seed=7, code=self.CODE)
        h = make(w)
        h.ctx = type('C', (), {'aborted': staticmethod(lambda: len(w.requests) > 60)})()
        with self.assertRaises(Abort):
            run_mission(w, h)

    def test_missing_color_is_skipped_and_stack_without_lower_is_skipped(self):
        """第一批的黑色没有：第二批的黑色在暂存区没有同色的可叠，规则只许码垛 -> 不放、不算放置(以前平放：0 分还多算一次放置)。"""
        w = SimWorld(seed=8, code=self.CODE, missing_batch1=(5,))        # 第一批的黑色没有
        lines = []
        h = make(w, log=lines.append)
        run_mission(w, h, log=lines.append)
        self.assertEqual(h.stats.grab_ok, 5)
        temp = rings_summary(w, 'TEMP')
        self.assertEqual(temp[1], [1, 1])
        self.assertEqual(temp[3], [6, 6])
        self.assertEqual(temp[2], [])                                   # 第二批的黑色没有下垫：不放
        self.assertEqual(w.collisions, 0)
        # 粗加工区 2+3、暂存区第一批 2、码垛 2：一共 9 次，都成功；没放的那个不算
        self.assertEqual((h.stats.place_ok, h.stats.place_total), (9, 9))
        self.assertIn('规则只许码垛，不放、不算放置', '\n'.join(lines))
        self.assertEqual(w.screen['t3'], 'PLACE 9')
        self.assertEqual([c for c in w.tray.values() if c], [5])        # 黑色留在车上

    def test_live_defaults_whatever_stops_under_the_claw_and_wheels_first(self):
        """比赛默认(10-10 用户要的)：原料区爪子下面停的是哪个就夹哪个、对准先慢慢动车轮：全抓到、全放对、不空夹。"""
        for seed in range(3):
            w = SimWorld(seed=300 + seed, code=self.CODE)
            h = make(w, dict(LIVE_DEFAULTS))
            run_mission(w, h)
            self.assertEqual(h.stats.grab_ok, 6)
            self.assertEqual(h.stats.place_ok, 12)
            self.assertEqual(w.air, 0)
            self.assertLess(max(e for (_, _, _, e, st) in w.placed if not st), 2.5)

    def test_stm32_error_on_one_grab_is_recovered(self):
        w = SimWorld(seed=9, code=self.CODE, fail_cmd=('GRAB', 2))
        h = make(w)
        run_mission(w, h)
        self.assertEqual(h.stats.grab_total, 6)
        self.assertEqual(h.stats.grab_ok, 5)                            # 失败的那个被跳过，其余照常
        self.assertTrue(any(r == 'STOW' for r in w.requests))

    def test_time_limit_skips_remaining_work(self):
        w = SimWorld(seed=10, code=self.CODE)
        h = make(w, dict(time_limit_s=60.0))
        run_mission(w, h)
        self.assertLess(h.stats.place_total, 12)
        self.assertEqual(w.screen['t1'], 'DONE')

    def test_noisy_camera_still_places_within_ring2(self):
        """摄像头噪声大(1.5 像素)、停车误差 15mm：平放的照样在 2 环以内(<4mm)；码垛容差本来就是 2.5mm(不掉下来就得分)，放宽到 6mm。"""
        errs, stacks = [], []
        for seed in range(8):
            w = SimWorld(seed=100 + seed, code=self.CODE, noise_px=1.5, stop_err_mm=15.0)
            h = make(w)
            run_mission(w, h)
            errs += [e for (_, _, _, e, st) in w.placed if not st]
            stacks += [e for (_, _, _, e, st) in w.placed if st]
            self.assertEqual(w.air, 0)
        errs = np.array(errs)
        print(f'\n  [噪声 1.5px、停车误差 15mm] 放置真实误差：平均 {errs.mean():.2f}mm / 最大 {errs.max():.2f}mm'
              f'(码垛最大 {max(stacks):.2f}mm)')
        self.assertLess(errs.max(), 4.0)
        self.assertLess(max(stacks), 6.0)

    def test_cached_calibration_is_reused(self):
        w = SimWorld(seed=11, code=self.CODE)
        h = make(w)
        run_mission(w, h)
        aps_cold = sum(1 for r in w.requests if r.startswith('AP'))
        n0 = len(w.requests)
        w.placed.clear()
        w.rings.clear()
        w.tray = {1: None, 2: None, 3: None}
        h2 = MissionHooks({'mission_cfg': dict(time_limit_s=1e9, servo_cal_file=None)}, log=lambda m: None, vision=w.vision,
                          now=w.now, sleep=w.advance, store=h.store)
        run_mission(w, h2)
        aps_warm = sum(1 for r in w.requests[n0:] if r.startswith('AP'))
        print(f'\n  [校准缓存] 第一次 AP {aps_cold} 次(含探测)，第二次 {aps_warm} 次')
        self.assertLessEqual(aps_warm, aps_cold)
        self.assertEqual(w.air, 0)


    def _fail_after_take(self, w, verbs):
        """从转盘取出物料(TAKE)以后，下面这几条指令各失败一次(按顺序)：模拟回不到对准姿态、升降出错。"""
        orig, st = w._handle, {'took': False, 'left': list(verbs)}

        def handle(text):
            v = text.split(' ', 1)[0]
            if v == 'TAKE' and not st['took']:
                st['took'] = True
            elif st['took'] and st['left'] and v == st['left'][0]:
                st['left'].pop(0)
                return False, 'ERR TIMEOUT', []
            return orig(text)
        w._handle = handle
        return st

    def test_material_put_back_when_placing_fails_after_take(self):
        """取出物料以后回不到对准姿态(AP 出错)：放回转盘再收臂。原来直接夹着物料收臂，下一个物料 OBS RING O 一张开爪子它就掉在半路上。"""
        w = SimWorld(seed=12, code=self.CODE)
        h = make(w)
        st = self._fail_after_take(w, ['AP'])
        run_mission(w, h)
        self.assertEqual(st['left'], [])
        self.assertEqual(w.loose, 0, '物料掉在半路上了')
        self.assertEqual(w.collisions, 0)
        self.assertIsNone(h.disabled)
        self.assertEqual(h.stats.place_ok, 11)                  # 只有出错的那一次没放成
        self.assertEqual(w.tray, {1: None, 2: None, 3: None})   # 放回转盘的那个后来在暂存区照常放了

    def test_put_back_failure_keeps_claw_closed_and_stops_arm_work(self):
        """放回转盘也失败了：爪子保持夹紧收臂，本轮不再夹放(不再张开爪子)。"""
        w = SimWorld(seed=12, code=self.CODE)
        h = make(w)
        st = self._fail_after_take(w, ['AP', 'LIFT'])
        n_obs = []
        orig = w._handle

        def handle(text):
            if text.startswith('OBS') and h.disabled:
                n_obs.append(text)
            return orig(text)
        w._handle = handle
        run_mission(w, h)
        self.assertEqual(st['left'], [])
        self.assertIsNotNone(h.disabled)
        self.assertEqual(w.loose, 0)
        self.assertIsNotNone(w.held)                            # 还夹在爪子里，没有乱放
        self.assertEqual(n_obs, [])

    def test_ring_with_our_leftover_material_counts_as_taken(self):
        """取回失败、物料还留在环上(记录里有)：这个环一定是占着的，不用看画面也不往上放。"""
        w = SimWorld(seed=14, code=self.CODE)
        h = make(w)
        h.on_ring[('ROUGH', 2)] = [object()]
        self.assertTrue(h._ring_taken('ROUGH', 2))
        self.assertFalse(h._ring_taken('ROUGH', 1))              # 假摄像头没有 ring_centre_free：看不清 = 照常放

    def test_blind_pickback_starts_from_the_recorded_pose(self):
        """取回时先按物料对准，手臂动过以后物料和圆环都认不到了：要回到放下时记下的位置再直接夹，不能在动过的位置夹。"""
        w = SimWorld(seed=13, code=self.CODE)

        def exact_move(cmd, val):                               # 底盘走得准(不然回到记下的位置本身就差几毫米，测不出手臂的问题)
            w.advance(1.0 + abs(val) / 150.0)
            if cmd == 'S':
                w.off_s += val
            else:
                w.off_f += val
            return True, 'DONE'
        w.move = exact_move
        h = make(w)
        real = h._align_covered

        def lost_after_moving(color, key, label, confirm):
            if key != 'PICK':
                return real(color, key, label, confirm)
            a1, a2 = h.arm.read_angles()
            h.arm.ap(a1 + 4.0, a2 + 25.0)                       # 对准过程中手臂挪开了
            return None, None
        h._align_covered = lost_after_moving
        run_mission(w, h, stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1'))
        self.assertEqual(w.air, 0, '在挪开的位置夹空了')
        self.assertEqual(rings_summary(w, 'ROUGH'), {1: [], 2: [], 3: []})
        self.assertEqual(w.loose + w.collisions, 0)

    def test_return_to_pose_retries_when_short(self):
        """取完物料回到对准姿态：回读差 0.4° 以上就再转一次(最多再 2 次)；读不到对准角度就报错不乱转。"""
        class Arm:
            def __init__(self, short):
                self.short, self.cmds, self.at = list(short), [], (0.0, 0.0)

            def ap(self, a1, a2):
                self.cmds.append((a1, a2))
                d = self.short.pop(0) if self.short else (0.0, 0.0)
                self.at = (a1 - d[0], a2 - d[1])

            def read_angles(self):
                return self.at
        h = make(SimWorld(seed=3, code=self.CODE))
        h.arm = Arm([(0.9, 0.0), (0.0, -0.5)])
        h._return_to_pose(300.0, -500.0)
        self.assertEqual(len(h.arm.cmds), 3)
        h.arm = Arm([(0.2, 0.1)])                              # 差得不多：不再转
        h._return_to_pose(300.0, -500.0)
        self.assertEqual(len(h.arm.cmds), 1)
        h.arm = Arm([(2.0, 0.0)] * 5)                          # 一直到不了：最多再转 2 次
        h._return_to_pose(300.0, -500.0)
        self.assertEqual(len(h.arm.cmds), 3)
        from arm_link import ArmError
        with self.assertRaises(ArmError):
            h._return_to_pose(None, -500.0)

        class BadRead(Arm):                                    # 回读失败：物料已经在爪子里，照常放，不报错
            def read_angles(self):
                raise ArmError('A? 1 失败：ERR READ')
        h.arm = BadRead([(2.0, 0.0)])
        h._return_to_pose(300.0, -500.0)
        self.assertEqual(len(h.arm.cmds), 1)


class ZoneFlowTests(unittest.TestCase):
    """粗加工区 / 暂存区的流程：到工位先看清三个圆环、按任务码顺序直接开到每个环、只前后挪不横移、手臂微调、放完不缩回。"""
    CODE = '156+123+516+231'

    def _run(self, seeds, wkw=None, cfg=None, stops=None):
        out = []
        for seed in seeds:
            w = SimWorld(seed=seed, code=self.CODE, **(wkw or {}))
            lines = []
            h = make(w, cfg, log=lines.append)
            if stops:
                run_mission(w, h, stops=stops, log=lines.append)
            else:
                run_mission(w, h, log=lines.append)
            out.append((seed, w, h, '\n'.join(lines)))
        return out

    def _assert_all_good(self, w, h, text, seed, batches=2):
        self.assertEqual((w.air, w.loose, w.collisions), (0, 0, 0), f'seed {seed}\n{text}')
        n = 6 * batches
        self.assertEqual((h.stats.place_ok, h.stats.place_total), (n, n), f'seed {seed}\n{text}')
        if batches == 2:
            self.assertEqual(rings_summary(w, 'TEMP'), {1: [1, 1], 2: [5, 5], 3: [6, 6]}, f'seed {seed}')

    def _zone_s_extent(self, w):
        """工位里车轮横移离停车点最远到过多少毫米(每次到工位从 0 算，做完会挪回去)。"""
        cur = worst = 0.0
        last = None
        for c, v, z in w.moves:
            if z != last:
                cur, last = 0.0, z
            if z in ('ROUGH', 'TEMP') and c == 'S':
                cur += v
                worst = max(worst, abs(cur))
        return worst

    def test_never_strafes_in_the_zones(self):
        """粗加工区、暂存区里手臂伸缩够得着时底盘只前后挪(横移会压进工位)；原料区照常可以横移。"""
        for seed, w, h, text in self._run(range(6)):
            self._assert_all_good(w, h, text, seed)
            zone_s = [m for m in w.moves if m[2] in ('ROUGH', 'TEMP') and m[0] == 'S']
            self.assertEqual(zone_s, [], f'seed {seed}: {zone_s}')

    def test_wheels_move_closer_when_the_arm_cannot_reach(self):
        """车停得离圆环太远/太近，手臂伸缩到头也够不着：车轮横着挪(只挪够不着的那一段)，照样放对；离停车点不超过 ring_strafe_max_mm(20)。"""
        for side in (55.0, -55.0):                               # 手臂伸缩 ±44mm + 横移 20mm 够得着(停车误差 ±8mm)
            for seed, w, h, text in self._run(range(3), dict(cam_deg=90.0, park_side={'ROUGH': side, 'TEMP': side})):
                self._assert_all_good(w, h, text, seed)
                self.assertIn('手臂伸缩够不着：车轮横着挪', text)
                ext = self._zone_s_extent(w)
                self.assertGreater(ext, 5.0, text)
                self.assertLessEqual(ext, 20.0, text)

    def test_strafe_limit_is_never_exceeded(self):
        """远近差得太多(手臂伸缩加上横移 20mm 也够不着)：不放(物料留在车上)，车轮横移不超过 20mm；ring_strafe_max_mm=0 时绝不横移。"""
        for seed, w, h, text in self._run([1], dict(cam_deg=90.0, park_side={'ROUGH': 120.0, 'TEMP': 120.0}),
                                          stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1')):
            self.assertEqual((w.air, w.loose, w.collisions), (0, 0, 0), text)
            self.assertLessEqual(self._zone_s_extent(w), 20.0, text)
            self.assertIn('车轮横着也挪到上限', text)
        for seed, w, h, text in self._run([2], dict(cam_deg=90.0, park_side={'ROUGH': 60.0, 'TEMP': 60.0}),
                                          cfg=dict(ring_strafe_max_mm=0), stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1')):
            self.assertEqual((w.air, w.loose, w.collisions), (0, 0, 0), text)
            self.assertEqual([m for m in w.moves if m[2] in ('ROUGH', 'TEMP') and m[0] == 'S'], [], text)

    def test_drives_straight_to_each_ring_in_code_order(self):
        """到工位先看清三个圆环，按任务码顺序一次开到位：每段一条 ±150/300mm 的指令，对准时底盘只修很少几下(其余靠手臂)。"""
        code = '156+213+516+231'                                 # 第一批去 2、1、3 号环：先不挪、再前进 150、再后退 300
        for seed in range(4):
            w = SimWorld(seed=seed, code=code, cam_deg=90.0)
            lines = []
            h = make(w, log=lines.append)
            run_mission(w, h, stops=('QR', 'RAW', 'ROUGH'), log=lines.append)
            text = '\n'.join(lines)
            self.assertIn('看到 3 个圆环(认出 1、2、3 号)', text, text)
            self.assertEqual([p[1] for p in w.placed], [2, 1, 3], text)
            big = [v for c, v, z in w.moves if z == 'ROUGH' and abs(v) > 60]
            # 放：去环1 +150、去环3 -300；取回(按任务码顺序)：环2 +150、环1 +150、环3 -300；最后回停车点 +150
            self.assertEqual(len(big), 6, (big, text))
            # 第一次对准时手臂要先小幅动几下测 J(探测)，之后可能前后修一下，所以放宽到 ±30mm；
            # 最后回停车点那一条要把对准时前后修的都退回去(每次对准最多 ring_fix_max_mm)，放宽到 ±60mm
            self.assertTrue(all(min(abs(abs(v) - 150), abs(abs(v) - 300)) <= 30 for v in big[:-1]), big)
            self.assertLessEqual(abs(abs(big[-1]) - 150), 60, big)
            fix = [v for c, v, z in w.moves if z == 'ROUGH' and abs(v) <= 60 and abs(v) != 40]                # 40 = 第一次测"底盘挪 40mm 画面怎么动"
            self.assertLessEqual(len(fix), 4, (fix, text))
            self.assertTrue(all(p[3] < 3.5 for p in w.placed), w.placed)       # 容差 2mm(差不多就行)+测量噪声

    def test_side_ring_missed_when_parked_at_ring2(self):
        """车停在 2 号环前面，画面里一边的圆环没认出来(被爪子挡住、反光)：不能当成"车停在 3 号(1 号)环"把整排认错一位
        (10-10 实车：只看到两个环，认成了 2、3 号，结果开到环1 的位置没有环)。"""
        for drop in ('right', 'left'):
            for seed in range(3):
                w = SimWorld(seed=seed, code=self.CODE, cam_deg=90.0)
                h = make(w)
                orig = w.vision.ring_list

                def ring_list(n=None, orig=orig, drop=drop):
                    r = orig(n)
                    if len(r) == 3:
                        return r[:2] if drop == 'right' else r[1:]
                    return r
                w.vision.ring_list = ring_list
                lines = []
                run_mission(w, h, log=lines.append)
                text = '\n'.join(lines)
                self._assert_all_good(w, h, text, seed)
                self.assertIn('没认出来', text)

    def test_car_parked_between_rings_still_finds_the_right_rings(self):
        """车停得离 2 号环偏了 55mm(两个环中间偏 2 号)：照样认对、放对。
        (停在 1 号/3 号环正前面、画面里只看到两个环时，按"车停在 2 号环"算，会认错一位：车要停在 2 号环前面)"""
        for park in ({'ROUGH': 55.0, 'TEMP': -55.0}, {'ROUGH': -55.0, 'TEMP': 55.0}):
            for seed, w, h, text in self._run(range(3), dict(cam_deg=90.0, park=park), stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1')):
                self._assert_all_good(w, h, text, seed, batches=1)
                self.assertEqual(rings_summary(w, 'TEMP'), {1: [1], 2: [5], 3: [6]}, f'{park} seed {seed}')

    def test_covered_end_ring_missed_does_not_shift_numbering(self):
        """暂存区第二批：圆环上都放着第一批的物料，一头那个没认出来：不能当成"那一头没有环"把编号整体挪一位(会叠错颜色)。"""
        for seed in range(3):
            w = SimWorld(seed=seed, code=self.CODE, cam_deg=90.0)
            h = make(w)
            orig = w.vision.ring_list

            def ring_list(n=None, orig=orig, w=w):
                r = orig(n)
                if w.zone == 'TEMP' and any(st for (z, _k), st in w.rings.items() if z == 'TEMP') and len(r) == 3:
                    return r[1:]                                 # 最左边那个被物料盖住，没认出来
                return r
            w.vision.ring_list = ring_list
            lines = []
            run_mission(w, h, log=lines.append)
            self._assert_all_good(w, h, '\n'.join(lines), seed)

    def test_nominal_ring_positions_follow_ring_order(self):
        """没看清圆环(这里 ring_survey=false)时按名义位置走：前后方向按 ring_order 和"底盘前进时画面往哪边动"推，
        和看清圆环时的认法一致(ring_offset_mm 的正负号写反了也不会放错环)。"""
        from visual_servo import JacStore
        for seed in range(3):
            w = SimWorld(seed=seed, code=self.CODE, cam_deg=90.0)
            bad = {'ROUGH': {'1': -150.0, '2': 0.0, '3': 150.0}, 'TEMP': {'1': 150.0, '2': 0.0, '3': -150.0}}   # 正负号全反
            st = JacStore(None)
            jf = -w.scale['RING'] * (w.Rcam @ np.array([0.0, 1.0]))          # 底盘前进 1mm，圆环在画面里动多少像素
            st.put('RING', 'ch', [[-jf[1], jf[0]], [jf[0], jf[1]]])
            cfg = dict(time_limit_s=1e9, servo_cal_file=None, vision_cal_file='', ring_survey=False, ring_offset_mm=bad,
                       ring_order={z: w.ring_order(z) for z in ('ROUGH', 'TEMP')})
            lines = []
            h = MissionHooks({'mission_cfg': cfg}, log=lines.append, vision=w.vision, now=w.now, sleep=w.advance, store=st)
            run_mission(w, h, log=lines.append)
            self._assert_all_good(w, h, '\n'.join(lines), seed)

    def test_middle_ring_missed_is_filled_in(self):
        """中间那个圆环没认出来(只看到两头隔了两个间距的两个)：补上中间那个，不会把间距当成两倍。"""
        w = SimWorld(seed=1, code=self.CODE, cam_deg=90.0)
        h = make(w)
        orig = w.vision.ring_list

        def ring_list(n=None):
            r = orig(n)
            return [r[0], r[2]] if len(r) == 3 else r
        w.vision.ring_list = ring_list
        lines = []
        run_mission(w, h, stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1'), log=lines.append)
        text = '\n'.join(lines)
        self.assertIn('中间那个圆环没认出来', text)
        self._assert_all_good(w, h, text, 1, batches=1)
        self.assertLess(abs(h.vision.scale('RING') - 1.46), 0.1)

    def test_chassis_travels_less_than_commanded(self):
        """底盘实际走的距离比指令少 15% / 多 15%(FPPM 不准)：对准时前后修一下，照样放对，不横移。"""
        for gain in (0.85, 1.15):
            for seed, w, h, text in self._run(range(3), dict(f_gain=gain)):
                self._assert_all_good(w, h, text, seed)
                self.assertEqual([m for m in w.moves if m[2] in ('ROUGH', 'TEMP') and m[0] == 'S'], [])

    def test_no_retract_after_drop(self):
        """放下以后不缩回(不发 DROP，只降、松手、抬起)；drop_retract=True 时还是用 DROP。"""
        (seed, w, h, text), = self._run([1])
        self._assert_all_good(w, h, text, seed)
        self.assertFalse([r for r in w.requests if r.startswith('DROP')])
        (seed, w, h, text), = self._run([1], cfg=dict(drop_retract=True))
        self._assert_all_good(w, h, text, seed)
        self.assertEqual(len([r for r in w.requests if r.startswith('DROP')]), 12)

    def test_does_not_go_back_to_photograph_after_placing(self):
        """放完不再回去拍物料(learn_pick 默认关)：取回按圆环对准，照样全夹回来。"""
        (seed, w, h, text), = self._run([2])
        self._assert_all_good(w, h, text, seed)
        self.assertNotIn('顺便量一次', text)
        self.assertFalse(h.vision.has_pick())

    def test_wrong_ring_next_to_the_target_is_ignored(self):
        """对准时只认离爪子点不到半个圆环间距的圆环：离得远的(旁边那个环)当没看到，不会被当成目标去追。"""
        w = SimWorld(seed=3, code=self.CODE, cam_deg=90.0)
        h = make(w)
        run_mission(w, h, stops=('QR', 'RAW', 'ROUGH'))
        gate = h.ring_gate_px
        self.assertIsNotNone(gate)
        self.assertLess(abs(gate - 0.45 * 219.0), 6.0)
        w.arrive('ROUGH', 1)
        h.arm.obs('RING', open_claw=True)
        w.stop_err[:] = (0.0, 30.0)                              # 爪子离 2 号环 30mm：认
        self.assertIsNotNone(h._ring_measure()())
        w.stop_err[:] = (0.0, 75.0)                              # 正好在 1、2 号环中间：两个都太远，当没看到
        self.assertIsNone(h._ring_measure()())

    def test_servo_read_glitch_is_retried(self):
        """总线舵机偶尔读角度没回话(A? 回 ERR READ，10-10 实车上蓝色因为这个没放)：再读一次就好，不能整个物料跳过。"""
        w = SimWorld(seed=6, code=self.CODE, cam_deg=90.0)
        h = make(w)
        orig = w._handle
        st = {'n': 0}

        def handle(text):
            if text.startswith('A?'):
                st['n'] += 1
                if st['n'] % 7 in (3, 4):                      # 每 7 次里连着两次读不到
                    return False, 'ERR READ', []
            return orig(text)
        w._handle = handle
        lines = []
        run_mission(w, h, log=lines.append)
        text = '\n'.join(lines)
        self._assert_all_good(w, h, text, 6)
        self.assertNotIn('ERR READ', text)

    def test_two_frame_measurements_are_faster(self):
        """工位里对准时每次测量只拍 2 帧(zone_frames=2)：比 5 帧快，照样全放对，误差还在 2 环以内(<4mm)。"""
        tot, errs = {}, {}
        for nf in (2, 5):
            tot[nf], errs[nf] = 0.0, []
            for seed in range(4):
                w = SimWorld(seed=seed, code=self.CODE, cam_deg=90.0)
                lines = []
                h = make(w, dict(zone_frames=nf), log=lines.append)
                run_mission(w, h, stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1'), log=lines.append)
                self._assert_all_good(w, h, '\n'.join(lines), seed, batches=1)
                tot[nf] += w.t
                errs[nf] += [p[3] for p in w.placed]
        self.assertLess(tot[2], tot[5] - 4.0, tot)
        self.assertLess(max(errs[2]), 4.0, errs)

    def test_next_ring_starts_at_the_reach_of_the_last_one(self):
        """车离圆环那一排远了 30mm：对准过一个环以后，去下一个环时手臂伸缩直接伸到同样远(ring_precorrect)，
        不再每个环都 OBS(缩回观察姿态)再伸出来；刚看完三个环时手臂还在观察姿态，第一个环也不用再 OBS。"""
        res = {}
        for pre in (True, False):
            obs, first = [], []
            for seed in range(3):
                w = SimWorld(seed=seed, code=self.CODE, cam_deg=90.0, park_side={'ROUGH': 30.0, 'TEMP': 30.0})
                lines = []
                h = make(w, dict(ring_precorrect=pre), log=lines.append)
                run_mission(w, h, stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1'), log=lines.append)
                self._assert_all_good(w, h, '\n'.join(lines), seed, batches=1)
                obs.append(sum(1 for r in w.requests if r.startswith('OBS RING')))
                first += self._start_errors(lines)[1:3] + self._start_errors(lines)[4:6]   # 每个工位第 2、3 个环
            res[pre] = (obs, first)
        self.assertEqual(res[True][0], [2, 2, 2], res)            # 每个工位只在看三个环时 OBS 一次
        self.assertEqual(res[False][0], [6, 6, 6], res)           # 不预先伸：每个工位第 2、3 个环各 OBS 一次
        self.assertLess(max(res[True][1]), 12.0, res)             # 一到就差不多(只剩底盘走得不准的那几毫米)
        self.assertGreater(min(res[False][1]), 20.0, res)         # 缩回观察姿态：每次都差 30mm 左右

    @staticmethod
    def _start_errors(lines):
        """每次放置(按日志顺序)对准时第一次测到的偏差(毫米)。"""
        out, want = [], False
        for l in lines:
            if l.startswith('  ▶') and ' 放 ' in l:
                want = True
            elif want and '] 偏差 ' in l:
                out.append(float(l.split('] 偏差 ')[1].split('mm')[0]))
                want = False
        return out

    def test_return_from_the_tray_learns_its_bias(self):
        """从转盘那边转回对准姿态总是多转一点(舵机过冲/齿轮间隙)：学出来提前补上，后面不用每次都"再转一次"。"""
        n = {}
        for learn in (True, False):
            w = SimWorld(seed=5, code=self.CODE, cam_deg=90.0, ap_bias=(0.9, 1.6))
            lines = []
            h = make(w, dict(return_bias=learn), log=lines.append)
            run_mission(w, h, log=lines.append)
            text = '\n'.join(lines)
            self._assert_all_good(w, h, text, 5)
            n[learn] = text.count('再转一次')
        self.assertGreaterEqual(n[False], 12, n)                  # 12 次放置，每次都差得多
        self.assertLessEqual(n[True], 4, n)

    def test_occupied_tray_slot_is_not_grabbed_into(self):
        """第一批有物料没放出去还在转盘里：第二批同一个槽的物料不夹(放进去会砸在上面)，也不会把它当成第二批的去放。"""
        w = SimWorld(seed=4, code=self.CODE)
        h = make(w)
        orig = w._handle
        st = {'n': 0}

        def handle(text):                                       # 暂存区第一次 TAKE 的时候 STM32 出错：那个物料留在车上
            if text.startswith('TAKE') and w.zone == 'TEMP' and st['n'] == 0:
                st['n'] += 1
                return False, 'ERR TIMEOUT', []
            return orig(text)
        w._handle = handle
        lines = []
        run_mission(w, h, log=lines.append)
        text = '\n'.join(lines)
        self.assertEqual(w.collisions, 0, text)
        self.assertIn('号槽里还有没放出去的', text)

    def test_wheels_first_option(self):
        """wheels_first(mtest … wheels)：沿圆环那一排先让车轮小步慢慢挪——照样全放对，工位里不横移，
        车轮每次最多 wheels_step_mm、用 wheels_rpm 的速度。"""
        steps = []
        for seed, w, h, text in self._run(range(3), dict(cam_deg=90.0),
                                          cfg=dict(wheels_first=True, wheels_step_mm=15.0, wheels_rpm=60, wheels_min_mm=4.0),
                                          stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1')):
            self._assert_all_good(w, h, text, seed, batches=1)
            self.assertEqual([m for m in w.moves if m[2] in ('ROUGH', 'TEMP') and m[0] == 'S'], [], text)
            wheel = [(m, sp) for m, sp in zip(w.moves, w.move_speeds) if sp == 60]
            self.assertTrue(all(m[0] == 'F' and abs(m[1]) <= 15 and m[2] in ('ROUGH', 'TEMP') for m, _sp in wheel), wheel)
            steps += wheel
        self.assertTrue(steps, '一次都没先动车轮')

    def test_servo_timeouts_use_the_sim_clock(self):
        """视觉闭环用模拟的时钟算超时(以前用真的时钟，模拟里永远不超时)：超时设得很短就会超时。"""
        (seed, w, h, text), = self._run([1], dict(cam_deg=90.0), cfg=dict(servo=dict(timeout_s=0.2)),
                                        stops=('QR', 'RAW', 'ROUGH', 'START1'))
        self.assertIn('超时', text)
        self.assertAlmostEqual(h.servo.clock(), w.t)


class TiltTests(unittest.TestCase):
    """车身和圆环那一排不平行(车停斜了几度)：看三个环时量出角度，挪到每个环以后离圆环远/近多少提前补到手臂上。
    任务码第一批去 1、3、2 号环：1 号环离看圆环的地方 150mm，3 号环离 1 号 300mm(斜 5° 时远近差 13mm / 26mm)。"""
    CODE = '156+132+516+231'

    def _run(self, tilt, pre, seeds=range(3)):
        from visual_servo import JacStore
        out = []
        for seed in seeds:
            w = SimWorld(seed=seed, code=self.CODE, cam_deg=90.0, tilt=tilt)
            st = JacStore(None)
            st.put('RING', 'arm', -w.scale['RING'] * w.Rcam @ np.diag([w.k2, w.k1]))     # 做过 vcal RING(手臂的 J 知道)
            lines, cur, first = [], [None], {}

            def log(m, lines=lines, cur=cur):
                lines.append(m)
                if m.startswith('  ▶') and ' 放 ' in m and '到环' in m:
                    cur[0] = (m.split('▶ ')[1].split(' ')[0], int(m.split('到环')[1][0]))
            orig = w.vision.ring_error

            def ring_error(n=None, max_px=None, w=w, cur=cur, first=first, orig=orig):
                if cur[0] is not None and cur[0] not in first:   # 每次放置对准时第一次测量：那一刻离圆环远/近差多少(真实的)
                    c = w.claw()
                    k = min((1, 2, 3), key=lambda k: np.linalg.norm(w.ring_center(k) - c))
                    first[cur[0]] = abs(float((w.ring_center(k) - c)[0]))
                return orig(n, max_px)
            w.vision.ring_error = ring_error
            cfg = dict(time_limit_s=1e9, servo_cal_file=None, vision_cal_file='', tilt_precorrect=pre,
                       ring_order={z: w.ring_order(z) for z in ('ROUGH', 'TEMP')})
            h = MissionHooks({'mission_cfg': cfg}, log=log, vision=w.vision, now=w.now, sleep=w.advance, store=st)
            run_mission(w, h, stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1'), log=log)
            self.assertEqual((w.air, w.loose, w.collisions), (0, 0, 0))
            self.assertEqual(h.stats.place_ok, 6, '\n'.join(lines))
            out.append((w, h, '\n'.join(lines), first))
        return out

    def test_precorrection_makes_the_first_error_small(self):
        """斜 5°(粗加工区)/-4°(暂存区)：到 1、3 号环第一次测量时离圆环远近只差几毫米；不提前补时差十几、二十几毫米。"""
        tilt = {'ROUGH': 5.0, 'TEMP': -4.0}
        on = [f for _w, _h, _t, f in self._run(tilt, True)]
        off = [f for _w, _h, _t, f in self._run(tilt, False)]
        pick = lambda fs: [e for f in fs for (z, r), e in f.items() if r in (1, 3)]
        print(f'\n  [车身斜 5°/-4°] 1、3 号环第一次测量时远近差：提前补 最大 {max(pick(on)):.1f}mm，不补 平均 {np.mean(pick(off)):.1f}mm')
        self.assertLess(max(pick(on)), 4.0, on)
        self.assertGreater(np.mean(pick(off)), 9.0, off)

    def test_warns_at_three_degrees_and_logs_the_angle(self):
        for _w, _h, text, _f in self._run({'ROUGH': 5.0, 'TEMP': -4.0}, True, seeds=[0]):
            self.assertIn('★ 车身和圆环那一排斜了 +', text)
            self.assertIn('★ 车身和圆环那一排斜了 -', text)
            self.assertIn('把车摆正', text)
        for _w, _h, text, _f in self._run({'ROUGH': 1.0, 'TEMP': -1.5}, True, seeds=[0, 1]):
            self.assertNotIn('斜了', text)
            self.assertGreaterEqual(text.count('车身和圆环那一排的夹角'), 2, text)     # 每个工位都报角度

    def test_refined_direction_is_stored_for_the_next_zone(self):
        """第一次换环时量准的"底盘前进方向"存下来：到暂存区直接用，不再显示"可能差 1° 左右"。"""
        (w, h, text, _f), = self._run({'ROUGH': 5.0, 'TEMP': -4.0}, True, seeds=[1])
        self.assertIsNotNone(h.store.extra('RING', 'ch_f_ref'))
        temp = text[text.index('TEMP'):]
        self.assertNotIn('可能差 1° 左右', temp[:temp.index('▶')])


class NavFlowTests(unittest.TestCase):
    """和新的导航一起跑(模拟)：prepare、按 START 计时、每个停车点告诉钩子回家要多久、开往下一个停车点前问来不来得及。"""

    def test_first_batch_fits_in_the_round_with_fast_params(self):
        """提速参数 + vcal 做过：第一批 QR→原料→粗加工→暂存→回家 在 3 分钟内(路上每段按 10 秒)；时间不够的就不做，一定按时回家。"""
        from visual_servo import JacStore
        for seed in range(3):
            w = SimWorld(seed=seed, code=MissionSimTests.CODE, params=FAST_PARAMS, cam_deg=90.0)
            st = JacStore(None)
            st.put('RING', 'arm', -w.scale['RING'] * w.Rcam @ np.diag([w.k2, w.k1]))
            st.put('RAW', 'arm', -w.scale['RAW'] * w.Rcam @ np.diag([w.k2, w.k1]))
            lines = []
            cfg = dict(time_limit_s=170.0, servo_cal_file=None, vision_cal_file='',
                       ring_order={z: w.ring_order(z) for z in ('ROUGH', 'TEMP')})
            h = MissionHooks({'mission_cfg': cfg, 'stops': QR_STOPS}, log=lines.append, vision=w.vision, now=w.now,
                             sleep=w.advance, store=st)
            nav = dict(leg_s=10.0, home_s={'QR': 25, 'RAW': 30, 'ROUGH': 22, 'TEMP': 25})
            done = run_mission(w, h, log=lines.append, nav=nav)
            text = '\n'.join(lines)
            self.assertLessEqual(w.t, 180.0, text)
            self.assertEqual(done[-1], 'START1')
            self.assertEqual((w.air, w.loose, w.collisions), (0, 0, 0), text)
            self.assertGreaterEqual(h.stats.place_ok, 3, text)
            self.assertEqual(w.parked, 1)
            print(f'\n  [提速参数 + 导航 seed {seed}] 去了 {"→".join(done)}；用时 {w.t:.0f} 秒；抓 {h.stats.grab_ok}、放 {h.stats.place_ok}')


def _demo(fast, batches=2, extra=None):
    params = dict(FAST_PARAMS) if fast else {}
    params.update(extra or {})
    w = SimWorld(seed=1, code=MissionSimTests.CODE, params=params)
    h = make(w, log=print)
    stops = ('QR', 'RAW', 'ROUGH', 'TEMP', 'START1') if batches == 1 else ('QR', 'RAW', 'ROUGH', 'TEMP', 'RAW', 'ROUGH', 'TEMP', 'START1')
    run_mission(w, h, stops=stops, log=print)
    print('\n==== 结果 ====')
    print('转盘:', w.tray, ' 夹空:', w.air, ' 碰倒/叠错:', w.collisions)
    print('粗加工区:', rings_summary(w, 'ROUGH'))
    print('暂存区  :', rings_summary(w, 'TEMP'))
    print('串口屏  :', w.screen)
    print('放置对准误差(mm)：', [round(e, 2) for (_, _, _, e, _) in w.placed])
    print(f'操作用时估计 {w.t:.0f} 秒(不含路线行驶；{"提速参数" if fast else "默认速度参数"}；只做第一批)' if batches == 1 else
          f'操作用时估计 {w.t:.0f} 秒(不含路线行驶；{"提速参数" if fast else "默认速度参数"})')
    print('各指令耗时：', time_report(w))


if __name__ == '__main__':
    args = sys.argv[1:]
    if args and args[0] == 'unittest':
        unittest.main(argv=[sys.argv[0]], verbosity=1)
    else:
        # 例：python3 sim_mission.py fast 1                    提速参数、只做第一批
        #     python3 sim_mission.py 1 LFRPM=300 ASPD=200      用您自己的速度参数估算(名字和 STM32 的参数名一样)
        extra = {}
        for a in args:
            if '=' in a:
                k, v = a.split('=', 1)
                extra[k.strip().upper()] = float(v)
        _demo('fast' in args, 1 if '1' in args else 2, extra)
