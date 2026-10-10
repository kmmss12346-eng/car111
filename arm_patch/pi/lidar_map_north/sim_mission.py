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

DEFAULT_PARAMS = dict(ARMOK=1.0, ZHI=0.0, ZGRAB=100.0, ZDROP=60.0, ZPLC=100.0, ZSTK=40.0, ZOBRAW=0.0, ZOBRNG=0.0,
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
        return self.w.move(cmd, val)

    def abort(self):
        self.w.abort_flag = True


class SimWorld:
    def __init__(self, seed=0, code='156+123+516+231', armok=True, qr_present=True, noise_px=0.6, stop_err_mm=8.0,
                 missing_batch1=(), fail_cmd=None, abort_at=None, params=None, cam_deg=None, f_gain=1.0, park=None):
        self.rng = np.random.default_rng(seed)
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
        self.moves = []                                        # 底盘每条指令 (S/F, 毫米, 当时在哪个工位)
        self.homed = 0                                         # 收到几次 HOME(把现在的车头方向记为要保持的方向)
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
        self.off_s = self.off_f = 0.0
        self.stop_err = np.clip(self.rng.normal(0, self.stop_err_mm / 2.0, 2), -self.stop_err_mm, self.stop_err_mm)
        self.stop_err[1] += float(self.park.get(role, 0.0))
        if role == 'RAW':
            # 转盘转到、停稳在爪子够得着的区域里的物料(视野只有 147x110mm，所以只会在 ±30mm 左右)
            try:
                colors = [it.color for it in parse_code(self.code).items(batch)]
            except Exception:
                colors = [1, 2, 3]
            self.raw_items = []
            for c in colors:
                if batch == 1 and c in self.missing_batch1:
                    continue
                for _ in range(200):                           # 离停车点 30mm 以内；两个物料不会叠在一起(不然夹的时候分不清夹到的是哪个)
                    pos = self.rng.uniform(-30, 30, 2)
                    if np.linalg.norm(pos) <= 30.0 and all(np.linalg.norm(pos - it['pos']) >= 20.0 for it in self.raw_items):
                        break
                self.raw_items.append(dict(color=c, pos=pos))
        if role in ('ROUGH', 'TEMP'):
            for k in (1, 2, 3):
                self.rings.setdefault((role, k), [])

    # ------------------------------------------------------------------ 几何
    def claw(self):
        """爪子在工位坐标系里的位置(rad, tan)。"""
        if self.a1_ref is None:
            return np.array([0.0, 0.0])
        return np.array([self.k2 * (self.a2 - self.a2_ref) - self.off_s + self.stop_err[0],
                         self.k1 * (self.a1 - self.a1_ref) + self.off_f + self.stop_err[1]])

    def ring_center(self, k):
        return np.array([0.0, self.ring_tan[self.zone][k]])

    def ring_order(self, zone):
        """这个假摄像头里，zone 的 1、2、3 号环在画面里怎么排('lr' / 'rl')：和 mission_hooks._survey 一样的定方向办法。"""
        t = self.ring_tan[zone]
        d = self.Rcam @ np.array([0.0, t[3] - t[1]])          # 画面里从 1 号到 3 号的方向
        a = d / np.linalg.norm(d)
        if abs(a[0]) >= abs(a[1]):
            a = a if a[0] > 0 else -a
        else:
            a = a if a[1] > 0 else -a
        return 'lr' if float(a @ d) > 0 else 'rl'

    def pixel_error(self, target, kind):
        e = np.asarray(target, float) - self.claw()
        p = self.scale[kind] * (self.Rcam @ e) + self.rng.normal(0, self.noise_px, 2)
        self.advance(MEASURE_S)
        if abs(p[0]) > 280 or abs(p[1]) > 200:
            return None                                   # 出了画面
        return (float(p[0]), float(p[1]))

    # ------------------------------------------------------------------ 底盘
    def move(self, cmd, val):
        self.advance(1.0 + abs(val) / 150.0)
        self.moves.append((cmd, val, self.zone))
        if cmd == 'S':
            self.off_s += val * (1 + self.rng.normal(0, 0.03)) + self.rng.normal(0, 1.0)
        elif cmd == 'F':
            self.off_f += val * self.f_gain * (1 + self.rng.normal(0, 0.03)) + self.rng.normal(0, 1.0)
        return True, 'DONE t=1000 e=10'

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
            info.append(f'LIFT {int(self.lift_known)} {self.lift_mm:.4f}')
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
            info.append(f'QR {self.code}' if self.qr_present else 'QR NONE')
            return True, 'DONE', info
        if verb == 'QR':
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
            if abs(a1_ - self.a1) > 10.0 or abs(a2_ - self.a2) > 10.0:
                spd = max(spd, P['ASPD'])                       # 角度变化大：用大动作速度(和 arm.c 一致)
            self._servos_to(a1_, a2_, spd, fine=True)
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
            c = self.claw()                                        # 夹的那一刻爪子在哪
            got = None
            if verb == 'GRAB' and self.zone == 'RAW':
                best = min(self.raw_items, key=lambda it: np.linalg.norm(it['pos'] - c), default=None)
                if best is not None and np.linalg.norm(best['pos'] - c) <= 8.0 and self.claw_open:
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
            self._tt_go(slot)
            self._lift_to(P['ZGRAB'] if verb == 'GRAB' else P['ZPLC'])
            self._claw_wait()
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
            w.advance(MEASURE_S)
            return None
        c = w.claw()
        it = min(tops, key=lambda i: np.linalg.norm(i['pos'] - c))
        e = w.pixel_error(it['pos'], 'PICK')
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
        it = min(items, key=lambda i: np.linalg.norm(i['pos'] - c))
        return self._off('RAW', w.pixel_error(it['pos'], 'RAW'))

    def wait_still(self, color_id, timeout_s=10.0, **kw):
        """和真的 Vision.wait_still 一样：要在画面里检测到才算(不在画面里 = 看不到)。"""
        w = self.w
        w.advance(0.8)
        items = [it for it in w.raw_items if it['color'] == int(color_id)]
        if not items:
            return False, None
        c = w.claw()
        it = min(items, key=lambda i: np.linalg.norm(i['pos'] - c))
        e = w.scale['RAW'] * (w.Rcam @ (it['pos'] - c))
        if abs(e[0]) > 280 or abs(e[1]) > 200:
            return False, None
        return True, (0.0, 0.0)

    def ring_error(self, n=None, max_px=None):
        w = self.w
        if w.zone not in ('ROUGH', 'TEMP'):
            return None
        c = w.claw()
        k = min((1, 2, 3), key=lambda k: np.linalg.norm(w.ring_center(k) - c))
        e = self._off('RING', w.pixel_error(w.ring_center(k), 'RING'))
        if e is not None and max_px and math.hypot(e[0] + self.claw('RING')[0] - self.TRUE_CLAW[0],
                                                     e[1] + self.claw('RING')[1] - self.TRUE_CLAW[1]) > max_px:
            return None                                       # 离爪子点太远：不是要对的那个环
        return e

    def ring_list(self, n=None):
        """画面里看得到的所有圆环 [(u, v, 最外圈半径)]，按 u 排。"""
        w = self.w
        w.advance(MEASURE_S)
        if w.zone not in ('ROUGH', 'TEMP'):
            return []
        c = w.claw()
        out = []
        for k in (1, 2, 3):
            e = w.scale['RING'] * (w.Rcam @ (w.ring_center(k) - c)) + w.rng.normal(0, w.noise_px, 2)
            if abs(e[0]) > 280 or abs(e[1]) > 200:
                continue
            out.append((self.TRUE_CLAW[0] + float(e[0]), self.TRUE_CLAW[1] + float(e[1]), 48.25 * w.scale['RING']))
        out.sort(key=lambda r: r[0])
        return out


# ======================================================================================================
def run_mission(world, hooks, stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'RAW', 'ROUGH', 'TEMP', 'START1'), log=lambda m: None):
    visits = {}
    for stop in stops:
        role = role_of(stop)
        visits[role] = visits.get(role, 0) + 1
        world.arrive(role, visits[role])
        hooks.task(stop, world.link, log)


def make(world, cfg_extra=None, log=lambda m: None):
    cfg = dict(time_limit_s=1e9, servo_cal_file=None, vision_cal_file='',      # 测试里不限时、不读写文件
               ring_order={z: world.ring_order(z) for z in ('ROUGH', 'TEMP')})  # 假摄像头装的方向是随机的
    cfg.update(cfg_extra or {})
    return MissionHooks({'mission_cfg': cfg}, log=log, vision=world.vision, now=world.now, sleep=world.advance)


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
        self.assertEqual((s['t0'], s['t7']), ('156+123', '516+231'))
        self.assertEqual(s['t2'], 'GRAB 6/6')
        self.assertEqual(s['t3'], 'PLACE 12/12')
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

    def test_missing_color_is_skipped_and_stack_falls_back_to_flat(self):
        w = SimWorld(seed=8, code=self.CODE, missing_batch1=(5,))        # 第一批的黑色没有
        h = make(w)
        run_mission(w, h)
        self.assertEqual(h.stats.grab_ok, 5)
        temp = rings_summary(w, 'TEMP')
        self.assertEqual(temp[1], [1, 1])
        self.assertEqual(temp[3], [6, 6])
        self.assertEqual(temp[2], [5])                                  # 第二批的黑色没有下垫，平放在 2 号环
        self.assertEqual(w.collisions, 0)

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
        errs = []
        for seed in range(8):
            w = SimWorld(seed=100 + seed, code=self.CODE, noise_px=1.5, stop_err_mm=15.0)
            h = make(w)
            run_mission(w, h)
            errs += [e for (_, _, _, e, st) in w.placed]
            self.assertEqual(w.air, 0)
        errs = np.array(errs)
        print(f'\n  [噪声 1.5px、停车误差 15mm] 放置真实误差：平均 {errs.mean():.2f}mm / 最大 {errs.max():.2f}mm')
        self.assertLess(errs.max(), 4.0)

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

    def test_never_strafes_in_the_zones(self):
        """粗加工区、暂存区里底盘只前后挪(横移会压进工位)；原料区照常可以横移。"""
        for seed, w, h, text in self._run(range(6)):
            self._assert_all_good(w, h, text, seed)
            zone_s = [m for m in w.moves if m[2] in ('ROUGH', 'TEMP') and m[0] == 'S']
            self.assertEqual(zone_s, [], f'seed {seed}: {zone_s}')

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
            self.assertTrue(all(min(abs(abs(v) - 150), abs(abs(v) - 300)) <= 15 for v in big), big)
            fix = [v for c, v, z in w.moves if z == 'ROUGH' and abs(v) <= 60 and abs(v) != 20]
            self.assertLessEqual(len(fix), 4, (fix, text))
            self.assertTrue(all(p[3] < 2.5 for p in w.placed), w.placed)

    def test_car_parked_at_ring1_or_ring3_still_finds_the_right_rings(self):
        """车没停在 2 号环前面(停在 1 号或 3 号环前面、或者两个环中间偏 2 号)：圆环空着时看清以后照样认对、放对。
        (圆环上放着物料时不靠"那一头没有环"认，按离爪子近的是 2 号算，所以这里只跑第一批)"""
        for park, sure in (({'ROUGH': 150.0, 'TEMP': -150.0}, True), ({'ROUGH': -150.0, 'TEMP': 150.0}, True),
                           ({'ROUGH': 55.0, 'TEMP': -55.0}, False)):
            for seed, w, h, text in self._run(range(3), dict(cam_deg=90.0, park=park), stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1')):
                self._assert_all_good(w, h, text, seed, batches=1)
                self.assertEqual(rings_summary(w, 'TEMP'), {1: [1], 2: [5], 3: [6]}, f'{park} seed {seed}')
                if sure:
                    self.assertNotIn('认不准', text)

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
