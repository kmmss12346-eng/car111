"""auto_run.drive 的导航测试(1010)：模拟一台"真车"(真实位姿、陀螺仪漂移、距离比例误差)，
检查每个停车点雷达定位后的修正、小移动不丢、车真的停到停车点、回家前定位失败不再照原路线走(冲出场地的 bug)、
回家每一步不出场地、时间不够直接回家、R 小数角度。python3 -m unittest test_nav_drive"""
import math
import random
import re
import sys
import types
import unittest

import test_auto_run  # noqa: F401  (先装好 stm32_link / route_plan 的替身)
import auto_run
from auto_run import Abort


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class SimCar:
    """一台模拟的车：STM32 按陀螺仪保持"目标车头" T；陀螺仪读数 = 真实车头 + g_off(会慢慢漂)。
    F/S 走 val*(1+scale)，横移时往前后串一点；R 转到 T(剩一点残差)。记录发出的每条指令。"""

    def __init__(self, pose, scale=0.0, drift_deg=0.0, resid_deg=0.0, slip=0.0, seed=1, rdec=False, zone_script=None,
                 park='ok'):
        self.x, self.y, self.yaw = (float(v) for v in pose)
        self.g_off = 0.0
        self.T = self.yaw
        self.scale, self.drift, self.resid, self.slip = scale, drift_deg, resid_deg, slip
        self.rng = random.Random(seed)
        self.rdec = rdec
        self.sent = []            # (cmd, val, speed)
        self.requests = []
        self.trace = [(self.x, self.y, self.yaw)]
        self.zone_script = list(zone_script or [])
        self.park = park
        self.aborted = 0
        self.ok = True
        self.err = ''

    # ---- 陀螺仪
    @property
    def value(self):
        return int(round((self.yaw + self.g_off) * 100))

    def fresh(self, max_age=1.0):
        return True

    def pose(self):
        return (self.x, self.y, self.yaw)

    def _settle(self):
        r = self.rng.uniform(-self.resid, self.resid)
        self.yaw = wrap(self.T - self.g_off + r)
        self.g_off += self.drift
        self.trace.append(self.pose())
        return r

    def _done(self, r):
        return True, f'DONE t=500 e={int(round(-r*100))}'     # e = 目标 - 实际

    # ---- stm32_link 接口
    def ping(self):
        return True, 'PONG'

    def sync_params(self, params, log=None):
        return []

    def home(self):
        self.T = wrap(self.yaw + self.g_off)
        return True, 'DONE'

    def get_params(self):
        ps = {'FPPM': 12.9, 'ALTOL': 1.0}
        if self.rdec:
            ps['PSPD'] = 70.0
        return ps, 'DONE'

    def abort(self):
        self.aborted += 1

    def _move(self, cmd, val):
        if cmd == 'R':
            self.T = wrap(self.T + val)
            for k in range(1, 5):              # 转弯过程也记下来(检查车身扫过的范围)
                self.yaw = wrap(self.yaw + val / 4.0)
                self.trace.append(self.pose())
            return self._done(self._settle())
        a = math.radians(self.yaw)
        d = val * (1 + self.scale)
        if cmd == 'F':
            ux, uy = math.cos(a), math.sin(a)
            px, py = -math.sin(a), math.cos(a)
            side = 0.0
        else:
            ux, uy = -math.sin(a), math.cos(a)
            px, py = math.cos(a), math.sin(a)
            side = self.slip * abs(val)
        n = max(1, int(abs(d) // 20))
        for k in range(n):
            self.x += d / n * ux + side / n * px
            self.y += d / n * uy + side / n * py
            self.trace.append(self.pose())
        return self._done(self._settle())

    def move(self, cmd, val, speed=None):
        self.sent.append((cmd, val, speed))
        if cmd == 'R' and val == 0:
            raise AssertionError('发了 R 0(STM32 上是 HOME)')
        if cmd == 'R' and val != int(val):
            raise AssertionError('move 不能发小数角度(stm32_link.move 怎么格式化不知道)')
        return self._move(cmd, val)

    def second_move(self, rel=None, log=None):
        rel = rel or auto_run.DEFAULT_SECOND_MOVES
        self._move('S', rel['left_mm'])
        self._move('F', rel['forward_mm'])
        return self._move('R', -rel['cw_deg'])

    def request(self, text, timeout=None, collect=False):
        self.requests.append(text)
        m = re.match(r'R (-?\d+(?:\.\d+)?)$', text)
        if m:
            if not self.rdec:
                out = (False, 'ERR ARG', [])
            else:
                v = float(m.group(1))
                if abs(v) < 0.05:
                    raise AssertionError('发了 R 0')
                self.sent.append(('R', v, None))
                ok, rep = self._move('R', v)
                out = (ok, rep, [])
        elif text == 'ZONE?':
            if not self.zone_script:
                out = (True, 'DONE', ['ZONE 0 0'])
            else:
                z, s = self.zone_script[0] if len(self.zone_script) == 1 else self.zone_script.pop(0)
                out = (True, 'DONE', [f'ZONE {z} {s}'])
        elif text.startswith('ZONE'):
            out = (True, 'DONE', [])
        elif text == 'PARK':
            out = (True, 'DONE', []) if self.park == 'ok' else (False, self.park, [])
        elif text in ('STOW', 'LIFT 60'):
            out = (True, 'DONE', [])
        else:
            out = (False, 'ERR CMD', [])
        return out if collect else out[:2]


class SimCtx:
    """ctx 的替身：relocalize 返回真车位置(加一点噪声)，可以设定第几次开始失败。"""

    def __init__(self, car, start_ref=None, start_est=None, fail_from=None, noise=(1.0, 0.1), obstacles=(), fail_noinit=True,
                 home_routes=None):
        self.car = car
        self.calls = 0
        self.fail_from = fail_from
        self.fail_noinit = fail_noinit
        self.noise = noise
        self._obs = list(obstacles)
        self.start_ref = start_ref
        self.start_est = start_est
        self.rng = random.Random(7)
        self.reloc_poses = []
        self.home_routes = home_routes
        self.tick = None

    def aborted(self):
        return False

    def obstacles(self):
        return list(self._obs)

    def route_start(self):
        return dict(est=self.start_est, ref=self.start_ref)

    def relocalize(self, plan_pose, frames=8, use_init=True, **kw):
        self.calls += 1
        if self.fail_from is not None and self.calls >= self.fail_from and (use_init or self.fail_noinit):
            return dict(ok=False, why='对不上(测试)', pose=None, dx=0, dy=0, dyaw=0, rms=0, inlier=0.2, t=0.01)
        n_mm, n_deg = self.noise
        p = self.car.pose()
        q = (p[0] + self.rng.uniform(-n_mm, n_mm), p[1] + self.rng.uniform(-n_mm, n_mm), wrap(p[2] + self.rng.uniform(-n_deg, n_deg)))
        self.reloc_poses.append((tuple(plan_pose), q))
        return dict(ok=True, why='', pose=q, dx=q[0] - plan_pose[0], dy=q[1] - plan_pose[1], dyaw=wrap(q[2] - plan_pose[2]),
                    rms=5.0, inlier=0.9, t=0.02)

    def home_route(self, name, goal):
        return (self.home_routes or {}).get(name)


class Hooks:
    def __init__(self, car, go_home_at=None):
        self.car = car
        self.at = []               # (停车点, 当时的真实位姿)
        self.go_home_at = go_home_at
        self.asked = []
        self.etas = []
        self.ctx = None

    def task(self, stop, link, log):
        self.at.append((stop, self.car.pose()))

    def go_home_now(self, role, est_leg_s, est_home_s):
        self.asked.append((role, est_leg_s, est_home_s))
        return self.go_home_at is not None and role == self.go_home_at

    def set_home_eta(self, s):
        self.etas.append(s)


def manhattan_leg(name, pose, goal, turn_first=False):
    """测试用的简单路线：沿现在的车头前进/横移到目标，再转到目标车头(转弯是 90° 的倍数)。
    turn_first=True：先在起点转到目标车头再走(停车点在场地边上时，在那里原地转会扫出场地)。"""
    x, y, h = pose
    gx, gy, gh = goal
    d = wrap(gh - h)
    turn = [('R', int(round(d)))] if abs(d) > 0.5 else []
    if turn_first:
        h = gh
    a = math.radians(h)
    f = (gx - x) * math.cos(a) + (gy - y) * math.sin(a)
    s = -(gx - x) * math.sin(a) + (gy - y) * math.cos(a)
    cmds = [(c, int(round(v))) for c, v in (('F', f), ('S', s)) if abs(v) >= 0.5]
    cmds = turn + cmds if turn_first else cmds + turn
    return dict(stop=name, start=tuple(pose), goal=tuple(goal), cmds=cmds, poses=[tuple(goal)], note='')


def route(start, stops):
    legs, p = [], start
    for name, g in stops:
        legs.append(manhattan_leg(name, p, g))
        p = g
    return legs


def edge_min(trace, cfg=None):
    m = auto_run.body_model(cfg or {})
    return min(auto_run.edge_clearance(p, m) for p in trace)


ZONE2 = (2250.0, 150.0, 90.0)
CFG = dict(car_length_mm=290, car_width_mm=260, lidar_overhang_mm=40, lidar_forward_mm=22.5, lidar_left_mm=134.8,
           route_speed_rpm=150, strafe_speed_rpm=140, reloc_slow_rpm=60, home_fix_rpm=60)


class FlattenTests(unittest.TestCase):
    def test_small_moves_are_carried_not_dropped(self):
        legs = [dict(stop='A', goal=(0, 0, 0), cmds=[('F', -8), ('S', 12), ('F', 500), ('S', 300), ('F', 15)])]
        seq = auto_run.flatten(0, legs)
        cmds = [(c, v) for _, c, v in seq if c]
        # F -8 加到 F 500 里(492)；F 15 是最后一点：>= 5 到停车点前补上；S 12 加到 S 300 里
        self.assertEqual(cmds, [('F', 492), ('S', 312), ('F', 15)])
        self.assertEqual(sum(v for c, v in cmds if c == 'F'), 507)
        self.assertEqual(sum(v for c, v in cmds if c == 'S'), 312)

    def test_remainder_follows_turns_in_world_frame(self):
        # 车头朝东时欠着 F 10(往东)，左转 90° 以后往东就是"右移"：加到下一条 S 里
        legs = [dict(stop='A', goal=None, cmds=[('F', 10), ('R', 90), ('F', 400), ('S', -200)])]
        seq = auto_run.flatten(0, legs)
        self.assertEqual([(c, v) for _, c, v in seq if c], [('R', 90), ('F', 400), ('S', -210)])

    def test_tiny_rest_is_recorded_in_marker(self):
        legs = [dict(stop='A', goal=(1, 2, 0), cmds=[('F', 400), ('S', 3)]),
                dict(stop='B', goal=(5, 6, 0), cmds=[('S', -3), ('F', 100)])]
        seq = auto_run.flatten(0, legs)
        marks = [(s, v) for s, c, v in seq if c is None]
        self.assertEqual(marks[0][0], 'A')
        self.assertAlmostEqual(marks[0][1]['rest']['S'], 3.0)
        self.assertEqual(marks[0][1]['goal'], (1.0, 2.0, 0.0))
        # 下一段不重复加(drive 按 rest 把 3mm 带过去)
        self.assertEqual([(c, v) for _, c, v in seq if c], [('F', 400), ('F', 100)])

    def test_turn_back_keeps_tenth_degree(self):
        seq = auto_run.flatten(59.63, [dict(stop='A', cmds=[('F', 100)])])
        self.assertEqual(seq[0], (None, 'R', 59.6))
        seq = auto_run.flatten(60.0, [dict(stop='A', cmds=[('F', 100)])])
        self.assertEqual(seq[0], (None, 'R', 60))
        self.assertIsInstance(seq[0][2], int)


class DriveTests(unittest.TestCase):
    def drive(self, car, ctx, legs, cfg=None, turn_back=0, hooks=None, caps=None, start=True):
        logs = []
        seq = auto_run.flatten(turn_back, legs)
        st = dict(est=car.pose(), ref=car.pose()) if start is True else start
        ctx.start_ref = st['ref'] if st else None
        auto_run.drive(ctx, car, seq, log=logs.append, stop_wait=0, hooks=hooks, speeds={'F': 150, 'S': 140},
                       motion=auto_run.Motion(dict(CFG, **(cfg or {}))), cfg=dict(CFG, **(cfg or {})), start=st,
                       caps=caps if caps is not None else {'rdec': False})
        self.logs = logs
        return seq

    def test_exact_arrival_without_lidar(self):
        # 没有雷达定位(ctx 没有 relocalize)：车按指令推算，到停车点前补上不到 20mm 的零头，车真的停在停车点上
        car = SimCar((1200.0, 1100.0, 90.0))
        ctx = types.SimpleNamespace(aborted=lambda: False)
        h = Hooks(car)
        stops = [('A', (1212.0, 1300.0, 90.0)), ('B', (2100.0, 1302.0, 0.0)), ('C', (2108.0, 1700.0, 0.0))]
        self.drive(car, ctx, route((1200.0, 1100.0, 90.0), stops), hooks=h)
        for (name, goal), (n2, p) in zip(stops, h.at):
            self.assertEqual(name, n2)
            # A 差 12mm、C 差 8mm(都 >= 5)：到停车点前补上，停得很准；B 只差 2mm：不单独发，带到下一段
            self.assertLess(math.hypot(p[0] - goal[0], p[1] - goal[1]), 1.0 if name != 'B' else auto_run.STOP_MIN_MM,
                            f'{name} 没停到停车点：{p}')
        for c, v, sp in car.sent:
            if c in ('F', 'S'):
                self.assertGreaterEqual(abs(v), auto_run.STOP_MIN_MM, '不能发小于 5mm 的移动')

    def test_relocalize_corrects_drift_at_every_stop(self):
        # 距离多走 2%、陀螺仪每条指令漂 0.15°：每个停车点雷达定位后，左右偏差修到 15mm 以内、车头 1° 以内
        car = SimCar((2100.0, 150.0, 90.0), scale=0.02, drift_deg=0.15, resid_deg=0.3, slip=0.01, rdec=True)
        ctx = SimCtx(car)
        h = Hooks(car)
        stops = [('QR', (2100.0, 1200.0, 90.0)), ('RAW', (1200.0, 2100.0, 180.0)), ('ROUGH', (1200.0, 340.0, 0.0)),
                 ('TEMP', (340.0, 1200.0, -90.0))]
        self.drive(car, ctx, route((2100.0, 150.0, 90.0), stops), hooks=h, start=dict(est=(2100.0, 150.0, 90.0), ref=(2100.0, 150.0, 90.0)),
                   caps={'rdec': True})
        self.assertEqual([n for n, _ in h.at], [n for n, _ in stops])
        for (name, goal), (_, p) in zip(stops, h.at):
            a = math.radians(goal[2])
            lat = -(p[0] - goal[0]) * math.sin(a) + (p[1] - goal[1]) * math.cos(a)
            self.assertLess(abs(lat), 16.0, f'{name} 左右还差 {lat:.1f}mm')
            self.assertLess(abs(wrap(p[2] - goal[2])), 1.0, f'{name} 车头还差 {wrap(p[2]-goal[2]):.2f}°')
        self.assertTrue(any(c == 'R' and v != int(v) for c, v, _ in car.sent), '新固件应该发 0.1° 的车头修正')
        self.assertTrue(any('雷达定位' in l for l in self.logs))

    def test_longitudinal_error_merged_into_next_forward(self):
        # 停车点上前后偏 30mm(下一条是同方向的前进)：不单独修，并进下一条
        car = SimCar((1200.0, 1000.0, 90.0))
        ctx = SimCtx(car)
        stops = [('A', (1200.0, 1500.0, 90.0)), ('B', (1200.0, 2000.0, 90.0))]
        legs = route((1200.0, 1000.0, 90.0), stops)
        car.scale = 0.06                     # 第一段多走 30mm

        class H(Hooks):
            def task(self, stop, link, log):
                Hooks.task(self, stop, link, log)
                car.scale = 0.0
        self.drive(car, ctx, legs, hooks=H(car))
        fs = [(c, v) for c, v, _ in car.sent if c == 'F']
        self.assertEqual(fs[0], ('F', 500))
        self.assertTrue(460 <= fs[1][1] <= 475, fs)          # 第二条前进少走了约 30mm
        self.assertEqual(len(fs), 2, '前后偏差不应该在停车点单独修')

    def test_lateral_error_fixed_slowly_before_task(self):
        car = SimCar((1200.0, 1000.0, 90.0))
        ctx = SimCtx(car)
        legs = [dict(stop='A', goal=(1200.0, 1500.0, 90.0), cmds=[('F', 500)])]
        car.x += 25                                           # 起步就偏了 25mm(向右)
        self.drive(car, ctx, legs, hooks=Hooks(car), start=dict(est=(1200.0, 1000.0, 90.0), ref=(1200.0, 1000.0, 90.0)))
        s = [(v, sp) for c, v, sp in car.sent if c == 'S']
        self.assertEqual(len(s), 1)
        self.assertTrue(20 <= abs(s[0][0]) <= 30, s)
        self.assertEqual(s[0][1], 60, '修正要用 60 转/分(STM32 精确模式)')
        self.assertLess(abs(car.x - 1200.0), 5)


class GuardTests(unittest.TestCase):
    def test_long_strafe_toward_wall_stops_short_then_lidar_fixes(self):
        # 往东横移 1900mm 到 PREHOME(车身离东边线只有 20mm)，横移多走 1.5%：
        # 先少走(2% + 10mm)，到停车点雷达定位后慢速补上，车身一直不出场地
        for scale in (0.0, 0.015, 0.025):
            car = SimCar((340.0, 400.0, 90.0), scale=scale)
            ctx = SimCtx(car)
            legs = [dict(stop='PREHOME', goal=(2250.0, 400.0, 90.0), cmds=[('S', -1910)])]
            logs = []
            auto_run.drive(ctx, car, auto_run.flatten(0, legs), log=logs.append, stop_wait=0, hooks=Hooks(car), speeds={},
                           motion=None, cfg=dict(CFG), start=dict(est=(340.0, 400.0, 90.0), ref=(340.0, 400.0, 90.0)),
                           caps={'rdec': False})
            self.assertGreaterEqual(edge_min(car.trace, CFG), 0.0, f'多走 {scale*100:.1f}% 出了场地')
            s = [(v, sp) for c, v, sp in car.sent if c == 'S']
            self.assertTrue(-1870 <= s[0][0] <= -1855, s)              # 横移按 3% + 10mm 留余量
            self.assertTrue(all(sp == 60 for v, sp in s[1:]), s)
            self.assertLess(abs(car.x - 2250.0), 7, scale)
            self.assertTrue(any('先少走' in l for l in logs))

    def test_moves_along_a_wall_are_not_shortened(self):
        car = SimCar((2250.0, 400.0, 90.0))
        ctx = SimCtx(car)
        legs = [dict(stop='A', goal=(2250.0, 1600.0, 90.0), cmds=[('F', 1200)])]
        auto_run.drive(ctx, car, auto_run.flatten(0, legs), log=lambda m: None, stop_wait=0, hooks=Hooks(car), speeds={},
                       motion=None, cfg=dict(CFG), start=dict(est=(2250.0, 400.0, 90.0), ref=(2250.0, 400.0, 90.0)),
                       caps={'rdec': False})
        self.assertEqual(car.sent[0][:2], ('F', 1200))


class HomeTests(unittest.TestCase):
    def run_home(self, car, ctx, legs, cfg=None, caps=None):
        logs = []
        p0 = legs[0]['start']
        legs = [dict(stop='X', start=p0, goal=p0, cmds=[])] + legs      # 前一个停车点(这里不做雷达定位)
        cfg = dict(dict(reloc_stops=[]), **(cfg or {}))
        seq = auto_run.flatten(0, legs)
        cfg = dict(CFG, **(cfg or {}))
        start = dict(est=p0, ref=p0)
        auto_run.drive(ctx, car, seq, log=logs.append, stop_wait=0, hooks=Hooks(car), speeds={'F': 150, 'S': 140},
                       motion=auto_run.Motion(cfg), cfg=cfg, start=start, caps=caps or {'rdec': False})
        self.logs = logs
        return seq

    def test_regression_approach_moved_then_failed_never_replays_route(self):
        """bug：回家前定位修了一次车，后面定位失败，drive 又接着发原路线剩下的指令，车冲出场地。"""
        car = SimCar((2250.0, 1300.0, 90.0))
        # 最后一段：从 (2250,1300) 后退 1150 到启停区 2；车实际多走了 3% -> 定位后要修
        legs = [dict(stop='START2', start=(2250.0, 1300.0, 90.0), goal=ZONE2, cmds=[('F', -1150)], poses=[ZONE2])]
        ctx = SimCtx(car, fail_from=2)           # 第 1 次定位成功(修了车)，以后都失败
        car.scale = 0.0
        car.y += 40                               # 真车比推算的靠北 40mm
        seq = self.run_home(car, ctx, legs)
        # home_cut 把 F -1150 拆成两截：前一截照走，到离启停区 800mm 处停下定位
        cut, _ = auto_run.home_cut(seq, {})
        planned = [v for _, c, v in cut if c == 'F']
        self.assertEqual(len(planned), 2)
        sent_f = [v for c, v, _ in car.sent if c == 'F']
        self.assertEqual(sent_f[0], planned[0])
        self.assertNotIn(planned[1], sent_f[1:], '定位修过车以后又发了原路线的指令')
        self.assertTrue(len(car.sent) >= 2, '应该修过车')
        self.assertTrue(all(sp == 60 for c, v, sp in car.sent[1:] if c in ('F', 'S')), '回家修正都要慢速')
        self.assertGreaterEqual(edge_min(car.trace, CFG), -0.5, '车身出了场地')
        self.assertTrue(any('不再照原路线走' in l for l in self.logs))

    def test_approach_fails_without_moving_continues_route_shortened(self):
        car = SimCar((2250.0, 1300.0, 90.0))
        legs = [dict(stop='START2', start=(2250.0, 1300.0, 90.0), goal=ZONE2, cmds=[('F', -1150)], poses=[ZONE2])]
        ctx = SimCtx(car, fail_from=1)            # 一次都测不到
        self.run_home(car, ctx, legs)
        f = [v for c, v, _ in car.sent if c == 'F']
        # 最后朝边线那条少走(至少 15mm，长的按 2% + 10mm)，宁短不长；定位一直失败就停在这里
        self.assertTrue(-1135 <= sum(f) <= -1120, f)
        self.assertGreaterEqual(edge_min(car.trace, CFG), 10)

    def test_prehome_final_approach_is_precise_and_inside(self):
        # 有 PREHOME：在离启停区 250mm 的直线上定位，最后一段直线后退，到了再对准(最多 2 轮)
        car = SimCar((2250.0, 1200.0, 90.0), scale=0.02, drift_deg=0.2, resid_deg=0.3)
        legs = [dict(stop='PREHOME', start=(2250.0, 1200.0, 90.0), goal=(2250.0, 400.0, 90.0), cmds=[('F', -800)]),
                dict(stop='START2', start=(2250.0, 400.0, 90.0), goal=ZONE2, cmds=[('F', -250)])]
        ctx = SimCtx(car)
        self.run_home(car, ctx, dict(reloc_stops=None) and legs, cfg=dict(reloc_stops=None), caps={'rdec': True})
        p = car.pose()
        goal = auto_run.inset_goal(ZONE2, CFG)                     # (2250,155)：车身离南边线 10mm
        self.assertLess(math.hypot(p[0] - goal[0], p[1] - goal[1]), 8.0, p)
        self.assertGreaterEqual(edge_min(car.trace, CFG), 3.0, '多走 2% 也不能出场地：最后一条先少走 15mm，再慢速修进去')
        self.assertTrue(auto_run.in_rect(p, auto_run.START_RECTS[2]))
        last_f = [v for c, v, sp in car.sent if c == 'F' and sp != 60]
        self.assertTrue(-245 <= last_f[-1] <= -205, car.sent)          # -250 加上 PREHOME 测到的前后偏差，再少走 15mm

    def test_home_correction_clamped_at_field_edge(self):
        # 目标点太靠边(车身会出场地 5mm)：每一步都限幅，车身离边线至少 home_edge_mm
        car = SimCar((2250.0, 200.0, 90.0))
        nav = auto_run.Nav(dict(est=(2250.0, 200.0, 90.0), ref=(2250.0, 200.0, 90.0)))
        ctx = SimCtx(car, noise=(0, 0))
        res = auto_run.home_approach(ctx, car, lambda m: None, nav, (2250.0, 140.0, 90.0),
                                     dict(CFG, home_edge_mm=3, home_goal_edge_mm=0), {'rdec': False})
        self.assertGreaterEqual(edge_min(car.trace, CFG), 3.0 - 0.01)
        self.assertIn(res['status'], ('ok', 'near'))
        self.assertLess(car.y, 150.0)

    def test_home_goal_moved_inside(self):
        # 出发位置车身离南边线只有 5mm：回家目标往北挪到离边线 10mm(区 1 往西挪)
        self.assertEqual(tuple(round(v, 3) for v in auto_run.inset_goal(ZONE2, CFG)), (2250.0, 155.0, 90.0))
        self.assertEqual(tuple(round(v, 3) for v in auto_run.inset_goal((2250.0, 2250.0, 180.0), CFG)), (2245.0, 2250.0, 180.0))
        self.assertEqual(auto_run.inset_goal((1200.0, 1200.0, 0.0), CFG), (1200.0, 1200.0, 0.0))
        car = SimCar((2250.0, 200.0, 90.0))
        nav = auto_run.Nav(dict(est=(2250.0, 200.0, 90.0), ref=(2250.0, 200.0, 90.0)))
        res = auto_run.home_approach(SimCtx(car, noise=(0, 0)), car, lambda m: None, nav, ZONE2, CFG, {'rdec': False})
        self.assertEqual(res['status'], 'ok')
        self.assertLess(abs(car.y - 155.0), 6.0)
        self.assertGreaterEqual(edge_min(car.trace, CFG), 4.0)

    def test_home_turn_clamped_in_corner(self):
        # 车在启停区角落(车身离南边线 7mm)，要转 3° 才对上出发时的车头：转 3° 车角会扫出场地 -> 只转能转的
        for rdec in (True, False):
            car = SimCar((2250.0, 152.0, 90.0), rdec=rdec)
            nav = auto_run.Nav(dict(est=(2250.0, 152.0, 90.0), ref=(2250.0, 152.0, 90.0)))
            ctx = SimCtx(car, noise=(0, 0))
            auto_run.home_approach(ctx, car, lambda m: None, nav, (2250.0, 152.0, 93.0),
                                   dict(CFG, home_edge_mm=3, home_goal_edge_mm=0), {'rdec': rdec})
            self.assertGreaterEqual(edge_min(car.trace, CFG), 3.0 - 0.01)
            self.assertGreater(car.yaw, 90.5, '能转的那一点还是要转')
            self.assertLess(car.yaw, 92.0)
        # 目标往场内挪(默认)：先往北挪，下一轮就能转到位，车角一直不出场地
        car = SimCar((2250.0, 152.0, 90.0), rdec=True)
        nav = auto_run.Nav(dict(est=(2250.0, 152.0, 90.0), ref=(2250.0, 152.0, 90.0)))
        auto_run.home_approach(SimCtx(car, noise=(0, 0)), car, lambda m: None, nav, (2250.0, 152.0, 93.0), dict(CFG), {'rdec': True})
        self.assertGreaterEqual(edge_min(car.trace, CFG), 3.0 - 0.01)
        self.assertAlmostEqual(car.yaw, 93.0, delta=1.0)

    def test_fail_moved_without_good_measure_stops_in_place(self):
        car = SimCar((2250.0, 500.0, 90.0))
        nav = auto_run.Nav(dict(est=(2250.0, 500.0, 90.0), ref=(2250.0, 500.0, 90.0)))
        ctx = SimCtx(car, fail_from=1)
        res = auto_run.home_approach(ctx, car, lambda m: None, nav, ZONE2, CFG, {'rdec': False})
        self.assertEqual(res['status'], 'fail_unmoved')
        self.assertEqual(car.sent, [])


class TurnTests(unittest.TestCase):
    def test_decimal_turn_via_request_never_r0(self):
        car = SimCar((1200.0, 1000.0, 90.0), rdec=True)
        caps = {'rdec': True}
        self.assertEqual(auto_run.send_turn(car, 0.04, caps)[0], 0)
        self.assertEqual(car.requests, [])
        self.assertEqual(auto_run.send_turn(car, -0.06, caps)[0], -0.1)
        self.assertEqual(car.requests, ['R -0.1'])
        self.assertEqual(auto_run.send_turn(car, 90.0, caps)[0], 90)       # 整数照旧用 move
        self.assertEqual(car.sent[-1], ('R', 90, None))

    def test_old_firmware_rounds_and_skips_small(self):
        car = SimCar((1000.0, 1000.0, 90.0), rdec=False)
        caps = auto_run.firmware_caps(car)
        self.assertFalse(caps['rdec'])
        self.assertEqual(auto_run.send_turn(car, 0.7, caps)[0], 1)
        self.assertEqual(auto_run.send_turn(car, 0.4, caps)[0], 0)
        self.assertEqual(car.sent, [('R', 1, None)])

    def test_firmware_without_decimals_falls_back(self):
        car = SimCar((1000.0, 1000.0, 90.0), rdec=False)
        caps = {'rdec': True}                       # 以为是新固件，其实不是：ERR ARG -> 改发整数
        sent, ok, _ = auto_run.send_turn(car, 1.4, caps)
        self.assertEqual((sent, ok), (1, True))
        self.assertFalse(caps['rdec'])

    def test_caps_from_pspd(self):
        self.assertTrue(auto_run.firmware_caps(SimCar((0, 0, 0), rdec=True))['rdec'])
        self.assertFalse(auto_run.firmware_caps(SimCar((0, 0, 0), rdec=True), {'r_decimals': False})['rdec'])


class TimeTests(unittest.TestCase):
    def test_go_home_now_switches_to_home_route(self):
        car = SimCar((2100.0, 150.0, 90.0))
        stops = [('QR', (2100.0, 1200.0, 90.0)), ('RAW', (1200.0, 2100.0, 180.0)), ('ROUGH', (1200.0, 340.0, 0.0)),
                 ('START2', ZONE2)]
        legs = route((2100.0, 150.0, 90.0), stops)
        home = route((2100.0, 1200.0, 90.0), [('START2', ZONE2)])
        ctx = SimCtx(car, home_routes={'QR': home})
        h = Hooks(car, go_home_at='RAW')
        logs = []
        cfg = dict(CFG)
        auto_run.drive(ctx, car, auto_run.flatten(0, legs), log=logs.append, stop_wait=0, hooks=h,
                       speeds={'F': 150, 'S': 140}, motion=auto_run.Motion(cfg), cfg=cfg,
                       start=dict(est=(2100.0, 150.0, 90.0), ref=(2100.0, 150.0, 90.0)), caps={'rdec': False})
        self.assertEqual([n for n, _ in h.at], ['QR', 'START2'], '时间不够：QR 之后直接回家')
        self.assertEqual(h.asked[0][0], 'RAW')
        self.assertGreater(h.asked[0][1], 0)
        self.assertGreater(h.asked[0][2], 0)
        self.assertTrue(h.etas and h.etas[0] > 0)
        self.assertLess(math.hypot(car.x - ZONE2[0], car.y - ZONE2[1]), 8)

    def test_go_home_without_route_skips_tasks(self):
        car = SimCar((2100.0, 150.0, 90.0))
        stops = [('QR', (2100.0, 1200.0, 90.0)), ('RAW', (1200.0, 2100.0, 180.0)), ('START2', ZONE2)]
        ctx = SimCtx(car)
        h = Hooks(car, go_home_at='RAW')
        auto_run.drive(ctx, car, auto_run.flatten(0, route((2100.0, 150.0, 90.0), stops)), log=lambda m: None, stop_wait=0,
                       hooks=h, speeds={}, motion=auto_run.Motion(CFG), cfg=dict(CFG),
                       start=dict(est=(2100.0, 150.0, 90.0), ref=(2100.0, 150.0, 90.0)), caps={'rdec': False})
        self.assertEqual([n for n, _ in h.at], ['QR', 'START2'], 'RAW 不做任务')


class SecondStationTests(unittest.TestCase):
    def test_turn_back_uses_stm32_target_heading(self):
        # 第二站：STM32 要保持的车头 = 90-60 = 30°，陀螺仪量到 30.4°(还差一点没转到)。转回去发 R 60(不是 59.6)
        car = SimCar((2208.0, 233.0, 30.4))
        car.T = 30.0
        ctx = SimCtx(car)
        legs = [dict(stop='A', goal=(2208.0, 900.0, 90.0), cmds=[('F', 667)])]
        seq = auto_run.flatten(59.6, legs)
        auto_run.drive(ctx, car, seq, log=lambda m: None, stop_wait=0, hooks=Hooks(car), speeds={}, motion=None,
                       cfg=dict(CFG, reloc_enabled=False), start=dict(est=(2208.0, 233.0, 30.0), ref=(2208.0, 233.0, 30.4)),
                       caps={'rdec': True})
        self.assertEqual(car.sent[0], ('R', 60, None))
        self.assertAlmostEqual(car.yaw, 90.0, places=3)


if __name__ == '__main__':
    unittest.main()
