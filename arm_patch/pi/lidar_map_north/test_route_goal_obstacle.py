"""停车点旁边有障碍物(10-12 用户："障碍物离我指定的点太近了……他就跑不到 1200,2100")：
真车(290×260+雷达)停在停车点上碰不到障碍物时，一定准确停在停车点上(只在停车点附近放宽到 goal_obstacle_min_margin_mm)；
路线其余部分照样按 margin_mm；真的停不下才沿车头方向挪到最近放得下的位置，并写清楚差多少。
python3 -m unittest test_route_goal_obstacle"""
import math
import time
import unittest

import route_plan as rp
from test_route_safety import CFG, STOPS, ZONES, BASE, run
import test_nav_drive as tnd
import auto_run

MIN = 10.0


def obs_beside(stop, side, gap, r=25.0, cfg=CFG):
    """障碍物放在停车点车身(含雷达外框)的 前/后/左/右 外面，离外框 gap mm。"""
    x, y, h = STOPS[stop]
    xb, xf, yr, yl = rp.Body(cfg).box
    a = math.radians(h)
    fx, fy = math.cos(a), math.sin(a)
    lx, ly = -fy, fx
    d, ux, uy = {'front': (xf, fx, fy), 'back': (-xb, -fx, -fy), 'left': (yl, lx, ly), 'right': (-yr, -lx, -ly)}[side]
    return (x + ux*(d + gap + r), y + uy*(d + gap + r), r)


def obs_gap(cfg, ob, pose):
    """真车(含雷达)在 pose 时离这个障碍物多远(按它给的半径)。"""
    return rp._Sweep(rp.Body(cfg), rp.obstacle_items(cfg, [ob], 0)).pose_gaps(tuple(pose))[0]


def sweep_poses(start, cmds, step=10.0):
    p = tuple(float(v) for v in start)
    out = [p]
    for c, v in cmds:
        n = max(1, int(math.ceil(abs(v) / (step if c != 'R' else 5.0))))
        for k in range(1, n + 1):
            out.append(rp.apply_body_move(p, c, v * k / n))
        p = rp.apply_body_move(p, c, v)
    return out


class GoalObstacleTests(unittest.TestCase):
    def leg_to(self, stop, obs, cfg=CFG, start=None):
        start = start or (STOPS['QR'] if stop != 'QR' else (2100, 600, 90))
        legs, why, pl = rp.plan_mission(cfg, obs, start, [stop])
        self.assertEqual(why, 'OK', f'{stop} {obs}')
        return legs[0], start

    def test_raw_exact_when_body_clears(self):
        # RAW(1200,2100,180)：障碍物在车身 前/后/左/右 10~40mm：都准确停在 (1200,2100)，整段离它至少 10mm
        for side in ('front', 'back', 'left', 'right'):
            for gap in (10, 20, 40):
                ob = obs_beside('RAW', side, gap)
                with self.subTest(side=side, gap=gap):
                    L, start = self.leg_to('RAW', [ob])
                    end = run([L], start)[0]
                    self.assertLess(math.hypot(end[0] - 1200, end[1] - 2100), 0.6, (side, gap, end, L['note']))
                    self.assertAlmostEqual(rp.wrap180(end[2] - 180), 0, delta=1e-6)
                    self.assertEqual(tuple(L['goal']), (1200.0, 2100.0, 180.0))
                    self.assertIsNone(L['goal_shift'])
                    self.assertIn('照样准确停在', L['note'])
                    ok, g, idx, what = rp.moves_clear(CFG, [ob], start, L['cmds'], margin_mm=MIN)
                    self.assertTrue(ok, what)
                    # 黄色区/工位/转盘/场地边：照样按原来的硬余量(没有障碍物时的检查)
                    ok, g, idx, what = rp.moves_clear(CFG, [], start, L['cmds'])
                    self.assertTrue(ok, what)

    def test_other_stops_exact(self):
        for stop in ('ROUGH', 'TEMP', 'QR'):
            for side, gap in (('front', 15), ('back', 30), ('left', 20)):
                ob = obs_beside(stop, side, gap)
                with self.subTest(stop=stop, side=side, gap=gap):
                    L, start = self.leg_to(stop, [ob])
                    end = run([L], start)[0]
                    g = STOPS[stop]
                    self.assertLess(math.hypot(end[0] - g[0], end[1] - g[1]), 0.6, (stop, side, end, L['note']))
                    self.assertIsNone(L['goal_shift'])
                    self.assertTrue(rp.moves_clear(CFG, [ob], start, L['cmds'], margin_mm=MIN)[0])

    def test_rest_of_route_keeps_full_margin(self):
        # 放宽只在停车点附近：离停车点远的地方，车身离这个障碍物照样至少 margin_mm(60)
        for side in ('left', 'back'):
            ob = obs_beside('RAW', side, 15)
            L, start = self.leg_to('RAW', [ob])
            far = [p for p in sweep_poses(start, L['cmds'])
                   if math.hypot(p[0] - 1200, p[1] - 2100) > rp.GOAL_RELAX_RADIUS_MM + 50]
            self.assertTrue(far)
            self.assertGreaterEqual(min(obs_gap(CFG, ob, p) for p in far), CFG['margin_mm'] - 0.5, side)

    def test_full_mission_with_obstacle_near_raw(self):
        # 用户的整场任务(启停区2，两批)：障碍物离 RAW 车头 20mm，两次到 RAW 都准确停在 (1200,2100)
        z = ZONES[2]
        ob = obs_beside('RAW', 'front', 20)
        mission = BASE + ['START2']
        t = time.monotonic()
        legs, why, _ = rp.plan_mission(CFG, [ob], z['start'], mission, start_pivot=z['pivot'])
        dt = time.monotonic() - t
        self.assertEqual(why, 'OK')
        self.assertLess(dt, 8.0, f'规划用了 {dt:.1f} 秒')
        for L, end in zip(legs, run(legs, z['start'])):
            g = STOPS[L['stop']]
            self.assertLess(math.hypot(end[0] - g[0], end[1] - g[1]), 0.6, (L['stop'], end))
        allc = [c for L in legs for c in L['cmds']]
        self.assertTrue(rp.moves_clear(CFG, [ob], z['start'], allc, margin_mm=MIN)[0])
        self.assertTrue(rp.moves_clear(CFG, [], z['start'], allc)[0])

    def test_weak_obstacle_inflation_not_used_at_goal(self):
        # 看不清的圆柱规划时半径 +80(25 → 105)；配置有 cylinder_diameter_mm 时停车点附近按真实的 25 算
        cfg = dict(CFG, cylinder_diameter_mm=50)
        real = obs_beside('RAW', 'left', 40)
        weak = (real[0], real[1], 105.0)
        L, start = self.leg_to('RAW', [weak], cfg)
        end = run([L], start)[0]
        self.assertLess(math.hypot(end[0] - 1200, end[1] - 2100), 0.6, L['note'])
        self.assertTrue(rp.moves_clear(cfg, [real], start, L['cmds'], margin_mm=MIN)[0])

    def test_min_margin_is_configurable(self):
        # 最少要 30mm、真车只有 20mm：停不下，沿车头方向往后挪 10mm
        cfg = dict(CFG, goal_obstacle_min_margin_mm=30)
        ob = obs_beside('RAW', 'front', 20)
        L, start = self.leg_to('RAW', [ob], cfg)
        self.assertEqual(L['goal_shift'], (-10.0, 0.0))
        self.assertGreaterEqual(obs_gap(cfg, ob, L['goal']), 30 - 0.5)

    def test_impossible_stop_moves_along_heading_and_says_so(self):
        # 圆柱压在 RAW 车头外框上(-5mm)：真车停不下 → 沿车头方向(x)往后挪到离它 10mm，y 和车头不变
        ob = obs_beside('RAW', 'front', -5)
        L, start = self.leg_to('RAW', [ob])
        gx, gy, gh = L['goal']
        self.assertAlmostEqual(gy, 2100.0, delta=1e-6)
        self.assertEqual(gh, 180.0)
        self.assertEqual(L['goal_shift'], (-15.0, 0.0))
        self.assertAlmostEqual(gx, 1215.0, delta=1e-6)
        self.assertGreaterEqual(obs_gap(CFG, ob, L['goal']), MIN - 0.5)
        for s in ('停不下', '沿车头方向', '往后 15mm', '(1215,2100)', '任务对准时要补上'):
            self.assertIn(s, L['note'])
        end = run([L], start)[0]
        self.assertLess(math.hypot(end[0] - gx, end[1] - gy), 0.6)
        self.assertTrue(rp.moves_clear(CFG, [ob], start, L['cmds'], margin_mm=MIN)[0])
        # 下一段从挪过的位置出发
        legs, why, _ = rp.plan_mission(CFG, [ob], start, ['RAW', 'ROUGH'])
        self.assertEqual(why, 'OK')
        self.assertEqual(tuple(legs[1]['start']), tuple(L['goal']))

    def test_old_qr_case_now_exact(self):
        # 以前的用例：QR 车头前 50mm 有圆柱(余量要 60) → 以前改停到别处；现在准确停在 QR
        legs, why, _ = rp.plan_mission(CFG, [(2100, 1420, 25)], (2100, 600, 90), ['QR'])
        self.assertEqual(why, 'OK')
        self.assertEqual(tuple(legs[0]['goal']), (2100.0, 1200.0, 90.0))
        end = run(legs, (2100, 600, 90))[0]
        self.assertLess(math.hypot(end[0] - 2100, end[1] - 1200), 0.6)

    def test_plan_leg_home_route_unaffected(self):
        leg = rp.plan_leg(CFG, [obs_beside('RAW', 'back', 20)], STOPS['TEMP'], 'RAW')
        end = run([leg], STOPS['TEMP'])[0]
        self.assertLess(math.hypot(end[0] - 1200, end[1] - 2100), 0.6)



class StopInputTests(unittest.TestCase):
    """(10-20 用户)四个停车点自己输入：改了停车点路线规划不出来报错、障碍物离停车点太近车跑不到指定位置。"""

    def test_stop_on_the_raw_plate_moves_to_the_nearest_free_pose(self):
        # 用户把 RAW 改成 (1200,2210)：以前"本身车身(含雷达)就会压到原料转盘"整场规划失败；现在改停最近停得下的 (1200,2155)
        cfg = dict(CFG, stops=dict(STOPS, RAW=[1200, 2210, 180]))
        legs, why, _ = rp.plan_mission(cfg, [], STOPS['QR'], ['RAW', 'ROUGH'])
        self.assertEqual(why, 'OK')
        L = legs[0]
        self.assertEqual(tuple(L['goal']), (1200.0, 2155.0, 180.0))
        self.assertEqual(L['goal_shift'], (0.0, 55.0))                     # 往左(离原料盘远) 55mm
        self.assertEqual(tuple(L['stop_pose']), (1200.0, 2210.0, 180.0))
        for t in ('压到原料转盘', '(1200,2155)', '往左 55mm', 'stop 命令'):
            self.assertIn(t, L['note'])
        self.assertTrue(rp.moves_clear(CFG, [], STOPS['QR'], L['cmds'])[0])
        end = run([L], STOPS['QR'])[0]
        self.assertLess(math.hypot(end[0] - 1200, end[1] - 2155), 0.6)
        self.assertEqual(tuple(legs[1]['start']), (1200.0, 2155.0, 180.0))  # 下一段从真正停的地方出发
        self.assertIsNone(legs[1]['goal_shift'])

    def test_stop_outside_the_field_moves_in(self):
        cfg = dict(CFG, stops=dict(STOPS, QR=[2350, 1200, 90]))
        legs, why, _ = rp.plan_mission(cfg, [], (2100, 600, 90), ['QR'])
        self.assertEqual(why, 'OK')
        self.assertEqual(tuple(legs[0]['goal']), (2240.0, 1200.0, 90.0))
        self.assertIn('出场地', legs[0]['note'])

    def test_unreachable_stop_parks_nearby_instead_of_failing(self):
        # RAW 两边的车道都被圆柱挡得车身过不去：以前整场"没有可用路线"；run_live 最后一次(goal_near_fallback)改停附近开得到的位置
        obs = [(1060, 1961, 25), (1432, 2208, 25)]
        cfg = dict(CFG, margin_mm=20)
        legs, why, _ = rp.plan_mission(cfg, obs, STOPS['QR'], ['RAW', 'ROUGH'])
        self.assertNotEqual(why, 'OK')
        t = time.monotonic()
        legs, why, pl = rp.plan_mission(dict(cfg, goal_near_fallback=True), obs, STOPS['QR'], ['RAW', 'ROUGH'])
        self.assertLess(time.monotonic() - t, 15.0)
        self.assertEqual(why, 'OK')
        self.assertTrue(getattr(pl, 'near_fallback', False))
        L = legs[0]
        f, l = L['goal_shift']
        self.assertLessEqual(abs(f), rp.GOAL_NEAR_MM)
        self.assertLessEqual(abs(l), rp.GOAL_NEAR_LATERAL_MM)
        self.assertGreater(math.hypot(f, l), 1.0)
        self.assertEqual(tuple(L['stop_pose']), (1200.0, 2100.0, 180.0))
        self.assertIn('开不过去', L['note'])
        self.assertIn('任务对准时要补上', L['note'])
        self.assertTrue(rp.moves_clear(cfg, obs, STOPS['QR'], L['cmds'], margin_mm=MIN)[0])
        self.assertTrue(rp.moves_clear(cfg, [], STOPS['QR'], L['cmds'])[0])
        end = run([L], STOPS['QR'])[0]
        self.assertLess(math.hypot(end[0] - L['goal'][0], end[1] - L['goal'][1]), 0.6)
        self.assertEqual(legs[1]['stop'], 'ROUGH')

    def test_check_stop_says_whether_the_race_moves_it(self):
        """(10-25 用户"我想随便改这四个停靠点的位置而不要报错")stop 输入什么就存什么：check_stop 的 moves 说跑的时候会不会挪走。
        只是离区域比余量近 = 照样准确停在那里；压到区域/出场地 = 自动停到最近停得下的(和 plan_mission 真走的一样)。"""
        r = rp.check_stop(CFG, [], 'ROUGH', (1200, 300, 0))            # 离粗加工区 20mm(余量 30)：不挪
        self.assertFalse(r['ok'])
        self.assertFalse(r['moves'])
        cfg = dict(CFG, stops=dict(STOPS, ROUGH=[1200, 300, 0]))
        legs, why, _ = rp.plan_mission(cfg, [], STOPS['QR'], ['ROUGH'])
        self.assertEqual(why, 'OK')
        self.assertEqual(tuple(legs[0]['goal']), (1200.0, 300.0, 0.0))  # 真的就停在输入的位置
        r = rp.check_stop(CFG, [], 'RAW', (1200, 2210, 180))             # 压到原料转盘：挪
        self.assertTrue(r['moves'])
        cfg = dict(CFG, stops=dict(STOPS, RAW=[1200, 2210, 180]))
        legs, why, _ = rp.plan_mission(cfg, [], STOPS['QR'], ['RAW'])
        self.assertEqual(tuple(legs[0]['goal'][:2]), r['near'][0][:2])   # 挪到的就是 stop 说的那个位置
        self.assertTrue(rp.check_stop(CFG, [], 'QR', (2350, 1200, 90))['moves'])   # 出场地：挪
        self.assertFalse(rp.check_stop(CFG, [], 'QR', STOPS['QR'])['moves'])

    def test_check_stop(self):
        r = rp.check_stop(CFG, [], 'RAW', (1200, 2210, 180))
        self.assertFalse(r['ok'])
        self.assertIn('压到原料转盘', r['why'])
        self.assertEqual(r['near'][0][:2], (1200, 2155))
        r = rp.check_stop(CFG, [], 'RAW', (1200, 2155, 180))
        self.assertTrue(r['ok'], r)
        self.assertEqual(r['warn'], [])
        for name in rp.STOP_NAMES:
            r = rp.check_stop(CFG, [], name, STOPS[name])
            self.assertTrue(r['ok'], (name, r))
            self.assertEqual(r['warn'], [], name)
        r = rp.check_stop(CFG, [], 'ROUGH', (1200, 300, 0))           # 离粗加工区只有 20mm(要 30)
        self.assertFalse(r['ok'])
        self.assertEqual(r['near'][0][:2], (1200, 310))
        r = rp.check_stop(CFG, [], 'ROUGH', (1300, 385, 0))           # 停得下(离黄色区 35)，但离工位远了、沿那一排偏了
        self.assertTrue(r['ok'], r)
        self.assertTrue(any('远 45mm' in w for w in r['warn']), r['warn'])
        self.assertTrue(any('2 号环可能不在摄像头里' in w for w in r['warn']), r['warn'])
        r = rp.check_stop(CFG, [], 'RAW', (1200, 2100, 0))            # 车头反了：原料盘在车的左边
        self.assertTrue(any('左边' in w for w in r['warn']), r['warn'])
        r = rp.check_stop(CFG, [], 'TEMP', (340, 1200, -80))
        self.assertFalse(r['ok'])
        self.assertIn('横平竖直', r['why'])
        r = rp.check_stop(CFG, [], 'QR', (2350, 1200, 90))
        self.assertFalse(r['ok'])
        self.assertEqual(r['near'][0][:2], (2240, 1200))
        ob = obs_beside('TEMP', 'front', 30)                           # 障碍物离车头 30mm：能停(停车点附近按 10mm 放宽)，提醒
        r = rp.check_stop(CFG, [ob], 'TEMP', STOPS['TEMP'])
        self.assertTrue(r['ok'])
        self.assertTrue(any('障碍物' in w for w in r['warn']), r['warn'])
        ob = obs_beside('TEMP', 'front', -5)                           # 压到车头：不能停，最近能停的往后挪
        r = rp.check_stop(CFG, [ob], 'TEMP', STOPS['TEMP'])
        self.assertFalse(r['ok'])
        self.assertTrue(r['near'])
        self.assertGreaterEqual(obs_gap(CFG, ob, (r['near'][0][0], r['near'][0][1], -90.0)), MIN - 0.5)


class GoalObstacleDriveTests(unittest.TestCase):
    """auto_run 按规划的路线开：停车点旁边有障碍物时车真的停在停车点上；挪了停车点时终端说清楚。"""

    def drive(self, legs, start, obs, cfg=None):
        car = tnd.SimCar(start, scale=0.01)
        ctx = tnd.SimCtx(car, obstacles=obs)
        logs = []
        # (10-23)比赛默认到站不用雷达定位(只有回启停区用)：这里测的是"到站雷达修正会不会被旁边的障碍物限幅"，照以前打开
        c = dict(tnd.CFG, **dict(tnd.OLD_RELOC, **(cfg or {})))
        hooks = tnd.Hooks(car)
        hooks.offsets = []
        hooks.stop_offset = lambda stop, f, l: hooks.offsets.append((stop, f, l))
        auto_run.drive(ctx, car, auto_run.flatten(0, legs), log=logs.append, stop_wait=0, hooks=hooks,
                       speeds={'F': 150, 'S': 140}, motion=None, cfg=c, start=dict(est=start, ref=start),
                       caps={'rdec': False})
        return car, hooks, logs

    def test_reaches_raw_next_to_obstacle(self):
        for exec_obs in (False, True):
            ob = obs_beside('RAW', 'front', 20)
            legs, why, _ = rp.plan_mission(CFG, [ob], STOPS['QR'], ['RAW'])
            self.assertEqual(why, 'OK')
            # 停车点修正的容差设成 5mm(用户配置是 15)：看修正会不会因为旁边的障碍物被限幅、到不了停车点
            car, hooks, logs = self.drive(legs, tuple(float(v) for v in STOPS['QR']), [ob],
                                          dict(exec_obstacles=exec_obs, reloc_pos_tol_mm=5, reloc_lateral_tol_mm=5))
            stop, p = hooks.at[0]
            self.assertEqual(stop, 'RAW')
            self.assertLess(math.hypot(p[0] - 1200, p[1] - 2100), 7, (exec_obs, p, logs))
            self.assertGreaterEqual(min(obs_gap(CFG, ob, q) for q in car.trace), 0.0)
            self.assertEqual(hooks.offsets, [])

    def test_shifted_stop_is_logged(self):
        ob = obs_beside('RAW', 'front', -5)
        legs, why, _ = rp.plan_mission(CFG, [ob], STOPS['QR'], ['RAW'])
        seq = auto_run.flatten(0, legs)
        self.assertEqual(seq[-1][2]['shift'], (-15.0, 0.0))
        car, hooks, logs = self.drive(legs, tuple(float(v) for v in STOPS['QR']), [ob], dict(reloc_pos_tol_mm=5, reloc_lateral_tol_mm=5))
        self.assertTrue(any('规定的停车点被障碍物挡住' in l and '往后 15mm' in l for l in logs), logs)
        self.assertEqual(hooks.offsets, [('RAW', -15.0, 0.0)])
        stop, p = hooks.at[0]
        self.assertLess(math.hypot(p[0] - 1215, p[1] - 2100), 7, p)


if __name__ == '__main__':
    unittest.main()
