"""审查时补的导航测试(1010)：没有 PREHOME 时从 home_cut 的地方(离启停区几百 mm)回家，距离差 2~3% 也不出场地；
回家前定位'near'但还在启停区外时到 START 再对准一次；手臂轻摆的默认间隔不被 hooks 的 5 秒限制吞掉；
race 第一次扫描失败不退出、过一会儿重扫。python3 -m unittest test_nav_audit"""
import unittest

import test_auto_run  # noqa: F401  (先装好 stm32_link / route_plan 的替身)
import auto_run
from auto_run import Abort
from test_nav_drive import SimCar, SimCtx, CFG, ZONE2, edge_min
from test_nav_race import Clock, RaceCtx


def drive_home(car, ctx, cfg=None):
    """从 (2250,1200) 朝北的车，最后一段直接后退 1050 到启停区 2(没有 PREHOME：走 home_cut)。"""
    p0 = (2250.0, 1200.0, 90.0)
    legs = [dict(stop='X', goal=p0, cmds=[]), dict(stop='START2', goal=ZONE2, cmds=[('F', -1050)])]
    seq = auto_run.flatten(0, legs)
    logs = []
    cfg = dict(CFG, **(cfg or {}))
    auto_run.drive(ctx, car, seq, log=logs.append, stop_wait=0, hooks=None, speeds={'F': 150, 'S': 140},
                   motion=auto_run.Motion(cfg), cfg=cfg, start=dict(est=p0, ref=p0), caps={'rdec': False})
    return logs


class CutApproachTests(unittest.TestCase):
    def test_long_home_move_stops_short_then_finishes(self):
        # 以前：home_cut 的地方(离目标 ~800mm)第一轮修正直接走满 F -796 到离边线 10mm 的目标，多走 2% 就出场地 5.8mm
        for scale in (0.0, 0.02, 0.03, -0.03):
            car = SimCar((2250.0, 1200.0, 90.0), scale=scale)
            drive_home(car, SimCtx(car, noise=(0, 0)))
            self.assertGreaterEqual(edge_min(car.trace[1:], CFG), 3.0, f'距离差 {scale:+.0%} 车身出场地/离边线太近')
            goal = auto_run.inset_goal(ZONE2, CFG)
            self.assertLess(abs(car.y - goal[1]), 6.5, (scale, car.pose()))
            self.assertTrue(auto_run.in_rect(car.pose(), auto_run.START_RECTS[2]))
            big = [v for c, v, sp in car.sent if c == 'F' and sp == 60 and abs(v) > 100]
            self.assertEqual(len(big), 1)
            self.assertLessEqual(abs(big[0]), 800 - 15 - 0.02*780 + 2, '长的回家修正要先少走 2% + 15mm')

    def test_short_home_corrections_are_not_shortened(self):
        # 只差几十毫米(第二轮/PREHOME 以后)：照走，不再少走(否则永远修不到误差范围里)
        car = SimCar((2250.0, 190.0, 90.0))
        nav = auto_run.Nav(dict(est=car.pose(), ref=car.pose()))
        res = auto_run.home_approach(SimCtx(car, noise=(0, 0)), car, lambda m: None, nav, ZONE2, CFG, {'rdec': False})
        self.assertEqual(res['status'], 'ok')
        self.assertEqual([(c, v) for c, v, _ in car.sent], [('F', -35)])


class StartReapproachTests(unittest.TestCase):
    def run_with(self, first):
        calls = []
        orig = auto_run.home_approach

        def fake(ctx, link, log, nav, goal, cfg, caps, obstacles=(), **kw):
            calls.append(kw.get('label', '回启停区对准'))
            return first if len(calls) == 1 else dict(status='ok', pose=tuple(goal))
        auto_run.home_approach = fake
        try:
            car = SimCar((2250.0, 1200.0, 90.0))
            drive_home(car, SimCtx(car, noise=(0, 0)))
        finally:
            auto_run.home_approach = orig
        return calls

    def test_near_outside_start_zone_tries_again_at_start(self):
        calls = self.run_with(dict(status='near', pose=(2250.0, 700.0, 90.0)))
        self.assertEqual(calls, ['回家前定位', '回启停区对准'])

    def test_near_inside_start_zone_is_enough(self):
        calls = self.run_with(dict(status='near', pose=(2250.0, 160.0, 90.0)))
        self.assertEqual(calls, ['回家前定位'])

    def test_ok_is_enough(self):
        calls = self.run_with(dict(status='ok', pose=(2250.0, 155.0, 90.0)))
        self.assertEqual(calls, ['回家前定位'])


class KeepaliveDefaultTests(unittest.TestCase):
    def test_default_period_longer_than_hooks_limit(self):
        # hooks.keepalive 离上次调用不到 5 秒就不摆：默认间隔要大于 5 秒，每次调用都真的摆
        clock = Clock()
        got = []

        class H:
            last = [-99.0]

            def keepalive(self):
                if clock.t - H.last[0] >= 5.0:
                    H.last[0] = clock.t
                    got.append(clock.t)
        tick = auto_run.make_tick(H(), {}, now=clock.now)
        t0 = clock.t
        for _ in range(250):
            tick()
            clock.sleep(0.1)
        gaps = [b - a for a, b in zip([t0] + got, got)]
        self.assertGreaterEqual(len(got), 4)
        self.assertLessEqual(max(gaps), 6.0, gaps)


class RaceScanRetryTests(unittest.TestCase):
    def test_scan_failure_rescans_instead_of_quitting(self):
        clock = Clock()

        class Flaky(RaceCtx):
            n = 0

            def scan(self):
                Flaky.n += 1
                self.calls.append(('scan', self.clock.t))
                self.clock.t += 4.0
                if Flaky.n == 1:
                    raise Abort('扫描失败，见上面的提示')
        car = SimCar((2250.0, 150.0, 90.0), zone_script=[(2, 0)] * 40 + [(2, 1)])
        ctx = Flaky(clock)
        runs = []
        st = auto_run.race_flow(ctx, car, lambda m: None, {'race_check_s': 0}, lambda z, sc: runs.append((z, sc)),
                                now=clock.now, sleep=clock.sleep)
        self.assertEqual(st, 'started')
        self.assertEqual([c[0] for c in ctx.calls].count('scan'), 2)
        self.assertIn('ZONE MSG SCAN ERR', car.requests)
        self.assertEqual(runs, [(2, True)])
        self.assertEqual(car.sent, [])

    def test_abort_during_scan_still_stops(self):
        clock = Clock()

        class Aborting(RaceCtx):
            def scan(self):
                self.abort_at = self.clock.t
                raise Abort('收到 abort')
        car = SimCar((2250.0, 150.0, 90.0), zone_script=[(2, 0)] * 40 + [(2, 1)])
        with self.assertRaises(Abort):
            auto_run.race_flow(Aborting(clock), car, lambda m: None, {'race_check_s': 0}, lambda z, sc: None,
                               now=clock.now, sleep=clock.sleep)


if __name__ == '__main__':
    unittest.main()
