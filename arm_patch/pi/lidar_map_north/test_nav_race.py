"""比赛流程((10-23)终端 race 1|2 选区、屏上只有 START -> 扫描、预先规划 -> 按 START 一键开跑)、手臂轻摆保活、退出收臂停 60mm、开跑后不再问 y 的测试(1010)。
python3 -m unittest test_nav_race"""
import threading
import time
import unittest

import test_auto_run  # noqa: F401  (先装好 stm32_link / route_plan 的替身)
import auto_run
from auto_run import Abort
from test_nav_drive import SimCar


class Clock:
    def __init__(self):
        self.t = 100.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s


class RaceCtx:
    """race 用的 ctx 替身：记下调用；gyro 可以按时间改；preplan 一开始就 'ok'(或指定)。"""

    def __init__(self, clock, gyro=lambda t: 90.0, plan='ok', still=None, abort_at=None):
        self.clock = clock
        self.calls = []
        self._gyro = gyro
        self.plan = plan
        self.still = still or (lambda t: None)
        self.abort_at = abort_at

    def aborted(self):
        return self.abort_at is not None and self.clock.t >= self.abort_at

    def zone_select(self, z):
        self.calls.append(('zone', z))

    def scan(self):
        self.calls.append(('scan', self.clock.t))
        self.clock.t += 4.0

    def preplan_start(self):
        self.calls.append(('preplan',))

    def preplan_state(self):
        return self.plan

    def gyro_yaw(self):
        return self._gyro(self.clock.t)

    def still_check(self):
        return self.still(self.clock.t)


def run_race(script, ctx_kw=None, park='ok', zone=None, logs=None):
    clock = Clock()
    car = SimCar((2250.0, 150.0, 90.0), zone_script=script)
    ctx = RaceCtx(clock, **(ctx_kw or {}))
    runs = []
    st = auto_run.race_flow(ctx, car, (logs.append if logs is not None else (lambda m: None)), {'race_check_s': 0, 'race_start': 'screen'},
                            lambda z, sc: runs.append((z, sc)), now=clock.now, sleep=clock.sleep, zone=zone)
    return st, car, ctx, runs, clock


class RaceTests(unittest.TestCase):
    def test_choose_zone_scan_plan_then_start(self):
        script = [(0, 0)] * 3 + [(2, 0)] * 15 + [(2, 1)]
        st, car, ctx, runs, clock = run_race(script)
        self.assertEqual(st, 'started')
        self.assertEqual(car.requests[0], 'ZONE 2', '(10-23)不再发 ZONE ASK(选区页)：直接发配置里的启停区，屏上只有 START')
        self.assertNotIn('ZONE ASK', car.requests)
        self.assertEqual([c[0] for c in ctx.calls], ['zone', 'scan', 'preplan'])
        self.assertEqual(ctx.calls[0], ('zone', 2))
        msgs = [r for r in car.requests if r.startswith('ZONE MSG')]
        self.assertEqual(msgs, ['ZONE MSG PLACE CAR', 'ZONE MSG SCAN', 'ZONE MSG PLAN', 'ZONE MSG READY Z2'])
        self.assertTrue(all(len(m) - len('ZONE MSG ') <= 20 for m in msgs))
        self.assertEqual(car.requests[-1], 'ZONE LOCK', '按了 START 先锁屏再开跑')
        self.assertEqual(runs, [(2, True)])
        self.assertEqual(car.sent, [], '开跑前车不能动')

    def test_waits_until_car_is_still(self):
        # 选区后车还在被摆(陀螺仪在变)：等稳定 2 秒以后才扫描
        t_still = [None]

        def gyro(t):
            if t < 110.0:
                return 90.0 + (t * 7) % 5
            return 91.0
        script = [(1, 0)] * 60 + [(1, 1)]
        st, car, ctx, runs, clock = run_race(script, dict(gyro=gyro))
        scans = [c for c in ctx.calls if c[0] == 'scan']
        self.assertEqual(len(scans), 1)
        self.assertGreaterEqual(scans[0][1], 112.0 - 1e-9)
        self.assertEqual(runs, [(1, True)])

    def test_moved_after_scan_rescans(self):
        def gyro(t):
            return 90.0 if t < 112.0 else 93.5             # 扫描完又被转了 3.5°
        script = [(2, 0)] * 40 + [(2, 1)]
        st, car, ctx, runs, clock = run_race(script, dict(gyro=gyro))
        self.assertEqual([c[0] for c in ctx.calls].count('scan'), 2, '车被挪了要重新扫描')
        self.assertEqual([c for c in ctx.calls if c[0] == 'zone'], [('zone', 2), ('zone', 2)])
        self.assertIn('ZONE MSG MOVED', car.requests)

    def test_lidar_sees_car_moved(self):
        clock_hits = []

        def still(t):
            clock_hits.append(t)
            return dict(moved=len(clock_hits) == 1, why='雷达看到车挪了 40mm')
        script = [(2, 0)] * 60 + [(2, 1)]
        clock = Clock()
        car = SimCar((2250.0, 150.0, 90.0), zone_script=script)
        ctx = RaceCtx(clock, still=still)
        runs = []
        auto_run.race_flow(ctx, car, lambda m: None, {'race_check_s': 3, 'race_start': 'screen'}, lambda z, sc: runs.append((z, sc)),
                           now=clock.now, sleep=clock.sleep)
        self.assertEqual([c[0] for c in ctx.calls].count('scan'), 2)
        self.assertEqual(runs, [(2, True)])

    def test_zone_lost_then_changed_rescans_new_zone(self):
        """(10-23)STM32 那边的区没了(重启过)：再发一次 ZONE 2；之后终端 zone 1 改了区：按 1 区重新扫描。"""
        script = [(2, 0)] * 15 + [(0, 0)] * 10 + [(1, 0)] * 15 + [(1, 1)]
        st, car, ctx, runs, clock = run_race(script)
        self.assertEqual([c for c in ctx.calls if c[0] == 'zone'], [('zone', 2), ('zone', 1)])
        self.assertEqual([c[0] for c in ctx.calls].count('scan'), 2)
        self.assertGreaterEqual(car.requests.count('ZONE 2'), 2, '区没了要再发一次 ZONE 2(屏上重新出 START)')
        self.assertEqual(runs, [(1, True)])

    def test_zone_from_race_command(self):
        """(10-23)race 1：发 ZONE 1(屏上只出 START，不显示区号)，按 1 区扫描、开跑；终端里说清楚是几区。"""
        logs = []
        st, car, ctx, runs, clock = run_race([(1, 0)] * 15 + [(1, 1)], zone=1, logs=logs)
        self.assertEqual(car.requests[0], 'ZONE 1')
        self.assertEqual(runs, [(1, True)])
        self.assertTrue(any('启停区 1' in l and 'race 1 或 race 2' in l for l in logs), logs[:3])
        self.assertTrue(any('可以按屏上的 START' in l for l in logs), logs)

    def test_zone_lost_for_long_resends_every_two_seconds(self):
        """STM32 一直回 0 区：每 2 秒再发一次 ZONE n，不是每次问都发(串口别刷屏)。"""
        st, car, ctx, runs, clock = run_race([(0, 0)] * 40 + [(2, 1)], zone=2)
        n = car.requests.count('ZONE 2')
        self.assertTrue(4 <= n <= 8, n)
        self.assertEqual(runs, [(2, False)])

    def test_start_before_scan(self):
        st, car, ctx, runs, clock = run_race([(2, 1)])
        self.assertEqual(runs, [(2, False)], '还没扫描就按了 START：开跑后再扫')
        self.assertNotIn('scan', [c[0] for c in ctx.calls])

    def test_plan_failure_shown(self):
        st, car, ctx, runs, clock = run_race([(2, 0)] * 15 + [(2, 1)], dict(plan='去 RAW 的路线失败'))
        self.assertIn('ZONE MSG PLAN ERR', car.requests)
        self.assertEqual(runs, [(2, True)], '规划失败也能开跑(第二次扫描后再规划)')

    def test_old_firmware_falls_back(self):
        clock = Clock()

        class Old(SimCar):
            def request(self, text, timeout=None, collect=False):
                self.requests.append(text)
                return (False, 'ERR CMD', []) if collect else (False, 'ERR CMD')
        car = Old((2250.0, 150.0, 90.0))
        ctx = RaceCtx(clock)
        logs = []
        st = auto_run.race_flow(ctx, car, logs.append, {'race_start': 'screen'}, lambda z, s: None, now=clock.now, sleep=clock.sleep)
        self.assertEqual(st, 'nozone')
        self.assertEqual(ctx.calls, [])
        self.assertTrue(any('zone 1' in l and 'go' in l for l in logs))

    def test_abort_while_waiting(self):
        with self.assertRaises(Abort):
            run_race([(0, 0)] * 100, dict(abort_at=105.0))

    def test_fixed_route_waits_for_screen_start_without_scanning(self):
        """(10-28)写死模式(ctx.fixed_route())：race_start 是 now 也等屏上按 START；等的时候不扫描、不预先规划、不看车挪没挪；
        选了区就告诉屏 READY，按了 START 先锁屏(屏幕变回白色)再开跑。"""
        class FixedCtx(RaceCtx):
            def fixed_route(self):
                return dict(wait1=3.0, wait2=3.0, t0=None)

            def still_check(self):
                self.calls.append(('still',))
                return dict(moved=True, why='雷达看到车挪了')
        for cfg in ({'race_check_s': 1, 'race_start': 'screen'}, {'race_check_s': 1}):
            clock = Clock()
            car = SimCar((2250.0, 150.0, 90.0), zone_script=[(0, 0)] * 2 + [(2, 0)] * 30 + [(2, 1)])
            ctx = FixedCtx(clock, gyro=lambda t: 90.0 + (t % 7))          # 陀螺仪一直在变：写死模式不管
            runs, logs = [], []
            st = auto_run.race_flow(ctx, car, logs.append, cfg, lambda z, sc: runs.append((z, sc)), now=clock.now, sleep=clock.sleep)
            self.assertEqual(st, 'started')
            self.assertEqual(car.requests[0], 'ZONE 2', '屏上出 START')
            self.assertEqual(ctx.calls, [('zone', 2)], '不扫描、不预先规划、不看车挪没挪')
            self.assertEqual([r for r in car.requests if r.startswith('ZONE MSG')], ['ZONE MSG READY Z2'])
            self.assertEqual(car.requests[-1], 'ZONE LOCK')
            self.assertEqual(runs, [(2, False)])
            self.assertTrue(any('写死模式' in l and '可以按屏上的 START' in l for l in logs), logs)
            self.assertFalse(any('还没扫描就按了 START' in l for l in logs), logs)
            self.assertEqual(car.sent, [], '开跑前车不能动')

    def test_race_now_starts_directly(self):
        """(10-23 晚 用户"开跑前全黑、不出 START，race 2 直接开跑")默认：清屏(不发 ZONE n，屏上不出 START)，切到选的区马上开跑；
        不等车静止、开跑前不扫描(开跑后再扫)；旧固件不认识 ZONE 也照样开跑。"""
        for zone, want in ((2, 2), (1, 1), (None, 2)):
            clock = Clock()
            car = SimCar((2250.0, 150.0, 90.0))
            ctx = RaceCtx(clock)
            runs, logs = [], []
            st = auto_run.race_flow(ctx, car, logs.append, {}, lambda z, sc: runs.append((z, sc)), now=clock.now,
                                    sleep=clock.sleep, zone=zone)
            self.assertEqual(st, 'started')
            self.assertEqual(car.requests, ['ZONE LOCK'])
            self.assertEqual(ctx.calls, [('zone', want)])
            self.assertEqual(runs, [(want, False)])
            self.assertEqual(clock.t, 100.0, '不等')
            self.assertTrue(any(f'启停区 {want}：直接开跑' in l for l in logs), logs)
        clock = Clock()

        class Old(SimCar):
            def request(self, text, timeout=None, collect=False):
                self.requests.append(text)
                return (False, 'ERR CMD', []) if collect else (False, 'ERR CMD')
        car = Old((2250.0, 150.0, 90.0))
        runs = []
        st = auto_run.race_flow(RaceCtx(clock), car, lambda m: None, {'start_zone': 1}, lambda z, sc: runs.append((z, sc)),
                                now=clock.now, sleep=clock.sleep)
        self.assertEqual((st, runs), ('started', [(1, False)]))


class ParkTests(unittest.TestCase):
    def test_terminal_park_forces_after_abort(self):
        # 急停以后 hooks.park() 默认不动；用户自己输入 park 时 force=True，要听用户的
        car = SimCar((0, 0, 0))

        class H:
            got = []

            def park(self, link=None, force=False):
                H.got.append(force)
                return force
        self.assertFalse(auto_run.park_arm(car, H(), lambda m: None))
        self.assertTrue(auto_run.park_arm(car, H(), lambda m: None, force=True))
        self.assertEqual(H.got, [False, True])

    def test_hooks_park_preferred(self):
        car = SimCar((0, 0, 0))

        class H:
            n = 0

            def park(self):
                H.n += 1
        self.assertTrue(auto_run.park_arm(car, H(), lambda m: None))
        self.assertEqual(H.n, 1)
        self.assertEqual(car.requests, [])

    def test_park_command(self):
        car = SimCar((0, 0, 0))
        self.assertTrue(auto_run.park_arm(car, None, lambda m: None))
        self.assertEqual(car.requests, ['PARK'])

    def test_old_firmware_stow_then_lift(self):
        car = SimCar((0, 0, 0), park='ERR CMD')
        self.assertTrue(auto_run.park_arm(car, None, lambda m: None))
        self.assertEqual(car.requests, ['PARK', 'STOW', 'LIFT 60'])

    def test_unknown_lift_does_not_move(self):
        car = SimCar((0, 0, 0), park='ERR NOZERO')
        self.assertFalse(auto_run.park_arm(car, None, lambda m: None))
        self.assertEqual(car.requests, ['PARK'])

    def test_stow_failure_does_not_lower(self):
        class C(SimCar):
            def request(self, text, timeout=None, collect=False):
                self.requests.append(text)
                out = {'PARK': (False, 'ERR CMD', []), 'STOW': (False, 'ERR NOZERO', [])}.get(text, (True, 'DONE', []))
                return out
        car = C((0, 0, 0))
        self.assertFalse(auto_run.park_arm(car, None, lambda m: None))
        self.assertEqual(car.requests, ['PARK', 'STOW'])

    def test_stow_nocal_still_lowers(self):
        class C(SimCar):
            def request(self, text, timeout=None, collect=False):
                self.requests.append(text)
                return {'PARK': (False, 'ERR CMD', []), 'STOW': (False, 'ERR NOCAL', [])}.get(text, (True, 'DONE', []))
        car = C((0, 0, 0))
        self.assertTrue(auto_run.park_arm(car, None, lambda m: None))
        self.assertEqual(car.requests, ['PARK', 'STOW', 'LIFT 60'])

    def test_quit_parks_only_when_idle_and_not_aborted(self):
        car = SimCar((0, 0, 0))
        self.assertFalse(auto_run.quit_park(car, None, lambda m: None, running=False, aborted=True))
        self.assertEqual(car.requests, [], '急停过：退出时不自动动')
        self.assertFalse(auto_run.quit_park(car, None, lambda m: None, running=True, aborted=False))
        self.assertEqual(car.requests, [])
        self.assertEqual(car.aborted, 1, '退出时还在跑：先急停')
        self.assertIsNone(auto_run.quit_park(car, None, lambda m: None, running=False, aborted=False, enabled=False))
        self.assertTrue(auto_run.quit_park(car, None, lambda m: None, running=False, aborted=False))
        self.assertEqual(car.requests, ['PARK'])
        self.assertIsNone(auto_run.quit_park(None, None, lambda m: None, running=False, aborted=False))


class KeepaliveTests(unittest.TestCase):
    def test_tick_rate_limited(self):
        clock = Clock()

        class H:
            n = 0

            def keepalive(self):
                H.n += 1
        tick = auto_run.make_tick(H(), {'keepalive_s': 4.0}, now=clock.now)
        for _ in range(100):
            tick()
            clock.sleep(0.1)
        self.assertEqual(H.n, 2)                          # 10 秒里每 4 秒一次
        self.assertIsNone(auto_run.make_tick(object(), {}))
        self.assertIsNone(auto_run.make_tick(None, {}))

    def test_run_bg_ticks_in_calling_thread(self):
        me = threading.get_ident()
        seen = []
        r = auto_run.run_bg(lambda: (time.sleep(0.3), 42)[1], tick=lambda: seen.append(threading.get_ident()), poll=0.02)
        self.assertEqual(r, 42)
        self.assertTrue(seen and all(t == me for t in seen), '手臂轻摆只能在行驶线程里调用')

    def test_run_bg_propagates_errors_and_abort(self):
        def bad():
            raise ValueError('规划出错')
        with self.assertRaises(ValueError):
            auto_run.run_bg(bad)
        with self.assertRaises(Abort):
            auto_run.run_bg(lambda: time.sleep(0.5), aborted=lambda: True, poll=0.01)


class MissionCtx:
    """run_mission 用的 ctx：规划在后台线程里慢慢做(0.3 秒)，等的时候调用 tick。"""

    def __init__(self, calls):
        self.calls = calls
        self.tick = None

    def aborted(self):
        return False

    def scan(self):
        self.calls.append('scan')

    def second(self):
        self.calls.append('second')

    def plan(self):
        self.calls.append('plan')
        auto_run.run_bg(lambda: time.sleep(0.3), tick=self.tick, poll=0.02)
        return 0, [{'stop': 'QR', 'cmds': [('R', -90), ('F', 300)]}], 'ROUTE OK'

    def ask(self, msg):
        self.calls.append('ask')
        return True


class RecLink(SimCar):
    def __init__(self, calls):
        SimCar.__init__(self, (2250.0, 150.0, 90.0))
        self.calls = calls

    def ping(self):
        self.calls.append('ping')
        return True, 'PONG'

    def home(self):
        self.calls.append('home')
        return SimCar.home(self)

    def second_move(self, rel=None, log=None):
        self.calls.append('second_move')
        return SimCar.second_move(self, rel, log)


class MissionHooksRec:
    def __init__(self, calls):
        self.calls = calls
        self.threads = []

    def start_clock(self):
        self.calls.append('start_clock')

    def prepare(self, link, log):
        self.calls.append('prepare')

    def keepalive(self):
        self.threads.append(threading.get_ident())

    def task(self, stop, link, log):
        self.calls.append('task ' + stop)


class RunMissionTests(unittest.TestCase):
    def test_go_order_no_y_and_keepalive_while_planning(self):
        calls = []
        ctx, link, h = MissionCtx(calls), RecLink(calls), MissionHooksRec(calls)
        me = threading.get_ident()
        auto_run.run_mission(ctx, link, log=lambda m: None, stop_wait=0, settle_s=0, hooks=h, cfg={'keepalive_s': 0.05})
        self.assertEqual(calls[:7], ['start_clock', 'ping', 'prepare', 'home', 'scan', 'second_move', 'second'])
        self.assertNotIn('ask', calls, '开跑后不能再要人输入 y')
        self.assertIn('task QR', calls)
        self.assertTrue(h.threads, '规划时间长要调用 hooks.keepalive()')
        self.assertTrue(all(t == me for t in h.threads))

    def test_first_scan_failure_rescans_once(self):
        """(10-23 晚)race 默认直接开跑，第一次扫描在开跑以后：雷达卡一下(扫描失败)就重扫一次接着跑；急停了不重扫；连着失败两次才停。"""
        calls = []

        class Flaky(MissionCtx):
            n = 0

            def scan(self):
                Flaky.n += 1
                calls.append(('scan', Flaky.n))
                if Flaky.n == 1:
                    raise Abort('扫描失败，见上面的提示')
        logs = []
        auto_run.run_mission(Flaky(calls), RecLink(calls), log=logs.append, stop_wait=0, settle_s=0, hooks=MissionHooksRec(calls), cfg={})
        self.assertEqual([c for c in calls if c[0] == 'scan'], [('scan', 1), ('scan', 2)])
        self.assertTrue(any('第一次扫描失败' in l and '再扫一次' in l for l in logs), logs)

        class Dead(MissionCtx):
            def scan(self):
                calls.append(('dead',))
                raise Abort('扫描失败，见上面的提示')
        calls.clear()
        with self.assertRaises(Abort):
            auto_run.run_mission(Dead(calls), RecLink(calls), log=lambda m: None, stop_wait=0, settle_s=0, hooks=MissionHooksRec(calls), cfg={})
        self.assertEqual(calls.count(('dead',)), 2)

        class Aborted(MissionCtx):
            def aborted(self):
                return True

            def scan(self):
                calls.append(('ab',))
                raise Abort('收到 abort')
        calls.clear()
        with self.assertRaises(Abort):
            auto_run.run_mission(Aborted(calls), RecLink(calls), log=lambda m: None, stop_wait=0, settle_s=0, hooks=MissionHooksRec(calls), cfg={})
        self.assertEqual(calls.count(('ab',)), 1, '急停了不重扫')

    def test_race_skips_first_scan(self):
        calls = []
        auto_run.run_mission(MissionCtx(calls), RecLink(calls), log=lambda m: None, stop_wait=0, settle_s=0,
                             hooks=MissionHooksRec(calls), cfg={}, scanned=True)
        self.assertNotIn('scan', calls)
        self.assertIn('second', calls)

    def test_confirm_route_only_for_bench(self):
        calls = []
        auto_run.run_mission(MissionCtx(calls), RecLink(calls), log=lambda m: None, stop_wait=0, settle_s=0,
                             cfg={'confirm_route': True})
        self.assertIn('ask', calls)

    def test_hooks_without_new_methods(self):
        # 旧版 MissionHooks(没有 start_clock / prepare / keepalive)照样能跑
        calls = []

        class Old:
            def task(self, stop, link, log):
                calls.append('task ' + stop)
        auto_run.run_mission(MissionCtx(calls), RecLink(calls), log=lambda m: None, stop_wait=0, settle_s=0, hooks=Old(), cfg={})
        self.assertIn('task QR', calls)

    def test_fixed_route_waits_instead_of_scanning(self):
        """(10-28)写死模式：两次扫描都不做、不规划——开跑(t0)以后原地等到 wait1 秒(机械臂准备也算在里面)，第二站动作，再原地等 wait2 秒，
        然后按存好的路线跑；日志里写明是写死模式。"""
        calls = []
        stamp = {}

        class FixedCtx(MissionCtx):
            t0 = None

            def fixed_route(self):
                return dict(wait1=0.4, wait2=0.3, t0=self.t0)

            def fixed_first(self):
                calls.append('fixed_first')

            def fixed_second(self):
                calls.append('fixed_second')
                stamp['second'] = time.monotonic()

            def plan(self):
                calls.append('plan')
                return 0, [{'stop': 'QR', 'cmds': [('R', -90), ('F', 300)]}], 'ROUTE OK'

        class Link(RecLink):
            def second_move(self, rel=None, log=None):
                stamp['move'] = time.monotonic()
                return RecLink.second_move(self, rel, log)
        ctx = FixedCtx(calls)
        ctx.t0 = time.monotonic()
        logs = []
        auto_run.run_mission(ctx, Link(calls), log=logs.append, stop_wait=0, settle_s=0, hooks=MissionHooksRec(calls), cfg={})
        self.assertEqual(calls[:8], ['start_clock', 'ping', 'prepare', 'home', 'fixed_first', 'second_move', 'fixed_second', 'plan'])
        self.assertNotIn('scan', calls)
        self.assertNotIn('second', calls)
        self.assertIn('task QR', calls)
        self.assertGreaterEqual(stamp['move'] - ctx.t0, 0.4 - 0.01, '开跑以后先原地等 wait1 秒')
        self.assertGreaterEqual(stamp['second'] - stamp['move'], 0.3 - 0.01, '第二站动作以后再原地等 wait2 秒')
        text = '\n'.join(logs)
        self.assertIn('① 写死模式：不扫描、不规划', text)
        self.assertIn('③ 写死模式：不扫描，原地等 0.3 秒', text)
        self.assertIn('④ 写死路线', text)
        for word in ('第一次扫描', '第二次扫描', '规划用时', '路线规划完成'):
            self.assertNotIn(word, text)
        # 关着(fixed_route 返回 None)：和以前一样扫描两次、规划
        calls.clear()

        class Off(MissionCtx):
            def fixed_route(self):
                return None
        auto_run.run_mission(Off(calls), RecLink(calls), log=lambda m: None, stop_wait=0, settle_s=0, hooks=MissionHooksRec(calls), cfg={})
        self.assertEqual(calls[:7], ['start_clock', 'ping', 'prepare', 'home', 'scan', 'second_move', 'second'])

    def test_fixed_route_abort_while_waiting(self):
        calls = []

        class FixedCtx(MissionCtx):
            def aborted(self):
                return 'fixed_first' in calls

            def fixed_route(self):
                return dict(wait1=5.0, wait2=5.0, t0=time.monotonic())

            def fixed_first(self):
                calls.append('fixed_first')
        t = time.monotonic()
        with self.assertRaises(Abort):
            auto_run.run_mission(FixedCtx(calls), RecLink(calls), log=lambda m: None, stop_wait=0, settle_s=0, hooks=MissionHooksRec(calls), cfg={})
        self.assertLess(time.monotonic() - t, 1.0, '等的时候 abort 马上停')
        self.assertNotIn('second_move', calls, '车没动')

    def test_fixed_route_presynced_waits_3s_then_goes(self):
        """(10-29 晚 用户"你先让这些准备好最后我按这个红色按钮的时候就不用准备了就是我按下之后就可是停三秒直接走了")写死模式 race：
        参数按 START 之前已经同步好(cfg _presynced)，按了 START 不再 GET/SET；原地等 wait1 → 第二站动作 → wait2=0 不再等，直接开；
        r_decimals 按 START 之前问好了，按了以后不再 GET。"""
        calls = []
        stamp = {}

        class FixedCtx(MissionCtx):
            t0 = None

            def fixed_route(self):
                return dict(wait1=0.4, wait2=0.0, t0=self.t0)

            def fixed_first(self):
                calls.append('fixed_first')

            def fixed_second(self):
                calls.append('fixed_second')
                stamp['second'] = time.monotonic()

            def plan(self):
                calls.append('plan')
                return 0, [{'stop': 'QR', 'cmds': [('R', -90), ('F', 300)]}], 'ROUTE OK'

        class Link(RecLink):
            def get_params(self):
                calls.append('GET')
                return {'PSPD': 60.0, 'TCV': 150.0}, 'DONE'

            def sync_params(self, params, log=None):
                calls.append('SYNC')
                return []

            def second_move(self, rel=None, log=None):
                stamp['move'] = time.monotonic()
                return RecLink.second_move(self, rel, log)
        ctx = FixedCtx(calls)
        ctx.t0 = time.monotonic()
        logs = []
        auto_run.run_mission(ctx, Link(calls), log=logs.append, stop_wait=0, settle_s=0, hooks=MissionHooksRec(calls),
                             cfg={'stm32_params': {'TCV': 120.0}, '_presynced': True, 'r_decimals': True})
        self.assertNotIn('GET', calls, '按了 START 不再读参数')
        self.assertNotIn('SYNC', calls, '按了 START 不再发参数')
        self.assertEqual(calls[:7], ['start_clock', 'ping', 'prepare', 'home', 'fixed_first', 'second_move', 'fixed_second'])
        self.assertGreaterEqual(stamp['move'] - ctx.t0, 0.4 - 0.01, '先原地等 wait1')
        self.assertLess(stamp['second'] - stamp['move'], 0.2, '第二站动作以后不再等')
        text = '\n'.join(logs)
        self.assertIn('STM32 参数按 START 之前已经同步好了', text)
        self.assertNotIn('③', text)
        # 没有同步好(_presynced 没有)：和以前一样按了 START 再同步
        calls.clear()
        ctx.t0 = time.monotonic()
        auto_run.run_mission(ctx, Link(calls), log=lambda m: None, stop_wait=0, settle_s=0, hooks=MissionHooksRec(calls),
                             cfg={'stm32_params': {'TCV': 120.0}})
        self.assertIn('GET', calls)
        self.assertIn('SYNC', calls)

    def test_prepare_error_does_not_stop(self):
        calls = []

        class H(MissionHooksRec):
            def prepare(self, link, log):
                raise RuntimeError('摄像头打不开')
        logs = []
        auto_run.run_mission(MissionCtx(calls), RecLink(calls), log=logs.append, stop_wait=0, settle_s=0, hooks=H(calls), cfg={})
        self.assertTrue(any('机械臂准备出错' in l for l in logs))
        self.assertIn('task QR', calls)


if __name__ == '__main__':
    unittest.main()
