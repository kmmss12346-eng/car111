"""mission_hooks 给导航的接口(prepare / start_clock / go_home_now / set_home_eta / keepalive / park)、
QR 读不到时前后挪着找、arm_link 的 PARK / ZONE：python3 -m unittest test_mission_api

都用 sim_mission 的假 STM32 + 假摄像头(电脑上就能跑，不要车上才有的模块)。"""
import math
import unittest

from sim_mission import SimWorld, make, run_mission, FakeCtx, QR_STOPS
from mission_hooks import Abort
from arm_link import ArmLink, unknown_cmd

CODE = '156+123+516+231'
RAW = {'stops': QR_STOPS, 'car_length_mm': 290, 'car_width_mm': 260, 'lidar_overhang_mm': 40}


def hooks(w, cfg=None, raw=RAW):
    lines = []
    h = make(w, cfg, log=lines.append, raw=raw)
    return h, lines


def f_moves(w, zone=None):
    """QR 点上底盘前后挪的指令(毫米)。"""
    return [v for c, v, z in w.moves if c == 'F' and z == zone]


class PrepareTests(unittest.TestCase):
    def test_prepare_inits_arm_clears_old_code_and_screen(self):
        """出发后马上 prepare：机械臂初始化 + 收臂、清掉 STM32 里存着的旧码、屏上任务码写 ---；再调一次什么也不重做。"""
        w = SimWorld(seed=1, code=CODE, qr_stale='111+222+333+123')
        h, lines = hooks(w)
        self.assertTrue(h.prepare(w.link, lines.append))
        self.assertIn('STOW', w.requests)
        self.assertIn('QR CLR', w.requests)
        self.assertIsNone(w.qr_code)                                 # 旧码清掉了
        self.assertEqual((w.screen['t0'], w.screen['t7']), ('---', '---'))
        n = len(w.requests)
        self.assertTrue(h.prepare(w.link, lines.append))
        self.assertEqual(len(w.requests), n)                         # 做过的不再做
        w.arrive('QR')
        h.task('QR', w.link, lines.append)                           # 到 QR 点读到的是现在的码，不是旧码
        self.assertEqual(h.plan.code, CODE)
        self.assertEqual(w.requests.count('STOW'), 1)                # task 不再初始化一遍

    def test_without_prepare_the_stale_code_is_used(self):
        """(对照)导航不调 prepare：STM32 里的旧码会被当成任务码——所以出发前一定要清。"""
        w = SimWorld(seed=1, code=CODE, qr_stale='111+222+333+123')
        h, lines = hooks(w)
        w.arrive('QR')
        h.task('QR', w.link, lines.append)
        self.assertEqual(h.plan.code, '111+222+333+123')

    def test_prepare_never_raises(self):
        """prepare 出错(GET 超时、急停)只记日志、返回 False，不抛异常；到 QR 点 task 再试。"""
        for kw in (dict(fail_cmd=('QR', 1)), dict(abort_at=('STOW', 1)), dict(armok=False)):
            w = SimWorld(seed=2, code=CODE, **kw)
            h, lines = hooks(w)
            try:
                ok = h.prepare(w.link, lines.append)
            except Exception as ex:                                  # noqa: BLE001
                self.fail(f'{kw}: prepare 抛了 {ex!r}')
            if 'armok' in kw:
                self.assertFalse(ok)
                self.assertEqual(w.screen.get('t4'), 'ARM OFF')
        w = SimWorld(seed=2, code=CODE)
        h, lines = hooks(w)
        h.vision.open = lambda: (_ for _ in ()).throw(RuntimeError('没有 /dev/video0'))
        self.assertTrue(h.prepare(w.link, lines.append))             # 摄像头打不开只提示
        self.assertIn('打开摄像头出错', '\n'.join(lines))

    def test_lift_boot_mode_is_reported(self):
        """新程序 LIFT? 带 BOOT=：开机没用编码器认高度(NOCAL/NOENC)时提醒一定要停 60 再关电。"""
        w = SimWorld(seed=3, code=CODE, lift_boot='NOENC')
        h, lines = hooks(w)
        h.prepare(w.link, lines.append)
        text = '\n'.join(lines)
        self.assertIn('读不到编码器', text)
        self.assertIn('一定要停在 60mm', text)
        self.assertEqual(h.arm.lift_boot, 'NOENC')


class ClockTests(unittest.TestCase):
    def test_clock_starts_at_start_clock(self):
        """计时从 start_clock(按 START)开始，不从创建钩子(扫描、规划、等人摆车)开始。"""
        w = SimWorld(seed=1, code=CODE)
        h, _ = hooks(w, dict(time_limit_s=150.0))
        w.advance(100.0)                                             # 摆车、扫描、预规划
        h.start_clock()
        self.assertAlmostEqual(h.elapsed(), 0.0, places=6)
        self.assertAlmostEqual(h.time_left(), 150.0, places=6)
        w.advance(30.0)
        self.assertAlmostEqual(h.time_left(), 120.0, places=6)

    def test_go_home_now(self):
        """去下一个停车点做一个物料再回家来不来得及；到 START 永远 False；时间不够过就一直 True。"""
        w = SimWorld(seed=1, code=CODE)
        h, lines = hooks(w, dict(time_limit_s=170.0))
        h.prepare(w.link, lines.append)
        h.start_clock()
        w.arrive('QR')
        h.task('QR', w.link, lines.append)
        self.assertFalse(h.go_home_now('RAW', 10.0, 30.0))           # 刚出发：来得及
        self.assertFalse(h.go_home_now('START2', 10.0, 0.0))
        w.advance(120.0)                                             # 已用 ~130 秒：抓一个约 14 秒 + 回家 30 + 余量 12 > 180
        self.assertTrue(h.go_home_now('RAW', 10.0, 30.0))
        self.assertTrue(h.out_of_time)
        self.assertTrue(h.go_home_now('ROUGH', 1.0, 1.0))            # 之后一律回家
        self.assertFalse(h.go_home_now('START', 30.0, 0.0))
        self.assertIn('时间不够去 RAW', '\n'.join(lines))

    def test_go_home_now_uses_learned_item_times(self):
        """每个物料的用时按实测的算：实测很慢时同样的剩余时间也不去了。"""
        w = SimWorld(seed=1, code=CODE)
        h, lines = hooks(w)
        h.prepare(w.link, lines.append)
        h.start_clock()
        w.arrive('QR')
        h.task('QR', w.link, lines.append)
        w.advance(100.0)
        self.assertFalse(h.go_home_now('RAW', 10.0, 30.0))           # 默认一个约 12 秒：100+10+2+12+30+12 < 180
        h.work_t['grab'] = 30.0                                      # 实测一个要 30 秒
        self.assertTrue(h.go_home_now('RAW', 10.0, 30.0))

    def test_no_work_left_goes_home(self):
        """到过 QR 点还是没有任务码 / 机械臂不能用：后面没活可干，直接回家(home_when_idle)；关掉就照走。"""
        w = SimWorld(seed=1, code=CODE, qr_present=False)
        h, lines = hooks(w, dict(qr_search=False, qr_timeout_s=0.5))
        h.start_clock()
        w.arrive('QR')
        h.task('QR', w.link, lines.append)
        self.assertIsNone(h.plan)
        self.assertTrue(h.go_home_now('RAW', 10.0, 30.0))
        self.assertIn('没有读到任务码', '\n'.join(lines))
        h.cfg['home_when_idle'] = False
        h.out_of_time = False
        self.assertFalse(h.go_home_now('RAW', 10.0, 30.0))

    def test_home_eta_stops_items_in_time_without_raising(self):
        """按回家时间判断：每个物料开始前算"这个物料 + 收臂 + 回家 + 余量"，来不及就不开始，记下 out_of_time，不抛异常。"""
        w = SimWorld(seed=4, code=CODE)
        h, lines = hooks(w)
        nav = dict(leg_s=10.0, home_s={'QR': 25, 'RAW': 30, 'ROUGH': 22, 'TEMP': 25})
        done = run_mission(w, h, log=lines.append, nav=nav)
        text = '\n'.join(lines)
        self.assertTrue(h.out_of_time, text)
        self.assertEqual(done[-1], 'START1')
        self.assertLess(len(done), 8, done)                          # 第二批做不完：中途直接回家
        self.assertLessEqual(w.t, 180.0, text)                       # 按估计的路上时间，3 分钟内回到启停区
        self.assertEqual(w.screen['t1'], 'DONE')
        self.assertEqual((w.air, w.loose, w.collisions), (0, 0, 0))

    def test_rough_without_time_for_temp_leaves_materials(self):
        """粗加工区放完，取回了也来不及去暂存区放：不取回(放下时已经算分)，直接回家。"""
        w = SimWorld(seed=1, code=CODE, params=dict(LFRPM=400.0, ASPD=270.0, ASPDF=120.0))
        h, lines = hooks(w)
        h.prepare(w.link, lines.append)
        h.start_clock()
        for role in ('QR', 'RAW', 'ROUGH'):
            if role == 'ROUGH':
                w.advance(95.0 - h.elapsed())                        # 到粗加工区时已经用了 95 秒
                h.set_home_eta(25.0)
            w.arrive(role)
            h.task(role, w.link, lines.append)
        text = '\n'.join(lines)
        self.assertIn('留在粗加工区', text)
        self.assertEqual(sum(1 for r in w.requests if r.startswith('PICK')), 0, text)
        self.assertEqual(sum(len(v) for (z, _k), v in w.rings.items() if z == 'ROUGH'), 3, text)
        self.assertTrue(h.out_of_time)
        self.assertTrue(h.go_home_now('TEMP', 12.0, 25.0))


class KeepaliveParkTests(unittest.TestCase):
    def test_keepalive_moves_id1_and_back_rate_limited(self):
        w = SimWorld(seed=1, code=CODE)
        h, lines = hooks(w)
        self.assertFalse(h.keepalive())                              # 机械臂还没初始化：什么也不做
        h.prepare(w.link, lines.append)
        a1 = w.a1
        n = len(w.requests)
        t = w.t
        self.assertTrue(h.keepalive())
        self.assertEqual(w.requests[n:], ['AD 1 3', 'AD 1 -3'])
        self.assertAlmostEqual(w.a1, a1, places=6)                   # 摆回来了
        self.assertLess(w.t - t, 1.0)                                # 不到 1 秒
        self.assertFalse(h.keepalive())                              # 5 秒以内不再摆
        w.advance(5.1)
        self.assertTrue(h.keepalive())
        h.disabled = '测试'
        w.advance(6.0)
        self.assertFalse(h.keepalive())

    def test_keepalive_never_raises(self):
        w = SimWorld(seed=1, code=CODE, fail_cmd=('AD', 1))
        h, lines = hooks(w)
        h.prepare(w.link, lines.append)
        self.assertFalse(h.keepalive())
        w2 = SimWorld(seed=1, code=CODE, abort_at=('AD', 1))
        h2, lines2 = hooks(w2)
        h2.prepare(w2.link, lines2.append)
        self.assertFalse(h2.keepalive())
        self.assertTrue(h2._aborted)

    def test_park_uses_park_command(self):
        w = SimWorld(seed=1, code=CODE)
        h, lines = hooks(w)
        h.prepare(w.link, lines.append)
        self.assertTrue(h.park())
        self.assertEqual(w.parked, 1)
        self.assertAlmostEqual(w.lift_mm, 60.0)

    def test_park_falls_back_on_old_firmware(self):
        """旧程序不认识 PARK(ERR CMD)：STOW + LIFT 60；没标定(ARMOK=0)只降升降。"""
        w = SimWorld(seed=1, code=CODE, park_cmd=False)
        h, lines = hooks(w)
        h.prepare(w.link, lines.append)
        w.lift_mm = 100.0
        self.assertTrue(h.park())
        self.assertEqual(w.requests[-3:], ['PARK', 'STOW', 'LIFT 60.0'])
        self.assertAlmostEqual(w.lift_mm, 60.0)
        w2 = SimWorld(seed=1, code=CODE, park_cmd=False, armok=False)
        h2, lines2 = hooks(w2)
        w2.lift_mm = 100.0
        self.assertTrue(h2.park(link=w2.link))                       # 机械臂没初始化过：用给的 link 连
        self.assertNotIn('STOW', w2.requests)
        self.assertAlmostEqual(w2.lift_mm, 60.0)

    def test_park_refuses_when_holding_or_after_abort_or_nozero(self):
        w = SimWorld(seed=1, code=CODE)
        h, lines = hooks(w)
        h.prepare(w.link, lines.append)
        h.maybe_holding = '红色物料可能还夹在爪子里'
        self.assertFalse(h.park())
        h.maybe_holding = None
        h._aborted = True
        self.assertFalse(h.park())                                   # 急停以后不自动动
        self.assertEqual(w.parked, 0)
        self.assertTrue(h.park(force=True))                          # 终端 park：确认安全了
        w.lift_known = False
        self.assertFalse(h.park(force=True))                         # 升降位置不知道：PARK 回 ERR NOZERO
        self.assertIn('NOZERO', '\n'.join(lines))

    def test_end_of_run_parks_and_shows_final_counts(self):
        w = SimWorld(seed=1, code=CODE)
        h, lines = hooks(w)
        run_mission(w, h, stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START2'), log=lines.append)
        self.assertEqual(w.parked, 1)
        self.assertEqual((w.screen['t2'], w.screen['t3']), ('GRAB 3', 'PLACE 6'))
        self.assertEqual((w.screen['t0'], w.screen['t7']), ('156+123+', '516+231'))

    def test_no_park_after_abort(self):
        w = SimWorld(seed=6, code=CODE, abort_at=('GRAB', 2))
        h, lines = hooks(w)
        with self.assertRaises(Abort):
            run_mission(w, h, log=lines.append)
        self.assertTrue(h._aborted)
        self.assertFalse(h.park())
        self.assertEqual(w.parked, 0)


class ErrorTests(unittest.TestCase):
    def test_unexpected_error_does_not_end_the_round(self):
        """停车点里出了意外的错(程序 bug)：收臂、挪回停车点，路线照走，后面的停车点照常做。"""
        w = SimWorld(seed=5, code=CODE)
        h, lines = hooks(w)
        real = h._survey
        st = {'n': 0}

        def broken(zone):
            st['n'] += 1
            if st['n'] == 1:
                real(zone)
                h.act.chassis_move(0, 37)                            # 挪过底盘才出错
                raise ZeroDivisionError('测试')
            return real(zone)
        h._survey = broken
        run_mission(w, h, stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1'), log=lines.append)
        text = '\n'.join(lines)
        self.assertIn('出了意外的错', text)
        self.assertEqual(w.screen['t1'], 'DONE')
        self.assertEqual(rings_placed(w, 'ROUGH'), 0)                # 粗加工区没做
        self.assertEqual(len([p for p in w.placed if p[0] == 'TEMP']), 3)   # 物料还在车上：暂存区照常放
        self.assertIn('底盘挪回停车点：前进 -37mm', text)             # 出错前挪过的底盘挪回去了
        self.assertEqual((w.air, w.loose, w.collisions), (0, 0, 0))

    def test_stow_glitch_is_retried(self):
        """收臂偶尔一次出错：再收一次，不结束这一轮。"""
        w = SimWorld(seed=2, code=CODE, fail_cmd=('STOW', 3))
        h, lines = hooks(w)
        run_mission(w, h, stops=('QR', 'RAW', 'ROUGH', 'TEMP', 'START1'), log=lines.append)
        self.assertIn('再收一次', '\n'.join(lines))
        self.assertEqual(w.screen['t1'], 'DONE')


def rings_placed(w, zone):
    return sum(len(v) for (z, _k), v in w.rings.items() if z == zone)


class QrSearchTests(unittest.TestCase):
    """扫码器在车右后方(离车尾 20、离右边 40：车中心后 125、右 90)。QR 停车点 (2100,1200) 车头朝北：扫码器在 y=1075。"""

    def run_qr(self, window, cfg=None, obstacles=None, latch=False, raw=RAW, abort=None):
        w = SimWorld(seed=3, code=CODE, qr_window=window, qr_latch_moving=latch)
        h, lines = hooks(w, cfg, raw=raw)
        h.ctx = FakeCtx(obstacles, abort=abort)
        h.prepare(w.link, lines.append)
        w.arrive('QR')
        h.task('QR', w.link, lines.append)
        return w, h, '\n'.join(lines)

    def test_offsets_follow_the_scanner_geometry(self):
        w = SimWorld(seed=3, code=CODE)
        h, _ = hooks(w)
        got = h._qr_offsets((2100.0, 1200.0, 90.0))
        self.assertEqual(got, [(1200.0, 125.0), (1260.0, 185.0), (1140.0, 65.0), (1320.0, 245.0)])   # 1080 和停车点差不多，不再挪
        got = h._qr_offsets((2100.0, 1200.0, -90.0))                 # 车头朝南(只看几何)：扫码器在 y=1325，往前开 y 变小；1320 和停车点差不多
        self.assertEqual([f for _t, f in got], [125.0, 65.0, 185.0, 245.0])
        self.assertEqual(h._qr_offsets((2100.0, 1200.0, 0.0)), [])   # 车头朝东(对着码板)：前后挪对不上，不挪

    def test_found_on_arrival_does_not_move(self):
        w, h, text = self.run_qr(None)
        self.assertEqual(h.plan.code, CODE)
        self.assertEqual(f_moves(w), [])

    def test_search_moves_until_read_and_returns(self):
        """码板在 1260 附近(扫码器挪 185mm 才看得到)：前进 125、再 60 读到，最后退回停车点(合起来 0)。"""
        w, h, text = self.run_qr((160.0, 210.0))
        self.assertEqual(h.plan.code, CODE, text)
        mv = f_moves(w)
        self.assertEqual(mv[:2], [125, 60], text)
        self.assertEqual(mv[-1], -185, text)
        self.assertLess(abs(h.act.disp['F']), 1e-6)
        self.assertIn('读到了(扫码器对着 1260 左右)', text)
        self.assertEqual(w.screen['t0'], '156+123+')

    def test_search_skips_positions_near_obstacles(self):
        """车道上前面有障碍物(车头离它不到 60mm)：那几个位置跳过，不往那边挪。"""
        w, h, text = self.run_qr((-140.0, -100.0), obstacles=[(2110.0, 1560.0, 25.0)])
        self.assertIsNone(h.plan)
        self.assertIn('离障碍物(2110,1560)', text)
        pos, far = 0.0, 0.0
        for v in f_moves(w):
            pos += v
            far = max(far, pos)
        self.assertLessEqual(1200 + 145 + far, 1560 - 25 - 60 + 1, f_moves(w))   # 车头一直离障碍物 60mm 以上
        self.assertLess(abs(h.act.disp['F']), 1e-6)

    def test_search_never_leaves_the_lane(self):
        """停车点挨着黄色区/场地边(假想的停车点)：会碰到的位置都跳过，一步也不挪。"""
        stops = dict(QR_STOPS, QR=[1945, 1200, 90])                  # 车左边(带雷达)离黄色区 x=1850 只有 ~0mm
        w, h, text = self.run_qr((160.0, 210.0), raw=dict(RAW, stops=stops))
        self.assertEqual(f_moves(w), [], text)
        self.assertIn('黄色区', text)
        stops = dict(QR_STOPS, QR=[2100, 2160, 90])                  # 往北挪会出场地
        w, h, text = self.run_qr((160.0, 210.0), raw=dict(RAW, stops=stops))
        self.assertTrue(all(v < 0 for v in f_moves(w)[:-1]) or not f_moves(w), (f_moves(w), text))

    def test_abort_during_search_does_not_move_back(self):
        """找码时急停：抛 Abort，不再挪回停车点(急停以后不动)。"""
        st = {'n': 0}

        def abort():
            st['n'] += 1
            return st['n'] > 6
        with self.assertRaises(Abort):
            self.run_qr((1000.0, 1001.0), abort=abort)

    def test_abort_flag_blocks_return_move(self):
        w = SimWorld(seed=3, code=CODE, qr_window=(1000.0, 1001.0))
        h, lines = hooks(w)
        st = {'n': 0}

        def abort():
            st['n'] += 1
            return st['n'] > 6
        h.ctx = FakeCtx(abort=abort)
        w.arrive('QR')
        with self.assertRaises(Abort):
            h.task('QR', w.link, lines.append)
        self.assertNotEqual(h.act.disp['F'], 0.0)                    # 没挪回去
        self.assertTrue(h._aborted)

    def test_not_found_tries_again_at_the_next_stops(self):
        """到 QR 点找了一圈也没读到：后面的停车点再问一次 QR?(新程序在路上读到也存着)，读到了照常做。"""
        w = SimWorld(seed=3, code=CODE, qr_window=(1000.0, 1001.0))
        h, lines = hooks(w)
        w.arrive('QR')
        h.task('QR', w.link, lines.append)
        self.assertIsNone(h.plan)
        w.qr_code = CODE                                             # STM32 后来读到了
        w.arrive('RAW', 1)
        h.task('RAW', w.link, lines.append)
        self.assertEqual(h.plan.code, CODE)
        self.assertEqual(h.stats.grab_total, 3)
        self.assertIn('路上读到的', '\n'.join(lines))

    def test_latching_firmware_reads_while_moving(self):
        """新程序车在走的时候读到的码也存下来：窗口很窄(停下的位置都看不到)，路过时读到就够了。"""
        w, h, text = self.run_qr((150.0, 152.0), latch=True)
        self.assertEqual(h.plan.code, CODE, text)
        w2, h2, text2 = self.run_qr((150.0, 152.0), latch=False)
        self.assertIsNone(h2.plan, text2)

    def test_no_stop_pose_falls_back_to_waiting(self):
        w, h, text = self.run_qr((160.0, 210.0), raw={})
        self.assertEqual(f_moves(w), [])
        self.assertIn('没有 QR 停车点的位置', text)

    def test_mtest_qr_holds_heading_and_resets_disp(self):
        import mission_cli
        import time
        w = SimWorld(seed=3, code=CODE, qr_window=(160.0, 210.0), qr_stale='111+222+333+123')
        h, lines = hooks(w)
        mission_cli._S['hooks'] = h
        state = {}
        h._ensure(w.link)
        h.act.disp = {'S': 0.0, 'F': 80.0}                           # 上一个测试剩下的位移：不能按它挪回去
        mission_cli.handle_cli('mtest', ['mtest', 'QR'], link=w.link, raw_cfg=RAW, state=state, log=lines.append)
        t0 = time.time()
        while state.get('busy') and time.time() - t0 < 10:
            time.sleep(0.01)
        text = '\n'.join(lines)
        self.assertEqual(w.homed, 1, text)
        self.assertIn('QR CLR', w.requests)
        self.assertEqual(h.plan.code, CODE, text)                    # 旧码清掉了，读到的是现在的
        self.assertEqual(sum(f_moves(w)), 0, text)
        mission_cli._S['hooks'] = None


class ArmLinkTests(unittest.TestCase):
    class Link:
        def __init__(self, replies):
            self.replies, self.sent = replies, []

        def request(self, text, timeout, collect=False):
            self.sent.append((text, timeout))
            out = self.replies.get(text.split(' ', 1)[0], (False, 'ERR CMD', []))
            return out if collect else out[:2]

    def test_park_and_zone_on_new_firmware(self):
        link = self.Link({'PARK': (True, 'DONE', []), 'ZONE?': (True, 'DONE', ['ZONE 2 1']), 'ZONE': (True, 'DONE', []),
                          'LIFT?': (True, 'DONE', ['LIFT 1 60.0 BOOT=ENC'])})
        a = ArmLink(link, log=lambda m: None)
        self.assertEqual(a.park(), (True, 'DONE'))
        self.assertEqual(dict(link.sent)['PARK'], 40.0)
        self.assertEqual(a.zone(), (2, 1))
        self.assertTrue(a.zone_cmd('lock'))
        self.assertTrue(a.zone_cmd('MSG scan 中 "ok"'))
        self.assertIn(('ZONE LOCK', 3.0), link.sent)
        self.assertIn("ZONE MSG scan ? 'ok'", [t for t, _ in link.sent])
        self.assertEqual(a.lift_state(), (True, 60.0))
        self.assertEqual(a.lift_boot, 'ENC')

    def test_old_firmware(self):
        link = self.Link({'LIFT?': (True, 'DONE', ['LIFT 1 60.0'])})
        a = ArmLink(link, log=lambda m: None)
        self.assertEqual(a.park(), (None, 'ERR CMD'))
        self.assertIsNone(a.zone())
        self.assertFalse(a.zone_cmd('ASK'))
        self.assertEqual(a.lift_state(), (True, 60.0))
        self.assertIsNone(a.lift_boot)
        self.assertTrue(unknown_cmd('ERR CMD'))
        self.assertFalse(unknown_cmd('ERR NOZERO'))
        link2 = self.Link({'PARK': (False, 'ERR NOZERO', [])})
        self.assertEqual(ArmLink(link2, log=lambda m: None).park(), (False, 'ERR NOZERO'))


if __name__ == '__main__':
    unittest.main(verbosity=1)
