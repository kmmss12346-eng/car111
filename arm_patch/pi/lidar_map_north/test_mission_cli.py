"""mission_cli.py / apply_mission_config.py 的冒烟测试(用 sim_mission 的假 STM32)：python3 test_mission_cli.py"""
import json
import os
import tempfile
import time
import unittest

import mission_cli
import apply_mission_config
from sim_mission import SimWorld, make, rings_summary


def read(path):
    with open(path, encoding='utf-8') as f:
        return f.read()


def load(path):
    return json.loads(read(path))


def wait_idle(state, timeout=10.0):
    t0 = time.time()
    while state.get('busy') and time.time() - t0 < timeout:
        time.sleep(0.01)
    assert not state.get('busy'), 'worker 没结束'


class CliTests(unittest.TestCase):
    def setUp(self):
        self.w = SimWorld(seed=21, code='156+123+516+231')
        self.h = make(self.w)
        mission_cli._S['hooks'] = self.h
        self.lines = []
        self.log = self.lines.append
        self.state = {}

    def run_cli(self, k, text):
        parts = text.split()
        mission_cli.handle_cli(k, parts, link=self.w.link, raw_cfg={}, state=self.state, log=self.log)
        wait_idle(self.state)

    def test_arm_raw_command(self):
        self.run_cli('arm', 'arm LIFT ZERO')
        self.assertTrue(any('DONE' in l for l in self.lines))
        self.assertTrue(self.w.lift_known)
        self.lines.clear()
        self.run_cli('arm', 'arm A? 1')
        self.assertTrue(any(l.strip().startswith('ANG 1') for l in self.lines))

    def test_arm_needs_arguments(self):
        with self.assertRaises(ValueError):
            mission_cli.handle_cli('arm', ['arm'], link=self.w.link, raw_cfg={}, state={}, log=self.log)

    def test_no_link(self):
        with self.assertRaises(ValueError):
            mission_cli.handle_cli('arm', ['arm', 'GET'], link=None, raw_cfg={}, state={}, log=self.log)

    def test_qr_and_mcode(self):
        self.run_cli('qr', 'qr')
        self.assertTrue(any('156+123+516+231' in l for l in self.lines))
        self.run_cli('mcode', 'mcode 156+123+516+231')
        self.assertIsNotNone(self.h.plan)
        with self.assertRaises(ValueError):
            self.run_cli('mcode', 'mcode 999')

    def test_mtest_raw_then_rough_then_temp(self):
        self.run_cli('mcode', 'mcode 156+123+516+231')
        self.w.arrive('RAW', 1)
        self.run_cli('mtest', 'mtest RAW 1')
        self.assertEqual((self.h.stats.grab_ok, self.h.stats.grab_total), (3, 3))
        self.w.arrive('ROUGH', 1)
        self.run_cli('mtest', 'mtest ROUGH 1')
        self.assertEqual(self.h.stats.place_ok, 3)
        self.assertEqual(self.w.air + self.w.collisions, 0)
        self.assertEqual(set(self.w.tray.values()), {1, 5, 6})
        self.w.arrive('TEMP', 1)
        self.run_cli('mtest', 'mtest TEMP 1')
        self.assertEqual(rings_summary(self.w, 'TEMP'), {1: [1], 2: [5], 3: [6]})

    # ---------------- 原料区：车是手放的，对准时车轮不能动(不能压进原料区)
    def _raw_setup(self):
        self.run_cli('arm', 'arm LIFT ZERO')
        self.run_cli('mcode', 'mcode 156+123+516+231')
        self.w.arrive('RAW', 1)
        moves = []
        orig = self.w.link.move
        self.w.link.move = lambda cmd, val, speed=None: (moves.append((cmd, val)), orig(cmd, val, speed))[1]
        self.w.requests.clear()
        self.lines.clear()
        return moves

    def test_mtest_raw_keeps_wheels_still(self):
        """默认对准只动手臂；上一个工位剩下的底盘位移不会让车开回去；先记下现在的车头方向。"""
        moves = self._raw_setup()
        self.h._ensure(self.w.link)
        self.h.act.disp = {'S': 120.0, 'F': -150.0}            # 上一个工位没挪回去
        self.run_cli('mtest', 'mtest RAW 1')
        text = '\n'.join(self.lines)
        self.assertEqual(moves, [], text)
        self.assertEqual(self.w.homed, 1, text)
        self.assertIn('车轮不动', text)
        self.assertEqual((self.h.stats.grab_ok, self.h.stats.grab_total), (3, 3), text)
        self.assertEqual(sorted(c for c in self.w.tray.values() if c), [1, 5, 6])

    def test_mtest_raw_out_of_reach_is_skipped_not_chased(self):
        """物料停在手臂够不着的地方：跳过，不开车去追。"""
        moves = self._raw_setup()
        self.w.raw_items[0]['pos'] = self.w.raw_items[0]['pos'] * 0 + (0.0, 60.0)
        self.run_cli('mtest', 'mtest RAW 1')
        text = '\n'.join(self.lines)
        self.assertEqual(moves, [], text)
        self.assertIn('手臂够不着的地方就跳过', text)
        self.assertEqual(self.w.air + self.w.collisions, 0, text)

    def test_raw_chassis_f_only_moves_forward_back(self):
        moves = self._raw_setup()
        self.h.cfg['raw_chassis'] = 'F'
        self.w.raw_items[0]['pos'] = self.w.raw_items[0]['pos'] * 0 + (0.0, 45.0)
        self.run_cli('mtest', 'mtest RAW 1')
        text = '\n'.join(self.lines)
        self.assertTrue(moves, text)
        self.assertEqual({c for c, v in moves}, {'F'}, moves)
        pos, far = 0.0, 0.0
        for c, v in moves:                                     # 离停车点最远多少(探测 + 对准)
            pos += v
            far = max(far, abs(pos))
        self.assertLessEqual(far, 60.0, moves)

    def test_mtest_raw_needs_empty_tray(self):
        """转盘里记着有物料(粗加工区测试取回的)：不加 force 不夹，加了 force 当作已经拿空。"""
        self._raw_setup()
        for it in self.h.plan.items(1):
            self.h.in_tray[it.slot] = it
        self.run_cli('mtest', 'mtest RAW 1')
        text = '\n'.join(self.lines)
        self.assertIn('先用手把车上转盘拿空', text)
        self.assertFalse([r for r in self.w.requests if r.startswith(('GRAB', 'OBS'))], self.w.requests)
        self.lines.clear()
        self.run_cli('mtest', 'mtest RAW 1 force')
        text = '\n'.join(self.lines)
        self.assertEqual((self.h.stats.grab_ok, self.h.stats.grab_total), (3, 3), text)

    # ---------------- 原料盘转一会儿停一会儿(大约转 3 秒停 6 秒)
    def _plate_world(self, stop_s=6.0, move_s=3.0, seed=5, seed_j=True, **kw):
        """换一个原料盘会转的模拟世界；seed_j = 已经做过 vcal RAW(手臂和画面的对应关系存好了)。"""
        import numpy as np
        from sim_mission import make
        self.w = SimWorld(seed=seed, code='156+123+516+231', plate_stop_s=stop_s, plate_move_s=move_s, raw_scale=1.97,
                          params=dict(ZOBRAW=100.0, ZGRAB=80.0, LFRPM=150.0, LFMRG=100.0), **kw)
        self.h = make(self.w)
        mission_cli._S['hooks'] = self.h
        self.run_cli('arm', 'arm LIFT ZERO')
        self.run_cli('mcode', 'mcode 156+123+516+231')
        self.h._ensure(self.w.link)
        if seed_j:
            w = self.w
            J = -w.scale['RAW'] * w.Rcam @ np.diag([w.k2, w.k1])        # 真实的：手臂 ID2/ID1 转 1° 画面里物料移动多少像素
            self.h.store.put('RAW', 'arm', J)
        self.w.arrive('RAW', 1)
        self.w.requests.clear()
        self.lines.clear()

    def test_stop_go_plate_grabs_when_it_stops(self):
        self._plate_world()
        self.run_cli('mtest', 'mtest RAW 1')
        text = '\n'.join(self.lines)
        self.assertEqual((self.h.stats.grab_ok, self.h.stats.grab_total), (3, 3), text)
        self.assertEqual(self.w.air, 0, text)
        self.assertEqual(sorted(c for c in self.w.tray.values() if c), [1, 5, 6])
        self.assertIn('原料盘停了', text)
        self.assertIn('下爪', text)
        self.assertNotIn('探测', text)                              # 停下的几秒里不现测 J

    def test_stop_go_plate_shorter_stops_still_work(self):
        """停的时间比说的短(可能更快)：照样一停就夹，不会在转盘转起来以后才下爪。"""
        for stop in (3.0, 4.0):
            self._plate_world(stop_s=stop, move_s=2.0, seed=7)
            self.run_cli('mtest', 'mtest RAW 1')
            text = '\n'.join(self.lines)
            self.assertEqual(self.w.air, 0, f'停 {stop}s\n{text}')
            self.assertEqual((self.h.stats.grab_ok, self.h.stats.grab_total), (3, 3), f'停 {stop}s\n{text}')

    def test_stop_too_short_never_grabs_a_moving_material(self):
        """停的时间短到来不及对准：不夹(不会夹空/把物料碰倒)，到时间就跳过。"""
        self._plate_world(stop_s=1.0, move_s=2.0, seed=3)
        self.h.cfg['raw_track_s'] = 20.0
        self.run_cli('mtest', 'mtest RAW 1')
        text = '\n'.join(self.lines)
        self.assertEqual(self.w.air, 0, text)

    def test_stop_go_without_vcal_does_not_probe(self):
        """没做 vcal RAW：转盘一停一转时不现测(测出来是错的还会存下来)，提示先停转盘做 vcal。"""
        self._plate_world(seed_j=False)
        self.h.cfg['raw_track_s'] = 15.0
        self.run_cli('mtest', 'mtest RAW 1')
        text = '\n'.join(self.lines)
        self.assertIn('vcal RAW', text)
        self.assertIsNone(self.h.store.get('RAW', 'arm'))
        self.assertEqual(self.w.air, 0, text)
        self.assertFalse([r for r in self.w.requests if r.startswith('GRAB')], text)

    def test_time_up_grabs_with_current_error(self):
        """没修到容差以内、但停的时间到了：只要偏差不超过 raw_grab_max_mm 就按现在的位置下爪，不放过这一次。"""
        self._plate_world()
        self.h.cfg['raw_tol_mm'] = 0.01                                     # 永远修不到
        self.h.cfg['raw_max_iter'] = 1
        self.run_cli('mtest', 'mtest RAW 1')
        text = '\n'.join(self.lines)
        self.assertIn('按现在的位置夹', text)
        self.assertEqual((self.h.stats.grab_ok, self.h.stats.grab_total), (3, 3), text)
        self.assertEqual(self.w.air, 0, text)

    def test_any_order_grabs_whatever_stops_under_the_claw(self):
        """mtest RAW 1 any：哪个颜色先停在爪子附近就先夹哪个，放进它自己的槽；比按顺序等快。"""
        self._plate_world(seed=7)
        t0 = self.w.t
        self.run_cli('mtest', 'mtest RAW 1 any')
        text = '\n'.join(self.lines)
        t_any = self.w.t - t0
        self.assertEqual((self.h.stats.grab_ok, self.h.stats.grab_total), (3, 3), text)
        self.assertEqual(self.w.air, 0, text)
        self.assertEqual(self.w.tray, {1: 1, 2: 5, 3: 6}, text)            # 每个颜色在它自己的槽里
        self.assertIn('这次停在爪子附近的是', text)
        self.assertIn('不按顺序', text)
        self.assertFalse(self.h.raw_any_once)                              # 只这一次
        self._plate_world(seed=7)                                          # 同一个世界按顺序抓：要等更久
        t0 = self.w.t
        self.run_cli('mtest', 'mtest RAW 1')
        self.assertEqual(self.h.stats.grab_ok, 3, '\n'.join(self.lines))
        self.assertLess(t_any, self.w.t - t0)

    def test_stop_phase_from_measured_cycle(self):
        """量到过停多久、转多久：一开始就看到它停着时，按周期推算已经停了几秒(知道还剩多久)。"""
        self._plate_world(stop_s=4.8, move_s=2.7, seed=11)
        self.h.raw_stops, self.h.raw_moves = [4.8], [2.7]
        t_stop = self.w.plate_t0
        while self.w.plate_phase(t_stop + 0.01)[1]:              # 找一个"停下"的时刻当作量到过的
            t_stop += 0.01
        self.h.raw_cycle['t_stop'] = t_stop
        got = self.h._raw_predict_stop_start(t_stop + 7.5 * 3 + 2.0)
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got, t_stop + 7.5 * 3, delta=0.05)
        self.assertIsNone(self.h._raw_predict_stop_start(t_stop + 7.5 * 3 + 6.0))   # 这会儿应该在转

    def test_long_wait_keeps_the_arm_alive(self):
        """等物料转过来等得久：手臂隔一会儿轻轻摆一下(规则：机器人停止运行 15 秒/等转盘 23 秒本轮结束)，照样夹到。"""
        self._plate_world()
        self.h.cfg['raw_keepalive_s'] = 3.0
        self.run_cli('mtest', 'mtest RAW 1')
        text = '\n'.join(self.lines)
        self.assertIn('轻轻摆一下', text)
        self.assertEqual((self.h.stats.grab_ok, self.h.stats.grab_total), (3, 3), text)
        self.assertEqual(self.w.air, 0, text)

    def test_gtest_nogo_on_stop_go_plate(self):
        self._plate_world()
        self.run_cli('gtest', 'gtest 1 nogo')
        text = '\n'.join(self.lines)
        self.assertIn('nogo', text)
        self.assertIn('原料盘', text)
        self.assertNotIn('CLAW C', self.w.requests)
        self.assertFalse([r for r in self.w.requests if r.startswith('GRAB')])

    def test_mtest_force_assumes_tray(self):
        self.run_cli('mcode', 'mcode 156+123+516+231')
        self.w.arrive('ROUGH', 1)
        self.w.tray = {1: 1, 2: 5, 3: 6}                    # 世界里真的有，force 让程序也认为有
        self.run_cli('mtest', 'mtest ROUGH 1 force')
        self.assertEqual(self.h.stats.place_ok, 3)

    def test_mtest_reset(self):
        self.run_cli('mtest', 'mtest reset')
        self.assertIsNone(mission_cli._S['hooks'])

    def test_vcal_ring(self):
        self.w.arrive('ROUGH', 1)
        self.run_cli('vcal', 'vcal RING')
        text = '\n'.join(self.lines)
        self.assertIn('手臂：ID2 每转 1°', text)
        self.assertIn('底盘：横移 1mm', text)
        self.assertIsNotNone(self.h.store.get('RING', 'arm'))
        self.assertIsNotNone(self.h.store.get('RING', 'ch'))

    def test_vcal_arm_only_does_not_move_chassis(self):
        self.w.arrive('ROUGH', 1)
        moves0 = sum(1 for r in self.w.requests if r.startswith(('S ', 'F ')))
        self.run_cli('vcal', 'vcal RING arm')
        self.assertIsNone(self.h.store.get('RING', 'ch'))
        self.assertIsNotNone(self.h.store.get('RING', 'arm'))
        self.assertEqual(abs(self.w.off_s) + abs(self.w.off_f), 0.0)

    def test_vcal_raw_wheels_first_probes_forward_only_and_returns(self):
        # 原料区先动车轮：vcal RAW 也测车轮，但只前后挪(不横移，不压进原料区)，测完回到原处
        self.h.cfg['raw_wheels_first'] = True
        self.run_cli('arm', 'arm LIFT ZERO')
        self.w.arrive('RAW', 1)
        for it in self.w.raw_items:
            if it['color'] == 1:
                it['pos'] = self.w.claw().copy()
        self.run_cli('vcal', 'vcal RAW 1')
        self.assertIsNotNone(self.h.store.get('RAW', 'ch'), '\n'.join(self.lines))
        self.assertFalse(any(c == 'S' for c, _v, _z in self.w.moves))
        self.assertTrue(any(c == 'F' for c, _v, _z in self.w.moves))
        self.assertLess(abs(sum(v for c, v, _z in self.w.moves if c == 'F')), 1.5)     # 挪回原处
        self.assertEqual(self.w.off_s, 0.0)

    def test_vclaw_raw_measures_and_saves(self):
        with tempfile.TemporaryDirectory() as td:
            self.h.cfg['vision_cal_file'] = os.path.join(td, 'vision_cal.json')
            self.run_cli('arm', 'arm LIFT ZERO')
            self.w.arrive('RAW', 1)
            for it in self.w.raw_items:                          # 物料放在爪子正下方(用户就是这么做的)
                if it['color'] == 1:
                    it['pos'] = self.w.claw().copy()
            self.lines.clear()
            self.run_cli('vclaw', 'vclaw RAW 1')
            text = '\n'.join(self.lines)
            self.assertIn('claw_px.RAW', text, text)
            saved = load(self.h.cfg['vision_cal_file'])['claw_px']['RAW']
            self.assertEqual(len(saved), 2)
            self.assertLess(abs(saved[0] - 320.0) + abs(saved[1] - 240.0), 3.0, saved)   # 测到的是真的爪子点
            for got, want in zip(self.h.vision.claw('RAW'), saved):
                self.assertAlmostEqual(got, want, delta=0.01)

    def test_vclaw_ring_saves_ring_size(self):
        with tempfile.TemporaryDirectory() as td:
            self.h.cfg['vision_cal_file'] = os.path.join(td, 'vision_cal.json')
            self.run_cli('arm', 'arm LIFT ZERO')
            self.w.arrive('ROUGH', 1)
            self.lines.clear()
            self.run_cli('vclaw', 'vclaw RING')
            text = '\n'.join(self.lines)
            self.assertIn('claw_px.RING', text, text)
            self.assertIn('圆环最外圈半径', text, text)
            saved = load(self.h.cfg['vision_cal_file'])
            self.assertEqual(len(saved['claw_px']['RING']), 2)
            self.assertGreater(saved['ring_rmax_px'], 10)
            self.assertIn('ZOBRNG', saved)

    def test_vclaw_bad_args_and_needs_zero(self):
        for bad in ('vclaw', 'vclaw RAW', 'vclaw RAW 9', 'vclaw XYZ'):
            with self.assertRaises(ValueError):
                mission_cli.handle_cli('vclaw', bad.split(), link=self.w.link, raw_cfg={}, state={}, log=self.log)
        self.lines.clear()
        self.w.lift_known = False                            # 升降位置丢了：不动，提示
        self.run_cli('vclaw', 'vclaw RAW 1')
        self.assertTrue(any('LIFT ZERO' in l for l in self.lines), self.lines)

    def test_vmask_without_matdet(self):
        self.lines.clear()
        self.run_cli('vmask', 'vmask')
        self.assertTrue(any('不是 matdet' in l for l in self.lines), self.lines)

    def test_vcal_bad_args(self):
        with self.assertRaises(ValueError):
            self.run_cli('vcal', 'vcal FOO')
        with self.assertRaises(ValueError):
            self.run_cli('vcal', 'vcal RAW')

    def test_gtest_align_and_grab(self):
        self.run_cli('arm', 'arm LIFT ZERO')
        self.w.arrive('RAW', 1)                              # 车停在原料盘前
        self.w.a1_ref, self.w.a2_ref = self.w.params['A1G'], self.w.params['A2E']   # 模拟世界的几何基准 = 原料上方姿态
        self.lines.clear()
        color = self.w.raw_items[0]['color']
        self.run_cli('gtest', f'gtest {color}')
        text = '\n'.join(self.lines)
        self.assertIn('对准结果', text, text)
        sent = [r for r in self.w.requests]
        self.assertIn('OBS RAW O', sent, text)                  # 先升高再摆过去(OBS)，不在低处横扫
        self.assertIn('CLAW C', sent, text)
        self.assertTrue(any(r.startswith('AP ') for r in sent))

    def test_rtest_aligns_without_placing(self):
        self.run_cli('arm', 'arm LIFT ZERO')
        self.w.arrive('TEMP', 1)
        self.w.a1_ref, self.w.a2_ref = self.w.params['A1P'], self.w.params['A2P']
        self.w.requests.clear()
        self.lines.clear()
        self.run_cli('rtest', 'rtest arm')
        text = '\n'.join(self.lines)
        self.assertIn('对准结果', text, text)
        self.assertIn('OBS RING O', self.w.requests)
        self.assertFalse([r for r in self.w.requests if r.split()[0] in ('TAKE', 'DROP', 'GRAB', 'PICK')], self.w.requests)
        self.assertFalse([r for r in self.w.requests if r.split()[0] in ('S', 'F')], '只动手臂')

    def test_rtest_falls_back_to_any_size(self):
        """按配置的大小认不到、不限大小认得到：提示做 vclaw RING，这次先接着测。"""
        self.run_cli('arm', 'arm LIFT ZERO')
        self.w.arrive('TEMP', 1)
        self.w.a1_ref, self.w.a2_ref = self.w.params['A1P'], self.w.params['A2P']
        vis = self.h.vision
        true_err = vis.ring_error
        cu, cv_ = vis.claw('RING')

        def ring_px(n=None, expect=None, any_size=False):
            if not any_size:
                return None
            e = true_err(n)
            return None if e is None else (cu + e[0], cv_ + e[1])
        vis.ring_px = ring_px
        vis.ring_error = lambda n=None: None
        vis._ring_rmax_range = lambda: (84.0, 225.0)
        self.lines.clear()
        self.run_cli('rtest', 'rtest arm')
        text = '\n'.join(self.lines)
        self.assertIn('不在现在认的范围', text, text)
        self.assertIn('对准结果', text, text)

    def test_vclaw_ring_with_diameter(self):
        with tempfile.TemporaryDirectory() as td:
            self.h.cfg['vision_cal_file'] = os.path.join(td, 'vision_cal.json')
            self.run_cli('arm', 'arm LIFT ZERO')
            self.w.arrive('ROUGH', 1)
            self.run_cli('vclaw', 'vclaw RING 100')
            self.assertEqual(load(self.h.cfg['vision_cal_file'])['ring_outer_diam_mm'], 100.0)
        with self.assertRaises(ValueError):
            mission_cli.handle_cli('vclaw', ['vclaw', 'RING', 'abc'], link=self.w.link, raw_cfg={}, state={}, log=self.log)

    def test_vwatch_shows_live_and_steps_aside(self):
        """vwatch：后台一直识别、写实时画面；用摄像头的命令(vdbg)运行时先停、做完接着看；vwatch off 停。"""
        import numpy as np
        import cv2
        from vision import Vision
        img = np.full((480, 640, 3), 225, np.uint8)
        for d in (53, 58, 65, 75, 85, 95):
            cv2.circle(img, (330, 200), int(d / 2 * 2.0), (20, 20, 20), 3, cv2.LINE_AA)

        class Cam:
            def read(self):
                return img.copy()
        with tempfile.TemporaryDirectory() as td:
            live = os.path.join(td, 'live.jpg')
            self.h._ensure_arm_only(self.w.link)
            self.h.vision = Vision(dict(live_view=live, matdet={'claw_mask': '/nonexistent'}, px_per_mm=dict(RING=2.0)),
                                   camera=Cam(), log=lambda m: None, sleep=lambda s: None)
            self.lines.clear()
            self.run_cli('vwatch', 'vwatch RING')
            for _ in range(100):
                if any('[vwatch] 认到圆环' in l for l in self.lines):
                    break
                time.sleep(0.05)
            self.assertTrue(any('[vwatch] 认到圆环' in l for l in self.lines), self.lines)
            self.assertTrue(os.path.exists(live))
            self.run_cli('vdbg', 'vdbg RING')                              # 要用摄像头的命令：vwatch 先停，做完接着看
            for _ in range(60):
                if mission_cli._W['thread'] is not None:
                    break
                time.sleep(0.05)
            self.assertIsNotNone(mission_cli._W['thread'])
            self.run_cli('vwatch', 'vwatch off')
            self.assertIsNone(mission_cli._W['thread'])
            self.assertTrue(any('vwatch 已停' in l for l in self.lines))
        with self.assertRaises(ValueError):
            mission_cli.handle_cli('vwatch', ['vwatch', 'XYZ'], link=self.w.link, raw_cfg={}, state={}, log=self.log)

    # ---------------- 粗加工区 / 暂存区：车是手放到工位上的
    def _zone_setup(self, zone, code='156+123+516+231'):
        self.w.code = code
        self.run_cli('arm', 'arm LIFT ZERO')
        self.run_cli('mcode', f'mcode {code}')
        self.w.arrive(zone, 1)
        for it in self.h.plan.items(1):                        # 物料手放进车上转盘
            self.w.tray[it.slot] = it.color
        self.w.requests.clear()
        self.lines.clear()

    def test_mtest_rough_places_and_picks_back(self):
        self._zone_setup('ROUGH')
        self.run_cli('mtest', 'mtest ROUGH 1 force')
        text = '\n'.join(self.lines)
        self.assertIn('2 号环正对手臂', text, text)
        self.assertEqual(self.w.air, 0, text)
        self.assertEqual(self.w.collisions, 0, text)
        self.assertEqual(len([p for p in self.w.placed if p[0] == 'ROUGH']), 3, text)
        self.assertEqual(sorted(c for c in self.w.tray.values() if c), sorted(it.color for it in self.h.plan.items(1)))
        self.assertTrue(all(not st for (z, k), st in self.w.rings.items() if z == 'ROUGH'))   # 全都夹回来了
        self.assertIn('现在车上转盘', text)

    def _hide_covered_rings(self):
        """模拟没量过圆环大小：圆环白心上放着物料时认不出圆环。"""
        import numpy as np
        vis, w = self.h.vision, self.w
        orig = vis.ring_error

        def ring_error(n=None, max_px=None):
            c = w.claw()
            k = min((1, 2, 3), key=lambda k: np.linalg.norm(w.ring_center(k) - c))
            return None if w.rings.get((w.zone, k)) else orig(n, max_px=max_px)
        vis.ring_error = ring_error

    def test_pickback_aligns_on_the_material(self):
        """取回时白心被物料盖住、认不出圆环：第一个物料放下后自动量 PICK 点，取回时直接认物料对准，全都夹回来。"""
        self._zone_setup('ROUGH', '234+123+342+213')           # 黄、蓝、绿(没有黑色)
        self.h.cfg['learn_pick'] = True                       # 这个功能默认关了(要多拍一张)，这里测它
        self._hide_covered_rings()
        self.assertFalse(self.h.vision.has_pick())
        self.run_cli('mtest', 'mtest ROUGH 1 force')
        text = '\n'.join(self.lines)
        self.assertIn('claw_px.PICK = (', text, text)
        self.assertTrue(self.h.vision.has_pick())
        pk = self.h.vision.claw('PICK')
        self.assertLess(abs(pk[0] - self.h.vision.TRUE_PICK[0]) + abs(pk[1] - self.h.vision.TRUE_PICK[1]), 3.0, pk)
        self.assertEqual(text.count('对准结果(按物料)'), 3, text)
        self.assertEqual(text.count('claw_px.PICK = ('), 1, text)           # 只量一次
        self.assertEqual((self.w.air, self.w.collisions), (0, 0), text)
        self.assertTrue(all(not st for (z, k), st in self.w.rings.items() if z == 'ROUGH'), text)
        self.assertEqual(sorted(c for c in self.w.tray.values() if c), [2, 3, 4])

    def test_black_material_uses_the_ring_seen_when_placing(self):
        """黑色物料在黑环上认不出来(不按物料对)：按放物料时认到的圆环对准取回。"""
        self._zone_setup('ROUGH')                              # 红、黑、浅蓝
        self.h.cfg['learn_pick'] = True                       # 这个功能默认关了(要多拍一张)，这里测它
        self.run_cli('mtest', 'mtest ROUGH 1 force')
        text = '\n'.join(self.lines)
        self.assertEqual(text.count('对准结果(按物料)'), 2, text)
        self.assertEqual(text.count('对准结果(按圆环)'), 1, text)
        self.assertEqual((self.w.air, self.w.collisions), (0, 0), text)

    def test_pickback_blind_when_nothing_is_visible(self):
        """物料、圆环都认不到：按放下时记下的位置直接夹(不会卡住、不会报错停下)。"""
        self._zone_setup('ROUGH')
        self.h.cfg['learn_pick'] = True                       # 这个功能默认关了(要多拍一张)，这里测它
        self._hide_covered_rings()
        self.h.vision.hide_pick = (1, 2, 3, 4, 5, 6)
        self.run_cli('mtest', 'mtest ROUGH 1 force')
        text = '\n'.join(self.lines)
        self.assertIn('没量成', text, text)
        self.assertEqual(text.count('物料和圆环都看不到：按放下时记下的位置直接取'), 3, text)
        self.assertEqual(len([r for r in self.w.requests if r.startswith('PICK ')]), 3)

    def test_mtest_restarts_the_clock(self):
        """前面摆物料、搬车花了很久：mtest 重新计时，不会一开始就"时间到"。"""
        self._zone_setup('TEMP')
        self.h.cfg['time_limit_s'] = 100.0
        self.h.t0 = self.w.now() - 5000.0
        self.run_cli('mtest', 'mtest TEMP 1 force')
        text = '\n'.join(self.lines)
        self.assertNotIn('时间到', text)
        self.assertEqual(len(self.w.placed), 3, text)

    def test_temp_stacking_aligns_on_the_lower_material(self):
        """暂存区第二批码垛：下面那层盖住了白心，按物料顶面对准(黑色认圆环外圈)，叠在同色物料上。"""
        self._zone_setup('TEMP')
        self.h.cfg['learn_pick'] = True                       # 这个功能默认关了(要多拍一张)，这里测它
        self.run_cli('mtest', 'mtest TEMP 1 force')
        self.assertTrue(self.h.vision.has_pick(), '\n'.join(self.lines))     # 第一批放下时量好了
        for it in self.h.plan.items(2):                        # 第二批手放进转盘
            self.w.tray[it.slot] = it.color
        self.lines.clear()
        self.run_cli('mtest', 'mtest TEMP 2 force')
        text = '\n'.join(self.lines)
        self.assertEqual(self.w.collisions, 0, text)
        self.assertEqual(rings_summary(self.w, 'TEMP'), {1: [1, 1], 2: [5, 5], 3: [6, 6]}, text)
        self.assertEqual(text.count('对准结果(按物料)'), 2, text)
        self.assertEqual(text.count('对准结果(按圆环)'), 1, text)           # 黑色物料在黑环上：认圆环

    def test_stacking_without_material_or_ring_keeps_it_on_the_car(self):
        """码垛时下面的物料和圆环都认不到：不盲放(会砸倒下面那个)，物料留在车上。"""
        self._zone_setup('TEMP')
        self.run_cli('mtest', 'mtest TEMP 1 force')
        self._hide_covered_rings()
        self.h.vision.hide_pick = (5,)
        for it in self.h.plan.items(2):
            self.w.tray[it.slot] = it.color
        self.lines.clear()
        self.run_cli('mtest', 'mtest TEMP 2 force')
        text = '\n'.join(self.lines)
        self.assertIn('下面那层物料和圆环都看不到，物料留在车上', text)
        self.assertEqual(self.w.collisions, 0, text)
        self.assertEqual(rings_summary(self.w, 'TEMP')[2], [5], text)

    def test_mtest_temp_places_on_rings(self):
        self._zone_setup('TEMP')
        self.run_cli('mtest', 'mtest TEMP 1 force')
        text = '\n'.join(self.lines)
        self.assertEqual(self.w.collisions, 0, text)
        got = {k: [it['color'] for it in st] for (z, k), st in self.w.rings.items() if z == 'TEMP'}
        want = {it.ring: [it.color] for it in self.h.plan.items(1)}
        self.assertEqual(got, want, text)
        self.assertTrue(max(p[3] for p in self.w.placed) < 3.0)

    def test_mtest_nogo_only_aligns(self):
        self._zone_setup('TEMP')
        self.run_cli('mtest', 'mtest TEMP 1 force nogo')
        text = '\n'.join(self.lines)
        self.assertIn('nogo', text)
        self.assertFalse([r for r in self.w.requests if r.split()[0] in ('TAKE', 'DROP', 'PICK', 'GRAB')], self.w.requests)
        self.assertEqual(sorted(c for c in self.w.tray.values() if c), sorted(it.color for it in self.h.plan.items(1)))
        self.assertTrue(any(r.startswith('OBS RING') for r in self.w.requests))

    def test_mtest_rev_and_fresh_position(self):
        """rev：圆环前后方向反过来；每次 mtest 车都是手放的，前一次的底盘位移、学到的停车误差都不带过来。"""
        self._zone_setup('TEMP')
        moves = []
        orig = self.w.link.move
        self.w.link.move = lambda cmd, val, speed=None: (moves.append((cmd, val)), orig(cmd, val, speed))[1]
        self.run_cli('mtest', 'mtest TEMP 1 force nogo')
        first = next(v for c, v in moves if c == 'F' and abs(v) > 40)          # 跳过第一次测"底盘挪 20mm 画面怎么动"
        self.h.act.disp = {'S': 40.0, 'F': -70.0}
        self.h.learn = {'S': 12.0, 'F': 9.0}
        moves.clear()
        self.run_cli('mtest', 'mtest TEMP 1 force nogo rev')
        first_rev = next(v for c, v in moves if c == 'F' and abs(v) > 40)
        self.assertLess(abs(first + first_rev), 12, (first, first_rev))       # 方向反过来
        self.assertLess(abs(abs(first) - 150), 12, moves)
        self.assertEqual([m for m in moves if m[0] == 'S'], [], moves)        # 工位里不横移
        with self.assertRaises(ValueError):
            mission_cli.handle_cli('mtest', ['mtest', 'TEMP', '1', 'xyz'], link=self.w.link, raw_cfg={}, state={}, log=self.log)

    def test_mtest_zone_holds_heading_surveys_and_does_not_retract(self):
        """mtest ROUGH/TEMP：先把现在的车头方向记为要保持的方向(HOME)，再看清三个圆环；放完不缩回(不发 DROP)；手臂够得着就不横移。"""
        self._zone_setup('TEMP')
        self.run_cli('mtest', 'mtest TEMP 1 force')
        text = '\n'.join(self.lines)
        self.assertEqual(self.w.homed, 1, text)
        self.assertIn('车头方向：以现在的方向为准', text)
        self.assertIn('个圆环(认出', text)
        self.assertIn('补不过来车轮再横着靠近/退远', text)
        self.assertFalse([r for r in self.w.requests if r.startswith('DROP')], text)
        self.assertEqual([m for m in self.w.moves if m[0] == 'S'], [], text)
        self.assertEqual(len(self.w.placed), 3, text)
        self.assertNotIn('PICK 点还没量', text)                   # 默认不回去拍物料，就不提 PICK 点

    def test_mcode_again_keeps_what_is_in_the_tray(self):
        """mtest RAW 以后又输了一次 mcode(同一个码)：转盘里的物料还认得，mtest ROUGH 照常放。"""
        self.run_cli('mcode', 'mcode 156+123+516+231')
        self.w.arrive('RAW', 1)
        self.run_cli('mtest', 'mtest RAW 1')
        self.assertEqual(self.h.stats.grab_ok, 3)
        self.run_cli('mcode', 'mcode 156+123+516+231')
        self.w.arrive('ROUGH', 1)
        self.lines.clear()
        self.run_cli('mtest', 'mtest ROUGH 1')
        text = '\n'.join(self.lines)
        self.assertEqual(self.h.stats.place_ok, 3, text)
        self.assertNotIn('槽里没有', text)

    def test_mcode_lists_slots(self):
        self.run_cli('mcode', 'mcode 156+123+516+231')
        text = '\n'.join(self.lines)
        self.assertIn('1号槽=RED(粗加工区、暂存区都去环1)', text)
        # 第二批：粗加工区按任务码第四组；暂存区码垛在同色的第一批物料上(红在 1 号环)，不是粗加工区的环
        self.assertIn('2号槽=RED(粗加工区去环3，暂存区码垛到环1)', text)
        self.assertIn('3号槽=LIGHT_BLUE(粗加工区去环1，暂存区码垛到环3)', text)

    def test_mtest_zone_batch1_starts_with_empty_rings(self):
        """mtest 暂存区/粗加工区第一批：圆环已经用手拿空了，上次测试记着的"环上有物料"要清掉，不然每个环都当成占着、一个都不放。"""
        self._zone_setup('TEMP')
        self.run_cli('mtest', 'mtest TEMP 1 force')
        self.assertEqual(len(self.w.placed), 3)
        for key in [k for k in self.w.rings if k[0] == 'TEMP']:      # 用手把圆环拿空，物料放回车上转盘
            self.w.rings[key] = []
        for it in self.h.plan.items(1):
            self.w.tray[it.slot] = it.color
        self.lines.clear()
        self.run_cli('mtest', 'mtest TEMP 1 force')
        text = '\n'.join(self.lines)
        self.assertIn('已清掉记录', text)
        self.assertNotIn('按记录环', text)
        self.assertEqual(len(self.w.placed), 6, text)
        self.assertEqual(self.w.collisions, 0, text)
        self.assertEqual({k: [it.color for it in v] for (z, k), v in self.h.on_ring.items() if z == 'TEMP' and v},
                         {1: [1], 2: [5], 3: [6]})

    def test_mtest_temp2_force_records_batch1_once(self):
        """mtest TEMP 2 force：暂存区记成"第一批各一个"(跑过 TEMP 1 也不重复记)；开头显示码垛去的环(同色那个环)。"""
        self._zone_setup('TEMP')
        self.run_cli('mtest', 'mtest TEMP 1 force')
        for it in self.h.plan.items(2):
            self.w.tray[it.slot] = it.color
        self.lines.clear()
        self.run_cli('mtest', 'mtest TEMP 2 force')
        text = '\n'.join(self.lines)
        self.assertIn('2号槽 RED -> 码垛到环1(叠在第一批的RED上)', text)
        self.assertIn('1号槽 BLACK -> 码垛到环2(叠在第一批的BLACK上)', text)
        self.assertEqual({k: [(it.batch, it.color) for it in v] for (z, k), v in self.h.on_ring.items() if z == 'TEMP' and v},
                         {1: [(1, 1), (2, 1)], 2: [(1, 5), (2, 5)], 3: [(1, 6), (2, 6)]}, text)
        self.assertEqual(rings_summary(self.w, 'TEMP'), {1: [1, 1], 2: [5, 5], 3: [6, 6]}, text)

    def test_ring_taken_by_record_says_so(self):
        """程序记着某个环上还放着物料(取回失败留下的)：不去那个环，提示里写清是按记录判断的(不是摄像头看到的)。"""
        self._zone_setup('ROUGH')
        self.h._ensure(self.w.link)
        self.h.in_tray = {it.slot: it for it in self.h.plan.items(1)}
        self.h.on_ring[('ROUGH', 2)] = [self.h.plan.items(2)[0]]     # 记着 2 号环上留着第二批的黑色
        self.h.visits = {}
        self.h.run_role('ROUGH', 1)
        text = '\n'.join(self.lines)
        self.assertIn('按记录环2 上还放着 BLACK', text)
        self.assertIn('不是摄像头看到的', text)
        self.assertEqual([p[1] for p in self.w.placed], [1, 3], text)
        self.assertEqual(self.w.collisions, 0, text)

    def test_gtest_nogo_and_needs_zero(self):
        self.w.lift_known = False                            # 模拟急停打断升降后位置丢了
        self.run_cli('gtest', 'gtest 1')
        self.assertTrue(any('LIFT ZERO' in l for l in self.lines), self.lines)
        self.run_cli('arm', 'arm LIFT ZERO')
        self.w.arrive('RAW', 1)
        self.w.a1_ref, self.w.a2_ref = self.w.params['A1G'], self.w.params['A2E']
        self.w.requests.clear()
        self.run_cli('gtest', f'gtest {self.w.raw_items[0]["color"]} nogo')
        self.assertTrue(any('nogo' in l for l in self.lines), self.lines)
        self.assertNotIn('CLAW C', self.w.requests)
        with self.assertRaises(ValueError):
            mission_cli.handle_cli('gtest', ['gtest', '9'], link=self.w.link, raw_cfg={}, state={}, log=self.log)

    def test_mtest_reset_closes_camera(self):
        closed = []
        self.h.close = lambda: closed.append(1)
        self.run_cli('mtest', 'mtest reset')
        self.assertEqual(closed, [1])
        self.assertIsNone(mission_cli._S['hooks'])

    def test_vmask_before_vclaw_ignores_guessed_claw_point(self):
        """还没 vclaw：配置里的爪子点只是估计值，落在爪子区域里也照样存。"""
        calls = []

        class MD:
            def save_claw_mask(self, frames, keep_clear=None):
                calls.append(keep_clear)
                return ('claw_mask.png', 0.4)
        with tempfile.TemporaryDirectory() as td:
            self.h.cfg['vision_cal_file'] = os.path.join(td, 'vision_cal.json')
            self.h.vision.material_detector_obj = lambda: MD()
            self.h.vision._frame = lambda: None
            mission_cli._vmask(self.h, self.log)
            self.assertEqual(calls, [None])
            from vision import save_vision_cal
            save_vision_cal('RAW', (330, 250), self.h.cfg['vision_cal_file'])
            mission_cli._vmask(self.h, self.log)
            self.assertIsNotNone(calls[-1])

    # ---------------- 10-10：mtest wheels / nofilt、park、码垛跳过、gtest 后夹着东西
    def test_mtest_wheels_and_nofilt_only_for_this_run(self):
        """mtest TEMP 1 force wheels：这次对准先动车轮(小步、wheels_rpm)；nofilt：这次不滤波。做完配置恢复原样，报这次用时。"""
        self._zone_setup('TEMP')
        self.run_cli('mtest', 'mtest TEMP 1 force wheels')
        text = '\n'.join(self.lines)
        self.assertIn('wheels：这次对准先动车轮', text)
        self.assertIn('这次用时', text)
        self.assertIn('先动车轮', text.split('这次用时')[-1])
        self.assertTrue(any(sp == 60 for sp in self.w.move_speeds), self.w.move_speeds)
        self.assertFalse(self.h.cfg['wheels_first'])                 # 只管这一次
        self.assertEqual(len(self.w.placed), 3, text)
        seen = []
        real = self.h.servo.run

        def run(*a, **kw):
            seen.append(kw.get('filt'))
            return real(*a, **kw)
        self.h.servo.run = run
        for key in [k for k in self.w.rings if k[0] == 'TEMP']:     # 圆环拿空，物料放回车上
            self.w.rings[key] = []
        for it in self.h.plan.items(1):
            self.w.tray[it.slot] = it.color
        self.lines.clear()
        self.run_cli('mtest', 'mtest TEMP 1 force nofilt')
        text = '\n'.join(self.lines)
        self.assertIn('nofilt：这次对准不用测量滤波', text)
        self.assertTrue(seen and all(f is False for f in seen), seen)
        self.assertTrue(self.h.cfg['zone_filter'])
        with self.assertRaises(ValueError):
            mission_cli.handle_cli('mtest', ['mtest', 'TEMP', '1', 'wheel'], link=self.w.link, raw_cfg={}, state={}, log=self.log)

    def test_park_command(self):
        """park：收臂、升降停 60(终端命令，急停以后也能做)。"""
        self.h._ensure(self.w.link)
        self.h._aborted = True
        self.w.lift_mm = 100.0
        self.run_cli('park', 'park')
        self.assertIn('PARK', self.w.requests)
        self.assertAlmostEqual(self.w.lift_mm, 60.0)
        self.assertTrue(any('park 完成' in l for l in self.lines), self.lines)

    def test_gtest_leaves_material_in_claw_until_claw_open(self):
        """gtest 夹起来以后物料还在爪子里：park 不降(会压到车上转盘)；arm CLAW O 以后才算空了。"""
        self.run_cli('arm', 'arm LIFT ZERO')
        self.w.arrive('RAW', 1)
        self.w.a1_ref, self.w.a2_ref = self.w.params['A1G'], self.w.params['A2E']
        self.run_cli('gtest', f'gtest {self.w.raw_items[0]["color"]}')
        self.assertIsNotNone(self.h.maybe_holding)
        self.lines.clear()
        self.run_cli('park', 'park')
        self.assertNotIn('PARK', self.w.requests)
        self.assertTrue(any('不收臂停 60' in l for l in self.lines), self.lines)
        self.run_cli('arm', 'arm CLAW O')
        self.assertIsNone(self.h.maybe_holding)
        self.run_cli('park', 'park')
        self.assertIn('PARK', self.w.requests)

    def test_mcode_says_stack_without_lower_is_skipped(self):
        self.run_cli('mcode', 'mcode 123+123+456+123')
        text = '\n'.join(self.lines)
        self.assertIn('没有同色的第一批物料可叠：不放(规则只许码垛在同色上)', text)

    def test_busy_refused(self):
        self.state['busy'] = True
        with self.assertRaises(ValueError):
            mission_cli.handle_cli('arm', ['arm', 'GET'], link=self.w.link, raw_cfg={}, state=self.state, log=self.log)
        self.state['busy'] = False


class MotLink:
    """只会回 MOT? / MOT EN 的假 STM32。"""

    def __init__(self, lines, known=True):
        self.lines = lines
        self.known = known
        self.sent = []

    def request(self, text, timeout, collect=False):
        self.sent.append(text)
        if not self.known:
            out = (False, 'ERR CMD', [])
        elif text == 'MOT?':
            out = (True, 'DONE', list(self.lines))
        else:
            out = (True, 'DONE', [])
        return out if collect else out[:2]


class MotTests(unittest.TestCase):
    GOOD = ['MOT 1 V=12.31 EN=1 ARR=1 STALL=0 PROT=0', 'MOT 2 V=12.29 EN=1 ARR=1 STALL=0 PROT=0',
            'MOT 3 V=12.30 EN=1 ARR=1 STALL=0 PROT=0', 'MOT 4 V=12.30 EN=1 ARR=1 STALL=0 PROT=0',
            'MOT 5 V=12.28 EN=1 ARR=1 STALL=0 PROT=0']

    def run_mot(self, text, link):
        lines, state = [], {}
        mission_cli.handle_cli('mot', text.split(), link=link, raw_cfg={}, state=state, log=lines.append)
        wait_idle(state)
        return '\n'.join(lines)

    def test_all_good(self):
        rows, advice = mission_cli.explain_mot(self.GOOD)
        self.assertEqual(len(rows), 5)
        self.assertIn('1号(右前轮)：电压 12.31V，已使能', rows[0])
        self.assertIn('send R 90', advice[0])

    def test_protect_low_voltage_and_silent(self):
        lines = ['MOT 1 V=10.10 EN=1 ARR=0 STALL=0 PROT=0', 'MOT 2 V=10.20 EN=0 ARR=0 STALL=1 PROT=1',
                 'MOT 3 NOREPLY', 'MOT 4 V=10.15 EN=1 ARR=1 STALL=0 PROT=0', 'MOT 5 NOREPLY RX=01 00 EE 6B']
        rows, advice = mission_cli.explain_mot(lines)
        text = '\n'.join(rows + advice)
        self.assertIn('堵转保护', rows[1])
        self.assertIn('没使能', rows[1])
        self.assertIn('3号(右后轮)：没有回复', text)
        self.assertIn('01 00 EE 6B', rows[4])
        self.assertIn('mot en', text)
        self.assertIn('电池快没电', text)

    def test_all_wheels_silent(self):
        rows, advice = mission_cli.explain_mot(['MOT %d NOREPLY' % i for i in range(1, 6)])
        self.assertIn('PC11', advice[0])

    def test_cli_mot_and_mot_en(self):
        link = MotLink(self.GOOD)
        out = self.run_mot('mot', link)
        self.assertEqual(link.sent, ['MOT?'])
        self.assertIn('4号(左后轮)', out)
        link = MotLink(self.GOOD)
        self.run_mot('mot en', link)
        self.assertEqual(link.sent, ['MOT EN', 'MOT?'])

    def test_old_firmware(self):
        out = self.run_mot('mot', MotLink([], known=False))
        self.assertIn('旧程序', out)

    def test_bad_args(self):
        with self.assertRaises(ValueError):
            mission_cli.handle_cli('mot', ['mot', 'xx'], link=MotLink([]), raw_cfg={}, state={}, log=print)


class ConfigScriptTests(unittest.TestCase):
    def test_apply_adds_only_missing(self):
        d = tempfile.mkdtemp()
        path = os.path.join(d, 'cfg.json')
        base = {'stops': {'QR': [1, 2, 90]}, 'mission': ['QR', 'RAW', 'ROUGH', 'TEMP', 'RAW', 'ROUGH', 'TEMP', 'START'],
                'mission_cfg': {'time_limit_s': 123.0, 'tol_mm': {'RING': 0.8}}, 'stm32_params': {'FPPM': 12.9}}
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(base, f)
        self.assertEqual(apply_mission_config.main([path]), 0)
        cfg = load(path)
        mc = cfg['mission_cfg']
        self.assertEqual(mc['time_limit_s'], 123.0)                 # 用户设的值不变
        self.assertEqual(mc['tol_mm']['RING'], 0.8)
        self.assertIn('RAW', mc['tol_mm'])                          # 缺的补上
        self.assertTrue(mc['enabled'])
        self.assertEqual(cfg['stm32_params'], {'FPPM': 12.9})       # 别的配置不动
        self.assertTrue(os.path.exists(path.replace('.json', '.json.bak_mission')))
        self.assertEqual(apply_mission_config.main([path, '--disable']), 0)
        self.assertFalse(load(path)['mission_cfg']['enabled'])
        # 再运行一次不改变内容
        before = read(path)
        apply_mission_config.main([path, '--disable'])
        self.assertEqual(before, read(path))

    def test_old_defaults_are_upgraded(self):
        d = tempfile.mkdtemp()
        path = os.path.join(d, 'cfg.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump({'mission_cfg': {'px_per_mm': {'RAW': 4.36, 'RING': 2.96}, 'tol_mm': {'RAW': 3.0},
                                       'accept_mm': {'RAW': 5.0}, 'learn_pick': True, 'chassis_fine_rpm': 80,
                                       'raw_stop_s': 5.0, 'raw_frames': 2}}, f)
        apply_mission_config.main([path])
        mc = load(path)['mission_cfg']
        self.assertEqual((mc['raw_stop_s'], mc['raw_frames']), (4.0, 1))       # 10-10 写进配置的旧默认值也升级
        self.assertEqual(mc['px_per_mm']['RAW'], 1.97)              # 没改过的旧默认值 -> 新默认值
        self.assertEqual(mc['tol_mm']['RAW'], 2.0)
        self.assertEqual(mc['accept_mm']['RAW'], 5.0)               # 用户自己改过的不动
        self.assertIs(mc['learn_pick'], False)                      # 10-10 改了默认值：放完不回去拍照
        self.assertEqual(mc['chassis_fine_rpm'], 80)                # 用户自己改过的不动
        self.assertEqual(mc['ring_order'], {'ROUGH': 'lr', 'TEMP': 'lr'})   # 新加的项补上
        self.assertIs(mc['chassis_strafe'], False)

    def test_10_10_keys_and_strafe_upgrade(self):
        """10-10：ring_strafe_max_mm 旧默认 40 升级成 20(用户自己改过的不动)；新加的项(时间、扫码、斜度…)都补上，用户的值不改。"""
        d = tempfile.mkdtemp()
        path = os.path.join(d, 'cfg.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump({'mission_cfg': {'ring_strafe_max_mm': 40.0, 'raw_stop_s': 5.0, 'qr_dwell_s': 1.5}}, f)
        apply_mission_config.main([path])
        mc = load(path)['mission_cfg']
        self.assertEqual(mc['ring_strafe_max_mm'], 20.0)
        self.assertEqual(mc['raw_stop_s'], 4.0)                     # 以前那次升级也还在
        self.assertEqual(mc['qr_dwell_s'], 1.5)                     # 用户的值不动
        for k in ('round_s', 'home_margin_s', 'work_item_s', 'qr_search', 'qr_targets_mm', 'qr_scanner_from_rear_mm',
                  'qr_scanner_from_right_mm', 'tilt_precorrect', 'tilt_warn_deg', 'keepalive_deg', 'code_plus', 'home_when_idle'):
            self.assertIn(k, mc)
        self.assertEqual(mc['wheels_min_mm'], 4.0)
        self.assertIs(mc['wheels_first'], True)          # 10-10：先慢慢动车轮
        self.assertIs(mc['raw_any_order'], True)         # 10-10：爪子下面停的是哪个就夹哪个
        self.assertIs(mc['raw_wheels_first'], False)      # 10-10：原料区只动爪子
        self.assertIs(mc['zone_filter'], True)
        with open(path, 'w', encoding='utf-8') as f:
            json.dump({'mission_cfg': {'ring_strafe_max_mm': 30}}, f)
        apply_mission_config.main([path])
        self.assertEqual(load(path)['mission_cfg']['ring_strafe_max_mm'], 30)   # 用户自己改过的不动
        with open(path, 'w', encoding='utf-8') as f:
            json.dump({'mission_cfg': {'ring_strafe_max_mm': 40}}, f)
        apply_mission_config.main([path])
        self.assertEqual(load(path)['mission_cfg']['ring_strafe_max_mm'], 20.0)   # 手写的整数 40 也算旧默认值
        with open(path, 'w', encoding='utf-8') as f:
            json.dump({'mission_cfg': {'wheels_min_mm': 1.5}}, f)
        apply_mission_config.main([path])
        self.assertEqual(load(path)['mission_cfg']['wheels_min_mm'], 4.0)        # 旧默认 1.5 -> 4
        with open(path, 'w', encoding='utf-8') as f:
            json.dump({'mission_cfg': {'wheels_min_mm': 2.5}}, f)
        apply_mission_config.main([path])
        self.assertEqual(load(path)['mission_cfg']['wheels_min_mm'], 2.5)        # 用户自己改过的不动

    def test_missing_file(self):
        self.assertEqual(apply_mission_config.main(['/nonexistent/x.json']), 1)


if __name__ == '__main__':
    unittest.main(verbosity=1)
