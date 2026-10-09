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
                                       'accept_mm': {'RAW': 5.0}}}, f)
        apply_mission_config.main([path])
        mc = load(path)['mission_cfg']
        self.assertEqual(mc['px_per_mm']['RAW'], 1.97)              # 没改过的旧默认值 -> 新默认值
        self.assertEqual(mc['tol_mm']['RAW'], 2.0)
        self.assertEqual(mc['accept_mm']['RAW'], 5.0)               # 用户自己改过的不动

    def test_missing_file(self):
        self.assertEqual(apply_mission_config.main(['/nonexistent/x.json']), 1)


if __name__ == '__main__':
    unittest.main(verbosity=1)
