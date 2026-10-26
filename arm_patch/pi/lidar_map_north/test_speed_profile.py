"""speed_profile.py 的检查(10-26 用户"整个车运动和转向都太慢了帮我加速")：speed fast / max / normal 改哪些、能不能原样还原、
TCKP 按 PPD 换算；turntune 在假车上选 TCLAG；Motion 按 SCDEC 估直行刹车。python3 -m unittest test_speed_profile"""
import copy
import unittest

import test_auto_run  # noqa: F401  (电脑上没有 stm32_link 时先装好替身)
import speed_profile as spf
from route_plan import Motion

# 车上 10-22 的配置(car_backup)里和速度有关的
CAR = {'route_speed_rpm': 150, 'strafe_speed_rpm': 140, 'turn90_s': 1.8, 'turn180_s': 2.5,
       'stm32_params': {'FPPM': 12.93, 'PPD': 47.0, 'SCACC': 100.0, 'SCDEC': 70.0, 'SSACC': 140.0, 'TCV': 80.0, 'TCA': 400.0,
                        'ALTOL': 0.5}}


class FakeStm:
    """STM32 的参数表 + 原地转：TCLAG 合适时转得快、停得准(按 lag_best)，太小冲过头(误差大)，太大慢慢爬。"""

    def __init__(self, lag_best=0.10, stall=False):
        self.P = {'SCACC': 100.0, 'SCDEC': 70.0, 'SSACC': 140.0, 'TCV': 80.0, 'TCA': 400.0, 'TCD': 250.0, 'TCKP': 3.0,
                  'TCMIN': 1.0, 'TCLAG': 0.16, 'PPD': 47.0}
        self.sets = []
        self.turns = []
        self.lag_best = lag_best
        self.stall = stall
        self.homed = 0

    def get_params(self):
        return dict(self.P), 'DONE'

    def set_param(self, name, value):
        if name not in self.P:
            return False, 'ERR NAME'
        self.P[name] = float(value)
        self.sets.append((name, float(value)))
        return True, 'DONE'

    def home(self):
        self.homed += 1
        return True, 'DONE'

    def move(self, cmd, val, speed=None):
        self.turns.append((cmd, val))
        if self.stall:
            return False, 'ERR STALL t=800 e=-9000'
        lag = self.P['TCLAG']
        d = lag - self.lag_best
        t = 1.6 + 6.0 * abs(d) if d >= 0 else 1.6 + 10.0 * abs(d)      # 偏大：最后慢慢爬；偏小：冲过头再回来
        e = 0.2 if d >= 0 else 0.2 + 30.0 * abs(d)
        return True, f'DONE t={int(t * 1000)} e={int(e * 100) * (1 if val > 0 else -1)}'


class SwitchTests(unittest.TestCase):
    def test_tckp_follows_ppd(self):
        self.assertEqual(spf.tckp_for(47), 6.0)                 # 车上标定的 PPD：车身"差 1° 转多快"和 PPD 25.4、TCKP 3 时一样
        self.assertEqual(spf.tckp_for(25.4), 3.3)
        self.assertEqual(spf.tckp_for(10), 3.0)                 # 限在 3~8
        self.assertEqual(spf.tckp_for(None), 3.3)

    def test_fast_then_normal_restores_everything(self):
        cfg = copy.deepcopy(CAR)
        stm = FakeStm()
        lines = []
        ch, bad = spf.switch(cfg, 'fast', stm, lines.append)
        self.assertEqual(bad, [])
        self.assertEqual((cfg['route_speed_rpm'], cfg['strafe_speed_rpm']), (190, 160))
        sp = cfg['stm32_params']
        self.assertEqual((sp['SCACC'], sp['SCDEC'], sp['SSACC'], sp['TCV'], sp['TCKP'], sp['TCMIN']), (150, 110, 180, 120, 6.0, 2))
        self.assertEqual((sp['FPPM'], sp['ALTOL']), (12.93, 0.5))      # 别的参数不动
        self.assertEqual(stm.P['TCKP'], 6.0)                            # STM32 马上收到
        self.assertEqual(spf.level(cfg), 'fast')
        self.assertIn(('route_speed_rpm', 150, 190), ch)
        self.assertIn(('TCKP', 3.0, 6.0), ch)                           # 配置里没有：按 STM32 现在的值显示
        # 再换 max：原来的样子还是第一次之前的
        spf.switch(cfg, 'max', stm, lines.append)
        self.assertEqual((cfg['route_speed_rpm'], cfg['stm32_params']['SCACC']), (220, 200))
        self.assertEqual(cfg['speed_saved']['cfg']['route_speed_rpm'], 150)
        self.assertEqual(spf.level(cfg), 'max')
        # normal：配置一模一样，STM32 也发回原来的值(配置里没有的 TCD/TCKP/TCMIN 发回当时 STM32 里的)
        stm.sets = []
        ch, bad = spf.switch(cfg, 'normal', stm, lines.append)
        self.assertEqual(cfg, CAR)
        self.assertEqual(spf.level(cfg), 'normal')
        self.assertEqual({k: stm.P[k] for k in ('SCACC', 'SCDEC', 'SSACC', 'TCV', 'TCA', 'TCD', 'TCKP', 'TCMIN')},
                         {'SCACC': 100.0, 'SCDEC': 70.0, 'SSACC': 140.0, 'TCV': 80.0, 'TCA': 400.0, 'TCD': 250.0, 'TCKP': 3.0, 'TCMIN': 1.0})
        self.assertTrue(ch)
        # 已经是 normal：什么都不改
        ch, bad = spf.switch(cfg, 'normal', stm, lines.append)
        self.assertEqual((ch, bad), ([], []))

    def test_without_stm32_only_config(self):
        cfg = copy.deepcopy(CAR)
        ch, bad = spf.switch(cfg, 'fast', None, lambda m: None)
        self.assertEqual(cfg['stm32_params']['TCKP'], 6.0)              # PPD 从配置里读
        self.assertEqual(bad, [])
        spf.switch(cfg, 'normal', None, lambda m: None)
        self.assertEqual(cfg, CAR)

    def test_old_firmware_refuses_a_param(self):
        cfg = copy.deepcopy(CAR)
        stm = FakeStm()
        del stm.P['TCMIN']
        lines = []
        ch, bad = spf.switch(cfg, 'fast', stm, lines.append)
        self.assertEqual(bad, ['TCMIN'])
        self.assertTrue(any('TCMIN' in m for m in lines))
        spf.report(ch, bad, 'fast', lines.append)
        self.assertTrue(any('没收' in m for m in lines))

    def test_bad_level(self):
        with self.assertRaises(ValueError):
            spf.switch(copy.deepcopy(CAR), 'turbo', None)

    def test_show(self):
        lines = []
        spf.show(copy.deepcopy(CAR), FakeStm(), lines.append)
        text = '\n'.join(lines)
        self.assertIn('speed normal', text)
        self.assertIn('TCKP=3', text)
        self.assertIn('PPD=47', text)


class TurnTuneTests(unittest.TestCase):
    def test_picks_fastest_accurate_lag_and_turns_back(self):
        stm = FakeStm(lag_best=0.10)
        lines = []
        best, old, rows = spf.turn_tune(stm, lines.append)
        self.assertEqual((best, old), (0.10, 0.16))
        self.assertEqual(stm.P['TCLAG'], 0.10)                          # 选好的发给 STM32
        self.assertEqual(stm.homed, 1)
        self.assertEqual(sum(v for _, v in stm.turns), 0)               # 最后转回原来的车头
        self.assertEqual(len(stm.turns), 16)
        bad = [r for r in rows if r['lag'] == 0.08][0]
        self.assertGreater(max(abs(e) for e in bad['es']), 0.6)         # 0.08 冲过头：不选
        self.assertEqual(spf.turn_estimates(rows, best), (1.7, 2.4))

    def test_stall_keeps_old_lag(self):
        stm = FakeStm(stall=True)
        best, old, rows = spf.turn_tune(stm, lambda m: None)
        self.assertEqual(best, 0.16)
        self.assertEqual(stm.P['TCLAG'], 0.16)
        self.assertEqual(len(stm.turns), 1)                             # 第一次就卡住：不再转

    def test_abort_stops_and_restores(self):
        stm = FakeStm()
        n = [0]

        def aborted():
            n[0] += 1
            return n[0] > 3
        with self.assertRaises(RuntimeError):
            spf.turn_tune(stm, lambda m: None, aborted=aborted)
        self.assertEqual(stm.P['TCLAG'], 0.16)                          # 没试完：还是原来的

    def test_pick_prefers_larger_lag_when_nearly_as_fast(self):
        rows = [dict(lag=0.16, ts=[1.80, 1.82], es=[0.1, -0.2], bad=None),
                dict(lag=0.13, ts=[1.78, 1.80], es=[0.2, -0.1], bad=None),
                dict(lag=0.10, ts=[1.50, 1.52], es=[0.7, -0.3], bad=None)]    # 快但不准
        self.assertEqual(spf.pick(rows, 0.16), 0.16)
        self.assertEqual(spf.pick([], 0.12), 0.12)


class MotionDecelTests(unittest.TestCase):
    def test_scdec_makes_straight_slower(self):
        base = dict(route_speed_rpm=150, fwd_mm_per_rev=249.0, cmd_overhead_s=0.0)
        same = Motion(dict(base, stm32_params={'SCACC': 100.0}))
        soft = Motion(dict(base, stm32_params={'SCACC': 100.0, 'SCDEC': 70.0}))
        zero = Motion(dict(base, stm32_params={'SCACC': 100.0, 'SCDEC': 0.0}))     # 0 = 和 SCACC 一样
        self.assertAlmostEqual(same.time_of('F', 1117), zero.time_of('F', 1117))
        self.assertGreater(soft.time_of('F', 1117), same.time_of('F', 1117) + 0.3)
        self.assertAlmostEqual(soft.time_of('F', 1117), 3.62, delta=0.05)          # 实车日志 F +1117 用 3.8 秒(含停稳、回复)
        a = 100.0 / 60.0 * 249.0
        self.assertAlmostEqual(same.time_of('F', 3000), 3000 / (150 / 60 * 249) + (150 / 60 * 249) / a, places=6)
        self.assertAlmostEqual(same.time_of('F', 100), 2 * (100 / a) ** 0.5, places=6)


if __name__ == '__main__':
    unittest.main()
