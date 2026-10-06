"""visual_servo.py 的模拟测试：python3 test_visual_servo.py

假的"摄像头+手臂+底盘"：随机的摄像头朝向/镜像、随机的符号，加测量噪声、手臂和底盘的动作误差。
验证：从零开始(没有 J)能自己探测并收敛；有 J 时更快；J 错了能发现并恢复；手臂够不着时底盘接手；
看不到目标/动了没反应时能给出明确的失败原因。
"""
import math
import unittest

import numpy as np

from visual_servo import VisualServo, JacStore, ServoError, DEFAULTS


class SimPlant:
    """p = 目标像素 - 爪子像素(真实值)。动作使 p 线性变化：p += A@[ID2度,ID1度] 或 B@[S毫米,F毫米]。"""

    def __init__(self, rng, scale=2.96, err_mm=None, noise_px=0.6, arm_gain_err=0.02, arm_noise_deg=0.05,
                 ch_gain_err=0.05, ch_noise_mm=1.5, mm_per_deg_id2=0.175, mm_per_deg_id1=2.6, dead_arm=False,
                 lose_after=None):
        self.rng = rng
        self.scale = scale
        th = rng.uniform(0, 2 * math.pi)
        r = np.array([math.cos(th), math.sin(th)])
        t = np.array([-r[1], r[0]])
        sa, sb, sS, sF = rng.choice([-1.0, 1.0], 4)
        self.A = np.stack([sa * mm_per_deg_id2 * scale * r, sb * mm_per_deg_id1 * scale * t], axis=1)
        self.B = np.stack([sS * scale * r, sF * scale * t], axis=1)
        mag = err_mm if err_mm is not None else rng.uniform(3, 25)
        ang = rng.uniform(0, 2 * math.pi)
        self.p = np.array([math.cos(ang), math.sin(ang)]) * mag * scale
        self.noise_px = noise_px
        self.arm_gain_err, self.arm_noise_deg = arm_gain_err, arm_noise_deg
        self.ch_gain_err, self.ch_noise_mm = ch_gain_err, ch_noise_mm
        self.dead_arm = dead_arm
        self.lose_after = lose_after
        self.n_measure = 0
        self.arm_moves = 0
        self.ch_moves = 0

    def measure(self):
        self.n_measure += 1
        if self.lose_after is not None and self.n_measure > self.lose_after:
            return None
        n = self.rng.normal(0, self.noise_px, 2)
        return (float(self.p[0] + n[0]), float(self.p[1] + n[1]))

    def arm_move(self, d2, d1):
        self.arm_moves += 1
        if self.dead_arm:
            return (0.0, 0.0)
        a2 = d2 * (1 + self.rng.normal(0, self.arm_gain_err)) + (self.rng.normal(0, self.arm_noise_deg) if d2 else 0.0)
        a1 = d1 * (1 + self.rng.normal(0, self.arm_gain_err)) + (self.rng.normal(0, self.arm_noise_deg) if d1 else 0.0)
        self.p = self.p + self.A @ np.array([a2, a1])
        return (a2, a1)

    def chassis_move(self, s, f):
        self.ch_moves += 1
        s2 = s * (1 + self.rng.normal(0, self.ch_gain_err)) + (self.rng.normal(0, self.ch_noise_mm) if s else 0.0)
        f2 = f * (1 + self.rng.normal(0, self.ch_gain_err)) + (self.rng.normal(0, self.ch_noise_mm) if f else 0.0)
        self.p = self.p + self.B @ np.array([s2, f2])

    @property
    def true_err_mm(self):
        return float(np.linalg.norm(self.p)) / self.scale


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


def make_servo(plant, store=None, cfg=None, clock=None):
    clock = clock or FakeClock()
    return VisualServo(plant, store=store or JacStore(None), cfg=cfg, log=lambda m: None, sleep=clock.sleep, clock=clock)


class ServoTests(unittest.TestCase):
    def test_converge_from_scratch(self):
        rng = np.random.default_rng(1)
        ok_n, errs, iters = 0, [], []
        N = 200
        for _ in range(N):
            pl = SimPlant(rng)
            sv = make_servo(pl)
            res = sv.run('RING', pl.measure, pl.scale, tol_mm=1.0, allow_chassis=False, max_iter=8, timeout_s=60)
            ok_n += res.ok
            errs.append(pl.true_err_mm)
            iters.append(res.iters)
        errs = np.array(errs)
        print(f'\n  [从零探测] 成功 {ok_n}/{N}，真实误差 平均 {errs.mean():.2f}mm / 95% {np.percentile(errs, 95):.2f}mm / 最大 {errs.max():.2f}mm，'
              f'平均修正 {np.mean(iters):.1f} 次')
        self.assertGreaterEqual(ok_n, int(0.97 * N))
        self.assertLess(errs.mean(), 1.0)
        self.assertLess(np.percentile(errs, 95), 1.6)

    def test_cached_jacobian_is_faster(self):
        rng = np.random.default_rng(2)
        iters_cold, iters_warm, errs = [], [], []
        for _ in range(60):
            pl = SimPlant(rng)
            store = JacStore(None)
            sv = make_servo(pl, store)
            r1 = sv.run('RING', pl.measure, pl.scale, 1.0, allow_chassis=False, max_iter=8, timeout_s=60)
            # 同一套机械(同一个 A、B)，新的目标位置
            ang = rng.uniform(0, 2 * math.pi)
            pl.p = np.array([math.cos(ang), math.sin(ang)]) * rng.uniform(3, 20) * pl.scale
            sv2 = make_servo(pl, store)
            r2 = sv2.run('RING', pl.measure, pl.scale, 1.0, allow_chassis=False, max_iter=8, timeout_s=60)
            self.assertFalse(r2.probed)
            iters_cold.append(r1.iters)
            iters_warm.append(r2.iters)
            errs.append(pl.true_err_mm)
        print(f'\n  [有 J] 平均修正 {np.mean(iters_warm):.1f} 次(冷启动 {np.mean(iters_cold):.1f} 次)，最终真实误差平均 {np.mean(errs):.2f}mm')
        self.assertLessEqual(np.mean(iters_warm), 3.0)
        self.assertLess(np.mean(errs), 1.0)

    def test_wrong_cached_jacobian_recovers(self):
        rng = np.random.default_rng(3)
        ok_n, N = 0, 60
        for _ in range(N):
            pl = SimPlant(rng)
            store = JacStore(None)
            store.put('RING', 'arm', -pl.A)                      # 符号全反：错的 J
            sv = make_servo(pl, store)
            res = sv.run('RING', pl.measure, pl.scale, 1.5, allow_chassis=False, max_iter=10, timeout_s=60)
            ok_n += res.ok
        print(f'\n  [J 全错] 恢复成功 {ok_n}/{N}')
        self.assertGreaterEqual(ok_n, int(0.85 * N))

    def test_chassis_takes_over_for_large_error(self):
        rng = np.random.default_rng(4)
        ok_n, used, N = 0, 0, 40
        errs = []
        for _ in range(N):
            pl = SimPlant(rng, err_mm=rng.uniform(50, 90))
            sv = make_servo(pl)
            res = sv.run('RAW', pl.measure, pl.scale, 2.0, allow_chassis=True, max_iter=10, timeout_s=60)
            ok_n += res.ok
            used += res.used_chassis
            errs.append(pl.true_err_mm)
        print(f'\n  [大偏差 50~90mm] 成功 {ok_n}/{N}，用了底盘 {used}/{N}，最终真实误差平均 {np.mean(errs):.2f}mm')
        self.assertGreaterEqual(ok_n, int(0.85 * N))
        self.assertEqual(used, N)

    def test_arm_only_gives_up_when_out_of_reach(self):
        rng = np.random.default_rng(5)
        pl = SimPlant(rng, err_mm=80.0)
        sv = make_servo(pl)
        res = sv.run('RAW', pl.measure, pl.scale, 1.0, allow_chassis=False, max_iter=10, timeout_s=60)
        self.assertFalse(res.ok)
        print(f'\n  [够不着且不许动底盘] {res.reason}')

    def test_target_lost(self):
        rng = np.random.default_rng(6)
        pl = SimPlant(rng, lose_after=0)
        res = make_servo(pl).run('RING', pl.measure, pl.scale, 1.0)
        self.assertFalse(res.ok)
        self.assertIn('看不到目标', res.reason)

    def test_target_lost_midway(self):
        rng = np.random.default_rng(7)
        pl = SimPlant(rng, lose_after=6)
        res = make_servo(pl).run('RING', pl.measure, pl.scale, 1.0, allow_chassis=False)
        self.assertFalse(res.ok)

    def test_probe_fails_if_arm_does_not_move(self):
        rng = np.random.default_rng(8)
        pl = SimPlant(rng, dead_arm=True)
        res = make_servo(pl).run('RING', pl.measure, pl.scale, 1.0, allow_chassis=False)
        self.assertFalse(res.ok)
        self.assertIn('探测失败', res.reason)

    def test_timeout(self):
        rng = np.random.default_rng(9)
        pl = SimPlant(rng, noise_px=4.0)                       # 噪声大到永远达不到 0.3mm
        res = make_servo(pl).run('RING', pl.measure, pl.scale, 0.3, allow_chassis=False, max_iter=50, timeout_s=3.0)
        self.assertFalse(res.ok)

    def test_arm_excursion_is_limited(self):
        rng = np.random.default_rng(10)
        pl = SimPlant(rng, err_mm=60.0)
        cfg = dict(arm_limit_deg=dict(id2=40.0, id1=3.0))
        sv = make_servo(pl, cfg=cfg)
        sv.run('RAW', pl.measure, pl.scale, 1.0, allow_chassis=True, max_iter=10, timeout_s=60)
        # 限位按"指令"算；模拟的手臂有 2% 比例误差和 0.05° 噪声，实际量会略多一点点，所以留 5%+0.1° 容差
        self.assertLessEqual(abs(sv.dev[0]), 40.0 * 1.05 + 0.1)
        self.assertLessEqual(abs(sv.dev[1]), 3.0 * 1.05 + 0.1)

    def test_store_roundtrip(self):
        import os, tempfile
        d = tempfile.mkdtemp()
        path = os.path.join(d, 'cal.json')
        s = JacStore(path)
        s.put('RING', 'arm', np.array([[1.0, 2.0], [3.0, 4.0]]))
        s.save()
        s2 = JacStore(path)
        self.assertTrue(np.allclose(s2.get('RING', 'arm'), [[1, 2], [3, 4]]))
        self.assertIsNone(s2.get('RAW', 'arm'))
        s2.drop('RING', 'arm')
        s2.save()
        self.assertIsNone(JacStore(path).get('RING', 'arm'))


if __name__ == '__main__':
    unittest.main(verbosity=1)
