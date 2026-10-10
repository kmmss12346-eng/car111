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

    def test_lost_after_move_reports_no_stale_error(self):
        """动了以后看不到了：不能把动之前测的偏差当结果(会按没人测过的位置去夹)。"""
        rng = np.random.default_rng(17)
        pl = SimPlant(rng, err_mm=5.0)
        store = JacStore(None)
        store.put('RAW', 'arm', pl.A)                          # J 已知，第一次测完就动
        pl.lose_after = 1
        res = make_servo(pl, store=store).run('RAW', pl.measure, pl.scale, 1.0, allow_chassis=False)
        self.assertFalse(res.ok)
        self.assertEqual(res.err_mm, float('inf'))

    def test_small_step_is_rounded_up_to_min_step(self):
        """剩下一点点偏差：太小的一步(舵机动不了)放大到最小步，还能接着收敛。"""
        rng = np.random.default_rng(18)
        pl = SimPlant(rng, err_mm=0.8, noise_px=0.05, arm_noise_deg=0.0, arm_gain_err=0.0)
        store = JacStore(None)
        store.put('RAW', 'arm', pl.A)
        calls = []
        real = pl.arm_move

        def arm_move(d2, d1):
            calls.append((d2, d1))
            return real(d2, d1)
        pl.arm_move = arm_move
        res = make_servo(pl, store=store).run('RAW', pl.measure, pl.scale, 0.3, allow_chassis=False)
        for d2, d1 in calls:
            for v in (d2, d1):
                self.assertTrue(v == 0 or abs(v) >= 0.35 - 1e-9, calls)
        self.assertTrue(res.ok or res.err_mm < 1.0, res)

    def test_commanded_but_not_moved_is_reported(self):
        rng = np.random.default_rng(19)
        pl = SimPlant(rng, err_mm=6.0)
        store = JacStore(None)
        store.put('RAW', 'arm', pl.A)
        pl.dead_arm = True
        res = make_servo(pl, store=store).run('RAW', pl.measure, pl.scale, 1.0, allow_chassis=False)
        self.assertFalse(res.ok)
        self.assertIn('没动', res.reason)

    def test_not_moved_keeps_last_measured_error(self):
        """回读说手臂一点没动：上次测的偏差还是对的，报告它(不是 inf)，调用的地方才能按 accept_mm 照常夹。"""
        rng = np.random.default_rng(23)
        pl = SimPlant(rng, err_mm=2.0)
        store = JacStore(None)
        store.put('RAW', 'arm', pl.A)
        pl.dead_arm = True
        res = make_servo(pl, store=store).run('RAW', pl.measure, pl.scale, 1.0, allow_chassis=False)
        self.assertFalse(res.ok)
        self.assertTrue(math.isfinite(res.err_mm), res)
        self.assertAlmostEqual(res.err_mm, 2.0, delta=0.5)

    def test_isolated_zero_moves_do_not_abort(self):
        """偶尔一次没动(舵机停顿一下)，中间动成了：不算"连着两次没动"。"""
        rng = np.random.default_rng(24)
        pl = SimPlant(rng, err_mm=8.0)
        store = JacStore(None)
        store.put('RAW', 'arm', pl.A)
        real, n = pl.arm_move, [0]

        def flaky(d2, d1):
            n[0] += 1
            return (0.0, 0.0) if n[0] in (1, 3) else real(d2, d1)
        pl.arm_move = flaky
        res = make_servo(pl, store=store).run('RAW', pl.measure, pl.scale, 1.0, allow_chassis=False, max_iter=10)
        self.assertTrue(res.ok, res)

    def test_jacobian_too_small_is_corrected(self):
        """存的 J 比实际小一半多(观察高度改过没重新 vcal)：在线修正能把比例修回来。"""
        rng = np.random.default_rng(25)
        ok = 0
        for _ in range(40):
            pl = SimPlant(rng, err_mm=10.0)
            store = JacStore(None)
            store.put('RING', 'arm', 0.4 * pl.A)
            res = make_servo(pl, store=store).run('RING', pl.measure, pl.scale, 1.0, allow_chassis=False, max_iter=10)
            ok += res.ok
        self.assertGreaterEqual(ok, 34)

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

    def test_clearly_inside_tolerance_skips_the_confirm(self):
        """偏差明显在容差里(不到 confirm_skip × 容差)：不再拍一次确认，直接算对准；刚好在容差边上才复测。"""
        rng = np.random.default_rng(26)
        for err, tol, want in ((0.3, 1.2, 1), (1.0, 1.2, 2)):
            pl = SimPlant(rng, err_mm=err, noise_px=0.0)
            store = JacStore(None)
            store.put('RING', 'arm', pl.A)
            res = make_servo(pl, store=store).run('RING', pl.measure, pl.scale, tol, allow_chassis=False, confirm=True)
            self.assertTrue(res.ok, res)
            self.assertEqual(res.iters, 0)
            self.assertEqual(pl.n_measure, want, (err, tol))

    # ---------------- 测量滤波(filter / filt)
    def test_filter_needs_fewer_corrections_when_noisy(self):
        """噪声大(1.5 像素)：滤波开着修正次数少(不去追一次测量的噪声)，最后的真实误差不比不开大。"""
        res = {}
        for filt in (False, True):
            rng = np.random.default_rng(31)
            moves, errs = [], []
            for _ in range(80):
                pl = SimPlant(rng, err_mm=rng.uniform(2, 12), noise_px=1.5)
                store = JacStore(None)
                store.put('RING', 'arm', pl.A)
                r = make_servo(pl, store=store).run('RING', pl.measure, pl.scale, 1.2, allow_chassis=False, max_iter=8,
                                                    filt=filt, confirm=True)
                moves.append(pl.arm_moves)
                errs.append(pl.true_err_mm)
            res[filt] = (float(np.mean(moves)), float(np.mean(errs)))
        print(f'\n  [滤波] 噪声 1.5 像素：修正 {res[False][0]:.2f} → {res[True][0]:.2f} 次，真实误差 {res[False][1]:.2f} → {res[True][1]:.2f}mm')
        self.assertLess(res[True][0], res[False][0])
        self.assertLess(res[True][1], res[False][1] + 0.15)

    def test_filter_gate_recovers_from_a_wrong_jacobian(self):
        """J 的符号全反(滤波按错的 J 推算)：测到的和推算的差太多(filter_gate)就只信测的，照样能恢复。"""
        rng = np.random.default_rng(32)
        ok_n, N = 0, 60
        for _ in range(N):
            pl = SimPlant(rng)
            store = JacStore(None)
            store.put('RING', 'arm', -pl.A)
            res = make_servo(pl, store).run('RING', pl.measure, pl.scale, 1.5, allow_chassis=False, max_iter=10, timeout_s=60,
                                            filt=True)
            ok_n += res.ok
        self.assertGreaterEqual(ok_n, int(0.85 * N))

    def test_filter_off_uses_each_measurement_as_is(self):
        """不开滤波：每次的偏差就是那一次测到的(和以前一样)；开了：和测到的不完全一样(合进了推算的)。"""
        for filt in (False, True):
            rng = np.random.default_rng(33)
            pl = SimPlant(rng, err_mm=8.0, noise_px=1.0)
            store = JacStore(None)
            store.put('RING', 'arm', pl.A)
            seen = []
            real = pl.measure

            def measure():
                p = real()
                if p is not None:
                    seen.append(math.hypot(*p) / pl.scale)
                return p
            logged = []
            sv = VisualServo(pl, store=store, cfg=dict(near_avg=0.0, confirm=False), log=logged.append,
                             sleep=FakeClock().sleep, clock=FakeClock())
            sv.run('RING', measure, pl.scale, 0.3, allow_chassis=False, max_iter=5, filt=filt)
            got = [float(l.split('偏差 ')[1].split('mm')[0]) for l in logged if '] 偏差 ' in l]
            same = all(abs(a - b) < 0.006 for a, b in zip(got, seen))
            self.assertEqual(same, not filt, (filt, got, seen))
            if not filt:
                self.assertEqual(sv.meas_var, {})                     # 不开滤波不估计噪声

    # ---------------- 发散判断：一次跳变不算
    def test_single_noisy_jump_is_not_divergence(self):
        """偏差一下变大(一次测量跳了)：先再测一次，复测正常就不算发散，不丢 J、不重新探测。"""
        rng = np.random.default_rng(34)
        pl = SimPlant(rng, err_mm=10.0, noise_px=0.3)
        store = JacStore(None)
        store.put('RING', 'arm', pl.A)
        n = [0]
        real = pl.measure

        def measure():
            n[0] += 1
            p = real()
            if n[0] == 2:                                          # 第二次测量跳了 30 像素
                p = (p[0] + 30.0, p[1])
            return p
        logged = []
        sv = VisualServo(pl, store=store, log=logged.append, sleep=FakeClock().sleep, clock=FakeClock())
        res = sv.run('RING', measure, pl.scale, 1.0, allow_chassis=False, max_iter=8)
        text = '\n'.join(logged)
        self.assertTrue(res.ok, text)
        self.assertFalse(res.probed, text)
        self.assertIn('再测一次', text)
        self.assertNotIn('J 可能不对', text)
        self.assertIsNotNone(store.get('RING', 'arm'))

    def test_divergence_never_deletes_the_stored_jacobian_file(self):
        """真的发散(目标自己一直在跑)：这次运行里不用这组 J(内存里停用)，但 servo_cal.json 里存的不删。"""
        import os
        import tempfile
        path = os.path.join(tempfile.mkdtemp(), 'cal.json')
        rng = np.random.default_rng(35)
        pl = SimPlant(rng, err_mm=6.0, noise_px=0.2)
        store = JacStore(path)
        store.put('RING', 'arm', pl.A)
        store.save()
        k = [0]
        real = pl.measure

        def running_away():
            k[0] += 1
            pl.p = pl.p * 1.8 + 3.0                                 # 每测一次目标就跑远一大截
            return real()
        sv = VisualServo(pl, store=store, log=lambda m: None, sleep=FakeClock().sleep, clock=FakeClock())
        res = sv.run('RING', running_away, pl.scale, 1.0, allow_chassis=False, max_iter=10)
        self.assertFalse(res.ok)
        self.assertIsNone(store.get('RING', 'arm'))                # 这次运行里不用了
        self.assertTrue(np.allclose(JacStore(path).get('RING', 'arm'), pl.A))   # 文件里的还在
        store.put('RING', 'arm', pl.A)                             # 重新测好的 J 放回来就照常用
        self.assertIsNotNone(store.get('RING', 'arm'))

    # ---------------- 手臂能偏的范围 / 横移 / 超时
    def test_arm_range_is_respected(self):
        """手臂已经提前伸出去了：这次只能往一边再伸一点(arm_range)，修正不能超出这个范围。"""
        rng = np.random.default_rng(36)
        pl = SimPlant(rng, err_mm=30.0)
        store = JacStore(None)
        store.put('RING', 'arm', pl.A)
        sv = make_servo(pl, store=store)
        sv.run('RING', pl.measure, pl.scale, 1.0, allow_chassis=False, max_iter=8, arm_range={'id2': (-20.0, 5.0)})
        self.assertGreaterEqual(sv.dev[0], -20.0 - 0.5)
        self.assertLessEqual(sv.dev[0], 5.0 + 0.5)
        self.assertEqual(sv._arm_range, {})                        # 只管这一次

    def test_small_residual_strafes_when_the_arm_is_at_its_end(self):
        """'Fs'：手臂伸缩已经到头，剩下几毫米(比 chassis_min_mm 小)也让车轮横着挪，不卡在那儿。"""
        rng = np.random.default_rng(37)
        ok = 0
        for _ in range(20):
            pl = SimPlant(rng, err_mm=4.0, noise_px=0.2, ch_noise_mm=0.3)
            r = pl.A[:, 0] / np.linalg.norm(pl.A[:, 0])
            pl.p = r * float(np.linalg.norm(pl.p))                  # 偏差全在"远近"方向(ID2 管的那个方向)
            store = JacStore(None)
            store.put('RING', 'arm', pl.A)
            store.put('RING', 'ch', pl.B)
            sv = make_servo(pl, store=store)
            res = sv.run('RING', pl.measure, pl.scale, 1.0, allow_chassis=True, chassis_axes='Fs', max_iter=6,
                         chassis_s_range=(-20.0, 20.0), arm_range={'id2': (0.0, 0.0)})   # 伸缩一点也动不了
            ok += res.ok
            self.assertGreater(pl.ch_moves, 0)
        self.assertGreaterEqual(ok, 18)

    def test_probing_time_does_not_count_against_the_timeout(self):
        """第一次要探测 J(动几下)：探测用的时间不算进对准超时(timeout_s)，不会刚测完 J 就超时。"""
        rng = np.random.default_rng(38)
        ok = 0
        for _ in range(20):
            pl = SimPlant(rng, err_mm=8.0)
            clock = FakeClock()
            sv = VisualServo(pl, store=JacStore(None), cfg=dict(settle_arm_s=0.6), log=lambda m: None, sleep=clock.sleep, clock=clock)
            res = sv.run('RING', pl.measure, pl.scale, 1.0, allow_chassis=False, max_iter=8, timeout_s=2.5)
            self.assertTrue(res.probed)
            ok += res.ok
        self.assertGreaterEqual(ok, 18)

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
