"""reloc.py(雷达重定位，快速 ICP)的测试：合成的场地点云上测精度、速度和各道门槛。python3 -m unittest test_reloc"""
import math
import time
import unittest

import numpy as np

import reloc

# 合成场景：场地四周的墙、场外的桌子/墙、二维码板、三根障碍圆柱、原料转盘
SEGS = [((0, 0), (2400, 0)), ((2400, 0), (2400, 2400)), ((2400, 2400), (0, 2400)), ((0, 2400), (0, 0)),
        ((-600, -500), (3000, -500)), ((3000, -500), (3000, 1000)), ((2330, 950), (2330, 1450)),
        ((-400, 3000), (1500, 3100))]
CIRC = [(430, 265, 25), (2240, 1690, 25), (285, 2060, 25), (1200, 2480, 150)]


def scan(pose, n=720, noise=3.0, rmax=4500.0, seed=1, segs=SEGS, circ=CIRC):
    """从 pose(世界)看出去的一圈点，返回车身坐标(x 朝车头、y 朝车左)。"""
    rng = np.random.default_rng(seed)
    x0, y0, yaw = pose
    out = []
    for k in range(n):
        a = math.radians(yaw) + 2 * math.pi * k / n
        dx, dy = math.cos(a), math.sin(a)
        best = rmax
        for (ax, ay), (bx, by) in segs:
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = ((ax - x0) * ey - (ay - y0) * ex) / den
            u = ((ax - x0) * dy - (ay - y0) * dx) / den
            if t > 150 and 0 <= u <= 1:
                best = min(best, t)
        for cx, cy, r in circ:
            fx, fy = x0 - cx, y0 - cy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - r * r
            disc = b * b - c
            if disc >= 0:
                t = -b - math.sqrt(disc)
                if t > 150:
                    best = min(best, t)
        if best < rmax:
            d = best + rng.normal(0, noise)
            out.append((x0 + d * dx, y0 + d * dy))
    W = np.array(out)
    return reloc.rotate(W - [x0, y0], -yaw)


START = (2250.0, 150.0, 90.0)
SECOND = (2208.0, 233.0, 30.0)


def make_ref(start=START, second=SECOND):
    P1 = reloc.thin(scan(start, seed=1), 20, 2000)
    P2 = reloc.thin(scan(second, seed=2), 20, 2000)
    return reloc.Reference(start, [P1, reloc.body_to_frame(P2, second, start)])


class FrameTests(unittest.TestCase):
    def test_rel_world_roundtrip(self):
        o = (2250.0, 150.0, 90.0)
        for p in [(2100.0, 1200.0, 90.0), (340.0, 1200.0, -90.0), (1200.0, 2100.0, 180.0)]:
            q = reloc.world_of(reloc.rel_of(p, o), o)
            self.assertAlmostEqual(q[0], p[0], places=6)
            self.assertAlmostEqual(q[1], p[1], places=6)
            self.assertAlmostEqual(reloc.wrap180(q[2] - p[2]), 0.0, places=6)
        # 出发时车头朝北：前方 100mm = 世界里往北 100mm
        self.assertEqual(tuple(round(v, 6) for v in reloc.world_of((0, 100, 0), o)), (2250.0, 250.0, 90.0))

    def test_body_points_mount_and_self_echo(self):
        c = dict(sdk_y_sign=-1, lidar_yaw_deg=90.0, lidar_forward_mm=20.0, lidar_left_mm=130.0,
                 car_length_mm=290, car_width_mm=260, self_echo_pad_mm=120)

        def cart(pol):
            return [(d * math.cos(math.radians(a)), d * math.sin(math.radians(a))) for a, d, *_ in pol]
        frames = [{'points': [(0.0, 1010.0, 9), (0.0, 1013.0, 9), (180.0, 160.0, 9), (90.0, 5000.0, 9), (45.0, 100.0, 9)]}]
        B = reloc.body_points(frames, c, cartesian=cart)
        # (0°,1010) 雷达坐标 (1010,0) -> y 取反 -> 转 90° (0,1010) -> + 安装位置 (20,1140)；1013 和它在同一个 20mm 格子里；
        # (180°,160) 落在车身上(自己的回波)，5000 太远，100 太近，都不要
        self.assertEqual(len(B), 1)
        self.assertAlmostEqual(B[0][0], 20.0, places=3)
        self.assertAlmostEqual(B[0][1], 1140.0, places=3)


class GridTests(unittest.TestCase):
    def test_nearest_matches_brute_force(self):
        rng = np.random.default_rng(3)
        ref = reloc.thin(rng.uniform(-2000, 2000, (3000, 2)), 20)
        g = reloc.RefGrid(ref, cell=20, reach=320)
        Q = rng.uniform(-2100, 2100, (2000, 2))
        j, d = g.nearest(Q)
        bd = np.sqrt(((Q[:, None, :] - ref[None, :, :]) ** 2).sum(-1)).min(1)
        near = bd < 300
        self.assertTrue(np.all(d[near] >= bd[near] - 1e-6))
        err = d[near] - bd[near]
        self.assertGreater(float((err < 0.5).mean()), 0.97, '最近点表找到的基本都是真正最近的点')
        self.assertLess(float(err.max()), 25.0)          # 偶尔差一点(不超过一格的对角线)，ICP 不在乎
        self.assertTrue(np.all(np.isinf(d[bd > 400])), 'reach 以外的点找不到')

    def test_empty_reference(self):
        g = reloc.RefGrid(np.empty((0, 2)))
        j, d = g.nearest([[0, 0]])
        self.assertTrue(np.isinf(d[0]))


class IcpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        t = time.monotonic()
        cls.ref = make_ref()
        cls.build_s = time.monotonic() - t

    def test_accuracy_at_every_stop(self):
        cases = [((2100, 1200, 90), (2160, 1110, 87)), ((1200, 2100, 180), (1290, 2180, 183)),
                 ((340, 1200, -90), (400, 1300, -87)), ((1200, 340, 0), (1100, 400, 4)),
                 ((2250, 400, 90), (2230, 450, 91)), ((2250, 150, 90), (2250, 150, 90)),
                 ((2250, 2250, 180), (2240, 2230, 181))]
        for true, plan in cases:
            cur = reloc.thin(scan(true, seed=5), 20, 600)
            r = self.ref.measure(cur, plan)
            self.assertTrue(r['ok'], f'{true}: {r["why"]}')
            e = math.hypot(r['pose'][0] - true[0], r['pose'][1] - true[1])
            self.assertLess(e, 6.0, f'{true}: 位置差 {e:.1f}mm')
            self.assertLess(abs(reloc.wrap180(r['pose'][2] - true[2])), 0.4, f'{true}: 车头差')

    def test_fast_enough(self):
        cur = reloc.thin(scan((1200, 2100, 180), seed=6), 20, 600)
        t = time.monotonic()
        for _ in range(5):
            r = self.ref.measure(cur, (1250, 2060, 182))
        dt = (time.monotonic() - t) / 5
        self.assertTrue(r['ok'])
        self.assertLess(dt, 0.25, f'一次对齐 {dt:.3f} 秒(树莓派会慢几倍)')
        self.assertLess(self.build_s, 2.0, f'建参考点云和最近点表 {self.build_s:.2f} 秒')

    def test_no_init_tries_three_angles(self):
        cur = reloc.thin(scan((2250, 170, 92), seed=7), 20, 600)
        r = self.ref.measure(cur, (2250, 150, 90), use_init=False)
        self.assertTrue(r['ok'], r['why'])
        self.assertLess(math.hypot(r['pose'][0] - 2250, r['pose'][1] - 170), 6)

    def test_gate_shift_too_far(self):
        # 推算位置和真实差 300mm：对齐的结果离推算位置超过 reloc_max_shift_mm，不信
        cur = reloc.thin(scan((2100, 1500, 90), seed=8), 20, 600)
        r = self.ref.measure(cur, (2100, 1200, 90), cfg=dict(reloc_max_shift_mm=150))
        self.assertFalse(r['ok'])

    def test_gate_yaw(self):
        res = dict(theta_deg=7.0, tx=0.0, ty=0.0, rms=5.0, inlier=0.9)
        self.assertIn('角度', reloc.judge(res, (0.0, 0.0, 0.0), max_deg=5.0))
        self.assertEqual(reloc.judge(res, (4.0, 0.0, 0.0), max_deg=5.0), '')

    def test_gate_inlier_and_rms(self):
        self.assertIn('对上的点只有', reloc.judge(dict(theta_deg=0, tx=0, ty=0, rms=5.0, inlier=0.3)))
        self.assertIn('残差', reloc.judge(dict(theta_deg=0, tx=0, ty=0, rms=20.0, inlier=0.9)))

    def test_changed_scene_is_rejected(self):
        # 场地完全变了(只有几根随机的墙)：对上的点太少，不信
        segs = [((100, 100), (900, 300)), ((1500, 2000), (1700, 900))]
        cur = reloc.thin(scan((1200, 1200, 0), seed=9, segs=segs, circ=[]), 20, 600)
        r = self.ref.measure(cur, (1200, 1200, 0))
        self.assertFalse(r['ok'])

    def test_too_few_points(self):
        r = self.ref.measure(np.zeros((10, 2)) + 500, (1200, 1200, 0))
        self.assertFalse(r['ok'])
        self.assertIn('点太少', r['why'])


class RelocalizeTests(unittest.TestCase):
    def fake(self, poses):
        it = iter(poses)

        def m(pp):
            p = next(it)
            if p is None:
                return dict(ok=False, why='对不上', pose=None, t=0.01)
            return dict(ok=True, why='', pose=p, rms=5.0, inlier=0.9, t=0.01)
        return m

    def test_small_fix_one_measurement(self):
        r = reloc.relocalize(self.fake([(1010.0, 1000.0, 90.5)]), (1000.0, 1000.0, 90.0))
        self.assertTrue(r['ok'])
        self.assertEqual(r['n_meas'], 1)
        self.assertAlmostEqual(r['dx'], 10.0)
        self.assertAlmostEqual(r['dyaw'], 0.5)

    def test_big_fix_needs_two_agreeing(self):
        r = reloc.relocalize(self.fake([(1080.0, 1000.0, 90.0), (1084.0, 1002.0, 90.4)]), (1000.0, 1000.0, 90.0))
        self.assertTrue(r['ok'])
        self.assertEqual(r['n_meas'], 2)
        self.assertAlmostEqual(r['pose'][0], 1082.0)
        self.assertAlmostEqual(r['pose'][2], 90.2)

    def test_big_fix_disagreeing_is_rejected(self):
        r = reloc.relocalize(self.fake([(1080.0, 1000.0, 90.0), (1040.0, 1000.0, 90.0)]), (1000.0, 1000.0, 90.0))
        self.assertFalse(r['ok'])
        self.assertIn('不一致', r['why'])
        r = reloc.relocalize(self.fake([(1080.0, 1000.0, 90.0), None]), (1000.0, 1000.0, 90.0))
        self.assertFalse(r['ok'])

    def test_failure_passes_reason(self):
        r = reloc.relocalize(self.fake([None]), (1000.0, 1000.0, 90.0))
        self.assertFalse(r['ok'])
        self.assertEqual(r['why'], '对不上')
        self.assertEqual(r['dx'], 0.0)

    def test_end_to_end_with_reference(self):
        ref = make_ref()
        true = (1200.0, 340.0, 0.0)
        cur = reloc.thin(scan(true, seed=11), 20, 600)
        r = reloc.relocalize(lambda pp: ref.measure(cur, pp), (1140.0, 300.0, 2.0))
        self.assertTrue(r['ok'], r['why'])
        self.assertEqual(r['n_meas'], 2)                 # 修正量 72mm > 50mm：测两次
        self.assertLess(math.hypot(r['pose'][0] - true[0], r['pose'][1] - true[1]), 6)


if __name__ == '__main__':
    unittest.main()
