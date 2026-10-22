"""ringdet.py(抗爪子遮挡的圆环识别)的测试：python3 -m unittest test_ringdet

圆环用合成的(1~6 环，外径 53~95mm，线宽 1.5mm)，爪子用真实画面 testdata/arm_cam_18.jpg 里的爪子盖在上面。"""
import math
import time
import unittest
from pathlib import Path

import numpy as np

try:
    import cv2
    HAVE_CV = True
except ImportError:
    HAVE_CV = False

HERE = Path(__file__).resolve().parent
DIAMS = (53, 58, 65, 75, 85, 95)


def rings(cx, cy, scale, bg=None, noise=0.0, seed=0):
    img = np.full((480, 640, 3), 232, np.uint8) if bg is None else bg.copy()
    for d in DIAMS:
        cv2.circle(img, (int(round(cx * 16)), int(round(cy * 16))), int(round(d / 2 * scale * 16)), (20, 20, 20),
                   max(2, int(round(1.5 * scale))), cv2.LINE_AA, shift=4)
    if noise:
        img = np.clip(img.astype(float) + np.random.default_rng(seed).normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img


def school_target(cx, cy, R, digit='2', material=None, mat_off=(6, 4), noise=3.0, seed=0):
    """校赛的靶：黑色圆盘(半径 R) + 中间白色满分区(半径 R/2，里面印着数字)。material = 物料颜色(BGR)，盖在中间(取回、码垛时)。"""
    img = np.full((480, 640, 3), 214, np.uint8)
    cv2.circle(img, (int(round(cx * 16)), int(round(cy * 16))), int(round(R * 16)), (38, 40, 42), -1, cv2.LINE_AA, shift=4)
    cv2.circle(img, (int(round(cx * 16)), int(round(cy * 16))), int(round(R * 8)), (222, 226, 224), -1, cv2.LINE_AA, shift=4)
    if digit:
        fs, th = R / 40.0, max(2, int(R / 14))
        (tw, tht), _ = cv2.getTextSize(digit, cv2.FONT_HERSHEY_SIMPLEX, fs, th)
        cv2.putText(img, digit, (int(cx - tw / 2), int(cy + tht / 2)), cv2.FONT_HERSHEY_SIMPLEX, fs, (38, 40, 42), th, cv2.LINE_AA)
    if material is not None:
        col = np.array(material, float)
        cv2.circle(img, (int(cx + mat_off[0]), int(cy + mat_off[1])), int(0.62 * R), tuple(int(v) for v in col * 0.8), -1, cv2.LINE_AA)
        cv2.circle(img, (int(cx + 2 * mat_off[0]), int(cy + 2 * mat_off[1])), int(0.62 * R), tuple(int(v) for v in col), -1, cv2.LINE_AA)
    if noise:
        img = np.clip(img.astype(float) + np.random.default_rng(seed).normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img


@unittest.skipUnless(HAVE_CV, '没有 opencv')
class SchoolTargetTests(unittest.TestCase):
    """校赛的粗黑环靶(黑环 + 中间白色满分区)。"""
    @classmethod
    def setUpClass(cls):
        import matdet
        cls.cam = cv2.imread(str(HERE / 'testdata' / 'arm_cam_18.jpg'))
        cls.claw = matdet.MaterialDetector({'claw_mask': '/nonexistent'}).auto_claw_mask(cv2.cvtColor(cls.cam, cv2.COLOR_BGR2HSV))

    def with_claw(self, img):
        img = img.copy()
        img[self.claw > 0] = self.cam[self.claw > 0]
        return img

    def find(self, img, c, r_expect=None):
        import ringdet
        res = ringdet.RingDetector().detect(img, self.claw, r_expect=r_expect)
        near = [r for r in res if math.hypot(r['center'][0] - c[0], r['center'][1] - c[1]) < 15]
        return (near[0], math.hypot(near[0]['center'][0] - c[0], near[0]['center'][1] - c[1])) if near else (None, None)

    def test_empty_target_near_and_under_claw(self):
        for R in (60, 90, 130):
            for c in ((320, 150), (337, 260), (337, 283), (380, 290)):
                best, e = self.find(self.with_claw(school_target(*c, R, seed=R)), c)
                self.assertIsNotNone(best, f'R={R} 中心 {c} 认不到')
                self.assertLess(e, 0.6)
                self.assertAlmostEqual(best['r_max'], R, delta=2.0)

    def test_material_covering_the_centre_needs_known_size(self):
        """取回/码垛时物料盖住白心，只剩黑环外圈：知道圆环大小(vclaw RING 量过)才认，认到的圆心要准。"""
        for mat in ((40, 40, 210), (28, 28, 30), (40, 200, 230)):           # 红、黑、黄色物料
            for R in (70, 100, 130):
                for c in ((337, 250), (337, 283), (337, 320)):
                    img = self.with_claw(school_target(*c, R, material=mat, seed=R))
                    best, e = self.find(img, c, r_expect=(0.85 * R, 1.2 * R))
                    self.assertIsNotNone(best, f'物料 {mat} R={R} 中心 {c} 认不到')
                    self.assertLess(e, 1.5)

    def test_coloured_material_alone_is_not_a_ring(self):
        """只有一个物料(没有圆环)：就算它的大小正好在圆环大小范围里，也不能当成圆环。"""
        import ringdet
        for mat in ((40, 40, 210), (190, 70, 20), (60, 170, 40)):
            img = np.full((480, 640, 3), 214, np.uint8)
            cv2.circle(img, (320, 170), 80, mat, -1, cv2.LINE_AA)
            res = ringdet.RingDetector().detect(self.with_claw(img), self.claw, r_expect=(68, 96))
            self.assertEqual(res, [], f'物料 {mat} 被当成了圆环')

    def test_real_frames_no_ring_even_with_size_hint(self):
        import matdet, ringdet
        for name in ('arm_cam_18.jpg', 'arm_cam_19.jpg'):
            im = cv2.imread(str(HERE / 'testdata' / name))
            claw = matdet.MaterialDetector({'claw_mask': '/nonexistent'}).auto_claw_mask(cv2.cvtColor(im, cv2.COLOR_BGR2HSV))
            for band in ((30, 70), (40, 250)):
                res = [r for r in ringdet.RingDetector().detect(im, claw, r_expect=band) if band[0] <= r['r_max'] <= band[1]]
                self.assertEqual(res, [], f'{name} {band}：{res}')


@unittest.skipUnless(HAVE_CV, '没有 opencv')
class RingDetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import matdet
        cls.cam = cv2.imread(str(HERE / 'testdata' / 'arm_cam_18.jpg'))
        cls.claw = matdet.MaterialDetector({'claw_mask': '/nonexistent'}).auto_claw_mask(cv2.cvtColor(cls.cam, cv2.COLOR_BGR2HSV))

    def setUp(self):
        import ringdet
        self.det = ringdet.RingDetector()

    def with_claw(self, img):
        img = img.copy()
        img[self.claw > 0] = self.cam[self.claw > 0]
        return img

    def nearest(self, res, cx, cy):
        self.assertTrue(res, '没认到圆环')
        best = min(res, key=lambda r: math.hypot(r['center'][0] - cx, r['center'][1] - cy))
        return best, math.hypot(best['center'][0] - cx, best['center'][1] - cy)

    def test_open_view(self):
        best, e = self.nearest(self.det.detect(rings(320.3, 160.7, 2.96)), 320.3, 160.7)
        self.assertLess(e, 0.3)
        self.assertGreaterEqual(best['n_circles'], 6)
        self.assertAlmostEqual(best['r_max'], (95 / 2 + 0.75) * 2.96, delta=3.0)

    def test_half_hidden_by_claw(self):
        """圆环中心就在爪子点附近：外面几圈下半截被爪子挡住。"""
        for scale in (1.6, 2.2, 2.96):
            for c in ((336.8, 282.9), (351.3, 297.7), (328.0, 261.0)):
                img = self.with_claw(rings(*c, scale, noise=5.0))
                best, e = self.nearest(self.det.detect(img, self.claw), *c)
                self.assertLess(e, 0.5, f'比例 {scale} 中心 {c} 偏了 {e:.2f}px')
                self.assertLess(best['coverage'], 0.9)                  # 确实挡住了一部分

    def test_old_method_fails_here(self):
        """同一个场景，原来只认闭合圆的办法认不到够大的圆环(说明上面那个测试有意义)。"""
        from vision import detect_rings_subpixel
        img = self.with_claw(rings(336.8, 282.9, 2.96, noise=5.0))
        self.assertEqual([r for r in detect_rings_subpixel(img) if r[3] > 0.8 * 47.5 * 2.96], [])

    def test_textured_background(self):
        img = self.with_claw(rings(300, 250, 2.2, bg=self.cam))
        best, e = self.nearest(self.det.detect(img, self.claw), 300, 250)
        self.assertLess(e, 0.5)

    def test_material_on_ring(self):
        """取回时圆环上放着物料(顶面圆，因为视差和圆环不完全同心)：圆环中心还要准。"""
        img = rings(320, 230, 2.96)
        cv2.circle(img, (326, 234), int(25 * 2.96), (40, 40, 200), -1, cv2.LINE_AA)
        best, e = self.nearest(self.det.detect(self.with_claw(img), self.claw), 320, 230)
        self.assertLess(e, 0.6)

    def test_single_circles_are_not_rings(self):
        """单个圆(物料顶面、被爪子切成两段的圆)只有一个半径，不算圆环。"""
        img = np.full((480, 640, 3), 232, np.uint8)
        cv2.circle(img, (200, 150), 48, (40, 40, 200), -1, cv2.LINE_AA)
        cv2.circle(img, (340, 300), 60, (40, 200, 40), -1, cv2.LINE_AA)            # 下半截在爪子里
        res = self.det.detect(self.with_claw(img), self.claw)
        self.assertEqual(res, [], res)

    def test_real_frames_have_no_ring_near_claw(self):
        for name in ('arm_cam_18.jpg', 'arm_cam_19.jpg'):
            im = cv2.imread(str(HERE / 'testdata' / name))
            import matdet
            claw = matdet.MaterialDetector({'claw_mask': '/nonexistent'}).auto_claw_mask(cv2.cvtColor(im, cv2.COLOR_BGR2HSV))
            res = self.det.detect(im, claw)
            self.assertFalse([r for r in res if r['r_max'] > 40], f'{name}：没有圆环却认出了 {res}')

    def test_partly_out_of_frame(self):
        best, e = self.nearest(self.det.detect(rings(600, 120, 2.2)), 600, 120)
        self.assertLess(e, 0.5)

    def test_call_format(self):
        out = self.det(rings(320, 160, 2.0))
        self.assertEqual(len(out), 1)
        x, y, rmean, rmax = out[0]
        self.assertLess(math.hypot(x - 320, y - 160), 0.3)
        self.assertGreater(rmax, rmean)

    def test_fast_enough(self):
        img = self.with_claw(rings(300, 250, 2.96, bg=self.cam))
        self.det.detect(img, self.claw)
        t0 = time.perf_counter()
        for _ in range(3):
            self.det.detect(img, self.claw)
        self.assertLess((time.perf_counter() - t0) / 3, 0.25)


if __name__ == '__main__':
    unittest.main()
