"""vision.py 的测试(用合成图片，不需要摄像头)：python3 test_vision.py
需要 numpy 和 opencv(树莓派上已经有)。"""
import math
import time
import unittest

try:
    import cv2
    import numpy as np
    HAVE_CV = True
except ImportError:      # 没装 opencv 就跳过依赖图片的测试
    HAVE_CV = False

from vision import Vision, _robust_mean


class FakeCam:
    def __init__(self, img=None):
        self.img = img

    def read(self):
        return None if self.img is None else self.img.copy()


def draw_rings(cx, cy, scale, size=(480, 640)):
    img = np.full((size[0], size[1], 3), 235, np.uint8)
    for d in (53, 58, 65, 75, 85, 95):                      # 1~6 环外径(毫米)，线宽 1.5mm
        cv2.circle(img, (int(round(cx)), int(round(cy))), int(round(d / 2 * scale)), (20, 20, 20),
                   max(2, int(round(1.5 * scale))), cv2.LINE_AA)
    return img


def quiet_vision(**kw):
    return Vision(log=lambda m: None, sleep=lambda s: None, **kw)


@unittest.skipUnless(HAVE_CV, '没有 opencv')
class RingTests(unittest.TestCase):
    def test_ring_error_matches_truth(self):
        img = draw_rings(340, 290, 2.96)
        v = quiet_vision(camera=FakeCam(img))
        du, dv = v.ring_error(n=3)
        self.assertAlmostEqual(du, 340 - 336.8, delta=0.6)
        self.assertAlmostEqual(dv, 290 - 282.9, delta=0.6)

    def test_subpixel_accuracy(self):
        rng = np.random.default_rng(0)
        worst = 0.0
        for _ in range(30):
            cx, cy = float(rng.integers(290, 380)), float(rng.integers(240, 330))
            v = quiet_vision(camera=FakeCam(draw_rings(cx, cy, 2.96)))
            p = v.ring_px(n=3)
            worst = max(worst, math.hypot(p[0] - cx, p[1] - cy))
        self.assertLess(worst, 0.6)

    def test_nearest_ring_is_chosen(self):
        # 比例缩小，让两个圆环同时在画面里(间隔 150 像素)
        img = draw_rings(200, 240, 1.0)
        img2 = draw_rings(350, 240, 1.0)
        img = np.minimum(img, img2)
        cfg = dict(px_per_mm=dict(RING=1.0), claw_px=dict(RING=[205, 240]))
        v = quiet_vision(camera=FakeCam(img), cfg=cfg)
        p = v.ring_px(n=3)
        self.assertLess(abs(p[0] - 200), 1.0)
        cfg['claw_px'] = dict(RING=[345, 240])
        v = quiet_vision(camera=FakeCam(img), cfg=cfg)
        p = v.ring_px(n=3)
        self.assertLess(abs(p[0] - 350), 1.0)

    def test_object_circles_are_not_rings(self):
        img = draw_rings(336, 283, 2.96)
        # 圆环旁边放一个物料(红色，大圆 r=74 + 顶面 r=44)，离圆环中心 60 像素，它自己的圆不应被当成圆环
        cv2.circle(img, (336 + 60, 283 + 5), 74, (40, 40, 200), -1, cv2.LINE_AA)
        cv2.circle(img, (336 + 60, 283 + 5), 44, (80, 80, 255), -1, cv2.LINE_AA)
        v = quiet_vision(camera=FakeCam(img))
        p = v.ring_px(n=3)
        self.assertIsNotNone(p)
        self.assertLess(math.hypot(p[0] - 336, p[1] - 283), 3.0)

    def test_blank_image_returns_none(self):
        v = quiet_vision(camera=FakeCam(np.full((480, 640, 3), 235, np.uint8)))
        self.assertIsNone(v.ring_px(n=3))
        self.assertIsNone(v.ring_error(n=3))

    def test_camera_returns_nothing(self):
        v = quiet_vision(camera=FakeCam(None))
        self.assertIsNone(v.ring_px(n=3))


class MaterialTests(unittest.TestCase):
    def test_outlier_rejected(self):
        seq = iter([(300.0, 200.0), (300.4, 200.2), (340.0, 260.0), (299.8, 199.9), (300.2, 200.1)])
        v = quiet_vision(camera=FakeCam(np.zeros((4, 4, 3), np.uint8) if HAVE_CV else None),
                         material_detector=lambda fr, cid: next(seq))
        p = v.material_px(1, n=5)
        self.assertAlmostEqual(p[0], 300.1, delta=0.4)
        self.assertAlmostEqual(p[1], 200.05, delta=0.4)

    def test_error_is_relative_to_claw(self):
        v = quiet_vision(camera=FakeCam(np.zeros((4, 4, 3), np.uint8) if HAVE_CV else None),
                         material_detector=lambda fr, cid: (346.8, 262.9))
        du, dv = v.material_error(1, n=3)
        self.assertAlmostEqual(du, 10.0, places=3)
        self.assertAlmostEqual(dv, -20.0, places=3)

    def test_too_few_detections_is_none(self):
        seq = iter([None, (1.0, 2.0), None, None, None])
        v = quiet_vision(camera=FakeCam(np.zeros((4, 4, 3), np.uint8) if HAVE_CV else None),
                         material_detector=lambda fr, cid: next(seq))
        self.assertIsNone(v.material_px(1, n=5))

    def test_wait_still(self):
        # 先在动，后停稳
        pts = [(100 + 8 * i, 100) for i in range(5)] + [(140.0, 100.0)] * 40
        it = iter(pts)
        v = Vision(camera=FakeCam(np.zeros((4, 4, 3), np.uint8) if HAVE_CV else None), log=lambda m: None,
                   sleep=lambda s: time.sleep(0.001), material_detector=lambda fr, cid: next(it))
        ok, last = v.wait_still(1, timeout_s=3.0, period_s=0.001)
        self.assertTrue(ok)
        self.assertAlmostEqual(last[0], 140.0, delta=0.5)

    def test_wait_still_times_out_when_moving(self):
        n = [0]

        def det(fr, cid):
            n[0] += 1
            return (100.0 + 5 * n[0], 100.0)
        v = Vision(camera=FakeCam(np.zeros((4, 4, 3), np.uint8) if HAVE_CV else None), log=lambda m: None,
                   sleep=lambda s: time.sleep(0.001), material_detector=det)
        ok, _ = v.wait_still(1, timeout_s=0.3, period_s=0.001)
        self.assertFalse(ok)


class MathTests(unittest.TestCase):
    def test_robust_mean(self):
        m = _robust_mean([(10, 10), (10.2, 9.8), (30, 30), (9.9, 10.1)], 3.0)
        self.assertAlmostEqual(m[0], 10.033, delta=0.05)


if __name__ == '__main__':
    unittest.main(verbosity=1)
