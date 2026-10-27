"""vision.py 的测试(用合成图片，不需要摄像头)：python3 test_vision.py
需要 numpy 和 opencv(树莓派上已经有)。"""
import math
import os
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

try:
    import cv2
    import numpy as np
    HAVE_CV = True
except ImportError:      # 没装 opencv 就跳过依赖图片的测试
    HAVE_CV = False

from vision import Camera, Vision, VisionError, _robust_mean, usb_cameras, video_users


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

    def test_ring_half_hidden_by_claw(self):
        """圆环中心在爪子点上、外面几圈被爪子挡住一截：还能认到，而且准(爪子从真实画面里自动找)。"""
        from pathlib import Path
        import matdet
        cam = cv2.imread(str(Path(__file__).resolve().parent / 'testdata' / 'arm_cam_18.jpg'))
        claw = matdet.MaterialDetector({'claw_mask': '/nonexistent'}).auto_claw_mask(cv2.cvtColor(cam, cv2.COLOR_BGR2HSV))
        img = draw_rings(336.8, 282.9, 2.96)
        img[claw > 0] = cam[claw > 0]
        v = quiet_vision(camera=FakeCam(img), cfg=dict(matdet={'claw_mask': '/nonexistent'}))
        du, dv = v.ring_error(n=3)
        self.assertLess(math.hypot(du, dv), 0.6)
        self.assertIsNotNone(v.last_claw)
        self.assertAlmostEqual(v.last_ring_rmax, 48.25 * 2.96, delta=4.0)
        v2 = quiet_vision(camera=FakeCam(img), cfg=dict(ring_detector='contour'))
        self.assertIsNone(v2.ring_px(n=3))                      # 原来的办法认不到

    def test_ring_size_filter_and_calibration(self):
        img = draw_rings(320, 200, 0.9)                          # 圆环比配置的 2.96 像素/毫米小很多(最外圈 43 像素)
        v = quiet_vision(camera=FakeCam(img))
        self.assertIsNone(v.ring_px(n=3))                        # 按配置的大小过滤掉了
        p = v.ring_px(n=3, any_size=True)                        # vclaw RING 第一次：不限大小
        self.assertLess(math.hypot(p[0] - 320, p[1] - 200), 0.6)
        r = v.last_ring_rmax
        self.assertAlmostEqual(r, 48.25 * 0.9, delta=2.0)
        v = quiet_vision(camera=FakeCam(img), cfg=dict(ring_rmax_cal=r, ring_line_mm=1.5))   # 量过以后按量到的大小过滤
        self.assertIsNotNone(v.ring_px(n=3))
        self.assertAlmostEqual(v.scale('RING'), 2 * r / 96.5, places=6)   # 6 条细线的靶：最外圈半径是线的外边
        self.assertAlmostEqual(v.scale('RING'), 0.9, delta=0.03)          # 和画图用的真实比例一致
        self.assertAlmostEqual(quiet_vision().scale('RING'), 2.96)
        v = quiet_vision(camera=FakeCam(img), cfg=dict(ring_rmax_cal=50.0, ring_outer_diam_mm=100.0))
        self.assertAlmostEqual(v.scale('RING'), 1.0)                       # 校赛粗黑环靶：外径 100mm、半径 50 像素

    def test_school_target_with_material_after_calibration(self):
        """校赛粗黑环靶上放着物料(取回/码垛)：vclaw RING 量过大小以后，Vision 会让识别只看外圈也算。"""
        import tempfile
        from pathlib import Path
        import matdet
        from test_ringdet import school_target
        cam = cv2.imread(str(Path(__file__).resolve().parent / 'testdata' / 'arm_cam_18.jpg'))
        claw = matdet.MaterialDetector({'claw_mask': '/nonexistent'}).auto_claw_mask(cv2.cvtColor(cam, cv2.COLOR_BGR2HSV))
        img = school_target(337, 283, 100, material=(40, 40, 210))
        img[claw > 0] = cam[claw > 0]
        with tempfile.TemporaryDirectory() as td:
            base = dict(matdet={'claw_mask': '/nonexistent'}, live_view=str(Path(td) / 'live.jpg'))
            v = quiet_vision(camera=FakeCam(img), cfg=base)
            self.assertIsNone(v.ring_px(n=3))                           # 没量过大小：只看到一圈不算
            v = quiet_vision(camera=FakeCam(img), cfg=dict(base, ring_rmax_cal=100.0))
            p = v.ring_px(n=3)
            self.assertIsNotNone(p)
            self.assertLess(math.hypot(p[0] - 337, p[1] - 283), 1.5)
            self.assertTrue((Path(td) / 'live.jpg').exists())           # 实时画面写出来了(给 vview.py 看)

    def test_overlay_and_live_view(self):
        import tempfile
        from pathlib import Path
        img = draw_rings(330, 200, 2.0)
        with tempfile.TemporaryDirectory() as td:
            path = str(Path(td) / 'live.jpg')
            v = quiet_vision(camera=FakeCam(img), cfg=dict(live_view=path, matdet={'claw_mask': '/nonexistent'}))
            self.assertIsNotNone(v.ring_px(n=2))
            out = cv2.imread(path)
            self.assertEqual(out.shape, img.shape)
            self.assertGreater(int(np.abs(out.astype(int) - img.astype(int)).sum()), 0)   # 画了东西
            v.cfg['live_view'] = ''                                     # 关掉：不写
            os_mtime = Path(path).stat().st_mtime
            v._live_t = 0.0
            v.ring_px(n=2)
            self.assertEqual(Path(path).stat().st_mtime, os_mtime)
            out2 = v.overlay(img, 'RAW', None, color_id=1)
            self.assertEqual(out2.shape, img.shape)

    def test_watching_plate_writes_live_view(self):
        """(10-27 用户"这个画面不是实时的已经卡住了")原料区等原料盘停、下爪前看(material_errors)也写实时画面：
        每个认出的物料画圈、写颜色和离爪子点多少毫米；live_view='' 时不写。"""
        import tempfile
        from pathlib import Path
        img = np.full((480, 640, 3), 200, np.uint8)
        cv2.circle(img, (380, 250), 60, (40, 200, 230), -1, cv2.LINE_AA)        # 黄色物料
        with tempfile.TemporaryDirectory() as td:
            path = str(Path(td) / 'live.jpg')
            v = quiet_vision(camera=FakeCam(img), cfg=dict(live_view=path, matdet={'claw_mask': '/nonexistent'}))
            cu, cv_ = v.claw('RAW')
            errs = v.material_errors([2, 1])
            self.assertIsNone(errs[1])
            self.assertAlmostEqual(errs[2][0], 380 - cu, delta=2.0)
            self.assertAlmostEqual(errs[2][1], 250 - cv_, delta=2.0)
            out = cv2.imread(path)
            self.assertEqual(out.shape, img.shape)
            diff = np.abs(out.astype(int) - img.astype(int)).sum(axis=2)
            self.assertGreater(int(diff[230:270, 360:400].sum()), 0)            # 圆心画了红叉
            self.assertGreater(int(diff[200:240, 392:520].sum()), 0)            # 旁边写了 YELLOW ..mm
            mm = math.hypot(380 - cu, 250 - cv_) / v.scale('RAW')
            lab = v.overlay(img, 'RAW', None, color_id='watch', pts=[(380.0, 250.0, 2, 60.0)])
            self.assertEqual(lab.shape, img.shape)
            self.assertGreater(mm, 0.0)
            v2 = quiet_vision(camera=FakeCam(img), cfg=dict(live_view='', matdet={'claw_mask': '/nonexistent'}))
            p2 = str(Path(td) / 'none.jpg')
            v2.material_errors([2])
            self.assertFalse(Path(p2).exists())

    def test_vview_reads_the_same_file(self):
        import vview
        self.assertEqual(vview.DEFAULT_PATH, Vision.DEFAULT['live_view'])

    def test_blank_image_returns_none(self):
        v = quiet_vision(camera=FakeCam(np.full((480, 640, 3), 235, np.uint8)))
        self.assertIsNone(v.ring_px(n=3))
        self.assertIsNone(v.ring_error(n=3))

    def test_camera_returns_nothing(self):
        v = quiet_vision(camera=FakeCam(None))
        self.assertIsNone(v.ring_px(n=3))

    @staticmethod
    def _two_rings():
        """现场的比例(相邻两个环隔 219 像素 = 150mm)：爪子下面一个环，右边还有一个。"""
        img = np.minimum(draw_rings(300, 250, 1.46), draw_rings(300 + 219, 250, 1.46))
        return img, dict(claw_px=dict(RING=[310.0, 245.0]), ring_rmax_hint=48.25 * 1.46)

    @staticmethod
    def _spy_roi(v, blind_first=False):
        rois, orig = [], v._rings_in_frame

        def spy(fr, any_size=False, roi=None):
            rois.append(roi)
            if blind_first and len(rois) == 1:
                return []                                    # 第一块里"没找到"(比如那一帧刚好被挡住)
            return orig(fr, any_size, roi)
        v._rings_in_frame = spy
        return rois

    def test_ring_search_near_claw_gives_the_same_centre(self):
        """对准时只在爪子点附近那一块画面里找(快好几倍)：找到的圆心和在整幅画面里找的一样，旁边那个环不会被当成目标。"""
        img, cfg = self._two_rings()
        full = quiet_vision(camera=FakeCam(img), cfg=cfg).ring_px(n=2)
        v = quiet_vision(camera=FakeCam(img), cfg=cfg)
        rois = self._spy_roi(v)
        near = v.ring_px(n=2, max_px=100.0)
        self.assertIsNotNone(rois[0])                        # 真的只看了一块
        x0, y0, x1, y1 = rois[0]
        self.assertLess((x1 - x0) * (y1 - y0), 0.6 * 640 * 480)
        self.assertLess(math.hypot(near[0] - full[0], near[1] - full[1]), 0.3)
        self.assertLess(math.hypot(near[0] - 300, near[1] - 250), 0.6)
        du, dv = v.ring_error(n=2, max_px=100.0)
        self.assertLess(math.hypot(du + 10.0, dv - 5.0), 0.6)

    def test_ring_search_falls_back_when_the_small_crop_misses(self):
        """对准中上一次刚认到过：先在"上次的位置到爪子点"那一小块里找；没找到就用整块(max_px 那么大)再找，这次测量剩下的帧也用整块。"""
        img, cfg = self._two_rings()
        v = quiet_vision(camera=FakeCam(img), cfg=cfg)
        v._ring_track = ((302.0, 249.0), time.monotonic())   # 上一次刚认到，离爪子点很近
        rois = self._spy_roi(v, blind_first=True)
        p = v.ring_px(n=2, max_px=100.0)
        self.assertIsNotNone(p)
        self.assertLess(math.hypot(p[0] - 300, p[1] - 250), 0.6)
        self.assertEqual(len(rois), 3, rois)
        area = [(r[2] - r[0]) * (r[3] - r[1]) for r in rois]
        self.assertLess(area[0], area[1])                    # 先找小的一块
        self.assertEqual(rois[2], rois[1])                   # 第二帧直接用整块

    def test_two_frame_measurement_accepts_one_detection(self):
        """对准时一次测量只拍 2 帧：认到 1 帧就算(要快)；3 帧以上还是要一半以上认到。"""
        img = draw_rings(340, 290, 2.96)
        blank = np.full_like(img, 235)

        class SeqCam:
            def __init__(self, frames):
                self.frames = list(frames)

            def read(self):
                return self.frames.pop(0).copy() if self.frames else None
        p = quiet_vision(camera=SeqCam([blank, img])).ring_px(n=2)
        self.assertIsNotNone(p)
        self.assertLess(math.hypot(p[0] - 340, p[1] - 290), 0.6)
        self.assertIsNone(quiet_vision(camera=SeqCam([blank, img, blank])).ring_px(n=3))


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


def pick_scene(top, r_top, color_bgr, R=None, seed=0):
    """校赛圆环(黑盘+白心)上立着一个物料：顶面圆心 top、半径 r_top；侧面(暗一点)在顶面下方露出来
    (和真实画面 arm_cam_18/19 一样：摄像头在爪子后上方往前下方看)；下面盖上真实画面里的爪子。R=None：只有物料没有圆环。"""
    from pathlib import Path
    import matdet
    from test_ringdet import school_target
    cam = cv2.imread(str(Path(__file__).resolve().parent / 'testdata' / 'arm_cam_18.jpg'))
    claw = matdet.MaterialDetector({'claw_mask': '/nonexistent'}).auto_claw_mask(cv2.cvtColor(cam, cv2.COLOR_BGR2HSV))
    bot = np.array(top, float) + (0.0, 0.25 * r_top)                            # 底面(和圆环中心)在顶面下方
    img = school_target(bot[0], bot[1], R, digit='', noise=0) if R else np.full((480, 640, 3), 214, np.uint8)
    c = np.array(color_bgr, float)
    for t in np.linspace(0.0, 1.0, 10):                                          # 侧面
        x, y = bot + (np.array(top, float) - bot) * t
        cv2.circle(img, (int(round(x)), int(round(y))), int(round(r_top * (0.92 + 0.08 * t))), tuple(int(v) for v in c * 0.7), -1, cv2.LINE_AA)
    cv2.circle(img, (int(round(top[0] * 16)), int(round(top[1] * 16))), int(round(r_top * 16)), tuple(int(v) for v in c), -1, cv2.LINE_AA, shift=4)
    img = np.clip(img.astype(float) + np.random.default_rng(seed).normal(0, 3, img.shape), 0, 255).astype(np.uint8)
    img[claw > 0] = cam[claw > 0]
    return img, claw, cam


@unittest.skipUnless(HAVE_CV, '没有 opencv')
class PickTests(unittest.TestCase):
    """取回/码垛：圆环上立着的物料，认它的顶面圆心(claw_px.PICK 对准用)。"""
    COLS = {1: (40, 40, 210), 2: (40, 200, 230), 4: (60, 170, 40), 3: (190, 70, 20), 6: (230, 200, 120)}

    def test_material_on_ring_found_under_claw(self):
        for color, bgr in self.COLS.items():
            for top in ((337, 283), (330, 300), (320, 220), (345, 255)):
                img, _claw, cam = pick_scene(top, 84, bgr, R=140, seed=top[0])
                v = quiet_vision(camera=FakeCam(img), cfg=dict(matdet={'claw_mask': '/nonexistent'}, live_view=''))
                if color in (3, 6):                         # 蓝色：要先有一份干净的爪子区域(对准空圆环时记下的)
                    v.last_claw = v._pick_det().auto_claw_mask(cv2.cvtColor(cam, cv2.COLOR_BGR2HSV))
                    v.note_claw_clear()
                p = v.pick_px(color, n=3)
                self.assertIsNotNone(p, f'颜色 {color} 顶面 {top} 认不到')
                self.assertLess(math.hypot(p[0] - top[0], p[1] - top[1]), 2.0, f'颜色 {color} 顶面 {top}：{p}')
                self.assertAlmostEqual(v.last_pick_r, 84, delta=4)

    def test_pick_point_and_scale(self):
        img, _c, _cam = pick_scene((340, 260), 84, self.COLS[1], R=140)
        v = quiet_vision(camera=FakeCam(img), cfg=dict(matdet={'claw_mask': '/nonexistent'}, live_view=''))
        self.assertFalse(v.has_pick())
        self.assertIsNone(v.pick_error(1))                   # 还没量 PICK：不能按物料对准
        self.assertEqual(v.claw('PICK'), v.claw('RING'))      # 画图先用圆环的点
        v.set_pick((330.0, 250.0), 84.0)
        self.assertTrue(v.has_pick())
        du, dv = v.pick_error(1, n=3)
        self.assertAlmostEqual(du, 10.0, delta=1.5)
        self.assertAlmostEqual(dv, 10.0, delta=1.5)
        self.assertAlmostEqual(v.scale('PICK'), 2 * 84.0 / 50.0, places=3)
        v.set_pick(None)
        self.assertFalse(v.has_pick())

    def test_wrong_size_is_rejected_once_known(self):
        img, _c, _cam = pick_scene((340, 260), 84, self.COLS[1], R=140)
        v = quiet_vision(camera=FakeCam(img), cfg=dict(matdet={'claw_mask': '/nonexistent'}, live_view='', pick_r_px=50.0))
        self.assertIsNone(v.pick_px(1, n=3))

    def test_ring_centre_free(self):
        empty, _c, _cam = pick_scene((337, 250), 1, (214, 214, 214), R=140)
        v = quiet_vision(camera=FakeCam(empty), cfg=dict(matdet={'claw_mask': '/nonexistent'}, live_view='', ring_rmax_cal=140.0))
        self.assertIs(v.ring_centre_free(), True)
        full, _c, _cam = pick_scene((337, 250), 84, self.COLS[2], R=140)
        v = quiet_vision(camera=FakeCam(full), cfg=dict(matdet={'claw_mask': '/nonexistent'}, live_view='', ring_rmax_cal=140.0))
        self.assertIs(v.ring_centre_free(), False)

    def test_ring_centre_free_in_dim_light(self):
        """光线暗/曝光低：白心只有灰白(亮度 120~130)，也不能当成"放了东西"而不放。"""
        for b in (0.75, 0.6, 0.5):
            empty, _c, _cam = pick_scene((337, 250), 1, (214, 214, 214), R=140)
            dim = np.clip(empty.astype(float) * b, 0, 255).astype(np.uint8)
            v = quiet_vision(camera=FakeCam(dim), cfg=dict(matdet={'claw_mask': '/nonexistent'}, live_view='', ring_rmax_cal=140.0))
            self.assertIsNot(v.ring_centre_free(), False, f'亮度 x{b}：空圆环被当成有东西')
        for col in ((40, 40, 210), (30, 30, 30)):              # 真放着物料(红、黑)，暗一点也认得出
            full, _c, _cam = pick_scene((337, 250), 84, col, R=140)
            dim = np.clip(full.astype(float) * 0.6, 0, 255).astype(np.uint8)
            v = quiet_vision(camera=FakeCam(dim), cfg=dict(matdet={'claw_mask': '/nonexistent'}, live_view='', ring_rmax_cal=140.0))
            self.assertIs(v.ring_centre_free(), False, f'{col}')

    def test_ring_centre_taken_by_blue_material_next_to_claw(self):
        """白心里放着蓝色物料、挨着蓝色爪子：现找的爪子区域会把物料算进去(白心"看不见" -> 照常放 -> 砸上去)。
        用对准空圆环时记下的干净爪子区域，就认得出白心被占了。"""
        empty, _c, _cam = pick_scene((337, 250), 1, (214, 214, 214), R=140)
        full, _c, _cam = pick_scene((337, 250), 84, self.COLS[3], R=140)
        v = quiet_vision(camera=FakeCam(empty), cfg=dict(matdet={'claw_mask': '/nonexistent'}, live_view='', ring_rmax_cal=140.0))
        self.assertIsNotNone(v.ring_px(n=2))                    # 对准空圆环(会记下这一帧的爪子区域)
        v.note_claw_clear()
        v.camera.img = full
        self.assertIs(v.ring_centre_free(), False)

    def test_ring_size_seen_on_empty_ring_finds_covered_ring(self):
        """没做 vclaw RING：放物料时认到的空圆环大小记下来，取回时白心被盖住也认得出外圈。"""
        img, _c, _cam = pick_scene((337, 250), 84, self.COLS[1], R=140)
        v = quiet_vision(camera=FakeCam(img), cfg=dict(matdet={'claw_mask': '/nonexistent'}, live_view=''))
        self.assertIsNone(v.ring_px(n=3))
        v.note_ring_size(141.0)
        p = v.ring_px(n=3)
        self.assertIsNotNone(p)
        self.assertLess(math.hypot(p[0] - 337, p[1] - 250), 30.0)        # 圆环中心(地面)在物料顶面附近

    def test_overlay_pick(self):
        img, _c, _cam = pick_scene((340, 260), 84, self.COLS[1], R=140)
        v = quiet_vision(camera=FakeCam(img), cfg=dict(matdet={'claw_mask': '/nonexistent'}, live_view=''))
        p = v.pick_px(1, n=2)
        out = v.overlay(img, 'PICK', p, color_id=1)
        self.assertEqual(out.shape, img.shape)


def plate_scene(cx, cy, R=276, orbit=201, seed=0):
    """(10-20)原料盘：白色圆盘(外圈半径 R 像素，边上一圈深色)放在灰色地面上，盘上画两圈线、三个物料(红蓝绿)；
    圆心 (cx, cy) 可以在画面外(车停偏了)。实车示教时圆心在画面 (340, 471)，差不多在下边上。"""
    rng = np.random.default_rng(seed)
    img = np.full((480, 640, 3), (95, 105, 110), np.uint8)
    img = (img.astype(np.int16) + rng.normal(0, 6, img.shape)).clip(0, 255).astype(np.uint8)
    cv2.circle(img, (int(cx), int(cy)), int(R), (235, 235, 235), -1, lineType=cv2.LINE_AA)
    cv2.circle(img, (int(cx), int(cy)), int(R), (60, 60, 60), 3, lineType=cv2.LINE_AA)
    for r in (int(orbit * 0.5), int(orbit)):
        cv2.circle(img, (int(cx), int(cy)), r, (150, 150, 150), 1, lineType=cv2.LINE_AA)
    for ang, col in ((90, (255, 80, 30)), (210, (40, 40, 220)), (330, (40, 180, 40))):
        a = math.radians(ang)
        cv2.circle(img, (int(cx + orbit * math.cos(a)), int(cy - orbit * math.sin(a))), 49, col, -1, lineType=cv2.LINE_AA)
    return (img.astype(np.int16) + rng.normal(0, 2, img.shape)).clip(0, 255).astype(np.uint8)


@unittest.skipUnless(HAVE_CV, '没有 opencv')
class PlateSeekTests(unittest.TestCase):
    """(10-20 用户"偏差大一点也要一直识别")车停偏了，原料盘圆心跑出画面：粗找(Hough)要求圆心在画面里，认不出；
    给了 hints 就按露出来的那段外圈投票算圆心，再沿径向找边拟合(圆心在画面外也行)。"""

    def _v(self, img):
        v = quiet_vision(camera=FakeCam(img))
        v.cfg['live_view'] = None
        return v

    def test_center_inside_frame_found_as_before(self):
        v = self._v(plate_scene(340, 471))
        pl, _ = v.plate_scan([])
        self.assertIsNotNone(pl)
        self.assertLess(math.hypot(pl['c_now'][0] - 340, pl['c_now'][1] - 471), 3.0)
        self.assertFalse(pl.get('seek'))

    def test_center_off_frame_found_by_vote_then_tracked(self):
        for cx, cy in ((340, 560), (340, 640), (-60, 250), (-150, 250), (700, 300), (150, 600)):
            img = plate_scene(cx, cy, seed=cx + cy)
            v = self._v(img)
            pl, _ = v.plate_scan([])
            self.assertIsNone(pl, (cx, cy))                         # 照原来的办法认不出
            pl, _ = v.plate_scan([], hints=[(340.0, 471.0, 270.0)])   # 猜的圆心离真的很远也行(按投票)
            self.assertIsNotNone(pl, (cx, cy))
            self.assertTrue(pl.get('seek'))
            self.assertLess(math.hypot(pl['c_now'][0] - cx, pl['c_now'][1] - cy), 4.0, (cx, cy, pl['c_now']))
            self.assertLess(abs(pl['r_now'] - 276), 6.0)
            pl2, _ = v.plate_scan([])                                # 认出来以后照原来的办法跟得住(圆心在画面外)
            self.assertIsNotNone(pl2, (cx, cy))
            self.assertLess(math.hypot(pl2['c_now'][0] - cx, pl2['c_now'][1] - cy), 4.0)

    def test_little_arc_gives_coarse_center_only(self):
        """外圈只露出一小段(圆心在画面下边外 210 像素)：拟合不准，只报大概的圆心(coarse)，够知道往哪边挪。"""
        v = self._v(plate_scene(340, 690, seed=7))
        pl, _ = v.plate_scan([], hints=[(340.0, 471.0, 270.0)])
        self.assertIsNotNone(pl)
        self.assertTrue(pl.get('coarse'))
        self.assertLess(math.hypot(pl['c_now'][0] - 340, pl['c_now'][1] - 690), 15.0)

    def test_no_plate_nothing_found(self):
        """画面里没有原料盘(只有地面)：不会把地面纹理认成外圈。"""
        v = self._v(plate_scene(-2000, -2000, seed=3))
        pl, _ = v.plate_scan([], hints=[(340.0, 471.0, 270.0)])
        self.assertIsNone(pl)

    def test_wrong_guess_rejected(self):
        """猜的圆心错了(那里没有圆)：_plate_seek 不会拟合到地面纹理上(边不清楚)；猜得差不多就找得到。"""
        img = plate_scene(340, 660, seed=5)
        v = self._v(img)
        tt = v._turntable()
        fit = v._plate_seek(tt, img, [(292.0, 316.0, 276.0), (383.0, 312.0, 276.0)])
        self.assertIsNone(fit)
        fit = v._plate_seek(tt, img, [(330.0, 640.0, 276.0)])
        self.assertIsNotNone(fit)
        self.assertLess(math.hypot(fit.center_x_px - 340, fit.center_y_px - 660), 4.0)


class LatestFrameTests(unittest.TestCase):
    def test_fresh_frames_and_close(self):
        from vision import LatestFrame

        class FakeCap:                      # 100 帧/秒，grab 要等到下一帧
            def __init__(self):
                self.t0, self.k = time.monotonic(), 0

            def grab(self):
                nxt = self.t0 + (int((time.monotonic() - self.t0) * 100) + 1) / 100.0
                time.sleep(max(0.0, nxt - time.monotonic()))
                self.k += 1
                return True

            def retrieve(self):
                return True, np.full((2, 2, 3), self.k % 250, np.uint8)
        lf = LatestFrame(FakeCap(), 0.01)
        try:
            t_req = time.monotonic()
            fr, t = lf.read()
            self.assertIsNotNone(fr)
            self.assertGreaterEqual(t, t_req + 0.01)              # 一定是要的时候以后才拍的
            fr2, t2 = lf.read(after=t, fresh=False)
            self.assertGreater(t2, t)                              # 下一帧比上一帧新
        finally:
            lf.close()
        self.assertFalse(lf.th.is_alive())
        self.assertEqual(lf.read(timeout=0.05), (None, None))

    def test_one_bad_frame_does_not_kill_thread(self):
        from vision import LatestFrame

        class BadOnce:
            def __init__(self):
                self.n = 0

            def grab(self):
                time.sleep(0.01)
                return True

            def retrieve(self):
                self.n += 1
                if self.n == 1:
                    raise RuntimeError('坏帧')
                return True, np.zeros((2, 2, 3), np.uint8)
        lf = LatestFrame(BadOnce(), 0.01)
        try:
            fr, _t = lf.read(timeout=1.0)
            self.assertIsNotNone(fr)
            self.assertTrue(lf.th.is_alive())
        finally:
            self.assertTrue(lf.close())

    def test_ring_settings_reach_vision(self):
        from mission_hooks import vision_cfg, DEFAULTS, deep_merge
        cfg = deep_merge(DEFAULTS, {'ring_detector': 'contour', 'ring_outer_diam_mm': 85.0, 'ring_rmax_px': [50, 300]})
        v = Vision(vision_cfg(cfg), camera=object(), log=lambda m: None)
        self.assertEqual(v.cfg['ring_detector'], 'contour')
        self.assertEqual(v.cfg['ring_outer_diam_mm'], 85.0)
        self.assertEqual(v._ring_rmax_range(), (50.0, 300.0))


class FakeCap:
    """假的 cv2.VideoCapture：这个节点在 v4l.devs 里是 'ok' 才有画面('blank' = 打得开、没画面，像 UVC 元数据节点)；不在里面的打不开。"""

    def __init__(self, v4l, dev):
        self.dev, self.state, self.k = dev, v4l.devs.get(dev), 0
        self.dead = self.released = False

    def isOpened(self):
        return self.state is not None

    def set(self, *a):
        return True

    def grab(self):
        time.sleep(0.002)
        return self.state == 'ok' and not self.dead and not self.released

    def retrieve(self):
        self.k += 1
        return True, [self.dev, self.k]                     # 一"帧" = [哪个节点拍的, 第几帧]

    def read(self):
        return self.retrieve() if self.grab() else (False, None)

    def release(self):
        self.released = True


class FakeV4L:
    """假的 cv2(只有摄像头用到的那几样，不要真的 opencv)：devs = {节点: 'ok' / 'blank'}；opened = 依次开过哪些节点。
    unplug(节点) = USB 掉线：节点没了，开着它的再也读不到画面。"""
    CAP_V4L2, CAP_PROP_FOURCC, CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_HEIGHT, CAP_PROP_FPS, CAP_PROP_BUFFERSIZE = 200, 6, 3, 4, 5, 38

    def __init__(self, devs):
        self.devs, self.opened, self.caps = dict(devs), [], []

    @staticmethod
    def VideoWriter_fourcc(*c):
        return 0

    def VideoCapture(self, dev, api=None):
        self.opened.append(dev)
        self.caps.append(FakeCap(self, dev))
        return self.caps[-1]

    def unplug(self, dev):
        self.devs.pop(dev, None)
        for c in self.caps:
            c.dead = c.dead or c.dev == dev


class UsbCameraTests(unittest.TestCase):
    """(10-19)实车：USB 摄像头掉线又连上，/dev/video0 没了，回来的是 video1(取画面) + video2(元数据，没画面)；
    video10 以上是树莓派自己的编解码/ISP。用假的 cv2 和临时文件夹里假的 /dev/v4l/by-id、/sys/class/video4linux，不要真的摄像头。"""

    def setUp(self):
        td = tempfile.TemporaryDirectory()
        self.addCleanup(td.cleanup)
        self.td, self.logs = td.name, []
        self.by_id = os.path.join(self.td, 'by-id')
        self.sysfs = os.path.join(self.td, 'video4linux')            # 先不建：by-id 里没有就什么都找不到
        os.makedirs(self.by_id)

    def link(self, name, dev):
        os.symlink(dev, os.path.join(self.by_id, name))

    def usb_back_as_video1(self):
        """USB 重连以后 udev 建的链接：index0 -> video1(取画面)，index1 -> video2(元数据)。"""
        self.link('usb-HD_USB_Camera_SN01-video-index0', '/dev/video1')
        self.link('usb-HD_USB_Camera_SN01-video-index1', '/dev/video2')

    def camera(self, fake, **kw):
        """用假 cv2(fake)、只在临时文件夹里找 USB 摄像头的 Camera(打开时不真的等)。测完自动关。"""
        if 'cv2' in sys.modules:
            self.addCleanup(sys.modules.__setitem__, 'cv2', sys.modules['cv2'])
        else:
            self.addCleanup(sys.modules.pop, 'cv2', None)
        sys.modules['cv2'] = fake                                    # Camera 里 import cv2 拿到的是假的
        for name, new in (('usb_cameras', lambda: usb_cameras(self.by_id, self.sysfs)), ('video_users', lambda: [])):
            p = mock.patch(f'vision.{name}', new)
            p.start()
            self.addCleanup(p.stop)
        cam = Camera(kw.pop('device', '/dev/video0'), log=self.logs.append, sleep=lambda s: None, **kw)
        self.addCleanup(cam.close)
        return cam

    def test_usb_cameras_from_by_id(self):
        """by-id：只要 USB 摄像头 index0(取画面)的，同一个节点只算一次，按名字排。"""
        self.usb_back_as_video1()
        self.link('usb-HD_USB_Camera_SN01b-video-index0', '/dev/video1')      # 同一个节点两个名字：只算一次
        self.link('usb-Other_Cam_SN02-video-index0', '/dev/video3')
        self.link('platform-fe00b840.mailbox-video-index0', '/dev/video10')   # 不是 USB 的：不要
        self.assertEqual(usb_cameras(self.by_id, self.sysfs), ['/dev/video1', '/dev/video3'])
        self.assertEqual(usb_cameras(os.path.join(self.td, 'none'), self.sysfs), [])

    def test_usb_cameras_from_sysfs(self):
        """by-id 没有：扫 /sys/class/video4linux(那次 ls /dev/video* 看到的那些)，只要 USB 上的、index 0 的。"""
        usb = os.path.join(self.td, 'devices/platform/scb/fd500000.pcie/pci0000:00/0000:01:00.0/usb1/1-1/1-1.3/1-1.3:1.0')
        soc = os.path.join(self.td, 'devices/platform/soc')
        nodes = {1: (usb, 0), 2: (usb, 1)}                                      # 摄像头：取画面 + 元数据
        nodes.update({n: (os.path.join(soc, 'bcm2835-codec'), 0) for n in (10, 11, 12, 18)})
        nodes.update({n: (os.path.join(soc, 'bcm2835-isp'), n % 2) for n in (13, 14, 15, 16, 20, 21, 22, 23)})
        nodes.update({n: (os.path.join(soc, 'rpivid'), 0) for n in (19, 31)})
        for n, (dev, idx) in nodes.items():
            d = os.path.join(self.sysfs, f'video{n}')
            os.makedirs(d)
            os.makedirs(dev, exist_ok=True)
            os.symlink(dev, os.path.join(d, 'device'))
            with open(os.path.join(d, 'index'), 'w') as f:
                f.write(f'{idx}\n')
        os.makedirs(os.path.join(self.sysfs, 'video5'))                         # 读不到 index 的：跳过
        self.assertEqual(usb_cameras(self.by_id, self.sysfs), ['/dev/video1'])

    def test_configured_camera_opens_without_searching(self):
        """配置的 /dev/video0 开得了：和以前一样，不去找别的。"""
        fake = FakeV4L({'/dev/video0': 'ok', '/dev/video1': 'ok'})
        self.usb_back_as_video1()
        cam = self.camera(fake)
        with mock.patch('vision.usb_cameras') as find:
            cam.open()
        find.assert_not_called()
        self.assertEqual((cam.device, fake.opened), ('/dev/video0', ['/dev/video0']))
        self.assertEqual(self.logs, ['  摄像头 /dev/video0 已打开 640x480'])
        self.assertEqual(cam.read()[0], '/dev/video0')
        cam.close()
        self.assertTrue(fake.caps[0].released)

    def test_falls_back_to_usb_camera_with_new_number(self):
        """配置的 /dev/video0 没了(USB 重连以后成了 video1 + video2)：改用 video1；元数据节点、video10 以上都不碰。"""
        fake = FakeV4L({'/dev/video1': 'ok', '/dev/video2': 'blank', '/dev/video10': 'blank'})
        self.usb_back_as_video1()
        cam = self.camera(fake).open()
        self.assertEqual(cam.device, '/dev/video1')
        self.assertEqual(fake.opened, ['/dev/video0', '/dev/video1'])
        self.assertEqual(self.logs, ['  摄像头 /dev/video1 已打开 640x480',
                                     '  (摄像头 /dev/video0 打不开，改用 /dev/video1：USB 掉线重连以后编号变了)'])
        self.assertEqual(cam.read()[0], '/dev/video1')

    def test_configured_node_without_frames_falls_back(self):
        """配置的节点打得开、读不到画面(比如成了元数据节点)：关掉它，也改用 USB 摄像头。"""
        fake = FakeV4L({'/dev/video0': 'blank', '/dev/video1': 'ok'})
        self.link('usb-HD_USB_Camera_SN01-video-index0', '/dev/video1')
        cam = self.camera(fake, threaded=False).open()
        self.assertEqual(cam.device, '/dev/video1')
        self.assertTrue(fake.caps[0].released)
        self.assertEqual(self.logs[-1], '  (摄像头 /dev/video0 打开了但读不到画面，改用 /dev/video1：USB 掉线重连以后编号变了)')
        self.assertEqual(cam.read()[0], '/dev/video1')

    def test_open_fails_with_what_was_found(self):
        """哪个都开不了：VisionError 还是"打不开摄像头 /dev/video0"开头，后面说找到了什么、谁开着摄像头。"""
        cam = self.camera(FakeV4L({}))
        with self.assertRaises(VisionError) as cm:
            cam.open()
        self.assertEqual(str(cm.exception), '打不开摄像头 /dev/video0(没找到 USB 摄像头：USB 没插好？ls /dev/video* 看看)')
        self.usb_back_as_video1()                                               # USB 摄像头在，可也打不开(被别的程序占着)
        with mock.patch('vision.video_users', lambda: ['1234「python3 vlive.py」开着 /dev/video1']):
            with self.assertRaises(VisionError) as cm:
                cam.open()
        self.assertEqual(str(cm.exception), '打不开摄像头 /dev/video0(USB 摄像头 /dev/video1 也打不开：可能被别的程序占着；'
                                            '1234「python3 vlive.py」开着 /dev/video1)')
        self.assertEqual((cam.device, cam.cap, self.logs), ('/dev/video0', None, []))

    def test_same_node_is_not_tried_twice(self):
        """找到的 USB 摄像头就是配置的那个(打得开、没画面)：不再开一次。"""
        fake = FakeV4L({'/dev/video0': 'blank'})
        self.link('usb-HD_USB_Camera_SN01-video-index0', '/dev/video0')
        with self.assertRaises(VisionError) as cm:
            self.camera(fake).open()
        self.assertEqual(str(cm.exception), '打不开摄像头 /dev/video0(打开了但读不到画面)')
        self.assertEqual(fake.opened, ['/dev/video0'])

    def test_reopen_after_usb_drop(self):
        """用着用着 USB 掉线：连着 2 秒读不到画面就重开(3 秒内只试一次；重开不了不抛、只说一次)，摄像头回来(换成 video1)就接着有画面。"""
        fake = FakeV4L({'/dev/video0': 'ok'})
        clk = [100.0]
        cam = self.camera(fake, threaded=False, clock=lambda: clk[0]).open()
        self.assertEqual(cam.read()[0], '/dev/video0')
        fake.unplug('/dev/video0')
        for dt, n_open in ((0.0, 1),                    # 第一次读不到：从这里开始算
                           (1.5, 1),                    # 才 1.5 秒没画面：不重开
                           (1.0, 2),                    # 2.5 秒：重开，摄像头还没回来 -> 打不开(不抛)
                           (2.0, 2),                    # 上次试才 2 秒：不试
                           (1.5, 3)):                   # 隔了 3 秒多：再试(还是不行，不再说)
            clk[0] += dt
            self.assertIsNone(cam.read())
            self.assertEqual(len(fake.opened), n_open, dt)
        self.assertTrue(fake.caps[0].released)
        self.assertEqual(self.logs[1:], ['  摄像头 2 秒没有画面了(USB 掉线？)：重新打开…',
                                         '  ★ 重开摄像头没成功(每 3 秒再试一次)：打不开摄像头 /dev/video0'
                                         '(没找到 USB 摄像头：USB 没插好？ls /dev/video* 看看)'])
        fake.devs.update({'/dev/video1': 'ok', '/dev/video2': 'blank'})        # USB 重连：成了 video1 + video2
        self.usb_back_as_video1()
        clk[0] += 3.0
        self.assertEqual(cam.read()[0], '/dev/video1')                          # 重开好了：这一次就拿到画面
        self.assertEqual(cam.device, '/dev/video1')
        self.assertEqual(fake.opened[-2:], ['/dev/video0', '/dev/video1'])
        self.assertEqual(self.logs[3:], ['  摄像头 /dev/video1 已打开 640x480',
                                         '  (摄像头 /dev/video0 打不开，改用 /dev/video1：USB 掉线重连以后编号变了)'])
        clk[0] += 10.0
        self.assertEqual(cam.read()[0], '/dev/video1')
        self.assertEqual(len(fake.opened), 5)                                   # 有画面：不再重开

    def test_reopen_with_background_thread(self):
        """平时的后台线程模式：掉线以后 LatestFrame 一直拿不到新帧，read 等 1 秒没有、发现 2 秒没画面了：停掉旧线程、重开。"""
        fake = FakeV4L({'/dev/video0': 'ok'})
        clk = [100.0]
        cam = self.camera(fake, clock=lambda: clk[0]).open()
        old = cam._lf
        self.assertEqual(cam.read()[0], '/dev/video0')
        fake.unplug('/dev/video0')
        fake.devs['/dev/video1'] = 'ok'
        self.usb_back_as_video1()
        self.assertIsNone(cam.read())                                           # 等 1 秒没有新帧：从这里开始算
        clk[0] += 2.5
        self.assertEqual(cam.read()[0], '/dev/video1')
        self.assertIsNotNone(cam.last_t)
        self.assertFalse(old.th.is_alive())                                     # 旧线程停了，旧的摄像头关了
        self.assertTrue(fake.caps[0].released)
        self.assertTrue(cam._lf.th.is_alive())
        self.assertEqual((cam.device, self.logs[1]), ('/dev/video1', '  摄像头 2 秒没有画面了(USB 掉线？)：重新打开…'))

    def test_only_one_caller_reopens(self):
        """后台识别线程和主线程同时要帧：只有一个去重开，抢不到的直接返回 None。"""
        fake = FakeV4L({'/dev/video0': 'ok'})
        clk = [100.0]
        cam = self.camera(fake, threaded=False, clock=lambda: clk[0]).open()
        fake.unplug('/dev/video0')
        self.assertIsNone(cam.read())                                           # 从这里开始算没画面
        clk[0] += 2.5
        self.assertIsNone(cam.read())                                           # 重开了一次，摄像头还没回来
        self.assertEqual(len(fake.opened), 2)
        clk[0] += 5.0
        with cam._lock:                                                         # 别的线程正在重开：抢不到锁就不等、不重开
            self.assertIsNone(cam._revive())
        self.assertEqual(len(fake.opened), 2)
        fake.devs['/dev/video0'] = 'ok'                                         # 原来的编号回来了
        got, go = [], threading.Barrier(4)

        def worker():
            go.wait()
            got.append(cam.read())
        ths = [threading.Thread(target=worker) for _ in range(4)]
        for th in ths:
            th.start()
        for th in ths:
            th.join(5.0)
        self.assertEqual(len(got), 4)
        self.assertEqual(len(fake.opened), 3)                                   # 四个线程一起要：只重开了一次
        self.assertTrue(any(fr is not None for fr in got))
        self.assertEqual(cam.device, '/dev/video0')

    def test_no_reopen_after_close_or_when_off(self):
        """close 以后、reopen_s=0(不重开，和以前一样)：读不到画面也不会自己去开摄像头。"""
        clk = [100.0]
        for reopen_s, closed in ((2.0, True), (0, False)):
            fake = FakeV4L({'/dev/video0': 'ok'})
            cam = self.camera(fake, threaded=False, clock=lambda: clk[0], reopen_s=reopen_s).open()
            fake.unplug('/dev/video0')
            fake.devs['/dev/video0'] = 'ok'
            if closed:
                cam.close()
            self.assertIsNone(cam.read())
            clk[0] += 60.0
            self.assertIsNone(cam.read())
            self.assertEqual(len(fake.opened), 1, reopen_s)

    def test_long_idle_then_one_miss_does_not_reopen(self):
        """开车时好几分钟不要帧，回来第一次偶尔没拿到：不算掉线、不重开(要连着读不到 2 秒才重开)。"""
        fake = FakeV4L({'/dev/video0': 'ok'})
        clk = [100.0]
        cam = self.camera(fake, threaded=False, clock=lambda: clk[0]).open()
        self.assertEqual(cam.read()[0], '/dev/video0')
        clk[0] += 300.0
        fake.caps[0].dead = True
        self.assertIsNone(cam.read())
        fake.caps[0].dead = False
        clk[0] += 0.5
        self.assertEqual(cam.read()[0], '/dev/video0')
        self.assertEqual(len(fake.opened), 1)
        self.assertEqual(len(self.logs), 1)

    def test_vision_passes_reopen_setting(self):
        """vision 配置里 camera.reopen_s 传给 Camera(默认 2 秒，0 = 不重开)。"""
        got = []

        class Spy:
            def __init__(self, *a, **kw):
                got.append(kw)

            def open(self):
                return self
        with mock.patch('vision.Camera', Spy):
            Vision(log=lambda m: None).open()
            Vision(dict(camera=dict(reopen_s=0)), log=lambda m: None).open()
        self.assertEqual([kw['reopen_s'] for kw in got], [2.0, 0])

    def test_video_users(self):
        """谁开着 /dev/video*：看 /proc/<pid>/fd 的链接(拔掉了的节点后面是 (deleted))；目录不对也不抛。"""
        proc = os.path.join(self.td, 'proc')
        for pid, cmd, fds in (('1234', b'python3\0vlive.py\0', {'3': '/dev/video0 (deleted)', '4': '/dev/null'}),
                              ('99', b'bash\0', {'0': '/dev/pts/0'})):
            os.makedirs(os.path.join(proc, pid, 'fd'))
            with open(os.path.join(proc, pid, 'cmdline'), 'wb') as f:
                f.write(cmd)
            for fd, tgt in fds.items():
                os.symlink(tgt, os.path.join(proc, pid, 'fd', fd))
        self.assertEqual(video_users(proc), ['1234「python3 vlive.py」开着 /dev/video0(已拔掉)'])
        self.assertEqual(video_users(os.path.join(self.td, 'none')), [])


class MathTests(unittest.TestCase):
    def test_robust_mean(self):
        m = _robust_mean([(10, 10), (10.2, 9.8), (30, 30), (9.9, 10.1)], 3.0)
        self.assertAlmostEqual(m[0], 10.033, delta=0.05)


if __name__ == '__main__':
    unittest.main(verbosity=1)
