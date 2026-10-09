"""matdet.py(抗爪子遮挡的物料识别)的测试：python3 -m unittest test_matdet

用两张真实画面(testdata/arm_cam_18.jpg 物料在爪子上方、arm_cam_19.jpg 物料被爪子挡住一部分，
从现场截图裁出来的，画面上还带着当时 vlive 画的十字和文字)和合成画面。"""
import math
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

try:
    import cv2
    HAVE_CV = True
except ImportError:                     # 没装 opencv 的电脑上跳过
    HAVE_CV = False

HERE = Path(__file__).resolve().parent
NO_MASK = {'claw_mask': '/nonexistent/claw_mask.png'}      # 不读文件夹里可能存在的 claw_mask.png

# 各颜色在 BGR 里的样子(落在 matdet 的 HSV 范围里)
BGR = {1: (40, 40, 220), 2: (40, 200, 230), 3: (190, 70, 20), 4: (60, 170, 40), 5: (30, 30, 30), 6: (235, 200, 140)}
CLAW_BGR = (200, 80, 20)


def scene(materials=(), claw_top=None, jaws=True, noise=3.0, seed=0, brightness=1.0, side=0.06):
    """白色转盘 + 细线纹理 + 圆柱物料(顶面圆 + 往画面中心那边露出一点侧面) + 下方蓝色爪子。
    materials: [(颜色号, cx, cy, r)]；claw_top: 爪子上沿的 y(None = 没有爪子)。"""
    rng = np.random.default_rng(seed)
    im = np.full((480, 640, 3), 228, np.uint8)
    for y in range(10, 480, 22):
        cv2.line(im, (0, y), (639, y + 30), (150, 150, 150), 1)
    for cid, cx, cy, r in materials:
        col = np.array(BGR[cid], float)
        off = ((320 - cx) * side, (240 - cy) * side)      # 侧面露在朝画面中心那边，离中心越远露得越多
        cv2.circle(im, (int(cx + off[0]), int(cy + off[1])), int(r), tuple(int(v) for v in col * 0.78), -1, cv2.LINE_AA)
        cv2.circle(im, (int(cx), int(cy)), int(r), tuple(int(v) for v in col), -1, cv2.LINE_AA)
    if claw_top is not None:
        cv2.rectangle(im, (0, int(claw_top) + 40), (639, 479), CLAW_BGR, -1)
        if jaws:
            for x0, x1 in ((150, 300), (340, 490)):
                pts = np.array([[x0, claw_top + 40], [x0 + 30, claw_top], [x1, claw_top + 10], [x1, claw_top + 40]], np.int32)
                cv2.fillPoly(im, [pts], CLAW_BGR)
        cv2.ellipse(im, (330, int(claw_top) + 55), (45, 14), 0, 0, 360, (240, 240, 240), -1)
    im = np.clip(im.astype(float) * brightness + rng.normal(0, noise, im.shape), 0, 255).astype(np.uint8)
    return im


def err(res, cx, cy):
    return math.hypot(res['center'][0] - cx, res['center'][1] - cy)


@unittest.skipUnless(HAVE_CV, '没有 opencv')
class RealFrameTests(unittest.TestCase):
    def setUp(self):
        import matdet
        self.matdet = matdet

    def load(self, name):
        im = cv2.imread(str(HERE / 'testdata' / name))
        self.assertIsNotNone(im, name)
        return im

    def test_red_in_open_view(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        r = d.detect(self.load('arm_cam_18.jpg'), 1)
        self.assertIsNotNone(r)
        self.assertLess(err(r, 306.6, 192.7), 1.5)
        self.assertAlmostEqual(r['radius'], 49.2, delta=2.0)

    def test_red_partly_behind_claw(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        r = d.detect(self.load('arm_cam_19.jpg'), 1)
        self.assertIsNotNone(r)
        self.assertLess(err(r, 354.5, 262.7), 2.0)
        self.assertGreater(r['occluded'], 0.03)

    def test_blue_material_not_confused_with_blue_claw(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        r = d.detect(self.load('arm_cam_18.jpg'), 3)
        self.assertIsNotNone(r)
        self.assertLess(r['center'][1], 150)                 # 是上面那个蓝色物料，不是下面的蓝色爪子

    def test_missing_colours_not_found(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        im = self.load('arm_cam_19.jpg')
        for cid in (4, 6):
            self.assertIsNone(d.detect(im, cid), f'颜色 {cid} 画面里没有，不能认出来')

    def test_fast_enough(self):
        import time
        d = self.matdet.MaterialDetector(NO_MASK)
        im = self.load('arm_cam_19.jpg')
        d.detect(im, 1)
        t0 = time.perf_counter()
        for _ in range(5):
            d.detect(im, 1)
        self.assertLess((time.perf_counter() - t0) / 5, 0.15)


@unittest.skipUnless(HAVE_CV, '没有 opencv')
class SyntheticTests(unittest.TestCase):
    def setUp(self):
        import matdet
        self.matdet = matdet

    def test_every_colour_open_view(self):
        for cid in range(1, 7):
            d = self.matdet.MaterialDetector(NO_MASK)
            im = scene([(cid, 320, 180, 48)], claw_top=330, seed=cid)
            r = d.detect(im, cid)
            self.assertIsNotNone(r, f'颜色 {cid} 没认出来')
            self.assertLess(err(r, 320, 180), 2.0, f'颜色 {cid} 偏了 {err(r, 320, 180):.1f}px')

    def test_occlusion_from_claw(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        d.detect(scene([(1, 330, 200, 48)], claw_top=330), 1)                # 先在没挡住时学到半径
        for hidden in (0.2, 0.4, 0.6, 0.75):
            top = int(200 + 48 - hidden * 96) - 40                          # 爪子大块上沿 = 物料被挡住 hidden 的位置
            r = d.detect(scene([(1, 330, 200, 48)], claw_top=top, jaws=False, seed=int(hidden * 10)), 1)
            self.assertIsNotNone(r, f'挡住 {hidden:.0%} 认不到了')
            self.assertLess(err(r, 330, 200), 2.5, f'挡住 {hidden:.0%} 偏了 {err(r, 330, 200):.1f}px')

    def test_occlusion_without_learned_radius(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        r = d.detect(scene([(2, 300, 250, 50)], claw_top=250 + 50 - 45 - 40, jaws=False), 2)
        self.assertIsNotNone(r)
        self.assertLess(err(r, 300, 250), 3.0)

    def test_two_same_colour_picks_a_real_one(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        r = d.detect(scene([(1, 200, 150, 45), (1, 450, 160, 45)], claw_top=350), 1)
        self.assertIsNotNone(r)
        self.assertLess(min(err(r, 200, 150), err(r, 450, 160)), 2.0)

    def test_partly_out_of_frame(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        r = d.detect(scene([(4, 320, 20, 48)], claw_top=350, side=0.0), 4)     # 顶面一半在画面外(看不到顶面就分不出侧面，这里不画侧面)
        self.assertIsNotNone(r)
        self.assertLess(err(r, 320, 20), 3.0)

    def test_brightness_and_noise(self):
        for b in (0.75, 1.15):
            d = self.matdet.MaterialDetector(NO_MASK)
            r = d.detect(scene([(1, 320, 200, 48)], claw_top=340, noise=6.0, brightness=b), 1)
            self.assertIsNotNone(r, f'亮度 x{b} 认不到')
            self.assertLess(err(r, 320, 200), 2.0)

    def test_nothing_there(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        im = scene([], claw_top=300)
        for cid in range(1, 7):
            self.assertIsNone(d.detect(im, cid), f'没有物料却认出了颜色 {cid}')

    def test_blue_touching_claw_with_static_mask(self):
        with tempfile.TemporaryDirectory() as td:
            d = self.matdet.MaterialDetector({'claw_mask': os.path.join(td, 'claw_mask.png')})
            empty = [scene([], claw_top=300, seed=s) for s in range(4)]
            path, frac = d.save_claw_mask(empty)
            self.assertTrue(os.path.exists(path))
            self.assertGreater(frac, 0.2)
            d2 = self.matdet.MaterialDetector({'claw_mask': os.path.join(td, 'claw_mask.png')})
            r = d2.detect(scene([(3, 330, 280, 48)], claw_top=300), 3)       # 蓝色物料下半截挨着/压在蓝色爪子上
            self.assertIsNotNone(r)
            self.assertLess(err(r, 330, 280), 3.0)


@unittest.skipUnless(HAVE_CV, '没有 opencv')
class ClawCacheTests(unittest.TestCase):
    def setUp(self):
        import matdet
        self.matdet = matdet

    def test_blue_uses_claw_found_while_looking_for_other_colours(self):
        """没有 claw_mask.png：找蓝色物料时不现找爪子(物料挨着爪子会连成一块被当成爪子)，用找别的颜色时记下的。"""
        d = self.matdet.MaterialDetector(NO_MASK)
        self.assertTrue(d.need_vmask(3))
        self.assertFalse(d.need_vmask(1))
        d.detect(scene([(1, 200, 120, 45)], claw_top=300), 1)            # 先找一次红色：记下爪子区域
        self.assertFalse(d.need_vmask(3))
        r = d.detect(scene([(3, 330, 280, 48)], claw_top=300), 3)        # 蓝色物料压在蓝色爪子上
        self.assertIsNotNone(r)
        self.assertLess(err(r, 330, 280), 3.0)

    def test_blue_without_any_mask_is_flagged(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        d.detect(scene([(3, 330, 280, 48)], claw_top=300), 3)            # 找蓝色不会记爪子区域
        self.assertTrue(d.need_vmask(3))
        self.assertIsNone(d._auto_cache)

    def test_static_mask_check_keeps_full_size(self):
        with tempfile.TemporaryDirectory() as td:
            d = self.matdet.MaterialDetector({'claw_mask': os.path.join(td, 'claw_mask.png')})
            self.assertFalse(d.has_static_mask())
            d.save_claw_mask([scene([], claw_top=300)])
            d2 = self.matdet.MaterialDetector({'claw_mask': os.path.join(td, 'claw_mask.png')})
            self.assertTrue(d2.has_static_mask())
            self.assertFalse(d2.need_vmask(3))
            self.assertEqual(d2.claw_mask(scene([], claw_top=300)).shape, (480, 640))


@unittest.skipUnless(HAVE_CV, '没有 opencv')
class ReviewFixTests(unittest.TestCase):
    def setUp(self):
        import matdet
        self.matdet = matdet

    def test_vmask_refuses_blue_material_in_jaws(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'claw_mask.png')
            d = self.matdet.MaterialDetector({'claw_mask': path})
            frames = [scene([(3, 330, 300, 45)], claw_top=300, seed=s) for s in range(3)]   # 蓝色物料夹在爪子里
            with self.assertRaises(ValueError):
                d.save_claw_mask(frames, keep_clear=(330, 300))
            self.assertFalse(os.path.exists(path))
            with self.assertRaises(ValueError):                                          # 画面里没有爪子
                d.save_claw_mask([scene([], claw_top=None)])
            d.save_claw_mask([scene([], claw_top=300)], keep_clear=(330, 250))           # 正常的能存
            self.assertTrue(os.path.exists(path))

    def test_cache_not_poisoned_by_blue_touching_claw(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        d.detect(scene([(1, 200, 120, 45)], claw_top=300), 1)
        n0 = int(np.count_nonzero(d._auto_cache))
        d.detect(scene([(1, 200, 120, 45), (3, 330, 290, 48)], claw_top=300), 1)       # 这一帧蓝色物料挨着爪子
        self.assertLessEqual(int(np.count_nonzero(d._auto_cache)), int(1.03 * n0))
        r = d.detect(scene([(3, 330, 290, 48)], claw_top=300), 3)
        self.assertIsNotNone(r)
        self.assertLess(err(r, 330, 290), 3.0)

    def test_light_blue_with_high_saturation_is_not_blue(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        hsv = np.uint8([[[100, 165, 225]]])
        bgr = tuple(int(v) for v in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])
        im = scene([], claw_top=330)
        cv2.circle(im, (320, 170), 48, bgr, -1, cv2.LINE_AA)
        self.assertIsNotNone(d.detect(im, 6))
        self.assertIsNone(d.detect(im, 3))

    def test_dark_blue_is_not_black(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        im = scene([], claw_top=330)
        cv2.circle(im, (320, 170), 48, (90, 30, 5), -1, cv2.LINE_AA)      # 很暗的蓝色(光线暗时)
        self.assertIsNone(d.detect(im, 5))
        im2 = scene([(5, 320, 170, 48)], claw_top=330)
        self.assertIsNotNone(d.detect(im2, 5))

    def test_wrong_radius_seed_does_not_lock_detection(self):
        d = self.matdet.MaterialDetector(NO_MASK)
        d.seed_radius(38.0)                                               # 存的半径是错的(观察高度改过)
        r = d.detect(scene([(1, 320, 180, 48)], claw_top=330), 1)
        self.assertIsNotNone(r)
        self.assertLess(err(r, 320, 180), 2.0)


@unittest.skipUnless(HAVE_CV, '没有 opencv')
class FitTests(unittest.TestCase):
    def test_fit_circle_arc(self):
        import matdet
        t = np.linspace(-0.2, 2.2, 120)                    # 只有大约 140° 的圆弧
        pts = np.c_[100 + 40 * np.cos(t), 80 + 40 * np.sin(t)] + np.random.default_rng(1).normal(0, 0.3, (120, 2))
        bad = np.c_[np.linspace(60, 140, 30), np.full(30, 130.0)]          # 一条直线(爪子边缘)混进来
        fit = matdet.fit_circle(np.vstack([pts, bad]), 10, 200)
        self.assertIsNotNone(fit)
        self.assertLess(math.hypot(fit[0] - 100, fit[1] - 80), 0.5)
        self.assertAlmostEqual(fit[2], 40, delta=0.5)


class CalFileTests(unittest.TestCase):
    def test_save_load_apply(self):
        from vision import save_vision_cal, load_vision_cal, apply_vision_cal
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, 'vision_cal.json')
            save_vision_cal('RAW', (331.26, 287.04), p)
            save_vision_cal('RING', (320, 250), p)
            d = load_vision_cal(p)
            self.assertEqual(d['claw_px']['RAW'], [331.26, 287.04])
            cfg = apply_vision_cal({'claw_px': {'RAW': [1, 2], 'RING': [3, 4]}}, p)
            self.assertEqual(cfg['claw_px']['RING'], [320.0, 250.0])
            save_vision_cal('RING', (321, 251), p, {'ring_rmax_px': 101.5})
            cfg = apply_vision_cal({'claw_px': {}}, p)
            self.assertEqual(cfg['ring_rmax_cal'], 101.5)
            self.assertEqual(cfg['claw_px']['RAW'], [331.26, 287.04])       # 存 RING 不会把 RAW 弄丢
            self.assertEqual(apply_vision_cal({'claw_px': {'RAW': [1, 2]}}, os.path.join(td, 'none.json'))['claw_px']['RAW'], [1, 2])


if __name__ == '__main__':
    unittest.main()
