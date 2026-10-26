"""(10-26 实车暂存区)圆环这里每毫米多少像素：没做 vclaw RING 时不用配置里默认的 2.96，按量过的来。"""
import os
import tempfile
import types
import unittest

import numpy as np

from mission_hooks import MissionHooks
from vision import Vision
from visual_servo import JacStore


def _hooks(vision, store):
    lines = []
    h = types.SimpleNamespace(vision=vision, store=store, log=lines.append)
    return h, lines


class RingScaleSeedTests(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.d.name, 'servo_cal.json')

    def tearDown(self):
        self.d.cleanup()

    def _store(self, J=None, seen=None, synth=False):
        st = JacStore(self.path)
        if J is not None:
            st.put('RING', 'ch', J)
        if synth:
            st.set_synth('RING', True)
        if seen is not None:
            st.set_extra('RING', 'px_mm_seen', seen)
        return st

    def test_vcal_j_beats_the_default(self):
        """实车：vcal RING 量的横移 1.12、前进 1.16 像素/mm，配置里默认 2.96：按 1.14 算，2.96 不用；半径 259 像素的大圆不再当圆环。"""
        v = Vision(cfg={}, log=lambda s: None)
        self.assertAlmostEqual(v.scale('RING'), 2.96)
        self.assertGreaterEqual(v._ring_rmax_range()[1], 259.0)          # 以前：按 2.96 算，259 像素的圆也算圆环
        h, lines = _hooks(v, self._store(J=[[1.12, 0.0], [0.0, -1.16]]))
        MissionHooks._ring_scale_seed(h)
        self.assertAlmostEqual(v.scale('RING'), 1.14, places=2)
        self.assertEqual(v.cfg['_ring_scale_src'], 'J')
        self.assertIn('vcal RING', '\n'.join(lines))
        self.assertIn('2.96', '\n'.join(lines))
        lo, hi = v._ring_rmax_range()
        self.assertLess(hi, 259.0)                                       # 那个大圆不是圆环
        self.assertTrue(lo < 55.0 < hi, (lo, hi))                        # 10-23 看到的圆环(半径约 55 像素)照样认
        self.assertIsNone(MissionHooks._raw_J_bad(np.array([[1.12, 0.0], [0.0, -1.16]]), v.scale('RING')) or None)
        n = len(lines)
        MissionHooks._ring_scale_seed(h)                                 # 一次运行只定一次
        self.assertEqual(len(lines), n)

    def test_seen_spacing_first_then_cal(self):
        v = Vision(cfg={}, log=lambda s: None)
        h, lines = _hooks(v, self._store(J=[[1.12, 0.0], [0.0, -1.16]], seen=1.27))
        MissionHooks._ring_scale_seed(h)
        self.assertAlmostEqual(v.scale('RING'), 1.27)                    # 以前看清三个环时按环距量的优先
        self.assertIn('环距', '\n'.join(lines))
        v2 = Vision(cfg=dict(ring_rmax_cal=120.0, ring_outer_diam_mm=95.0), log=lambda s: None)
        h2, lines2 = _hooks(v2, self._store(J=[[1.12, 0.0], [0.0, -1.16]], seen=1.27))
        MissionHooks._ring_scale_seed(h2)                                # vclaw RING 量过圆环大小：照旧按它
        self.assertAlmostEqual(v2.scale('RING'), 2.0 * 120.0 / 95.0)
        self.assertEqual(lines2, [])

    def test_nothing_measured_keeps_config(self):
        v = Vision(cfg={}, log=lambda s: None)
        h, lines = _hooks(v, self._store())
        MissionHooks._ring_scale_seed(h)
        self.assertAlmostEqual(v.scale('RING'), 2.96)
        self.assertEqual(lines, [])
        v2 = Vision(cfg={}, log=lambda s: None)
        h2, _l = _hooks(v2, self._store(J=[[-1.2, 0.0], [0.0, 1.2]], synth=True))
        MissionHooks._ring_scale_seed(h2)                                # 只测过前进那一列：按前进那一列
        self.assertAlmostEqual(v2.scale('RING'), 1.2)

    def test_rescale_only_when_columns_are_square(self):
        J = np.array([[1.12, 0.05], [0.02, -1.16]])
        R = MissionHooks._j_rescaled(J, 2.5)
        np.testing.assert_allclose(np.linalg.norm(R, axis=0), [2.5, 2.5])
        np.testing.assert_allclose(R[:, 0] / np.linalg.norm(R[:, 0]), J[:, 0] / np.linalg.norm(J[:, 0]))
        self.assertIsNone(MissionHooks._j_rescaled(np.array([[1.0, 0.9], [0.0, 0.3]]), 2.5))   # 两列几乎同一个方向：不是大小的问题
        self.assertIsNone(MissionHooks._j_rescaled(np.array([[np.nan, 0.0], [0.0, 1.0]]), 2.5))


if __name__ == '__main__':
    unittest.main()
