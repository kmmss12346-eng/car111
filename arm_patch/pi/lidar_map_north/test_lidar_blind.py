"""lidar_blind.py：从扫描存的原始帧看雷达哪些方向被车上的东西挡住。python3 -m unittest test_lidar_blind"""
import io
import json
import os
import tempfile
import unittest
from contextlib import redirect_stdout

import lidar_blind


def frame(blocks, far=(30, 60, 1500)):
    """一圈 720 个方向：blocks 里的方向近处有回波(车上的东西)；far 那一段 1.5 米处有东西(场地里)，别的方向没回波。"""
    pts = []
    for i in range(720):
        a = -180 + i * 0.5
        d = next((dd for lo, hi, dd in blocks if lo <= a < hi), 0)
        pts.append([a, d or (far[2] if far[0] <= a < far[1] else 0)])
    return dict(points=pts, seq=1)


class LidarBlindTests(unittest.TestCase):
    def run_on(self, views):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 'merged_x.json')
            with open(p, 'w') as f:
                json.dump(dict(views=views), f)
            out = io.StringIO()
            with redirect_stdout(out):
                lidar_blind.plot = lambda *a, **k: None       # 不画图
                lidar_blind.main([p])
            return out.getvalue()

    def test_blocked_sectors_and_wrong_heading(self):
        c = dict(car_x_mm=2250, car_y_mm=150, car_yaw_deg=90, lidar_forward_mm=22.5, lidar_left_mm=134.8,
                 lidar_yaw_deg=0, sdk_y_sign=1)
        blocks = [(0, 20, 150), (80, 150, 180)]
        out = self.run_on([dict(config=c, raw_frames=[frame(blocks)] * 3),
                           dict(config=dict(c, car_yaw_deg=-90), raw_frames=[frame(blocks)] * 3),
                           dict(config=c)])
        self.assertIn('+0°~+20°(最近 15cm)', out)
        self.assertIn('+80°~+150°(最近 18cm)', out)
        self.assertIn('车头按 90° 算', out)                   # 第 2 次记成 -90°：全落在场外
        self.assertIn('没有存原始帧', out)


if __name__ == '__main__':
    unittest.main()
