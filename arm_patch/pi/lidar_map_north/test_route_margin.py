"""route_plan.py 黄色区硬性余量(yellow_margin_mm)的测试：python3 -m unittest test_route_margin"""
import math
import unittest

import route_plan as rp

# 停车点、车身尺寸和您的 map_config_start2_roi.json 一样
CFG = dict(stops={'QR': [2100, 1200, 90], 'RAW': [1200, 2100, 180], 'ROUGH': [1200, 340, 0], 'TEMP': [340, 1200, -90],
                  'START2': [2250, 150, 90]},
           car_length_mm=290, car_width_mm=260, lidar_overhang_mm=40, margin_mm=60, strafe_cost_factor=3.0,
           strafe_max_mm_per_leg=300, turn_margin_mm=20, raw_center_mm=[1200, 2480], raw_radius_mm=150)
MISSION = ['QR', 'RAW', 'ROUGH', 'TEMP', 'RAW', 'ROUGH', 'TEMP', 'START2']
START = (2250, 150, 90)


def min_gap(legs, planner):
    g = 1e9
    for L in legs:
        for x, y, yaw in L['poses']:
            w, e, so, no = planner.ext[rp.heading_index(round(yaw / 90) * 90)]
            for r in rp.YELLOW:
                g = min(g, math.hypot(max(r[0] - (x + e), (x - w) - r[2], 0), max(r[1] - (y + no), (y - so) - r[3], 0)))
    return g


class YellowMarginTests(unittest.TestCase):
    def test_margin_keeps_route_away_from_yellow(self):
        legs0, why0, pl0 = rp.plan_mission(dict(CFG, yellow_margin_mm=0), [], START, MISSION)
        legs, why, pl = rp.plan_mission(dict(CFG, yellow_margin_mm=30), [], START, MISSION)
        self.assertEqual((why0, why), ('OK', 'OK'))
        self.assertGreaterEqual(min_gap(legs, pl), 30)
        self.assertGreater(min_gap(legs, pl), min_gap(legs0, pl0))
        self.assertFalse(any(L.get('note') for L in legs), '没有障碍物时每段都应该按 30mm 规划出来')

    def test_default_is_30(self):
        legs, why, pl = rp.plan_mission(dict(CFG), [], START, MISSION)
        self.assertEqual(why, 'OK')
        self.assertGreaterEqual(min_gap(legs, pl), 30)

    def test_narrow_lane_falls_back_per_leg_with_note(self):
        # 障碍物把外圈车道堵了，只能走 400mm 宽的中间车道：这几段自动放宽并提示，路线照样规划得出来
        obs = [(430, 265, 25), (2240, 1690, 25), (285, 2060, 25)]
        legs, why, pl = rp.plan_mission(dict(CFG, yellow_margin_mm=30), obs, START, MISSION)
        self.assertEqual(why, 'OK')
        notes = [L['note'] for L in legs if L.get('note')]
        self.assertTrue(notes and all('只留了' in n for n in notes))

    def test_why_blocked_mentions_yellow(self):
        p = rp.Planner(dict(CFG, yellow_margin_mm=30), [])
        self.assertIn('离黄色区不到 30mm', p.why_blocked(1600, 1850 + 150 + 10, 1))   # 车身后边离黄色区只有 10mm


if __name__ == '__main__':
    unittest.main()
