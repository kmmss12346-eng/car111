"""route_plan.py 黄色区余量(yellow_margin_mm / yellow_min_margin_mm)的测试：python3 -m unittest test_route_margin"""
import unittest

import route_plan as rp

# 停车点、车身尺寸和您的 map_config_start2_roi.json 一样
CFG = dict(stops={'QR': [2100, 1200, 90], 'RAW': [1200, 2100, 180], 'ROUGH': [1200, 340, 0], 'TEMP': [340, 1200, -90],
                  'START2': [2250, 150, 90]},
           car_length_mm=290, car_width_mm=260, lidar_overhang_mm=40, lidar_forward_mm=22.45, lidar_left_mm=134.77,
           margin_mm=60, strafe_cost_factor=3.0, strafe_max_mm_per_leg=300, turn_margin_mm=20,
           raw_center_mm=[1200, 2480], raw_radius_mm=150, zone_margin_mm=30)
MISSION = ['QR', 'RAW', 'ROUGH', 'TEMP', 'RAW', 'ROUGH', 'TEMP', 'START2']
START = (2250, 150, 90)
OUTER_BLOCKED = [(430, 265, 25), (2240, 1690, 25), (285, 2060, 25)]   # 外圈车道被堵，只能走 400mm 宽的中间车道


def yellow_gap(cfg, legs, start=START):
    """按真实车身(含雷达)把整条路线一条条扫过去，离黄色区最近多少 mm(转弯按角度扫)。"""
    items = [it for it in rp.fixed_items(cfg) if it[1] == 'yellow']
    sw = rp._Sweep(rp.Body(cfg), items)
    pose = start
    g = min(sw.pose_gaps(pose))
    for L in legs:
        for c, v in L['cmds']:
            pose, gaps, _ = sw.move_gaps(pose, c, v)
            g = min(g, min(gaps))
    return g


class YellowMarginTests(unittest.TestCase):
    def test_margin_keeps_route_away_from_yellow(self):
        legs0, why0, _ = rp.plan_mission(dict(CFG, yellow_margin_mm=0), [], START, MISSION)
        legs, why, _ = rp.plan_mission(dict(CFG, yellow_margin_mm=30), [], START, MISSION)
        self.assertEqual((why0, why), ('OK', 'OK'))
        self.assertGreaterEqual(yellow_gap(CFG, legs), 30 - 0.1)
        self.assertGreaterEqual(yellow_gap(CFG, legs), yellow_gap(CFG, legs0))
        self.assertFalse(any(L.get('note') for L in legs), '没有障碍物时每段都应该按 30mm 规划出来')

    def test_default_is_30(self):
        legs, why, pl = rp.plan_mission(dict(CFG), [], START, MISSION)
        self.assertEqual(why, 'OK')
        self.assertEqual(pl.yellow_margin, 30)
        self.assertEqual(pl.yellow_min, 15)
        self.assertGreaterEqual(yellow_gap(CFG, legs), 30 - 0.1)

    def test_narrow_lane_only_squeezes_where_needed_with_note(self):
        # 中间车道 400 宽，车身+雷达 300：两边各最多 50mm。按 60mm 走不过去的那几段放宽并提示，其余段照样 60
        cfg = dict(CFG, yellow_margin_mm=60)
        legs, why, _ = rp.plan_mission(cfg, OUTER_BLOCKED, START, MISSION)
        self.assertEqual(why, 'OK')
        notes = [L['note'] for L in legs if L.get('note')]
        self.assertTrue(notes and all('只留了' in n for n in notes))
        self.assertLess(len(notes), len(legs), '只有挤不过去的那几段才放宽')
        for L in legs:
            self.assertGreaterEqual(L['yellow_mm'], 15 - 0.1)
            if not L.get('note'):
                self.assertGreaterEqual(L['yellow_mm'], 60 - 0.1)
        self.assertGreaterEqual(yellow_gap(cfg, legs), 15 - 0.1)

    def test_never_below_min_tier(self):
        # 最低也要 55mm：中间车道(最多 50)过不去，外圈又被堵 → 规划失败并说明原因，不会悄悄贴着黄色区走
        cfg = dict(CFG, yellow_margin_mm=60, yellow_min_margin_mm=55)
        legs, why, _ = rp.plan_mission(cfg, OUTER_BLOCKED, START, MISSION)
        self.assertNotEqual(why, 'OK')
        self.assertIn('路线失败', why)
        for L in legs:
            self.assertGreaterEqual(L['yellow_mm'], 55 - 0.1)

    def test_zero_is_not_a_fallback(self):
        # 配置写 yellow_min_margin_mm=0 也按 1mm 算(不会压线)
        pl = rp.Planner(dict(CFG, yellow_min_margin_mm=0), [])
        self.assertGreaterEqual(pl.yellow_min, 1)

    def test_why_blocked_mentions_yellow(self):
        p = rp.Planner(dict(CFG, yellow_margin_mm=30), [])
        # 车头朝北，车身后边离黄色区只有 10mm(最低要 15)
        self.assertIn('离黄色区不到 15mm', p.why_blocked(1600, 1850 + 145 + 10, 1))
        self.assertEqual(p.why_blocked(1600, 1850 + 145 + 20, 1), '')


if __name__ == '__main__':
    unittest.main()
