"""route_plan.py 黄色区余量(yellow_margin_mm / yellow_min_margin_mm)的测试：python3 -m unittest test_route_margin"""
import unittest

import route_plan as rp

# 停车点、车身尺寸和您的 map_config_start2_roi.json 一样
CFG = dict(mid_lane_shift_mm=0, no_jog_from=[],  # 这里测规划本身的黄色区余量(no_jog 不按地图查，单独测)；去 ROUGH 中间车道往西挪见 MidLaneShiftTests
           stops={'QR': [2100, 1200, 90], 'RAW': [1200, 2100, 180], 'ROUGH': [1200, 340, 0], 'TEMP': [340, 1200, -90],
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

    def test_default_is_50_and_floor_35(self):
        # (10-26 用户"多留点冗余出来一定不要碰到黄色区域")希望 50、最低 35；配置里写的 30(车上以前配的)也按 35
        # (10-28)先按 yellow_prefer_floor_mm(45)规划，规划得出来就用它；0 = 以前直接按 35
        legs, why, pl = rp.plan_mission(dict(CFG), [], START, MISSION)
        self.assertEqual(why, 'OK')
        self.assertEqual(pl.yellow_margin, 50)
        self.assertEqual(pl.yellow_min, 45)
        self.assertGreaterEqual(yellow_gap(CFG, legs), 45 - 0.1)
        legs, why, pl = rp.plan_mission(dict(CFG, yellow_prefer_floor_mm=0), [], START, MISSION)
        self.assertEqual(why, 'OK')
        self.assertEqual(pl.yellow_min, 35)
        self.assertGreaterEqual(yellow_gap(CFG, legs), 35 - 0.1)
        pl = rp.Planner(dict(CFG, yellow_margin_mm=50, yellow_min_margin_mm=30), [])
        self.assertEqual(pl.yellow_min, 35)
        pl = rp.Planner(dict(CFG, yellow_margin_mm=50, yellow_min_margin_mm=30, yellow_floor_mm=30), [])
        self.assertEqual(pl.yellow_min, 30)

    def test_narrow_lane_only_squeezes_where_needed_with_note(self):
        # 中间车道 400 宽，车身+雷达 300：两边各最多 50mm。按 60mm 走不过去的那几段放宽，其余段照样 60。
        # (10-26)窄车道走在正中间(两边 45~55)是正常的最近路线，不再提示；离黄色区不到 40mm 才提示“只留了”
        cfg = dict(CFG, yellow_margin_mm=60)
        legs, why, _ = rp.plan_mission(cfg, OUTER_BLOCKED, START, MISSION)
        self.assertEqual(why, 'OK')
        narrow = [L for L in legs if L['yellow_mm'] < 60 - 0.1]
        self.assertTrue(narrow, '外圈被堵，有几段只能走中间的窄车道')
        self.assertLess(len(narrow), len(legs), '只有挤不过去的那几段才放宽')
        for L in legs:
            self.assertGreaterEqual(L['yellow_mm'], 35 - 0.1)
            self.assertEqual('只留了' in (L.get('note') or ''), L['yellow_mm'] < 40 - 0.5, L)
        self.assertGreaterEqual(yellow_gap(cfg, legs), 35 - 0.1)

    def test_center_lane_is_driven_in_the_middle(self):
        # (10-28 实车：两个障碍物在两边的车道里，中间那条 400 宽的车道空着，去粗加工区从中间穿；按硬余量 35 车走在离西边黄色区 35mm、
        # 离东边 65mm 的地方，推算的车位一路偏几厘米就压到黄色区)先按 45 规划：同一条车道走正中间(两边约 50mm)，用时最多多 1~2 秒
        stops = dict(CFG['stops'], QR=[2100, 1250, 90], RAW=[1080, 2185, 180], ROUGH=[1135, 330, 0], TEMP=[310, 1275, -90])
        cfg = dict(CFG, stops=stops, strafe_max_mm_per_leg=None)
        obs = [(754, 1257, 25.0), (1523, 1247, 25.0)]
        res = {}
        for pf in (45, 0):
            legs, why, _ = rp.plan_mission(dict(cfg, yellow_prefer_floor_mm=pf), obs, (2208, 233, 90), ['QR', 'RAW', 'ROUGH'])
            self.assertEqual(why, 'OK')
            leg = legs[-1]
            self.assertTrue(any(c == 'F' and abs(v) >= 1500 for c, v in leg['cmds']), leg['cmds'])   # 从中间车道穿过去
            res[pf] = (leg['yellow_mm'], sum(L['time_s'] for L in legs))
        self.assertGreaterEqual(res[45][0], 45 - 0.1, res)
        self.assertLess(res[0][0], 40, res)                                  # 以前：贴着一边走
        self.assertLessEqual(res[45][1], res[0][1] + 2.0, res)

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
        # 车头朝北，车身后边离黄色区只有 10mm(最低要 35)
        self.assertIn('离黄色区不到 35mm', p.why_blocked(1600, 1850 + 145 + 10, 1))
        self.assertEqual(p.why_blocked(1600, 1850 + 145 + 35, 1), '')


class MidLaneShiftTests(unittest.TestCase):
    """(10-29 用户"从原料区域去粗加工区域…走中间这条路的时候靠近暂停区域45mm")去 ROUGH 走中间南北车道时整段往西(暂存区那边)挪。"""

    def plan(self, shift):
        c = dict(CFG, mid_lane_shift_mm=shift, no_jog_from=[])
        c['stops'] = dict(CFG['stops'], QR=[2100, 1250, 90], RAW=[1080, 2155, 180], ROUGH=[1130, 350, 0], TEMP=[310, 1250, -90])
        legs, why, _ = rp.plan_mission(c, [], (2198, 228, 90), ['QR', 'RAW', 'ROUGH', 'TEMP'])
        self.assertEqual(why, 'OK')
        return legs

    @staticmethod
    def lane_x(leg):
        xs = [p[0] for p, q in zip(leg['poses'], leg['poses'][1:]) if abs(p[0] - q[0]) < 0.5 and abs(p[1] - q[1]) > 1000]
        return xs[0]

    def test_rough_leg_moves_45_west_same_goal(self):
        old, new = self.plan(0), self.plan(45)
        self.assertAlmostEqual(self.lane_x(old[2]) - self.lane_x(new[2]), 45, delta=0.5)
        self.assertEqual(len(new[2]['cmds']), len(old[2]['cmds']))          # 不加指令，只改前后两条沿 x 的移动
        self.assertEqual(new[2]['goal'], old[2]['goal'])
        self.assertGreaterEqual(new[2]['yellow_mm'], rp.MID_LANE_YELLOW_MIN_MM - 0.1)
        self.assertIn('往暂存区那边挪 45mm', new[2]['note'])
        for a, b in zip(old, new):                                          # 别的段不动
            if a['stop'] != 'ROUGH':
                self.assertEqual(a['cmds'], b['cmds'])
        c = dict(CFG)
        ok, g, idx, what = rp.moves_clear(c, [], new[2]['start'], new[2]['cmds'], margin_mm=0)
        self.assertTrue(ok, what)

    def test_raw_stop_already_in_lane_and_no_x_move_after(self):
        # (10-29 实车：原料区停车点 x 1100、粗加工区 x 1180，以前前面那条沿 x 的移动或后面没有就不挪)
        c = dict(CFG, mid_lane_shift_mm=45, stop_exact=True, no_jog_from=[])
        c['stops'] = dict(CFG['stops'], QR=[2100, 1250, 90], RAW=[1100, 2080, 180], ROUGH=[1180, 300, 0], TEMP=[210, 1250, -90])
        legs, why, _ = rp.plan_mission(c, [], (2198, 228, 90), ['QR', 'RAW', 'ROUGH'])
        self.assertEqual(why, 'OK')
        L = legs[2]
        self.assertEqual(L.get('mid_lane_shift'), 45)
        self.assertEqual(tuple(round(v) for v in L['goal']), (1180, 300, 0))
        self.assertAlmostEqual(self.lane_x(L), 1135, delta=0.5)
        # midlane 80 -30：原料区后面那一下不走了(转完弯直接往下开)，按地图允许压进黄色区 30mm
        c.update(mid_lane_shift_mm=80, mid_lane_yellow_min_mm=-30)
        legs, why, _ = rp.plan_mission(c, [], (2198, 228, 90), ['QR', 'RAW', 'ROUGH'])
        L = legs[2]
        self.assertEqual(L['cmds'][0][0], 'R')
        self.assertAlmostEqual(self.lane_x(L), 1100, delta=0.5)
        self.assertIn('压进黄色区 30mm', L['note'])

    def test_shrinks_when_45_would_touch_yellow(self):
        legs = self.plan(80)
        self.assertGreaterEqual(legs[2]['yellow_mm'], rp.MID_LANE_YELLOW_MIN_MM - 0.1)
        self.assertLess(legs[2]['mid_lane_shift'], 80)


class StopExactStraightInTests(unittest.TestCase):
    """(10-29 用户"只要我改了这个点的位置这个路线就能重新规划…这个拐角应该是直接到走到前面的")stopexact on、TEMP 改到 x 210
    (按地图压进暂存区)：不再走到 x 310 再在停车点横移过去，转完弯先横移到 x 210、一条直行开进停车点。"""

    def legs(self, exact):
        c = dict(CFG, stop_exact=exact, mid_lane_shift_mm=0)
        c['stops'] = dict(CFG['stops'], QR=[2100, 1250, 90], RAW=[1200, 2100, 180], ROUGH=[1200, 300, 0], TEMP=[210, 1250, -90])
        legs, why, _ = rp.plan_mission(c, [(1594, 1250, 25), (732, 1195, 25)], (2198, 228, 90), ['QR', 'RAW', 'ROUGH', 'TEMP', 'RAW'])
        self.assertEqual(why, 'OK')
        return legs

    def test_temp_210_drives_straight_in(self):
        off, on = self.legs(False), self.legs(True)
        t = on[3]
        self.assertEqual(t['stop'], 'TEMP')
        self.assertEqual(tuple(round(v) for v in t['goal']), (210, 1250, -90))
        self.assertEqual(t['cmds'][-1][0], 'F')                     # 最后一条是直行开进去，不是横移
        self.assertLess(abs(t['poses'][-2][0] - 210), 0.5)          # 开进去之前已经在 x 210
        self.assertEqual(tuple(round(v) for v in off[3]['goal']), (310, 1250, -90))   # stopexact off 照旧挪到 310
        for a, b in zip(off[:3], on[:3]):                            # 别的段不变
            self.assertEqual(a['cmds'], b['cmds'])


class NoJogTests(unittest.TestCase):
    """(10-29 用户"原料区域左走一下和这个暂停区域这么走一下就压线了，你不要看地图了")从 RAW、TEMP 出发开头那一下不单独走。"""

    def test_raw_and_temp_departures_start_without_the_small_move(self):
        c = dict(CFG, stop_exact=True)
        c.pop('mid_lane_shift_mm', None)
        c.pop('no_jog_from', None)
        c['stops'] = dict(CFG['stops'], QR=[2100, 1250, 90], RAW=[1100, 2080, 180], ROUGH=[1180, 300, 0], TEMP=[210, 1250, -90])
        names = ['QR', 'RAW', 'ROUGH', 'TEMP', 'RAW']
        legs, why, _ = rp.plan_mission(c, [(1594, 1250, 25), (732, 1195, 25)], (2198, 228, 90), names)
        self.assertEqual(why, 'OK')
        rough, temp_raw = legs[2], legs[4]
        self.assertEqual(rough['cmds'][0][0], 'R')                       # 原料区：转完弯直接往下开
        self.assertTrue(rough.get('no_jog'))
        self.assertEqual(temp_raw['cmds'][0][0], 'F')                    # 暂存区：不先横移，直接往北开
        self.assertTrue(temp_raw.get('no_jog'))
        for L, n in zip(legs, names):
            self.assertEqual(tuple(round(v) for v in L['goal'][:2]), tuple(c['stops'][n][:2]))   # 停车点不变
        off = dict(c, no_jog_from=[])
        legs2, _, _ = rp.plan_mission(off, [(1594, 1250, 25), (732, 1195, 25)], (2198, 228, 90), names)
        self.assertEqual(legs2[2]['cmds'][0][0], 'F')                    # no_jog_from=[] 照旧
        self.assertEqual(legs2[4]['cmds'][0][0], 'S')


if __name__ == '__main__':
    unittest.main()
