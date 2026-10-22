"""route_plan.py 的安全检查测试(真实车身、各区域硬余量、两个启停区、准确停车、规划速度)：
python3 -m unittest test_route_safety"""
import math
import time
import unittest

import route_plan as rp

# 和您 10 月 6 日的 map_config_start2_roi.json 一样(路线规划用到的那些键)；启停区1 的车头按 180(朝西)
STOPS = {'QR': [2100, 1200, 90], 'RAW': [1200, 2100, 180], 'ROUGH': [1200, 340, 0], 'TEMP': [340, 1200, -90],
         'START1': [2250, 2250, 180], 'START2': [2250, 150, 90],
         'PREHOME2': [2250, 400, 90], 'PREHOME1': [2000, 2250, 180]}
CFG = dict(stops=STOPS, car_length_mm=290, car_width_mm=260, lidar_overhang_mm=40,
           lidar_forward_mm=22.453557227708068, lidar_left_mm=134.7743150753031,
           margin_mm=60, strafe_cost_factor=3.0, strafe_max_mm_per_leg=300, turn_margin_mm=20,
           raw_center_mm=[1200, 2480], raw_radius_mm=150, zone_margin_mm=30, drive_mode='mecanum',
           clearance_pref_mm=120, clearance_penalty=1.0, route_speed_rpm=150, route_acc_rpm_s=300,
           strafe_speed_rpm=140, strafe_acc_rpm_s=200, fwd_mm_per_rev=249.0, strafe_mm_per_rev=235.0,
           turn90_s=1.8, turn180_s=2.5, cmd_overhead_s=0.25, turn_pivot_back_mm=0,
           stm32_params={'SCACC': 200.0, 'SSACC': 140.0})
SIM_OBS = [(430, 265, 25), (2240, 1690, 25), (285, 2060, 25)]
BASE = ['QR', 'RAW', 'ROUGH', 'TEMP', 'RAW', 'ROUGH', 'TEMP']


def rot1(p):
    """启停区2 → 启停区1：绕场地中心转 +90°。"""
    return (2400 - p[1], p[0], p[2] + 90)


# 第二站实测停在 (2208,233)、车头转回起步方向后的规划起点；启停区1 是它转 90° 的位置
ZONES = {2: dict(start=(2208, 233, 90), pivot=(2208, 233), home='START2', pre='PREHOME2'),
         1: dict(start=(rot1((2208, 233, 0))[0], rot1((2208, 233, 0))[1], 180),
                 pivot=rot1((2208, 233, 0))[:2], home='START1', pre='PREHOME1')}


def run(legs, start):
    """把每段指令按航位推算走一遍，返回每段走完的位姿。"""
    pose = start; out = []
    for L in legs:
        for c, v in L['cmds']:
            pose = rp.apply_body_move(pose, c, v)
        out.append(pose)
    return out


class FootprintTests(unittest.TestCase):
    def test_real_body_and_lidar(self):
        b = rp.Body(CFG)
        self.assertEqual(b.box, (-145.0, 145.0, -130.0, 170.0))      # 左边(雷达那边)多出 40
        self.assertAlmostEqual(b.r_turn, math.hypot(145, 130), delta=0.1)
        self.assertAlmostEqual(b.disk[2], 35.2, delta=0.1)

    def test_lidar_side_counts(self):
        # 中间竖车道里，车身离西边黄色区 20mm(≥15)；车头朝北时雷达在西边、伸进黄色区，朝南时雷达在东边
        ok, g, what = rp.pose_clear(CFG, [], (1150, 775, 90))
        self.assertFalse(ok)
        self.assertIn('黄色区', what)
        ok, g, what = rp.pose_clear(CFG, [], (1150, 775, -90))
        self.assertTrue(ok, what)

    def test_all_stops_are_clear(self):
        for name, p in STOPS.items():
            ok, g, what = rp.pose_clear(CFG, [], p)
            self.assertTrue(ok, f'{name}: {what}')
        # 粗加工区/暂存区停车点：真车离白区 60mm
        self.assertAlmostEqual(rp.pose_clear(CFG, [], STOPS['ROUGH'])[1], 60, delta=0.5)
        self.assertAlmostEqual(rp.pose_clear(CFG, [], STOPS['TEMP'])[1], 60, delta=0.5)
        for z in ZONES.values():
            self.assertTrue(rp.pose_clear(CFG, [], z['start'])[0])

    def test_turntable_whole_range(self):
        # 转盘中心按 x 1100~1300 都可能：离 (1200,2480) 那一个点还有 10mm，但转盘挪到 x=1100、y=2465 就压上了
        ok, g, what = rp.pose_clear(CFG, [], (1100, 2190, 180))
        self.assertFalse(ok)
        self.assertIn('原料转盘', what)
        ok, g, what = rp.pose_clear(dict(CFG, raw_area_mm=[1200, 2480, 1200, 2480], raw_margin_mm=0), [], (1100, 2190, 180))
        self.assertTrue(ok, what)

    def test_field_edge_message_and_start_zone(self):
        ok, g, what = rp.pose_clear(CFG, [], (2250, 150, 90))
        self.assertTrue(ok)                       # 启停区里贴边可以
        self.assertAlmostEqual(g, 5, delta=0.5)
        ok, g, what = rp.pose_clear(CFG, [], (1800, 150, 90))
        self.assertFalse(ok)                      # 别的地方离场地边要 30
        self.assertIn('场地边', what)
        self.assertNotIn('障碍物', what)
        ok, g, what = rp.pose_clear(CFG, [], (1800, 100, 90))
        self.assertIn('出场地', what)
        pl = rp.Planner(CFG, [])
        self.assertIn('出场地', pl.why_blocked(1800, 100, 1))
        self.assertNotIn('障碍物', pl.why_blocked(1800, 100, 1))

    def test_margin_override_and_obstacles(self):
        self.assertFalse(rp.pose_clear(CFG, [], STOPS['QR'], margin_mm=200)[0])
        self.assertTrue(rp.pose_clear(CFG, [], STOPS['QR'], margin_mm=100)[0])
        ok, g, what = rp.pose_clear(CFG, [(2100, 1400, 25)], STOPS['QR'])
        self.assertFalse(ok)
        self.assertIn('障碍物(2100,1400)', what)
        self.assertTrue(rp.pose_clear(CFG, [(2100, 1600, 25)], STOPS['QR'])[0])


class MovesClearTests(unittest.TestCase):
    def test_straight_sweep_through_yellow(self):
        start = (2100, 700, 180)
        self.assertTrue(rp.pose_clear(CFG, [], start)[0])
        self.assertTrue(rp.pose_clear(CFG, [], (300, 700, 180))[0])
        ok, g, idx, what = rp.moves_clear(CFG, [], start, [('F', 1800)])
        self.assertFalse(ok)
        self.assertEqual(idx, 0)
        self.assertIn('黄色区', what)

    def test_turn_sweep(self):
        # 竖车道中间：停着、转完都放得下，但转的过程中车角扫到黄色区
        self.assertTrue(rp.pose_clear(CFG, [], (1200, 775, 90))[0])
        self.assertTrue(rp.pose_clear(CFG, [], (1200, 775, 180))[0])
        ok, g, idx, what = rp.moves_clear(CFG, [], (1200, 675, 90), [('F', 100), ('R', 90)])
        self.assertFalse(ok)
        self.assertEqual(idx, 1)
        self.assertIn('黄色区', what)
        # 路口正中转得开
        ok, g, idx, what = rp.moves_clear(CFG, [], (1200, 1200, 0), [('R', 90), ('R', 180)])
        self.assertTrue(ok, what)
        self.assertGreater(g, 60)
        # 转轴在车后 200mm：扫过的圆大很多，路口也转不开
        self.assertFalse(rp.moves_clear(CFG, [], (1200, 1200, 0), [('R', 90)], pivot=(200, 0))[0])

    def test_check_moves_compat(self):
        pl = rp.Planner(CFG, [])
        ok, msg, end = rp.check_moves(pl, (1200, 1200, 0), [('R', 90), ('F', 200)])
        self.assertTrue(ok, msg)
        self.assertAlmostEqual(end[1], 1400)
        ok, msg, end = rp.check_moves(pl, (2100, 700, 180), [('F', 1800)])
        self.assertFalse(ok)


    def test_explicit_margin_applies_to_turns_too(self):
        # 回家对准在启停区角落转几度：auto_run 问 margin_mm=0，转弯也只看真的会不会出界(不再另加 turn_margin_mm)
        cfg = dict(car_length_mm=290, car_width_mm=260, lidar_overhang_mm=40)
        ok, gap, _, _ = rp.moves_clear(cfg, [], (2250, 170, 90), [('R', 3)], margin_mm=0)
        self.assertTrue(ok)
        self.assertGreater(gap, 0)
        ok, _, _, _ = rp.moves_clear(cfg, [], (2250, 170, 90), [('R', 3)])      # 默认余量(规划用)照旧要 turn_margin_mm
        self.assertFalse(ok)

class MissionTests(unittest.TestCase):
    def check_mission(self, zone, obs, prehome):
        z = ZONES[zone]
        mission = BASE + ([z['pre']] if prehome else []) + [z['home']]
        cfg = dict(CFG)
        legs, why, pl = rp.plan_mission(cfg, obs, z['start'], mission, start_pivot=z['pivot'])
        tag = f'区{zone} 障碍物{len(obs)} PREHOME={prehome}'
        self.assertEqual(why, 'OK', tag)
        self.assertEqual([L['stop'] for L in legs], mission)
        # 每一段都准确停在停车点上
        for L, end in zip(legs, run(legs, z['start'])):
            g = STOPS[L['stop']]
            self.assertLess(math.hypot(end[0] - g[0], end[1] - g[1]), 0.6, f'{tag} {L["stop"]} {end}')
            self.assertAlmostEqual(rp.wrap180(end[2] - g[2]), 0, delta=1e-6)
            self.assertEqual(tuple(L['goal']), tuple(float(v) for v in g))
            for c, v in L['cmds']:
                self.assertIsInstance(v, int)
                if c in ('F', 'S'):
                    self.assertGreaterEqual(abs(v), rp.MIN_MOVE_MM, f'{tag} {L["stop"]} 有小移动 {c} {v}')
            self.assertGreaterEqual(L['yellow_mm'], 15 - 0.1)
        # 整条指令(按顺序接起来)按真实车身、硬余量扫一遍
        allc = [c for L in legs for c in L['cmds']]
        ok, g, idx, what = rp.moves_clear(cfg, obs, z['start'], allc)
        self.assertTrue(ok, f'{tag}: {what}')
        return legs

    def test_zone2_and_zone1_all_variants(self):
        for zone in (2, 1):
            for obs in ([], SIM_OBS):
                for pre in (False, True):
                    with self.subTest(zone=zone, obs=len(obs), prehome=pre):
                        legs = self.check_mission(zone, obs, pre)
                        if pre:
                            # PREHOME → START 只剩一条直线
                            self.assertEqual(len(legs[-1]['cmds']), 1)
                            self.assertEqual(legs[-1]['cmds'][0][0], 'F')

    def test_no_obstacles_keeps_full_yellow_margin(self):
        for zone in (2, 1):
            z = ZONES[zone]
            legs, why, _ = rp.plan_mission(CFG, [], z['start'], BASE + [z['home']], start_pivot=z['pivot'])
            self.assertEqual(why, 'OK')
            self.assertFalse(any(L['note'] for L in legs), [L['note'] for L in legs])
            self.assertGreaterEqual(min(L['yellow_mm'] for L in legs), 30 - 0.1)

    def test_planning_is_fast(self):
        # 改之前这台电脑上一次要 10~19 秒；现在每次几秒以内(给得很宽，机器忙的时候也能过)
        for zone in (2, 1):
            z = ZONES[zone]
            for obs in ([], SIM_OBS):
                t = time.monotonic()
                legs, why, _ = rp.plan_mission(dict(CFG, margin_mm=60), obs, z['start'], BASE + [z['home']], start_pivot=z['pivot'])
                dt = time.monotonic() - t
                self.assertEqual(why, 'OK')
                self.assertLess(dt, 8.0, f'区{zone} 障碍物{len(obs)} 规划用了 {dt:.1f} 秒')


class StopTests(unittest.TestCase):
    def test_stop_too_close_to_zone_says_so(self):
        cfg = dict(CFG, stops=dict(STOPS, ROUGH=[1200, 300, 0]))     # 真车离粗加工区只有 20mm
        legs, why, _ = rp.plan_mission(cfg, [], STOPS['QR'], ['ROUGH', 'TEMP'])
        self.assertEqual(why, 'OK')
        self.assertIn('粗加工区', legs[0]['note'])
        self.assertIn('20mm', legs[0]['note'])
        self.assertIn('粗加工区', legs[1]['note'])         # 下一段从这里出发，也说清楚
        end = run(legs, STOPS['QR'])[0]
        self.assertLess(math.hypot(end[0] - 1200, end[1] - 300), 0.6)

    def test_stop_inside_yellow_fails_with_reason(self):
        cfg = dict(CFG, stops=dict(STOPS, BAD=[1600, 1600, 0]))
        legs, why, _ = rp.plan_mission(cfg, [], STOPS['QR'], ['BAD'])
        self.assertNotEqual(why, 'OK')
        self.assertIn('BAD', why)
        self.assertIn('黄色区', why)

    def test_stop_blocked_by_obstacle_parks_nearby(self):
        legs, why, _ = rp.plan_mission(CFG, [(2100, 1420, 25)], (2100, 600, 90), ['QR'])
        self.assertEqual(why, 'OK')
        self.assertIn('被障碍物(2100,1420)挡住', legs[0]['note'])
        g = legs[0]['goal']
        self.assertTrue(rp.pose_clear(CFG, [(2100, 1420, 25)], g)[0])
        end = run(legs, (2100, 600, 90))[0]
        self.assertLess(math.hypot(end[0] - g[0], end[1] - g[1]), 0.6)


class StartTests(unittest.TestCase):
    def test_start_close_to_obstacle_only_relaxes_that_obstacle(self):
        # 车已经停在离障碍物 37mm 的地方(要 60)：离开时只对这个障碍物放宽到 37，并写进 note
        obs = [(2073, 440, 25)]
        legs, why, _ = rp.plan_mission(CFG, obs, (2208, 233, 90), ['QR'], start_pivot=(2208, 233))
        self.assertEqual(why, 'OK')
        self.assertIn('起点离障碍物(2073,440)只有', legs[0]['note'])
        ok, g, idx, what = rp.moves_clear(dict(CFG, margin_mm=36), obs, (2208, 233, 90), legs[0]['cmds'])
        self.assertTrue(ok, what)

    def test_grid_api_compat(self):
        pl = rp.Planner(CFG, [])
        acts, why = pl.plan_leg((2100, 1200, 90), (1200, 2100, 180))
        self.assertEqual(why, 'OK')
        cmds, _ = rp.to_commands((2100, 1200, 90), acts)
        end = run([dict(cmds=cmds)], (2100, 1200, 90))[0]
        self.assertEqual((round(end[0]), round(end[1])), (1200, 2100))
        self.assertTrue(pl.point_free(2100, 1200, 1))
        self.assertFalse(pl.point_free(1600, 1600, 1))


class PlanLegTests(unittest.TestCase):
    def test_home_leg_from_a_stop(self):
        leg = rp.plan_leg(CFG, [], STOPS['TEMP'], 'START2')
        self.assertEqual(leg['stop'], 'START2')
        end = run([leg], STOPS['TEMP'])[0]
        self.assertLess(math.hypot(end[0] - 2250, end[1] - 150), 0.6)
        self.assertTrue(rp.moves_clear(CFG, [], STOPS['TEMP'], leg['cmds'])[0])

    def test_pose_goal_and_start_pivot(self):
        z = ZONES[2]
        leg = rp.plan_leg(CFG, SIM_OBS, z['start'], (2100, 1200, 90), start_pivot=z['pivot'])
        self.assertEqual(leg['stop'], 'GOAL')
        end = run([leg], z['start'])[0]
        self.assertLess(math.hypot(end[0] - 2100, end[1] - 1200), 0.6)

    def test_start_heading_snaps_or_refuses(self):
        leg = rp.plan_leg(CFG, [], (2100, 1200, 88.5), 'RAW')
        self.assertEqual(leg['stop'], 'RAW')
        with self.assertRaises(ValueError):
            rp.plan_leg(CFG, [], (2100, 1200, 45), 'RAW')

    def test_impossible_goal_raises(self):
        with self.assertRaises(ValueError) as cm:
            rp.plan_leg(CFG, [], STOPS['QR'], (1600, 1600, 0))
        self.assertIn('黄色区', str(cm.exception))


class MotionTests(unittest.TestCase):
    def test_time_model_uses_stm32_accelerations(self):
        m = rp.Motion(CFG)
        self.assertAlmostEqual(m.a_f, 200 / 60 * 249.0)
        self.assertAlmostEqual(m.a_s, 140 / 60 * 235.0)
        m = rp.Motion(dict(CFG, stm32_params={}))
        self.assertAlmostEqual(m.a_f, 300 / 60 * 249.0)


if __name__ == '__main__':
    unittest.main()
