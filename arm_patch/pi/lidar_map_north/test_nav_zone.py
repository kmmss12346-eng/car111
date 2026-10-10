"""启停区 1/2 切换的测试(1010)：区 1 = 区 2 绕场地中心转 90°，第二站动作原样用；配置里可以逐项指定；
写回配置文件时启停区相关的键保持原样。python3 -m unittest test_nav_zone"""
import copy
import json
import os
import unittest

import test_auto_run  # noqa: F401  (先装好 stm32_link / route_plan 的替身)
import auto_run

HERE = os.path.dirname(os.path.abspath(__file__))
USER_CFG = os.path.join(HERE, '..', '..', '..', 'docs', 'pi_snapshot_1006', 'map_config_start2_roi.json')


def user_cfg():
    if os.path.exists(USER_CFG):
        with open(USER_CFG, encoding='utf-8') as f:
            return json.load(f)
    return dict(car_x_mm=2250, car_y_mm=150, car_yaw_deg=90, start_zone=2,
                second_relative_moves={'left_mm': 42, 'forward_mm': 83, 'cw_deg': 60},
                stops={'QR': [2100, 1200, 90], 'START1': [2250, 2250, 90], 'START2': [2250, 150, 90]},
                mission=['QR', 'RAW', 'ROUGH', 'TEMP', 'RAW', 'ROUGH', 'TEMP', 'START'], _second_pose_mm_old=[2208, 233, 33.0])


def close(a, b, tol=1e-6):
    return all(abs(float(x) - float(y)) <= tol for x, y in zip(a, b))


class ZonePoseTests(unittest.TestCase):
    def test_rotation_about_field_centre(self):
        self.assertTrue(close(auto_run.zone_pose((2250, 150, 90), 2, 1), (2250, 2250, 180)))
        self.assertTrue(close(auto_run.zone_pose((2250, 400, 90), 2, 1), (2000, 2250, 180)))
        self.assertTrue(close(auto_run.zone_pose((2208, 233, 33), 2, 1), (2167, 2208, 123)))
        for p in [(2250, 150, 90), (2100, 1200, 90), (340, 1200, -90)]:
            q = auto_run.zone_pose(auto_run.zone_pose(p, 2, 1), 1, 2)
            self.assertTrue(close(q, p), (p, q))
        self.assertTrue(close(auto_run.zone_pose((1, 2, 3), 2, 2), (1, 2, 3)))


class ZoneSettingsTests(unittest.TestCase):
    def setUp(self):
        self.raw = user_cfg()
        self.snap = auto_run.zone_snapshot(self.raw)

    def test_zone2_from_user_config(self):
        s = auto_run.zone_settings(self.snap, 2)
        self.assertTrue(close(s['car'], (2250, 150, 90)))
        self.assertEqual(s['start_name'], 'START2')
        self.assertTrue(close(s['start_stop'], (2250, 150, 90)))
        self.assertTrue(close(s['prehome'], (2250, 400, 90)))
        self.assertEqual(s['rel'], {'left_mm': 42, 'forward_mm': 83, 'cw_deg': 60})

    def test_zone1_is_zone2_rotated(self):
        s = auto_run.zone_settings(self.snap, 1)
        self.assertTrue(close(s['car'], (2250, 2250, 180)), '区 1 车头朝西')
        self.assertEqual(s['start_name'], 'START1')
        self.assertTrue(close(s['start_stop'], (2250, 2250, 180)), 'START1 的车头要和起点一样(180)，不是配置里的 90')
        self.assertTrue(close(s['prehome'], (2000, 2250, 180)))
        self.assertEqual(s['rel'], {'left_mm': 42, 'forward_mm': 83, 'cw_deg': 60}, '第二站动作原样用')
        self.assertFalse(s['use_p2'])
        self.assertIsNone(s['second_pose'])

    def test_second_pose_is_rotated(self):
        snap = dict(self.snap, second_pose_mm=[2208, 233, 33.0])
        s = auto_run.zone_settings(snap, 1)
        self.assertTrue(close(s['second_pose'], (2167, 2208, 123)))

    def test_explicit_overrides(self):
        ov = {'1': {'car_x_mm': 2260, 'second_relative_moves': {'left_mm': 40, 'forward_mm': 80, 'cw_deg': 60},
                    'stops': {'PREHOME': [2000, 2240, 180]}, 'second_use_p2': False}}
        s = auto_run.zone_settings(self.snap, 1, ov)
        self.assertTrue(close(s['car'], (2260, 2250, 180)))
        self.assertTrue(close(s['start_stop'], (2260, 2250, 180)), '起点改了，START 跟着改')
        self.assertTrue(close(s['prehome'], (2000, 2240, 180)))
        self.assertEqual(s['rel']['left_mm'], 40)
        s2 = auto_run.zone_settings(self.snap, 2, ov)
        self.assertTrue(close(s2['car'], (2250, 150, 90)), '区 2 不受区 1 的设置影响')

    def test_prehome_can_be_disabled(self):
        s = auto_run.zone_settings(self.snap, 2, {'2': {'stops': {'PREHOME': None}}})
        self.assertIsNone(s['prehome'])
        s = auto_run.zone_settings(self.snap, 1, None, prehome_mm=0)
        self.assertIsNone(s['prehome'])

    def test_file_written_for_zone1_derives_zone2(self):
        raw1 = copy.deepcopy(self.raw)
        auto_run.apply_zone(raw1, 1, self.snap)
        snap1 = auto_run.zone_snapshot(raw1)                 # 好比配置文件里写的是区 1
        s = auto_run.zone_settings(snap1, 2)
        self.assertTrue(close(s['car'], (2250, 150, 90)))
        self.assertTrue(close(s['prehome'], (2250, 400, 90)))

    def test_bad_zone(self):
        with self.assertRaises(ValueError):
            auto_run.zone_settings(self.snap, 3)


class ApplyZoneTests(unittest.TestCase):
    def test_switch_in_place_and_restore_for_file(self):
        raw = user_cfg()
        orig = copy.deepcopy(raw)
        rel = raw['second_relative_moves']
        snap = auto_run.zone_snapshot(raw)
        auto_run.apply_zone(raw, 1, snap)
        self.assertEqual(raw['start_zone'], 1)
        self.assertEqual((raw['car_x_mm'], raw['car_y_mm'], raw['car_yaw_deg']), (2250.0, 2250.0, 180.0))
        self.assertIs(raw['second_relative_moves'], rel, '第二站动作的 dict 要原地改(map_merge_live 里一直引用同一个)')
        self.assertEqual(raw['stops']['START1'], [2250.0, 2250.0, 180.0])
        self.assertEqual(raw['stops']['PREHOME'], [2000.0, 2250.0, 180.0])
        self.assertFalse(raw['second_use_p2'])
        self.assertEqual(raw['stops']['QR'], orig['stops']['QR'], '工位停车点不变')
        self.assertEqual(auto_run.mission_names(raw)[-2:], ['PREHOME', 'START1'])
        auto_run.apply_zone(raw, 2, snap)
        self.assertEqual(raw['stops']['PREHOME'], [2250.0, 400.0, 90.0])
        self.assertEqual(auto_run.mission_names(raw)[-2:], ['PREHOME', 'START2'])
        out = auto_run.zone_restore(copy.deepcopy(raw), snap)
        self.assertEqual(out, orig, '写回文件的内容和原来一样(切换启停区只在内存里)')

    def test_mission_names_without_prehome(self):
        raw = dict(start_zone=2, stops={'START2': [2250, 150, 90]}, mission=['QR', 'START'])
        self.assertEqual(auto_run.mission_names(raw), ['QR', 'START2'])
        raw['stops']['PREHOME'] = [2250, 400, 90]
        raw['mission'] = ['QR', 'PREHOME', 'START']
        self.assertEqual(auto_run.mission_names(raw), ['QR', 'PREHOME', 'START2'])


if __name__ == '__main__':
    unittest.main()
