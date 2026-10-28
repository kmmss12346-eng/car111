"""写死路线(fixroute，fixed_route.py)的测试：存什么、读回来是不是一样、什么时候不让存/不让跑。
python3 -m unittest test_fixed_route"""
import json
import os
import shutil
import tempfile
import unittest

import numpy as np

import fixed_route


def _state(**kw):
    legs = [dict(stop='QR', start=(2208.0, 233.0, 90.0), goal=(2100.0, 1200.0, 90.0), cmds=[('F', 967), ('S', 108)],
                 poses=[(2100.0, 1200.0, 90.0)], note='', time_s=np.float64(3.7), goal_shift=None),
            dict(stop='START2', start=(2100.0, 1200.0, 90.0), goal=(2250.0, 150.0, 90.0), cmds=[('R', -90), ('F', np.int64(900))],
                 poses=[(2250.0, 150.0, 90.0)], note='', time_s=4.0)]
    st = dict(legs=legs, reason='ROUTE OK', legs_partial=False, turn_back=60.0, plan_start=None, start_pivot=(2208.0, 233.0),
              route_ref=(2208.0, 235.0, 30.0), path=[(2208.0, 233.0), (2100.0, 1200.0)], plan_margin=60.0,
              plan_obs=[(1650.0, 1250.0, 25.0)], sealed=[], seal_soft=True, seal_s=2.0, box=(145.0, 145.0, -130.0, 160.0),
              strafe_fallback=False, lane_fallback=False)
    st.update(kw)
    return st


VIEWS = [dict(config=dict(car_x_mm=2250.0, car_y_mm=150.0, car_yaw_deg=90.0), raw_frames=[{'points': [[10.0, 900, 200, 1]], 'seq': 1}],
              field_returns=[[1.0, 2.0]], source='unchanged_external_pose'),
         dict(config=dict(car_x_mm=2208.0, car_y_mm=235.0, car_yaw_deg=30.0), raw_frames=[{'points': [[11.0, 905, 200, 2]], 'seq': 2}],
              field_returns=[], source='second_relative_default')]
OBJS = [dict(id=1, x=1650.0, y=1250.0, radius=25.0, seen=9, count=40, anchor_view=1, ambiguous=False)]


class FixedRouteTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='fixroute_')
        self.path = fixed_route.path_for(os.path.join(self.tmp, 'map_config_start2_roi.json'))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_saved_next_to_the_config(self):
        self.assertEqual(self.path, fixed_route.path_for(os.path.join(self.tmp, 'cfg.json')))
        self.assertEqual(self.path.name, 'fixed_route.json')
        self.assertEqual(str(self.path.parent), self.tmp)

    def test_roundtrip_gives_back_the_same_route(self):
        snap = fixed_route.snapshot(_state(), VIEWS, OBJS, 2)
        fixed_route.save(self.path, snap)
        back = fixed_route.load(self.path)
        self.assertEqual(back['zone'], 2)
        self.assertEqual([L['stop'] for L in back['legs']], ['QR', 'START2'])
        self.assertEqual(back['legs'][0]['cmds'], [('F', 967), ('S', 108)])
        self.assertEqual(back['legs'][1]['cmds'], [('R', -90), ('F', 900)])
        self.assertEqual(back['legs'][0]['goal'], (2100.0, 1200.0, 90.0))
        self.assertIsInstance(back['legs'][0]['poses'][0], tuple)
        self.assertEqual(back['plan_start'], (2208.0, 233.0, 90.0), '跑完 plan_start 清掉了：按第一段的起点存')
        self.assertEqual(back['route_ref'], (2208.0, 235.0, 30.0))
        self.assertEqual(back['turn_back'], 60.0)
        self.assertEqual(back['plan_obs'], [(1650.0, 1250.0, 25.0)])
        self.assertEqual(back['views'][1]['config']['car_yaw_deg'], 30.0)
        self.assertEqual(back['views'][0]['raw_frames'][0]['points'][0], [10.0, 900, 200, 1])
        self.assertEqual(back['objects'], OBJS)
        for k in fixed_route.STATE_KEYS:
            self.assertIn(k, back)

    def test_only_the_first_two_scans_are_kept(self):
        snap = fixed_route.snapshot(_state(), VIEWS + [dict(VIEWS[1], source='station_QR')], OBJS, 2)
        self.assertEqual(len(snap['views']), 2)

    def test_resave_keeps_a_backup(self):
        fixed_route.save(self.path, fixed_route.snapshot(_state(), VIEWS, OBJS, 2))
        fixed_route.save(self.path, fixed_route.snapshot(_state(turn_back=-60.0), VIEWS, OBJS, 1))
        self.assertEqual(fixed_route.load(self.path)['zone'], 1)
        with open(str(self.path) + '.bak', encoding='utf-8') as f:
            self.assertEqual(json.load(f)['zone'], 2)
        self.assertFalse(os.path.exists(str(self.path) + '.tmp'))

    def test_refuses_to_save_without_a_full_route(self):
        with self.assertRaisesRegex(ValueError, '还没有两次扫描'):
            fixed_route.snapshot(_state(), VIEWS[:1], OBJS, 2)
        with self.assertRaisesRegex(ValueError, '没有规划好的路线'):
            fixed_route.snapshot(_state(legs=[], reason='NOT SCANNED'), VIEWS, OBJS, 2)
        with self.assertRaisesRegex(ValueError, '没有规划好的路线'):
            fixed_route.snapshot(_state(reason='NO PATH'), VIEWS, OBJS, 2)
        with self.assertRaisesRegex(ValueError, '后半段'):
            fixed_route.snapshot(_state(legs_partial=True), VIEWS, OBJS, 2)
        with self.assertRaisesRegex(ValueError, '哪个车位'):
            fixed_route.snapshot(_state(route_ref=None), VIEWS, OBJS, 2)

    def test_load_says_what_to_do(self):
        with self.assertRaisesRegex(ValueError, '还没存写死路线.*fixroute save'):
            fixed_route.load(self.path)
        self.path.write_text('{坏的', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, '读不出来'):
            fixed_route.load(self.path)
        self.path.write_text(json.dumps(dict(version=99, legs=[1], views=[1, 2])), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, '不是这一版'):
            fixed_route.load(self.path)

    def test_zone_must_match(self):
        snap = fixed_route.snapshot(_state(), VIEWS, OBJS, 2)
        self.assertIsNone(fixed_route.check(snap, 2))
        why = fixed_route.check(snap, 1)
        self.assertIn('写死路线是启停区 2 的，现在是启停区 1', why)
        self.assertIn('race 2', why)

    def test_describe(self):
        d = fixed_route.describe(fixed_route.snapshot(_state(), VIEWS, OBJS, 2))
        self.assertIn('启停区 2', d)
        self.assertIn('2 站 QR→START2', d)
        self.assertIn('障碍物 1 个', d)
        self.assertIn('预计行驶 8 秒', d)


if __name__ == '__main__':
    unittest.main()
