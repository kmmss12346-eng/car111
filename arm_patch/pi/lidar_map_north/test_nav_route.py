"""用真的路线规划(route_plan.plan_mission，您 10 月 6 日的配置 + 模拟障碍物)跑一整轮的导航测试(1010)，两个启停区都跑：
模拟的车有距离误差(1.5%)、横移串动、陀螺仪漂移；每个停车点、紧的转弯前雷达定位(这里按真实位置加 1mm 噪声)。
检查：离开启停区以后车身(含雷达)一直离黄区/工位区/转盘/障碍物/场地边至少 5mm，最后停在启停区里(目标往场内挪了 5mm)。
规划一次要十几秒，所以这个测试慢一点。python3 -m unittest test_nav_route"""
import copy
import json
import math
import os
import unittest

import test_auto_run  # noqa: F401  (先装好 stm32_link / route_plan 的替身)
import auto_run
from test_nav_drive import SimCar, SimCtx, Hooks

HERE = os.path.dirname(os.path.abspath(__file__))
USER_CFG = os.path.join(HERE, '..', '..', '..', 'docs', 'pi_snapshot_1006', 'map_config_start2_roi.json')


@unittest.skipUnless(os.path.exists(USER_CFG) and hasattr(auto_run._rp, 'plan_mission'), '没有配置文件或 route_plan')
class RealRouteTests(unittest.TestCase):
    def run_zone(self, zone, car_kw):
        with open(USER_CFG, encoding='utf-8') as f:
            raw = json.load(f)
        snap = auto_run.zone_snapshot(raw)
        cfg = copy.deepcopy(raw)
        s = auto_run.apply_zone(cfg, zone, snap)
        names = auto_run.mission_names(cfg)
        self.assertEqual(names[-2:], ['PREHOME', f'START{zone}'])
        obs = [tuple(o) for o in raw['sim_obstacles']]
        x0, y0, h0 = s['car']
        a = math.radians(h0)
        x2, y2 = x0 + 83 * math.cos(a) - 42 * math.sin(a), y0 + 83 * math.sin(a) + 42 * math.cos(a)   # 第二站
        legs, why, _ = auto_run._rp.plan_mission(dict(cfg, margin_mm=60), obs, (x2, y2, h0), names, start_pivot=(x2, y2))
        self.assertEqual(why, 'OK')
        car = SimCar((x2, y2, h0 - 60), rdec=True, **car_kw)
        car.T = h0 - 60
        ctx = SimCtx(car, obstacles=obs)
        logs = []
        auto_run.drive(ctx, car, auto_run.flatten(60.0, legs), log=logs.append, stop_wait=0, hooks=Hooks(car),
                       speeds={'F': 150, 'S': 140}, motion=auto_run.Motion(cfg), cfg=cfg,
                       start=dict(est=(x2, y2, h0 - 60), ref=(x2, y2, h0 - 60), yaw_real=h0 - 60), caps={'rdec': True})
        m = auto_run.body_model(cfg)
        regs = auto_run.regions(cfg, obs)
        k0 = next(i for i, q in enumerate(car.trace) if not auto_run.in_rect(q, auto_run.START_RECTS[zone]))
        worst = min(auto_run.pose_clearance(cfg, obs, q, m, regs)[0] for q in car.trace[k0:])
        goal = auto_run.inset_goal(s['start_stop'], cfg)
        return car, logs, worst, goal

    def check(self, car, logs, worst, goal, zone):
        self.assertGreaterEqual(worst, 5.0, '车身离黄区/工位区/转盘/障碍物/场地边太近')
        p = car.pose()
        self.assertLess(math.hypot(p[0] - goal[0], p[1] - goal[1]), 8.0, p)
        self.assertTrue(auto_run.in_rect(p, auto_run.START_RECTS[zone]))
        self.assertGreaterEqual(sum('雷达定位：车在' in l for l in logs), 8)

    def test_zone2(self):
        car, logs, worst, goal = self.run_zone(2, dict(scale=0.015, drift_deg=0.08, resid_deg=0.3, slip=0.01))
        self.check(car, logs, worst, goal, 2)

    def test_zone1(self):
        car, logs, worst, goal = self.run_zone(1, dict(scale=-0.012, drift_deg=-0.08, resid_deg=0.3, slip=-0.01))
        self.check(car, logs, worst, goal, 1)


if __name__ == '__main__':
    unittest.main()
