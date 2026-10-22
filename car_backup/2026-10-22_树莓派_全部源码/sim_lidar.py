#!/usr/bin/env python3
"""桌面模拟雷达：按“真实”车位、安装参数和障碍物，生成和LD14P一样格式的帧。
只用于没车时测试程序流程，不代表真实雷达效果。
会模拟：圆柱遮挡、车身挡住另一侧（雷达装在车侧面时的盲区）、原料转盘、测距噪声。
"""
import math
import time
import numpy as np


def world_to_sdk(pts, pose, c):
    """地图坐标 -> SDK坐标（to_map 的逆变换）。"""
    x, y, yaw = pose
    p = np.asarray(pts, dtype=float).reshape(-1, 2) - [x, y]
    a = math.radians(-yaw); co, si = math.cos(a), math.sin(a)
    body = np.c_[p[:, 0]*co - p[:, 1]*si, p[:, 0]*si + p[:, 1]*co]
    body -= [c['lidar_forward_mm'], c['lidar_left_mm']]
    a = math.radians(-c['lidar_yaw_deg']); co, si = math.cos(a), math.sin(a)
    sdk = np.c_[body[:, 0]*co - body[:, 1]*si, body[:, 0]*si + body[:, 1]*co]
    sdk[:, 1] *= c['sdk_y_sign']
    return sdk


class SimStream:
    """接口和 ld14p_scan.Stream 一样：snapshot()、proc.poll()、close()。"""
    def __init__(self, mount, obstacles, raw_center=(1200, 2480), raw_radius=150,
                 car_len=290, car_wid=260, noise_mm=4, hz=6.0, seed=1):
        self.mount = mount; self.obstacles = list(obstacles)
        self.raw = (raw_center[0], raw_center[1], raw_radius)
        self.car = (car_len, car_wid)
        self.noise = noise_mm; self.hz = hz
        self.rng = np.random.default_rng(seed)
        self.pose = None; self.count = 0; self.t0 = time.monotonic()
        self.proc = self

    def poll(self):
        return None

    def set_true_pose(self, pose):
        self.pose = tuple(pose)

    def _frame(self):
        c = self.mount
        circles = [(x, y, r) for x, y, r in self.obstacles] + [self.raw]
        centers = world_to_sdk([(x, y) for x, y, _ in circles], self.pose, c)
        radii = [r for _, _, r in circles]
        # 车身矩形在SDK坐标下的四个角
        L, W = self.car
        x, y, yaw = self.pose; a = math.radians(yaw)
        corners = [(x + bx*math.cos(a) - by*math.sin(a), y + bx*math.sin(a) + by*math.cos(a))
                   for bx, by in ((L/2, W/2), (-L/2, W/2), (-L/2, -W/2), (L/2, -W/2))]
        body = world_to_sdk(corners, self.pose, c)
        pts = []
        for ang in np.arange(0, 360, 0.54):
            u = np.array([math.cos(math.radians(ang)), math.sin(math.radians(ang))])
            best = math.inf
            for cen, r in zip(centers, radii):
                along = u @ cen; disc = along**2 - (cen @ cen - r*r)
                if disc >= 0 and along > 0:
                    best = min(best, along - math.sqrt(disc))
            for k in range(4):                     # 射线打到车身
                p0, p1 = body[k], body[(k+1) % 4]
                e = p1 - p0; m = np.array([[u[0], -e[0]], [u[1], -e[1]]])
                if abs(np.linalg.det(m)) < 1e-9:
                    continue
                t, s = np.linalg.solve(m, p0)
                if t > 1 and 0 <= s <= 1:
                    best = min(best, t)
            if best < 4000:
                d = best + self.rng.normal(0, self.noise)
                pts.append([float(ang), int(max(d, 1)), 200, 0])
        return dict(seq=self.count, hz=self.hz, points=pts)

    def snapshot(self):
        if self.pose is None:
            return None, 0.0, 0
        t = time.monotonic() - 0.05      # 和真雷达一样：返回的是“收到这一帧的时间”，比调用时刻稍早
        time.sleep(1/self.hz)
        self.count += 1
        return self._frame(), t, self.count

    def close(self):
        pass
