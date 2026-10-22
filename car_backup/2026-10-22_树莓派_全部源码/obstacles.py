"""静止雷达圆柱候选诊断；不识别颜色、不提供地图定位或通行保证。"""
import math
import numpy as np


def clusters(points, minimum=100, maximum=4000):
    # 按角度邻接分组；跨过过大的角度空隙或距离突变就分开。
    polar = sorted((float(a) % 360, float(d)) for a, d, _, _ in points
                   if math.isfinite(float(a)) and math.isfinite(float(d)) and minimum <= d <= maximum)
    if not polar:
        return []
    items = [(a, d, np.array([d*math.cos(math.radians(a)),
                              d*math.sin(math.radians(a))])) for a, d in polar]

    def adjacent(a, b):
        gap = (b[0]-a[0]) % 360
        limit = min(70.0, 15.0 + 1.8*min(a[1], b[1])*math.radians(gap))
        return gap <= 2.5 and np.linalg.norm(b[2]-a[2]) <= limit

    groups = [[items[0]]]
    for prev, item in zip(items, items[1:]):
        if adjacent(prev, item):
            groups[-1].append(item)
        else:
            groups.append([item])
    if len(groups) > 1 and adjacent(items[-1], items[0]):
        groups[0] = groups[-1] + groups[0]
        groups.pop()
    return [np.array([p[2] for p in group]) for group in groups if len(group) >= 2]


def fit_known_circle(xy, diameter):
    """已知半径的可见圆弧拟合。圆弧太少时拒绝输出中心。"""
    radius = diameter/2
    mean = xy.mean(axis=0)
    norm = np.linalg.norm(mean)
    span = np.linalg.norm(np.ptp(xy, axis=0))
    if norm <= radius or not 0.30*diameter <= span <= 1.35*diameter:
        return None
    center = mean + mean/norm*radius
    for _ in range(15):
        diff = center-xy
        distances = np.linalg.norm(diff, axis=1)
        if np.any(distances < 1e-6):
            return None
        jac = diff/distances[:, None]
        delta, _, rank, _ = np.linalg.lstsq(jac, distances-radius, rcond=None)
        if rank < 2 or not np.all(np.isfinite(delta)):
            return None
        center -= delta
        if np.linalg.norm(delta) < .01:
            break
    errors = np.linalg.norm(xy-center, axis=1)-radius
    rms = float(np.sqrt(np.mean(errors**2)))
    # 只接受中心在回波后方、绝大部分点位于朝向雷达的可见半圆。
    facing = (xy-center) @ (-center)
    if (np.linalg.norm(center) <= norm or np.mean(facing >= 0) < .85 or
            rms > max(3.0, diameter*.08) or np.linalg.norm(center-mean) > radius*1.4):
        return None
    return center, rms


def detect(points, diameter=0, minimum=100, maximum=4000):
    """2/3-point returns are SURFACE candidates, never fitted cylinder centers.
    No intensity/color classification. Detection range is independent of GUI zoom.
    """
    if not math.isfinite(diameter) or diameter < 0:
        raise ValueError('diameter must be finite and nonnegative')
    result = []
    for xy in clusters(points, minimum, maximum):
        width = float(np.linalg.norm(np.ptp(xy, axis=0)))
        nearest = float(np.linalg.norm(xy, axis=1).min())
        fit = fit_known_circle(xy, diameter) if diameter and len(xy) >= 4 else None
        sparse = fit is None
        if sparse:
            # A compact return is useful to display, but cannot establish a cylinder.
            # Do not admit 80-100mm clusters as D50 candidates.
            limit = diameter + 10.0 if diameter else 250.0
            if not 3 <= width <= limit:
                continue
            pos = xy.mean(axis=0)
            # Covers observed spread plus nominal radius and a provisional uncertainty.
            # This is a preview envelope, NOT a guaranteed complete object footprint.
            preview_radius = float(np.linalg.norm(xy-pos, axis=1).max()) + diameter/2 + 15
        else:
            pos = fit[0]
            preview_radius = diameter/2 + 10
        result.append(dict(x=float(pos[0]), y=float(pos[1]),
                           nearest_mm=nearest, width_mm=width, count=len(xy),
                           kind='circle_estimate' if fit else 'surface_cluster',
                           sparse=sparse, preview_radius_mm=preview_radius,
                           diameter_mm=diameter, rms_mm=fit[1] if fit else None))
    return result


class Tracker:
    """连续三帧的候选；漏检立即不显示旧位置。ID仅本次运行有效。"""
    def __init__(self):
        self.previous = []
        self.next_id = 1

    def clear(self):
        self.previous = []

    def update(self, detections):
        available = list(self.previous)
        current = []
        for item in detections:
            item = dict(item)
            distances = [math.hypot(item['x']-p['x'], item['y']-p['y']) for p in available]
            j = int(np.argmin(distances)) if distances else -1
            if j >= 0 and distances[j] <= 60 + .02*math.hypot(item['x'], item['y']):
                old = available.pop(j)
                item.update(id=old['id'], hits=old['hits']+1)
            else:
                item.update(id=self.next_id, hits=1)
                self.next_id += 1
            item['stable'] = item['hits'] >= (5 if item.get('sparse') else 3)
            current.append(item)
        self.previous = current
        return current
