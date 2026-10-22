"""两次扫描的位置校正，以及在起点用已知圆柱标定雷达安装参数。都不控制电机。
v10：去掉了“路线中途用雷达重新定位”(locate/candidates)——路线只靠 STM32 的陀螺仪和电机脉冲；这里只留第二站扫描合并用的 refine_second。"""
import math
import numpy as np


def _rigid_fit(P, Q):
    """求 R、t 使 R@P+t ≈ Q（二维，最小二乘）。返回 (角度deg, R, t, 最大残差)。"""
    P = np.asarray(P, float); Q = np.asarray(Q, float)
    pc, qc = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - pc).T @ (Q - qc))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1] *= -1; R = Vt.T @ U.T
    t = qc - pc @ R.T
    res = float(np.max(np.linalg.norm(P @ R.T + t - Q, axis=1)))
    return math.degrees(math.atan2(R[1, 0], R[0, 0])), R, t, res


def relocalize(anchors, mapping, max_pair_mm=300, max_angle=10, max_res=40, max_shift_1=250, trust_yaw=False):
    """第二次扫描的障碍物和第一次的锚点配对；>=2对时求整体修正。
    trust_yaw=True：车头角度已经用陀螺仪实测(比两个远处圆柱拟合出来的角度准得多)，只修位置。"""
    if not anchors:
        return None, '第一次没有锚点，第二站按推算位置'
    pairs = []
    for d in mapping.latched:
        o = min(anchors, key=lambda o: math.hypot(o['x'] - d['x'], o['y'] - d['y']))
        if math.hypot(o['x'] - d['x'], o['y'] - d['y']) <= max_pair_mm:
            pairs.append(((d['x'], d['y']), (o['x'], o['y'])))
    if not pairs:
        return None, '两次没有看到同一个障碍物，第二站按推算位置'
    if trust_yaw:
        T = np.array([[q[0] - p[0], q[1] - p[1]] for p, q in pairs])
        med = np.median(T, axis=0)
        keep = T[np.hypot(*(T - med).T) <= 60]          # 和大多数对不上的那一对丢掉
        if len(keep) == 0:
            keep = T
        t = keep.mean(axis=0)
        if float(np.hypot(*t)) > max_shift_1:
            return None, f'第二站位置差{np.hypot(*t):.0f}mm太大，可能配错，不校正'
        spread = float(np.max(np.hypot(*(keep - t).T))) if len(keep) > 1 else 0.0
        return (np.eye(2), t, 0.0), (f'用{len(keep)}个共同障碍物校正第二站位置：平移({t[0]:.0f},{t[1]:.0f})mm，'
                                     f'车头角度用陀螺仪实测值(不拟合)，各圆柱一致性{spread:.0f}mm')
    if len(pairs) == 1:
        # 只有1个共同障碍物：车头角度靠陀螺仪闭环转，比较准，只修位置(平移)
        (dx, dy), (ax, ay) = pairs[0]
        t = np.array([ax - dx, ay - dy])
        if float(np.hypot(*t)) > max_shift_1:
            return None, f'只有1个共同障碍物，但相差{np.hypot(*t):.0f}mm太大，可能配错，不校正'
        return (np.eye(2), t, 0.0), f'用1个共同障碍物校正第二站位置：平移({t[0]:.0f},{t[1]:.0f})mm（角度按陀螺仪，不修）'
    ang, R, t, res = _rigid_fit([p for p, _ in pairs], [q for _, q in pairs])
    if abs(ang) > max_angle or res > max_res:
        return None, f'配对不可靠（转角{ang:.1f}度，残差{res:.0f}mm），不校正'
    return (R, t, ang), f'用{len(pairs)}个共同障碍物校正第二站：转{ang:.1f}度，残差{res:.0f}mm'


def apply_fix(mapping, R, t, ang):
    for d in mapping.latched:
        d['x'], d['y'] = (np.array([d['x'], d['y']]) @ R.T + t).tolist()
    if len(mapping.points):
        mapping.points = mapping.points @ R.T + t
    p = np.array([mapping.c['car_x_mm'], mapping.c['car_y_mm']]) @ R.T + t
    yaw = (mapping.c['car_yaw_deg'] + ang + 180) % 360 - 180
    mapping.c = dict(mapping.c, car_x_mm=float(p[0]), car_y_mm=float(p[1]), car_yaw_deg=float(yaw))


def calibrate_mount(c, detected, known, lidar_world):
    """车停在已知起点，用已知圆柱位置修正雷达安装角(和位置)。
    detected/known: 地图坐标，一一对应。1个点只修角度；>=2个点修角度+位置。"""
    det = np.asarray(detected, float); kn = np.asarray(known, float); L = np.asarray(lidar_world, float)
    if len(det) == 1:
        a1 = math.atan2(*(det[0] - L)[::-1]); a2 = math.atan2(*(kn[0] - L)[::-1])
        d = math.degrees(a2 - a1); d = (d + 180) % 360 - 180
        newL = L; res = abs(math.hypot(*(kn[0] - L)) - math.hypot(*(det[0] - L)))
    else:
        d, R, t, res = _rigid_fit(det - L, kn)
        newL = t
    a = math.radians(-c['car_yaw_deg'])
    rel = newL - [c['car_x_mm'], c['car_y_mm']]
    fwd = rel[0] * math.cos(a) - rel[1] * math.sin(a)
    left = rel[0] * math.sin(a) + rel[1] * math.cos(a)
    return dict(lidar_yaw_deg=(c['lidar_yaw_deg'] + d + 180) % 360 - 180,
                lidar_forward_mm=float(fwd), lidar_left_mm=float(left), delta_deg=d, residual_mm=float(res))


class _Cands:
    def __init__(self, latched):
        self.latched = latched


def refine_second(m, anchors, accumulated_detections, roi_class, to_map, min_points=10, min_frames=6, min_seen=6, trust_yaw=False):
    """第二次扫描：先用宽松条件(允许超出边界150mm)找候选去和第一次配对、校正位置，
    再用校正后的位置严格重新识别（场地边界、不识别区域都按校正后的位置算）。"""
    import copy as _copy
    loose_c = dict(m.c, boundary_guard_mm=-150, scan_field_margin_mm=-150)
    loose, _ = accumulated_detections(m.raw_frames, loose_c)
    # 只用回波足够多的障碍物去配对：点太少的位置不可靠，拿它校正反而会把整张图带偏
    solid = [dict(x=o['x'], y=o['y']) for o in m.latched
             if o.get('kind') != 'accumulated' and o.get('seen', 0) >= min_seen]
    solid += [dict(x=d['x'], y=d['y']) for d in loose if d['count'] >= min_points and d['frames'] >= min_frames]
    cands = []
    for p in solid:
        if all(math.hypot(p['x'] - q['x'], p['y'] - q['y']) > 80 for q in cands):   # 同一个障碍物只算一次
            cands.append(p)
    fix, msg = relocalize(anchors, _Cands(cands), trust_yaw=trust_yaw)
    single = _copy.deepcopy(m.latched)
    if fix:
        R, t, ang = fix
        apply_fix(m, R, t, ang)               # 更新车位、单帧识别结果和显示点
        single = _copy.deepcopy(m.latched)
    strict, P = accumulated_detections(m.raw_frames, m.c)
    r = float(m.c['cylinder_diameter_mm']) / 2
    out = []
    for o in single:
        if roi_class(o['x'], o['y'], m.c) == 'interior':
            out.append(o)
    for d in strict:
        if any(math.hypot(o['x'] - d['x'], o['y'] - d['y']) <= 60 for o in out):
            continue
        out.append({'id': 1000 + len(out), 'x': d['x'], 'y': d['y'], 'radius': r,
                    'seen': d['frames'], 'count': d['count'], 'kind': 'accumulated'})
    m.latched = out
    if len(P):
        m.points = P
    return fix, msg
