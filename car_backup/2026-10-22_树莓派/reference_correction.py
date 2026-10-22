"""Pose-scoped provisional corrections; no automatic localization.
The first pose uses one-point translation, the second retains the earlier rigid fit.
Neither is an independently validated, general sensor calibration.
"""
import math
import numpy as np
VERSION = '2026-09-29-first-reference'
OBSERVED = np.array([[1040., 906.], [2217., 1033.]])
KNOWN = np.array([[1200., 1200.], [2400., 1200.]])
ANGLE_DEG = -math.degrees(math.atan2(127., 1177.))
a = math.radians(ANGLE_DEG)
ROT = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
SHIFT = KNOWN.mean(axis=0) - OBSERVED.mean(axis=0) @ ROT.T
FIRST_OBSERVED = np.array([625., 174.])
FIRST_KNOWN = np.array([470., 255.])
FIRST_SHIFT = FIRST_KNOWN - FIRST_OBSERVED


def profile(c):
    values = [c.get(k, float('nan')) for k in (
        'car_x_mm', 'car_y_mm', 'car_yaw_deg', 'lidar_forward_mm',
        'lidar_left_mm', 'lidar_yaw_deg', 'sdk_y_sign')]
    mount = [0, 140, -139.52993528774806, -1]
    for name, pose in [('first_translation', [2250, 150, 180]),
                       ('second_rigid', [2186, 241, 120])]:
        if np.allclose(values, pose + mount, atol=1e-5, rtol=0):
            return name
    return None


def applicable(c):
    return profile(c) is not None


def description(c):
    if not c.get('reference_correction_enabled', True):
        return '未启用：配置已关闭'
    name = profile(c)
    if name == 'first_translation':
        return '启用：首站单点平移 X-155 / Y+81 mm（角度误差未知）'
    if name == 'second_rigid':
        return '启用：第二站两点临时校正（未独立验证）'
    return '未启用：当前车位/安装参数与参考条件不符'


def metadata():
    return dict(version=VERSION, independently_validated=False,
                first=dict(pose=[2250,150,180], observed_mm=FIRST_OBSERVED.tolist(),
                           known_mm=FIRST_KNOWN.tolist(), shift_mm=FIRST_SHIFT.tolist(),
                           method='one_point_translation', source='user_screenshot_2026-09-29'),
                second=dict(pose=[2186,241,120], observed_mm=OBSERVED.tolist(),
                            known_mm=KNOWN.tolist(), angle_deg=ANGLE_DEG,
                            shift_mm=SHIFT.tolist(), method='two_point_rigid'))


def apply(world, c):
    world = np.asarray(world)
    if not c.get('reference_correction_enabled', True):
        return world
    name = profile(c)
    if name == 'first_translation':
        return world + FIRST_SHIFT
    if name == 'second_rigid':
        return world @ ROT.T + SHIFT
    return world
