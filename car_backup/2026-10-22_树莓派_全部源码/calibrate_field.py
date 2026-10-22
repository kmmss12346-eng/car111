#!/usr/bin/env python3
"""Fit mounting yaw from known map landmarks, keeping measured pose/offset fixed.
Do not infer field edges from horizontal scans of a flat printed mat.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from lidar_map_live import validate_config, rotate, to_map
ROOT=Path(__file__).resolve().parent


def calibrate(config, landmarks):
    c=dict(validate_config(config))
    if len(landmarks)<2:
        raise ValueError('至少提供两个不同方向的已知对应点')
    sdk=np.asarray([p['sdk_mm'] for p in landmarks],float)
    world=np.asarray([p['map_mm'] for p in landmarks],float)
    if sdk.shape!=(len(landmarks),2) or world.shape!=sdk.shape or not np.all(np.isfinite(sdk)) or not np.all(np.isfinite(world)):
        raise ValueError('对应点必须为有限二维坐标')
    if np.any(world<0) or np.any(world>2400):
        raise ValueError('标定点必须在场地内')
    if len(np.unique(sdk,axis=0))<2 or np.linalg.matrix_rank(sdk)<2:
        raise ValueError('对应点不能相同或均在同一径向直线上')
    sdk=sdk.copy();sdk[:,1]*=c['sdk_y_sign']
    target=rotate(world-[c['car_x_mm'],c['car_y_mm']],-c['car_yaw_deg'])
    target-=np.array([c['lidar_forward_mm'],c['lidar_left_mm']])
    cross=np.sum(sdk[:,0]*target[:,1]-sdk[:,1]*target[:,0])
    dot=np.sum(sdk*target)
    c['lidar_yaw_deg']=math.degrees(math.atan2(cross,dot))
    predicted=to_map([p['sdk_mm'] for p in landmarks],c)
    errors=np.linalg.norm(predicted-world,axis=1)
    if errors.max()>30:
        raise ValueError(f'拟合最大残差{errors.max():.1f}mm超过30mm，请检查点对应、车头方向或安装偏移；未保存')
    report=dict(method='fixed pose and translation; fit yaw only',
                max_landmark_residual_mm=float(errors.max()),
                predicted_map_mm=predicted.tolist(),residuals_mm=errors.tolist(),
                independent_validation=False,
                note='Residual is fit consistency, NOT global map accuracy. Car position/heading and 145mm offset are assumed measurements.')
    c['calibration_report']=report
    return c


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',default=str(ROOT/'calibration_start2.json'))
    ap.add_argument('--output',default=str(ROOT/'map_config_start2_roi.json'))
    args=ap.parse_args()
    data=json.loads(Path(args.input).read_text())
    c=calibrate(data['config'],data['landmarks'])
    out=Path(args.output);out.write_text(json.dumps(c,ensure_ascii=False,indent=2)+'\n')
    print('已生成配置:',out)
    print('安装角:',round(c['lidar_yaw_deg'],5),'deg')
    print('已知点拟合残差:',c['calibration_report']['residuals_mm'],'mm')
    print('这不是全场精度保证；固定车位、安装偏移仍按输入测量值。')

if __name__=='__main__':main()
