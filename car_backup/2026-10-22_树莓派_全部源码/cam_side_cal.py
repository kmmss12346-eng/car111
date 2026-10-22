#!/usr/bin/env python3
"""侧边摄像头重新标定(摄像头调过角度以后用)：车每次往右(往分界线)横移，拍照，自动算出换算公式，
写到 cam_side_cal.json，以后 cam / camteach / 跑图都自动用新的标定。
用法(树莓派终端；run_live.sh 和摄像头窗口 cam_view 都要先关掉)：
    cd ~/lidar_map_north && python3 cam_side_cal.py --start 200
--start：右前角蓝色摄像头座底边 到 右边灰/白分界线 的横向距离(毫米)，车头和分界线平行。
默认往右挪 5 次、每次 30mm(一共 150mm)，所以 --start 至少要 160。"""
import argparse
import json
import os
import statistics
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cam_side import edge_fit
from stm32_link import Stm32Link

ROWS = (360, 540, 690)      # 用画面第几行(像素)上分界线的左右位置来换算


def grab(cap, n=4):
    for _ in range(6):
        cap.read()
    out = []
    for _ in range(n):
        ok, f = cap.read()
        if ok:
            out.append(f)
        time.sleep(0.15)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', type=float, required=True)
    ap.add_argument('--step', type=float, default=-30.0, help='负数=往右(靠近分界线)')
    ap.add_argument('--n', type=int, default=6)
    ap.add_argument('--stm-port', default='/dev/serial0')
    ap.add_argument('--cam', type=int, default=0)
    a = ap.parse_args()
    if a.start + a.step * (a.n - 1) < 5:
        sys.exit(f'起始距离 {a.start:.0f}mm 太近：往右挪 {abs(a.step)*(a.n-1):.0f}mm 会碰到分界线，请把 --start 设大一点或减少 --n')

    link = Stm32Link(a.stm_port)
    if not link.ok:
        sys.exit(f'打不开 STM32 串口：{link.err}（run_live.sh 关了吗？）')
    ok, rep = link.ping()
    if not ok:
        sys.exit(f'STM32 没有回应：{rep}')
    cap = cv2.VideoCapture(a.cam)
    if not cap.isOpened():
        sys.exit(f'打不开摄像头 {a.cam}（cam_view 窗口关了吗？）')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    out_dir = os.path.expanduser('~/camsidecal')
    os.makedirs(out_dir, exist_ok=True)
    time.sleep(1.5)

    data = {rf: [] for rf in ROWS}
    moved = 0.0
    for i in range(a.n):
        if i > 0:
            ok, rep = link.request(f'S {int(round(a.step))} 40', 20.0)
            if not ok:
                print('横移失败，停止：', rep)
                break
            moved += a.step
            time.sleep(0.8)
        dist = a.start + moved
        frames = grab(cap)
        msg = []
        for j, f in enumerate(frames):
            cv2.imwrite(os.path.join(out_dir, f'd{int(round(dist)):04d}_{j}.jpg'), f)
        fits_here = [edge_fit(f) for f in frames]
        for rf in ROWS:
            xs = []
            for r in fits_here:
                if r is not None and r[2] - 60 <= rf <= r[3] + 60:
                    x = r[0] * rf + r[1]
                    if 10 <= x <= 1240:
                        xs.append(x)
            if len(xs) >= 2:
                x = statistics.median(xs)
                data[rf].append((dist, x))
                msg.append(f'第{rf}行:{x:.0f}')
            else:
                msg.append(f'第{rf}行:没看到')
        print(f'距离 {dist:.0f}mm → ' + '  '.join(msg), flush=True)
    cap.release()
    if moved != 0:
        print(f'横移回起始位置……', flush=True)
        link.request(f'S {-int(round(moved))} 40', 30.0)

    fits = []
    lo, hi = 1e9, -1e9
    print()
    for rf in ROWS:
        pts = data[rf]
        if len(pts) < 3:
            continue
        D = np.array([p[0] for p in pts]); X = np.array([p[1] for p in pts])
        if np.ptp(X) < 30:          # 这一行分界线几乎不动，说明不灵敏，不用
            continue
        c = np.polyfit(X, D, 2)
        res = float(np.max(np.abs(np.polyval(c, X) - D)))
        print(f'第{rf}行：用了{len(pts)}个位置，最大误差 {res:.1f}mm，范围 {D.min():.0f}～{D.max():.0f}mm')
        if res <= 8:
            fits.append((rf, c.tolist()))
            lo, hi = min(lo, D.min()), max(hi, D.max())
    if not fits:
        print('没有一行能用：每个位置画面里都要看得到右边那条斜着的灰/白分界线。把 ~/camsidecal 打包发我看看。')
        return
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cam_side_cal.json')
    json.dump(dict(method='row_profile', row_fits=fits, cal_min_mm=float(lo), cal_max_mm=float(hi), date=time.strftime('%Y-%m-%d %H:%M')),
              open(p, 'w', encoding='utf-8'), indent=2, ensure_ascii=False)
    print(f'\n已保存标定到 {p}（测量范围 {lo:.0f}～{hi:.0f}mm）。以后 cam / camteach / 跑图都会自动用它。')
    print('注意：摄像头调过角度，之前 camteach 记住的读数作废，要重新 camteach。')


if __name__ == '__main__':
    main()
