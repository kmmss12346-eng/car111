#!/usr/bin/env python3
"""自动标定：车离黑色虚线多远 ↔ 虚线在画面里的哪一行。
只需要你手量一次起始距离，剩下的由程序自己做：
  车每次向左横移 step 毫米(用 STM32 的 S 指令，陀螺仪保持车头不变)，
  每个位置拍几张照片，找出画面中间那一列上虚线所在的行号，
  最后打印 距离↔行号 对照表和拟合公式，照片存到 ~/camdist/ 。
用法(树莓派终端，run_live.sh 和摄像头窗口都要先关掉)：
    cd ~/lidar_map_north && python3 cam_dist_cal.py --start 120
--start 后面填你量的：车右侧量尺点 到 黑色虚线靠车那条边 的距离(毫米)。"""
import argparse
import math
import os
import statistics
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stm32_link import Stm32Link


def line_row(img, max_abs_deg=20.0):
    """找画面里近似水平的黑色虚线，返回它在画面正中间那一列的行号(像素)；找不到返回 None。"""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    bl = cv2.GaussianBlur(g, (5, 5), 0)
    th = (bl < 70).astype(np.uint8) * 255
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    ls = cv2.HoughLinesP(th, 1, np.pi / 720, 60, minLineLength=100, maxLineGap=80)
    if ls is None:
        return None
    h, w = g.shape
    xc = w / 2.0
    segs = []
    for l in ls:
        x1, y1, x2, y2 = [float(v) for v in l.reshape(-1)[:4]]
        a = math.degrees(math.atan2(y2 - y1, x2 - x1))
        if abs(a) > max_abs_deg or x2 == x1:
            continue
        k = (y2 - y1) / (x2 - x1)
        segs.append((math.hypot(x2 - x1, y2 - y1), y1 + k * (xc - x1)))   # 延长到中间一列的行号
    if not segs:
        return None
    segs.sort(reverse=True)
    best = segs[0][1]
    near = [s for s in segs if abs(s[1] - best) < 25][:6]          # 只用和最长那段是同一条线的
    wsum = sum(s[0] for s in near)
    return sum(s[0] * s[1] for s in near) / wsum


def grab(cap, n=3):
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
    ap.add_argument('--start', type=float, required=True, help='起始距离(毫米)：车右侧量尺点到虚线靠车那条边')
    ap.add_argument('--step', type=float, default=30.0, help='每次横移多少毫米：正数向左(离线更远)，负数向右(离线更近)')
    ap.add_argument('--n', type=int, default=5, help='一共测几个位置')
    ap.add_argument('--stm-port', default='/dev/serial0')
    ap.add_argument('--cam', type=int, default=0)
    a = ap.parse_args()

    link = Stm32Link(a.stm_port)
    if not link.ok:
        sys.exit(f'打不开 STM32 串口：{link.err}（run_live.sh 关了吗？）')
    ok, rep = link.ping()
    if not ok:
        sys.exit(f'STM32 没有回应：{rep}')
    cap = cv2.VideoCapture(a.cam)
    if not cap.isOpened():
        sys.exit(f'打不开摄像头 {a.cam}（摄像头窗口 cam_view 关了吗？）')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    out_dir = os.path.expanduser('~/camdist')
    os.makedirs(out_dir, exist_ok=True)
    time.sleep(1.5)

    rows = []
    moved = 0.0
    for i in range(a.n):
        if i > 0:
            ok, rep = link.request(f'S {int(round(a.step))} 40', 20.0)     # 慢速横移：正数向左，负数向右
            if not ok:
                print('横移失败，停止：', rep)
                break
            moved += a.step
            time.sleep(0.8)
        dist = a.start + moved
        frames = grab(cap)
        vals = []
        for j, f in enumerate(frames):
            cv2.imwrite(os.path.join(out_dir, f'd{int(round(dist)):04d}_{j}.jpg'), f)
            r = line_row(f)
            if r is not None:
                vals.append(r)
        m = statistics.median(vals) if vals else None
        rows.append((dist, m))
        print(f'距离 {dist:.0f}mm → 虚线在第 ' + ('(没找到)' if m is None else f'{m:.1f}') +
              f' 行（{len(vals)}/{len(frames)} 张有效）', flush=True)
    cap.release()

    # 回到起始位置
    if moved != 0:
        print(f'横移回起始位置（{"向右" if moved > 0 else "向左"} {abs(moved):.0f}mm）……', flush=True)
        link.request(f'S {-int(round(moved))} 40', 30.0)

    pts = [(d, r) for d, r in rows if r is not None]
    print()
    if len(pts) >= 3:
        D = np.array([p[0] for p in pts]); R = np.array([p[1] for p in pts])
        c = np.polyfit(R, D, 2)                         # 距离 = c0*行² + c1*行 + c2
        res = float(np.max(np.abs(np.polyval(c, R) - D)))
        print(f'拟合：距离(mm) = {c[0]:.6f}×行² + {c[1]:.4f}×行 + {c[2]:.2f}   最大残差 {res:.1f}mm')
        print('每移动 1 个像素大约对应 %.2f mm' % abs(np.polyval(np.polyder(c), R.mean())))
    else:
        print('有效数据太少，拟合不了；检查每个位置画面里是否都看得到黑色虚线')
    print(f'照片存在 {out_dir}/ ，把这个终端截图发我就行，照片暂时不用发。')


if __name__ == '__main__':
    main()
