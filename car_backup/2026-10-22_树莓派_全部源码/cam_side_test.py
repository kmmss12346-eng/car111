#!/usr/bin/env python3
"""实时测试：每 0.5 秒打印一次 车右侧(摄像头座底边) 到 右边分界线 的距离。Ctrl+C 退出。
用法(树莓派终端，摄像头窗口 cam_view 要先关掉)：
    cd ~/lidar_map_north && python3 cam_side_test.py"""
import os
import statistics
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cam_side import side_distance


def main():
    cam = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cap = cv2.VideoCapture(cam)
    if not cap.isOpened():
        sys.exit(f'打不开摄像头 {cam}（摄像头窗口 cam_view 关了吗？）')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    time.sleep(1.0)
    print('开始测量，按 Ctrl+C 退出', flush=True)
    try:
        while True:
            vals = []
            for _ in range(5):
                ok, f = cap.read()
                if not ok:
                    continue
                r = side_distance(f)
                if r is not None:
                    vals.append(r)
            if not vals:
                print('没找到右边的分界线', flush=True)
            else:
                d = statistics.median(v[0] for v in vals)
                a = statistics.median(v[1] for v in vals)
                inside = all(v[2] for v in vals)
                print(f'离右边分界线 {d:6.1f} mm   (斜边角度 {a:4.1f}°)' + ('' if inside else '   ← 超出标定范围，仅供参考'), flush=True)
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    cap.release()


if __name__ == '__main__':
    main()
