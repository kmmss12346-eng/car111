#!/usr/bin/env python3
"""LD14P静止扫描诊断。调用官方SDK；没有STM32或电机控制功能。
角度/原点是SDK修正后的雷达坐标，尚未标定到车身或场地。
关闭窗口只关闭数据读取，不保证断电或停止雷达内部电机。
"""
import argparse
import csv
import glob
import json
import math
from pathlib import Path
import subprocess
import threading
import time
from datetime import datetime

ROOT = Path(__file__).resolve().parent


def list_ports():
    matches = sorted(set(glob.glob('/dev/serial/by-id/*') + glob.glob('/dev/ttyUSB*') +
                         glob.glob('/dev/ttyACM*') + glob.glob('/dev/serial0')))
    print('候选串口（不是自动确认的雷达）：')
    for path in matches:
        print(' ', path, ' -> ', Path(path).resolve())
    if not matches:
        print('未找到候选串口。请检查USB连接；GPIO串口可能需要先启用。')
    print('用插拔雷达USB前后的变化确认端口，不要选择STM32或其他传感器的串口。')


def validate_frame(frame):
    if not isinstance(frame, dict) or not isinstance(frame.get('points'), list):
        raise ValueError('invalid frame')
    hz = float(frame['hz'])
    if not math.isfinite(hz) or hz < 0:
        raise ValueError('invalid hz')
    clean = []
    for p in frame['points']:
        if len(p) != 4:
            raise ValueError('invalid point')
        a, d, intensity, stamp = p
        if not all(math.isfinite(float(v)) for v in (a, d, intensity, stamp)):
            raise ValueError('nonfinite point')
        if not (0 <= a <= 360 and 0 <= d <= 65535 and 0 <= intensity <= 255):
            raise ValueError('out of range point')
        clean.append([float(a), int(d), int(intensity), int(stamp)])
    return {'seq': int(frame['seq']), 'hz': hz, 'points': clean}


def cartesian(points):
    return [(d*math.cos(math.radians(a)), d*math.sin(math.radians(a)))
            for a, d, intensity, stamp in points if d > 0]


class Stream:
    def __init__(self, port, model):
        self.latest = None
        self.received = 0.0
        self.count = 0
        self.lock = threading.Lock()
        self.proc = subprocess.Popen([str(ROOT/'scan_bridge'), port, model],
                                     stdout=subprocess.PIPE, text=True, bufsize=1)
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        try:
            for line in self.proc.stdout:
                if not line.startswith('SCAN '):
                    if line.strip(): print('[SDK]', line.strip(), flush=True)
                    continue
                try:
                    frame = validate_frame(json.loads(line[5:]))
                except (ValueError, KeyError, TypeError, OverflowError):
                    print('忽略一条格式异常的输出', flush=True)
                    continue
                with self.lock:
                    self.latest, self.received = frame, time.monotonic()
                    self.count += 1
        finally:
            self.proc.stdout.close()

    def snapshot(self):
        with self.lock:
            return self.latest, self.received, self.count

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try: self.proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.proc.kill(); self.proc.wait(timeout=2)
        self.thread.join(timeout=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--port', help='确认过的雷达串口，优先使用/dev/serial/by-id/...')
    ap.add_argument('--model', choices=('LD14P_2300HZ','LD14P_4000HZ'), default='LD14P_4000HZ')
    ap.add_argument('--range-mm', type=int, default=4000, help='图中每个轴的正负范围，仅改变显示')
    ap.add_argument('--diameter-mm', type=float, default=0, help='实测圆柱直径mm；不填时仅显示紧凑点簇，不估计圆心')
    ap.add_argument('--no-gui', action='store_true', help='终端诊断')
    ap.add_argument('--seconds', type=float, default=15, help='终端诊断持续秒数')
    args = ap.parse_args()
    if args.list:
        list_ports(); return 0
    if not args.port:
        list_ports(); print('\n请指定: python3 ld14p_scan.py --port 实际雷达端口'); return 2
    if not math.isfinite(args.diameter_mm) or args.diameter_mm < 0:
        ap.error('圆柱直径必须为非负有限数')
    if args.range_mm <= 0 or not math.isfinite(args.seconds) or args.seconds <= 0:
        ap.error('显示范围与诊断时间必须为正数')
    if not (ROOT/'scan_bridge').is_file():
        print('请先在本目录运行: bash build.sh'); return 2
    if not Path(args.port).exists():
        print('串口不存在:', args.port); return 2
    if not args.no_gui:
        import matplotlib.pyplot as plt
        import numpy as np
        from obstacles import detect, Tracker
    print('当前型号配置:', args.model, '| 波特率230400 | 只读扫描，不驱动车轮', flush=True)
    print('默认4000HZ沿用本车已通过扫描测试的配置。', flush=True)
    stream = Stream(args.port, args.model)
    started = time.monotonic()
    try:
        if args.no_gui:
            seen = -1
            while time.monotonic()-started < args.seconds:
                frame, received, count = stream.snapshot()
                if frame and count != seen:
                    valid = [p for p in frame['points'] if p[1] > 0]
                    print(f"帧={frame['seq']} 转频={frame['hz']:.2f}Hz 有效点={len(valid)}/{len(frame['points'])}", flush=True)
                    seen = count
                if stream.proc.poll() is not None:
                    return stream.proc.returncode or (0 if frame else 3)
                time.sleep(.2)
            return 0 if stream.snapshot()[2] > 0 else 3

        fig, ax = plt.subplots(figsize=(8,8))
        fig.subplots_adjust(bottom=.22,top=.87)
        lim = args.range_mm
        ax.set(xlim=(-lim,lim),ylim=(-lim,lim),xlabel='LIDAR SDK X / mm',ylabel='LIDAR SDK Y / mm')
        ax.set_aspect('equal'); ax.grid(alpha=.25)
        ax.axhline(0,color='gray',lw=.7); ax.axvline(0,color='gray',lw=.7)
        ax.plot(0,0,'r+',ms=15,mew=2)
        dots = ax.scatter([],[],s=9,c='#146ad1')
        for radius in (500,1000,1500,2000):
            if radius <= lim:
                ax.add_patch(plt.Circle((0,0),radius,fill=False,color='gray',alpha=.25))
        info = fig.text(.5,.06,'Waiting for scans...',ha='center',fontsize=10)
        fig.text(.5,.02,'Q: quit | S: save fresh scan CSV + PNG | Sensor frame only, NOT field localization',ha='center',fontsize=9)
        seen = [-1]
        tracker = Tracker()
        marks = []
        candidates = []

        def clear_marks():
            for artist in marks:
                artist.remove()
            marks.clear()

        def show_candidates(frame, advance=True):
            nonlocal candidates
            if advance:
                candidates = tracker.update(detect(frame['points'], args.diameter_mm))
            clear_marks()
            for c in candidates:
                color = '#15803d' if c['stable'] and not c.get('sparse') else '#d97706'
                x, y = c['x'], c['y']
                marks.extend(ax.plot(x,y,'x',color=color,ms=9,mew=2))
                if c['kind'] == 'circle_estimate':
                    circle = plt.Circle((x,y),args.diameter_mm/2,fill=False,color=color)
                    ax.add_patch(circle); marks.append(circle)
                label = 'CENTER?' if c['kind'] == 'circle_estimate' else 'SURFACE'
                marks.append(ax.annotate(f"C{c['id']} {label} ({x:.0f},{y:.0f})\nnear={c['nearest_mm']:.0f}mm n={c['count']}",
                                        (x,y),xytext=((8 if x >= 0 else -8),8),
                                        ha=('left' if x >= 0 else 'right'),
                                        textcoords='offset points',fontsize=8,color=color))


        def update():
            frame, received, count = stream.snapshot()
            age = time.monotonic()-received if received else math.inf
            rc = stream.proc.poll()
            if frame is None or age > 1.0 or rc is not None:
                dots.set_offsets(np.empty((0,2)))
                clear_marks(); tracker.clear(); candidates.clear(); seen[0] = -1
                state = f'DRIVER EXIT ({rc})' if rc is not None else ('WAITING' if frame is None else 'STALE - NO LIVE DATA')
                ax.set_title('LD14P SCAN TEST | '+state,color='red')
                info.set_text('No fresh scan. Check terminal logs; missing returns do NOT prove free space.')
            else:
                if seen[0] != count:
                    if seen[0] >= 0 and count-seen[0] > 2:
                        tracker.clear()
                    show_candidates(frame)
                points = cartesian(frame['points'])
                dots.set_offsets(np.asarray(points,dtype=float).reshape(-1,2))
                ax.set_title(f"LD14P LIVE | {frame['hz']:.2f} Hz | valid={len(points)} | frame={frame['seq']}",color='black')
                stable = sum(c['stable'] for c in candidates)
                info.set_text(f'Candidates={len(candidates)} stable={stable} | green=fitted + repeated; orange=unconfirmed; NOT color recognition\nSDK frame only | age={age:.2f}s | diameter={args.diameter_mm:g}mm (0=surface only)')
                if seen[0] != count and (seen[0] < 0 or count % 30 == 0):
                    print(f"收到扫描：frame={frame['seq']} 有效点={len(points)} 转频={frame['hz']:.2f}Hz",flush=True)
                seen[0] = count
            fig.canvas.draw_idle()

        def key(event):
            if (event.key or '').lower() in ('q','escape'):
                plt.close(fig)
            elif (event.key or '').lower() == 's':
                frame, received, count = stream.snapshot()
                if not frame or time.monotonic()-received > 1.0 or stream.proc.poll() is not None:
                    print('没有新鲜扫描数据，未保存。'); return
                out = ROOT/'captures'; out.mkdir(exist_ok=True)
                stem = 'scan_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                with (out/(stem+'.csv')).open('w',newline='') as f:
                    writer=csv.writer(f)
                    writer.writerow(['sdk_angle_deg','distance_mm','intensity','sdk_stamp_ns','sdk_x_mm','sdk_y_mm'])
                    for a,d,intensity,stamp in frame['points']:
                        writer.writerow([a,d,intensity,stamp,d*math.cos(math.radians(a)),d*math.sin(math.radians(a))])
                # 保存帧若尚未显示，先处理；重复保存不增加稳定计数。
                if count != seen[0]:
                    if seen[0] >= 0 and count-seen[0] > 2:
                        tracker.clear()
                    show_candidates(frame)
                    seen[0] = count
                with (out/(stem+'_candidates.json')).open('w') as f:
                    json.dump({'frame':frame['seq'], 'coordinate_frame':'lidar_sdk',
                               'vehicle_stationary_required':True,
                               'candidates_not_verified_obstacles':candidates},f,indent=2)
                # 截图与CSV显示同一份捕获帧，不包括下一次串口更新。
                xy=cartesian(frame['points'])
                dots.set_offsets(np.asarray(xy,dtype=float).reshape(-1,2))
                ax.set_title(f"LD14P CAPTURE | frame={frame['seq']} | valid={len(xy)}")
                fig.savefig(out/(stem+'.png'),dpi=140)
                print('已保存:',out/(stem+'.csv'),flush=True)
        if hasattr(fig.canvas.manager,'key_press_handler_id'):
            fig.canvas.mpl_disconnect(fig.canvas.manager.key_press_handler_id)
        fig.canvas.mpl_connect('key_press_event',key)
        timer=fig.canvas.new_timer(interval=100)
        timer.add_callback(update); timer.start()
        fig.canvas.mpl_connect('close_event',lambda event: timer.stop())
        update(); plt.show()
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        stream.close()


if __name__ == '__main__':
    raise SystemExit(main())
