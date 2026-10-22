#!/usr/bin/env python3
"""第二部分：固定朝向300mm方车的A*模拟。与field_map_stage1.py放在同一目录。
只在电脑画图，不连接任何电机、串口或传感器。地图仍是名义地图。
25mm节点间隔；鼠标目标明确吸附至最近节点，不代表实际定位精度。
圆柱采用外接正方形保守处理。所有运动段检查完整车身扫过的区域。
"""
import argparse
import heapq
import itertools
import math
from pathlib import Path

from field_map_stage1 import (FIELD_MM, CAR_MM, RES_MM, YELLOW_RECTS,
                              TEMP_ZONE, ROUGH_ZONE, START_RECTS,
                              START_CENTERS, POINTS)

STEP = RES_MM
FIXED = YELLOW_RECTS + [TEMP_ZONE, ROUGH_ZONE]
# 模拟障碍，不是传感器检测结果。三个圆柱直径均为50mm。
DEMO = [(2150, 900, 25), (1200, 1750, 25), (700, 1200, 25)]


def expanded_rects(obstacles, margin=0):
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('margin必须为有限非负数')
    half = CAR_MM / 2 + margin
    physical = list(FIXED)
    for x, y, r in obstacles:
        if not all(math.isfinite(v) for v in (x, y, r)) or r <= 0:
            raise ValueError('障碍物坐标必须有限，半径必须大于0')
        physical.append((x-r, y-r, x+r, y+r))
    return [(x0-half, y0-half, x1+half, y1+half)
            for x0, y0, x1, y1 in physical]


def enters_open_rect(a, b, rect):
    """线段是否进入矩形内部。边界接触允许，仅供零余量几何模拟。"""
    lo, hi = 0.0, 1.0
    for p, q, lower, upper in ((a[0], b[0], rect[0], rect[2]),
                                (a[1], b[1], rect[1], rect[3])):
        delta = q-p
        if abs(delta) < 1e-12:
            if not lower < p < upper:
                return False
        else:
            t0, t1 = sorted(((lower-p)/delta, (upper-p)/delta))
            lo, hi = max(lo, t0), min(hi, t1)
            if lo >= hi:
                return False
    return lo < hi


def segment_free(a, b, rects, margin=0):
    half = CAR_MM / 2 + margin
    for p in (a, b):
        if not all(math.isfinite(v) and half <= v <= FIELD_MM-half for v in p):
            return False
    return not any(enters_open_rect(a, b, r) for r in rects)


def simplify(path):
    """只合并共线且同向的段，不切角。"""
    result = []
    for p in path:
        if len(result) >= 2:
            a, b = result[-2:]
            u, v = (b[0]-a[0], b[1]-a[1]), (p[0]-b[0], p[1]-b[1])
            if u[0]*v[1] == u[1]*v[0] and u[0]*v[0]+u[1]*v[1] > 0:
                result.pop()
        result.append(p)
    return result


def plan(start, goal, obstacles=(), margin=0):
    """返回(折线路径,说明)。四邻接A*；输入必须是25mm节点。"""
    rects = expanded_rects(obstacles, margin)
    for name, p in (('START', start), ('GOAL', goal)):
        if len(p) != 2 or any(not math.isfinite(v) or v % STEP != 0 for v in p):
            raise ValueError('起终点必须位于25mm节点上')
        if not segment_free(p, p, rects, margin):
            return [], name + ' BLOCKED'
    start, goal = tuple(start), tuple(goal)
    def heuristic(p):
        return abs(p[0]-goal[0]) + abs(p[1]-goal[1])
    order = itertools.count()
    queue = [(heuristic(start), next(order), 0, start)]
    cost, parent = {start: 0}, {}
    while queue:
        _, _, g, p = heapq.heappop(queue)
        if g != cost.get(p):
            continue
        if p == goal:
            path = [p]
            while p != start:
                p = parent[p]
                path.append(p)
            return simplify(path[::-1]), 'PATH OK'
        for dx, dy in ((0, STEP), (-STEP, 0), (0, -STEP), (STEP, 0)):
            q = (p[0]+dx, p[1]+dy)
            if g+STEP >= cost.get(q, math.inf):
                continue
            if not segment_free(p, q, rects, margin):
                continue
            cost[q], parent[q] = g+STEP, p
            heapq.heappush(queue, (g+STEP+heuristic(q), next(order), g+STEP, q))
    return [], 'NO PATH ON 25mm GRID'


def path_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zone', type=int, choices=(1, 2), default=2)
    parser.add_argument('--demo', action='store_true', help='预放三个模拟障碍')
    parser.add_argument('--margin', type=float, default=0,
                        help='每侧额外余量mm；正余量会使贴边的启停区中心无效')
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--output', default='field_map_stage2_preview.png')
    args = parser.parse_args()
    if not math.isfinite(args.margin) or args.margin < 0:
        parser.error('margin必须为有限非负数')
    import matplotlib
    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle

    fig, ax = plt.subplots(figsize=(9, 9))
    fig.subplots_adjust(bottom=.16, top=.89, left=.10, right=.95)
    state = dict(start=START_CENTERS[args.zone], goal=POINTS['RAW_STOP'],
                 obstacles=list(DEMO) if args.demo else [], path=[], frames=[], index=0)
    timer = fig.canvas.new_timer(interval=40)

    def draw_map():
        ax.clear()
        ax.set_facecolor('#c7c9cc')
        ax.set(xlim=(0, FIELD_MM), ylim=(0, FIELD_MM), xlabel='X / mm', ylabel='Y / mm')
        ax.set_aspect('equal')
        ax.set_xticks(range(0, 2401, 300)); ax.set_yticks(range(0, 2401, 300))
        for r in YELLOW_RECTS:
            ax.add_patch(Rectangle((r[0], r[1]), r[2]-r[0], r[3]-r[1],
                                   facecolor='#fff0a6', edgecolor='#c2a348'))
        for r in (TEMP_ZONE, ROUGH_ZONE):
            ax.add_patch(Rectangle((r[0], r[1]), r[2]-r[0], r[3]-r[1],
                                   facecolor='white', edgecolor='gray'))
        for i, r in START_RECTS.items():
            ax.add_patch(Rectangle((r[0], r[1]), 300, 300, facecolor='#dce8f8', edgecolor='#6482aa'))
            ax.text(r[0]+150, r[1]+150, 'START '+str(i), ha='center', fontsize=8)
        ax.axvline(1200, color='gray', ls='-.', lw=.8)
        ax.axhline(1200, color='gray', ls='-.', lw=.8)
        for name, p in POINTS.items():
            ax.plot(*p, '.', color='steelblue')
            ax.annotate(name, p, xytext=(4, 6), textcoords='offset points', fontsize=8)
        # 膨胀框表示车中心禁入区域，不是障碍物真实大小。
        half = CAR_MM/2+args.margin
        for x, y, r in state['obstacles']:
            ax.add_patch(Circle((x, y), r, color='black', zorder=5))
            ax.add_patch(Rectangle((x-r-half, y-r-half), 2*(r+half), 2*(r+half),
                                   facecolor='#e96262', alpha=.18, edgecolor='red', ls='--'))
        ax.plot(*state['start'], 'o', color='green', ms=7)
        ax.plot(*state['goal'], '*', color='purple', ms=14)
        if state['path']:
            xs, ys = zip(*state['path'])
            ax.plot(xs, ys, color='#1565d8', lw=2.5)
        state['body'] = Rectangle((state['start'][0]-150, state['start'][1]-150),
                                   300, 300, fill=False, edgecolor='green', lw=2.5, zorder=10)
        ax.add_patch(state['body'])

    def replan():
        timer.stop()
        state['frames'] = []
        path, status = plan(state['start'], state['goal'], state['obstacles'], args.margin)
        state['path'] = path
        draw_map()
        state['body'].set_edgecolor('green' if path else 'red')
        length = path_length(path)
        ax.set_title('STAGE 2 | A* SIMULATION | 300 x 300 mm | FIXED HEADING\n'
                     f'{status} | length={length:.0f} mm | margin={args.margin:g} mm')
        print(f"{status} | 起点={state['start']} 终点={state['goal']} | 路程={length:.0f}mm", flush=True)
        if path:
            print('车中心途经点:', path, flush=True)
        else:
            print('未生成路线：检查起终点或障碍布局；此结果仅限当前网格与固定朝向。', flush=True)
        fig.canvas.draw_idle()

    def animate():
        if state['index'] >= len(state['frames']):
            timer.stop()
            return
        x, y = state['frames'][state['index']]
        state['body'].set_xy((x-150, y-150))
        state['index'] += 1
        fig.canvas.draw_idle()
    timer.add_callback(animate)

    def key(event):
        k = (event.key or '').lower()
        if k in ('q', 'escape'):
            timer.stop(); plt.close(fig); return
        if k == ' ':
            if not state['path']:
                return
            timer.stop()
            frames = [state['path'][0]]
            for a, b in zip(state['path'], state['path'][1:]):
                n = max(1, math.ceil(math.dist(a, b)/10))
                frames.extend((a[0]+(b[0]-a[0])*j/n, a[1]+(b[1]-a[1])*j/n)
                              for j in range(1, n+1))
            state['frames'], state['index'] = frames, 0
            timer.start(); return
        if k in ('1', '2'):
            state['start'] = START_CENTERS[int(k)]
        elif k in ('3', '4', '5', '6'):
            state['goal'] = POINTS[{'3':'QR_SCAN','4':'RAW_STOP','5':'TEMP_STOP','6':'ROUGH_STOP'}[k]]
        elif k == 'b':
            state['obstacles'] = list(DEMO)
        elif k == 'r':
            state['obstacles'] = []
        elif k == 's':
            fig.savefig(args.output, dpi=150)
            print('已保存:', Path(args.output).resolve()); return
        else:
            return
        replan()

    def click(event):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        toolbar = getattr(fig.canvas.manager, 'toolbar', None)
        if toolbar is not None and toolbar.mode:
            return
        x, y = round(event.xdata/STEP)*STEP, round(event.ydata/STEP)*STEP
        if event.button == 1:
            state['goal'] = (x, y)
            print(f'目标已吸附到最近25mm节点: ({x}, {y})')
        elif event.button == 3:
            state['obstacles'].append((x, y, 25))
        else:
            return
        replan()

    fig.text(.5, .065,
             '1/2: start | 3: QR | 4: raw | 5: temp | 6: rough | SPACE: simulate\n'
             'Left click: goal (snaps to 25mm) | Right click: add D50 obstacle\n'
             'B: load 3 demo obstacles | R: clear | S: save | Q: quit\n'
             'Geometry only. Zero margin permits contact. No rotation or hardware control.',
             ha='center', va='center', fontsize=9)
    fig.canvas.mpl_connect('key_press_event', key)
    fig.canvas.mpl_connect('button_press_event', click)
    fig.canvas.mpl_connect('close_event', lambda event: timer.stop())
    # 解除Matplotlib默认快捷键，避免S与工具栏保存、Q与退出等冲突。
    if hasattr(fig.canvas.manager, 'key_press_handler_id'):
        fig.canvas.mpl_disconnect(fig.canvas.manager.key_press_handler_id)
    print('只模拟，不驱动小车。绿色方框=车身，蓝线=车中心路线，黑圆=模拟障碍。')
    print('按1/2选起点；3扫码、4原料、5暂存、6粗加工；空格播放；B加载障碍；R清除。')
    replan()
    fig.savefig(args.output, dpi=150)
    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
