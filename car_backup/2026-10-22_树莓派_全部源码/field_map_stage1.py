# -*- coding: utf-8 -*-
"""
2027 工创智能搬运赛道：固定场地坐标地图
坐标系：
    左下角 = (0, 0)
    右下角 = (2400, 0)
    左上角 = (0, 2400)
    右上角 = (2400, 2400)
单位：mm

说明：
1. 这是“固定地图层”，不包含随机黑色障碍物。
2. 四个 450x450 淡黄色禁区坐标，是依据规则图中：
   - 场地 2400x2400
   - 中央十字车道宽 400
   - 四个淡黄色区域 450x450
   - 四区关于场地中心对称
   推导得到。正式比赛前请用实物赛道再核对一次。
3. 二维码板、原料转盘的位置存在规定范围内的随机偏差，
   所以程序里保存“范围 + 名义停车点”，名义停车点必须实车标定。
"""

import math
import numpy as np

FIELD_MM = 2400
RES_MM = 25                  # 25 mm / 格，96x96，树莓派运行很轻松
GRID_N = FIELD_MM // RES_MM

FREE = 0
BLOCKED = 1

# ------------------------------------------------------------
# 1. 坐标工具
# ------------------------------------------------------------
def mm_to_grid(x_mm, y_mm):
    """毫米 -> 格编号；越界报错，不偷偷夹到边界内。"""
    if not (math.isfinite(x_mm) and math.isfinite(y_mm) and
            0 <= x_mm < FIELD_MM and 0 <= y_mm < FIELD_MM):
        raise ValueError("坐标必须在[0,2400) mm内")
    return int(x_mm // RES_MM), int(y_mm // RES_MM)


def grid_to_mm(gx, gy):
    """格编号 -> 格中心；格中心不一定等于输入目标点。"""
    if not (isinstance(gx, int) and isinstance(gy, int) and
            0 <= gx < GRID_N and 0 <= gy < GRID_N):
        raise ValueError("格编号必须为0到95的整数")
    return (gx + 0.5) * RES_MM, (gy + 0.5) * RES_MM


# ------------------------------------------------------------
# 2. 固定几何
# ------------------------------------------------------------

# 四个淡黄色禁区（根据图纸几何关系推导）
YELLOW_ZONES = {
    "左上": (550, 1400, 1000, 1850),
    "右上": (1400, 1400, 1850, 1850),
    "左下": (550, 550, 1000, 1000),
    "右下": (1400, 550, 1850, 1000),
}

# 两个启停区：300 x 300
START_ZONES = {
    "启停区1": (2100, 2100, 2400, 2400),
    "启停区2": (2100, 0, 2400, 300),
}

# 左侧暂存区：150 x 580，按图示关于 y=1200 对称
TEMP_ZONE = (0, 910, 150, 1490)

# 下侧粗加工区：580 x 150，按图示关于 x=1200 对称
ROUGH_ZONE = (910, 0, 1490, 150)

# 二维码板：位于右侧，沿 y 方向在 1100~1300 mm 范围内变化
QR_BOARD = {
    "x": 2400,
    "y_range": (1100, 1300),
}

# 原料区：顶部，中心沿 x 方向在 1100~1300 mm 范围内变化
RAW_AREA = {
    "x_range": (1100, 1300),
    "y": 2400,
    "diameter_mm": 300,
}

# ------------------------------------------------------------
# 3. 名义导航节点
# ------------------------------------------------------------
# 这些是“车中心”的名义目标点，不是设备本身中心。
# 后续必须根据你的车长宽、扫码器距离、机械臂可达距离进行实车标定。
NODES = {
    "START1": (2250, 2250),
    "START2": (2250, 150),

    # 先给一个便于调试的名义值：
    # 二维码板在右边界 x=2400；这里让车中心停在 x=2100、y=1200
    "QR_SCAN": (2100, 1200),

    # 原料区在上边界附近，名义停车点先放在其正下方
    "RAW_STOP": (1200, 2100),

    # 暂存区在左边界附近，名义停车点先放在其右侧
    "TEMP_STOP": (300, 1200),

    # 粗加工区在下边界附近，名义停车点先放在其上方
    "ROUGH_STOP": (1200, 300),
}


# ------------------------------------------------------------
# 4. 建立固定占据栅格
# ------------------------------------------------------------
def make_fixed_grid():
    """
    0 = 地面未占用区域（尚未考虑车身，不能直接用于整车规划）
    1 = 固定禁止区域

    当前把：
    - 四个淡黄色区
    - 暂存区设备区域
    - 粗加工区设备区域
    标为禁止。

    启停区是允许进入的，因此不封锁。
    二维码板和原料转盘在场地边缘，当前作为“目标特征”处理，
    不直接封整片内部车道。
    """
    grid = np.zeros((GRID_N, GRID_N), dtype=np.uint8)

    for rect in YELLOW_ZONES.values():
        mark_rect(grid, *rect, BLOCKED)

    mark_rect(grid, *TEMP_ZONE, BLOCKED)
    mark_rect(grid, *ROUGH_ZONE, BLOCKED)

    return grid


def mark_rect(grid, x0, y0, x1, y1, value=BLOCKED):
    """把毫米坐标矩形 [x0,x1) x [y0,y1) 写入栅格"""
    gx0 = max(0, min(GRID_N, int(math.floor(x0 / RES_MM))))
    gy0 = max(0, min(GRID_N, int(math.floor(y0 / RES_MM))))
    gx1 = max(0, min(GRID_N, int(math.ceil(x1 / RES_MM))))
    gy1 = max(0, min(GRID_N, int(math.ceil(y1 / RES_MM))))
    grid[gy0:gy1, gx0:gx1] = value


# ------------------------------------------------------------
# 6. 基本检查
# ------------------------------------------------------------
def print_map_info():
    print("场地:", FIELD_MM, "x", FIELD_MM, "mm")
    print("分辨率:", RES_MM, "mm/格")
    print("栅格:", GRID_N, "x", GRID_N)
    print()
    print("固定禁区:")
    for name, rect in YELLOW_ZONES.items():
        print(" ", name, rect)
    print("  暂存区:", TEMP_ZONE)
    print("  粗加工区:", ROUGH_ZONE)
    print()
    print("导航节点:")
    for name, pt in NODES.items():
        print(" ", name, pt, "-> grid", mm_to_grid(*pt))



import argparse
from pathlib import Path

# 第一部分新增：300x300 mm轴对齐车身检查。
# 不假设整圈150mm白边；仅使用图示工位矩形和黄色禁区。
# 灰色底面、黄色块位置、设备投影仍需实物核对；文字标签不作为独立区域。
# 原料区y=2400仅是边界参考线，不是已测量的转盘中心。
CAR_MM = 300.0
CELL_MM = RES_MM
YELLOW_RECTS = list(YELLOW_ZONES.values())
START_RECTS = {1: START_ZONES["启停区1"], 2: START_ZONES["启停区2"]}
START_CENTERS = {1: NODES["START1"], 2: NODES["START2"]}
POINTS = {k: v for k, v in NODES.items() if not k.startswith("START")}


def overlaps(a, b):
    return max(a[0], b[0]) < min(a[2], b[2]) and \
           max(a[1], b[1]) < min(a[3], b[3])


def center_allowed(x, y, zone=2, margin=0.0):
    """检查固定朝向车身（300mm正方形）；边界接触允许。
    margin是每侧增加的余量；0仅用于理论几何演示。
    两个启停区均保留；zone只选择出发点，不改变地图。
    返回True只代表此名义静态地图中的该姿态可放置，不代表可旋转/可到达。
    """
    if zone not in (1, 2):
        raise ValueError("zone必须为1或2")
    if not all(math.isfinite(v) for v in (x, y, margin)) or margin < 0:
        raise ValueError("坐标必须有限，margin必须非负")
    h = CAR_MM / 2 + margin
    body = (x-h, y-h, x+h, y+h)
    if body[0] < 0 or body[1] < 0 or body[2] > FIELD_MM or body[3] > FIELD_MM:
        return False
    return not any(overlaps(body, rect) for rect in
                   YELLOW_RECTS + [TEMP_ZONE, ROUGH_ZONE])


def build_center_grid(zone=2, margin=0.0):
    """bool数组[gy,gx]，True表示该格中心可放车。
    仅采样格中心，不能据此证明整格或格间线段安全。
    """
    return np.array([[center_allowed(*grid_to_mm(c, r), zone, margin)
                      for c in range(GRID_N)] for r in range(GRID_N)], dtype=bool)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zone', type=int, choices=[1, 2], default=2)
    parser.add_argument('--margin', type=float, default=0.0,
                        help='每侧额外余量mm，默认0为理论边界展示')
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--output', default='field_map_preview.png')
    args = parser.parse_args()
    if not math.isfinite(args.margin) or args.margin < 0:
        parser.error('margin必须为非负有限数')
    import matplotlib
    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from matplotlib.colors import ListedColormap
    import numpy as np

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.subplots_adjust(bottom=0.15, top=0.90)
    state = {'zone': args.zone, 'xy': (1200.0, 1200.0)}

    def rectangle(r, **kwargs):
        ax.add_patch(Rectangle((r[0], r[1]), r[2]-r[0], r[3]-r[1], **kwargs))

    def redraw():
        ax.clear()
        rectangle((0, 0, 2400, 2400), facecolor='white', edgecolor='black')
        rectangle((0, 0, 2400, 2400), facecolor='#c3c7cc', edgecolor='#777777')
        for r in (TEMP_ZONE, ROUGH_ZONE):
            rectangle(r, facecolor='white', edgecolor='#777777')
        for r in YELLOW_RECTS:
            rectangle(r, facecolor='#fff1a8', edgecolor='#d4b13b')
        for z, r in START_RECTS.items():
            rectangle(r, facecolor='#bdd3ff' if z == state['zone'] else '#e2e6ee',
                      edgecolor='#41659e')
            ax.text(*START_CENTERS[z], f'START {z}', ha='center', va='center', fontsize=8)
        grid = build_center_grid(state['zone'], args.margin)
        ax.imshow(np.ma.masked_where(~grid, grid), origin='lower',
                  extent=(0, 2400, 0, 2400), interpolation='nearest',
                  cmap=ListedColormap(['#8ccda9']), alpha=0.65, vmin=0, vmax=1, zorder=2)
        # 点划线仅作地图参考，不表示障碍物。
        ax.plot([150,2250], [1200,1200], '-.', color='#555', lw=0.8)
        ax.plot([1200,1200], [150,2250], '-.', color='#555', lw=0.8)
        for name, (px, py) in POINTS.items():
            ax.plot(px, py, 'o', color='#245d9d', ms=4)
            ax.annotate(name, (px, py), xytext=(5, 8), textcoords='offset points', fontsize=8)
        x, y = state['xy']
        ok = center_allowed(x, y, state['zone'], args.margin)
        color = '#067b3d' if ok else '#cc253b'
        rectangle((x-150,y-150,x+150,y+150), facecolor='none', edgecolor=color, linewidth=2.5, zorder=5)
        if args.margin:
            h = 150 + args.margin
            rectangle((x-h,y-h,x+h,y+h), facecolor='none', edgecolor=color, linestyle='--')
        ax.plot(x, y, '+', color=color, ms=12)
        ax.set(xlim=(-60,2460), ylim=(-60,2460), xlabel='X / mm', ylabel='Y / mm', aspect='equal')
        ax.set_xticks(range(0,2401,300)); ax.set_yticks(range(0,2401,300))
        ax.set_title('STAGE 1 | NOMINAL MAP | 300 x 300 mm | FIXED HEADING\n'
                     f'Center=({x:.0f}, {y:.0f})  '
                     f'{"FITS" if ok else "OUTSIDE ALLOWED AREA"}  margin={args.margin:g} mm', fontsize=10)
        fig.canvas.draw_idle()
        return ok

    def click(event):
        if event.inaxes is ax and event.button == 1:
            state['xy'] = (event.xdata, event.ydata)
            ok = redraw()
            print('中心(mm)=', tuple(round(v,1) for v in state['xy']),
                  '可放置' if ok else '越界或进入禁区')

    def key(event):
        if event.key in ('1','2'):
            state['zone'] = int(event.key)
            state['xy'] = START_CENTERS[state['zone']]
            redraw()
        elif event.key in ('q', 'escape'):
            plt.close(fig)

    fig.text(0.5,0.035,'Green: sampled valid CENTER positions | Gray: lane | Yellow/white: forbidden\n'
             'Click: place car | 1/2: select start | Q: quit\n'
             'Geometry only. Zero margin allows boundary contact. No rotation or motor control.',
             ha='center', fontsize=8)
    redraw()
    fig.canvas.mpl_connect('button_press_event', click)
    fig.canvas.mpl_connect('key_press_event', key)
    output = Path(args.output).expanduser().resolve()
    fig.savefig(output, dpi=150)
    print('已保存地图预览：', output)
    print('名义地图；绿色表示格中心可放下轴对齐车体，不表示已定位或有可执行路线。')
    print('车宽300mm，中央车道宽400mm，居中时两侧各50mm。')
    print('默认margin=0仅展示几何边界；旋转和实车运动需要另外验证。')
    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == '__main__':
    main()
