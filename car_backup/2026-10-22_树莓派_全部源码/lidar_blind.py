"""(10-19)看雷达哪些方向被车上的东西挡住了(只读文件：不连雷达、不动车，run_live 开着也能跑)。
用 run_live 每次扫描自动存的 map_captures/merged_*.json(里面有每次扫描的原始帧和当时的车位)。

    cd ~/lidar_map_north
    python3 lidar_blind.py              # 最新存的那个
    python3 lidar_blind.py 文件.json

方向：0° = 车头正前方，正数往左(雷达装在车左边)，负数往右，±180° = 正后方。
一个方向上 35cm 以内(near_mm)就有回波 = 被车上/车旁边的东西挡住了，场地里这个方向的障碍物雷达看不到。
一个回波都没有的方向：可能贴着雷达被挡住(比雷达最近能测的距离还近，比如轮子)，也可能那边很远都没东西。
另外存一张 lidar_blind.png(车在中间、车头朝上；红点 = 35cm 以内的回波，蓝点 = 远处)，桌面上双击打开看。
"""
import glob
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
NEAR_MM = 350.0
FIELD_MM = 2400.0


def lidar_xy(points):
    """雷达坐标系里的 (x, y)。车上用 ld14p_scan.cartesian(和建图一样)；电脑上没有它就按 (角度°, 距离mm) 算。"""
    pts = [p for p in points if p[1] > 0]
    try:
        from ld14p_scan import cartesian
        return [(float(q[0]), float(q[1])) for q in cartesian(pts)]
    except ImportError:
        return [(p[1] * math.cos(math.radians(p[0])), p[1] * math.sin(math.radians(p[0]))) for p in pts]


def rot(x, y, deg):
    a = math.radians(deg)
    return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)


def body_rel(xy, c):
    """离雷达的 (前, 左)：和 lidar_map_live.to_map 一样先乘 sdk_y_sign、再转 lidar_yaw_deg。"""
    return [rot(x, y * float(c.get('sdk_y_sign', 1)), float(c.get('lidar_yaw_deg', 0))) for x, y in xy]


def in_field(rel, c):
    """按记下的车位换算到场地上，落在场地里(不算车身自己)的点数。"""
    yaw = float(c['car_yaw_deg'])
    lf, ll = float(c.get('lidar_forward_mm', 0)), float(c.get('lidar_left_mm', 0))
    hl, hw = float(c.get('car_length_mm', 290)) / 2 + 120, float(c.get('car_width_mm', 260)) / 2 + 120
    n = 0
    for f, l in rel:
        bf, bl = f + lf, l + ll
        if abs(bf) <= hl and abs(bl) <= hw:
            continue
        wx, wy = rot(bf, bl, yaw)
        wx, wy = wx + float(c['car_x_mm']), wy + float(c['car_y_mm'])
        n += 30 <= wx <= FIELD_MM - 30 and 30 <= wy <= FIELD_MM - 30
    return n


def ranges(bins, ok):
    """连着的 10° 格子合成一段：[(起, 止, 这几格里最近的距离)]。"""
    out, cur = [], None
    for i, b in enumerate(bins):
        if ok(b):
            if cur is None:
                cur = [i, i, b['min']]
            else:
                cur[1], cur[2] = i, min(cur[2], b['min'])
        elif cur is not None:
            out.append(cur)
            cur = None
    if cur is not None:
        out.append(cur)
    if len(out) > 1 and out[0][0] == 0 and out[-1][1] == len(bins) - 1:     # 跨过 ±180° 的接起来
        last = out.pop()
        out[0] = [last[0], out[0][1] + len(bins), min(last[2], out[0][2])]
    return [(-180 + 10 * a, -180 + 10 * (b + 1), m) for a, b, m in out]


def fmt(rs, with_min=True):
    def deg(v):
        v = (v + 180) % 360 - 180 if v > 180 else v
        return f'{v:+.0f}°'
    return '、'.join(f'{deg(a)}~{deg(b)}' + (f'(最近 {m / 10:.0f}cm)' if with_min and m < 1e9 else '') for a, b, m in rs) or '没有'


def analyse(view, k, near_mm=NEAR_MM):
    c = view.get('config') or {}
    frames = view.get('raw_frames') or []
    print(f"第{k}次扫描：车在 ({float(c.get('car_x_mm', 0)):.0f},{float(c.get('car_y_mm', 0)):.0f}) "
          f"车头 {float(c.get('car_yaw_deg', 0)):.1f}°，{len(frames)} 帧")
    if not frames:
        print('  这一次扫描没有存原始帧，看不了')
        return None
    bins = [dict(n=0, near=0, far=0, min=1e9) for _ in range(36)]
    rel_all, total = [], 0
    for f in frames:
        pts = f.get('points') or []
        total += len(pts)                                    # 没回波(距离 0)的也算：每格应该有多少个方向
        rel = body_rel(lidar_xy(pts), c)
        rel_all += rel
        for fwd, left in rel:
            d = math.hypot(fwd, left)
            b = bins[min(35, int((math.degrees(math.atan2(left, fwd)) + 180) // 10))]
            if d < near_mm:
                b['near'] += 1
                b['min'] = min(b['min'], d)
            else:
                b['far'] += 1
    per_bin = max(1.0, total / 36.0)                         # 每格每次扫描应该有的点数(所有帧加起来)
    blocked = ranges(bins, lambda b: b['near'] >= 0.5 * per_bin)
    partly = ranges(bins, lambda b: 0.15 * per_bin <= b['near'] < 0.5 * per_bin)
    print(f'  被车上的东西挡住的方向({near_mm / 10:.0f}cm 以内一直有回波)：{fmt(blocked)}')
    if partly:
        print(f'  挡住一部分的方向：{fmt(partly)}')
    print(f'  {near_mm / 10:.0f}cm 以外有回波的方向(看得到东西)：{fmt(ranges(bins, lambda b: b["far"] >= 0.1 * per_bin), False)}')
    none = ranges(bins, lambda b: b['near'] + b['far'] == 0)
    if none:
        print(f'  一个回波都没有的方向：{fmt(none, False)}(可能贴着雷达被挡住了(比雷达最近能测的还近)，也可能那边很远都没东西)')
    print(f'  按记下的车位换算，落在场地里的回波 {in_field(rel_all, c)} 个点')
    return bins, rel_all, per_bin


def plot(results, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as ex:
        print(f'(画不了图：{ex!r})')
        return
    n = len(results)
    fig, axs = plt.subplots(1, n, figsize=(6 * n, 6), squeeze=False)
    for ax, (k, c, rel) in zip(axs[0], results):
        lf, ll = float(c.get('lidar_forward_mm', 0)), float(c.get('lidar_left_mm', 0))
        near = [(-(l + ll), f + lf) for f, l in rel if math.hypot(f, l) < NEAR_MM]
        far = [(-(l + ll), f + lf) for f, l in rel if NEAR_MM <= math.hypot(f, l) <= 3000]
        if far:
            ax.scatter(*zip(*far), s=2, c='tab:blue')
        if near:
            ax.scatter(*zip(*near), s=3, c='tab:red')
        hl, hw = float(c.get('car_length_mm', 290)) / 2, float(c.get('car_width_mm', 260)) / 2
        ax.plot([-hw, hw, hw, -hw, -hw], [hl, hl, -hl, -hl, hl], 'k-')
        ax.plot([-ll], [lf], 'o', color='orange', ms=8)
        ax.annotate('FRONT', (0, hl), ha='center', va='bottom')
        ax.set_xlim(-1500, 1500)
        ax.set_ylim(-1500, 1500)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f'scan {k}: front up, left = car left (red < {NEAR_MM / 10:.0f}cm)')
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    print(f'图存在 {path}(红点 = {NEAR_MM / 10:.0f}cm 以内的回波，橙点 = 雷达)，桌面上双击打开看')


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv:
        path = Path(argv[0])
    else:
        files = sorted(glob.glob(str(ROOT / 'map_captures' / 'merged_*.json')))
        if not files:
            print('map_captures 里没有扫描存的文件：先在 run_live 里 scan 或 go 一次')
            return 1
        path = Path(files[-1])
    print(f'读 {path}')
    data = json.loads(path.read_text(encoding='utf-8'))
    results = []
    for k, view in enumerate(data.get('views') or [], 1):
        r = analyse(view, k)
        if r is not None:
            results.append((k, view.get('config') or {}, r[1]))
    if results:
        try:
            plot(results, ROOT / 'lidar_blind.png')
        except Exception as ex:                              # 图画不出来不影响上面的结果
            print(f'(画图出错：{ex!r})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
