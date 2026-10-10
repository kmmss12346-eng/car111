#!/usr/bin/env python3
"""全流程路线规划（25mm栅格A*，按“用时”算代价）。只算路线和指令，不直接驱动电机。

坐标：场地左下角原点，X向右，Y向上，单位mm。
航向：东0°，北90°，西180°，南-90°；逆时针为正。
车体指令：F=前进(负数后退)，S=横移(正数向左，负数向右，麦轮才有)，R=原地转(正数逆时针)。

规则约束（2027智能搬运）：
- 离开启停区后车身铅垂投影只能在灰色车道上：不能压黄色区、暂存区、粗加工区、原料转盘，不能出场地。
- 原地转弯时车身扫过一个圆，只在转得开的地方转。

v10 和之前的不同：
1. 现在 STM32 的 R 指令是绕“车中心”转(转的同时叠加平移，车中心基本不动)，所以 turn_pivot_back_mm 默认 0。
   (转轴参数还保留：以后换了车、转轴又不在中心时照样能用。)
2. A* 的代价从“毫米数”改成“预计用时(秒)”：直行、横移各有自己的速度和加减速；每多一次起停、每多转一次弯，都按
   实测的时间加上。横移现在又快又稳，所以不再限制横移长度，规划器会自己权衡“多转弯”和“横着走”哪个更省时间。
3. 离黄色区、墙、障碍物越近代价越高：同样的用时，规划器会选更宽的车道、走在车道中间。

v17(10-10 整合)：
1. 车身按真实尺寸算：car_length_mm × car_width_mm 的矩形，加上左侧雷达圆盘(lidar_forward_mm/lidar_left_mm/
   lidar_overhang_mm)，不再用 300 的正方形。每种区域单独留硬余量：
     黄色区 yellow_margin_mm(默认30)，车道太窄时只在挤不过去的地方放宽，最低到 yellow_min_margin_mm(默认15)，不会到 0；
     暂存区/粗加工区 zone_margin_mm(默认30)；原料转盘按它可能出现的整个范围(中心 x±100、y 2465~2480，半径150)
     留 raw_margin_mm(默认30)；场地边 edge_margin_mm(默认30)，启停区里面不算，启停区旁边 start_corridor_mm(默认300)
     这一段允许贴边(车要开进角上的启停区)，但尽量不走；障碍物 margin_mm。
   原地转弯按转轴到车身最远点的圆算(比真车扫过的范围略大)，离每种区域至少 max(该区域余量, turn_margin_mm)。
2. 规划快了很多：每种栅格偏移的“车身放得下”表只算一次(numpy)，A* 用整数状态和按车道算的启发值(BFS)，
   走不通的先用 BFS 判断，不再“按大余量把整个 A* 搜完再放宽重来”。
3. 每一段都准确停在停车点上：栅格对齐这一段的起点，终点不在栅格上的那一点差值并进最后一条同方向的移动，
   不再另外发 8~15mm 的小移动。
4. 新函数：pose_clear(某个位姿离各区域多远)、moves_clear(一串指令扫过的范围，转弯按角度一步步扫)、plan_leg(单段规划)。
5. 停车点本身离某个区域不够余量时，这一段只在这个区域上放宽到停车点自己的距离，并在 note 里写清楚是哪个区域、差多少。
"""
import heapq
import itertools
import math

import numpy as np

FIELD = 2400
STEP = 25
N = FIELD // STEP + 1            # 0,25,...,2400 共97个节点
DIRS = ((1, 0), (0, 1), (-1, 0), (0, -1))   # 世界方向：0东 1北 2西 3南

YELLOW = [(550, 1400, 1000, 1850), (1400, 1400, 1850, 1850),
          (550, 550, 1000, 1000), (1400, 550, 1850, 1000)]
TEMP_ZONE = (0, 910, 150, 1490)
ROUGH_ZONE = (910, 0, 1490, 150)
START_ZONES = ((2100, 0, 2400, 300), (2100, 2100, 2400, 2400))   # 启停区2(东南)、启停区1(东北)

# 车身离黄色区至少留多少 mm(配置 yellow_margin_mm 可改)。挤不过去的地方放宽，最低到 YELLOW_MIN_MARGIN_MM。
YELLOW_MARGIN_MM = 30
YELLOW_MIN_MARGIN_MM = 15
ZONE_MARGIN_MM = 30              # 暂存区/粗加工区
RAW_MARGIN_MM = 30               # 原料转盘
EDGE_MARGIN_MM = 30              # 场地边(启停区里面和旁边除外)
START_CORRIDOR_MM = 300          # 启停区旁边这一段场地边允许贴边(开进启停区要用)
MIN_MOVE_MM = 20                 # 比这小的前进/横移尽量不出现(执行时太短，车本身的精度就在这个量级)
EPS_MM = 0.05                    # 浮点误差：差这么一点不算不够余量

_BIG = 1.0e5

# v11：场地按黄色区的边分成 5×5 格：偶数格是车道，奇数×奇数是黄色区。
#   路口 = 车道×车道(9 个)，路段 = 车道×黄色列/行(12 段)。
LANE_BANDS = ((0, 550), (550, 1000), (1000, 1400), (1400, 1850), (1850, 2400))


def _band(v):
    for k, (a, b) in enumerate(LANE_BANDS):
        if a <= v <= b:
            return k
    return 0 if v < 0 else len(LANE_BANDS) - 1


def lanes_connected(sealed, points):
    """封死这些车道以后，points(起点和各停车点)之间按车道还连得通吗？只看车道格子，不跑A*，很快。"""
    n = len(LANE_BANDS)
    blocked = {(_band((r[0]+r[2])/2), _band((r[1]+r[3])/2)) for r in sealed}
    cells = [(_band(x), _band(y)) for x, y in points]
    if not cells or any(c in blocked for c in cells):
        return False
    seen = {cells[0]}; todo = [cells[0]]
    while todo:
        i, j = todo.pop()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i+di, j+dj
            if 0 <= a < n and 0 <= b < n and not (a % 2 and b % 2) and (a, b) not in blocked and (a, b) not in seen:
                seen.add((a, b)); todo.append((a, b))
    return all(c in seen for c in cells)


def lane_block_rects(cfg, obstacles):
    """v11：障碍物在哪一段车道上，就把那一整段(两个路口之间)封死，不管扫出来的位置准不准。
    障碍物在路口里：没有停车点的路口(左上、左下、正中)整块封死；有停车点的路口不能封，只按障碍物本身绕开。
    障碍物离格子的边不到 lane_block_tol_mm(默认80mm)时，旁边那一格也一起封(扫描位置有误差)。
    返回 [(x0, y0, x1, y1), ...]。"""
    tol = float(cfg.get('lane_block_tol_mm', 80))
    stops = [(float(v[0]), float(v[1])) for v in (cfg.get('stops') or {}).values()]

    def bands(v):
        return [k for k, (a, b) in enumerate(LANE_BANDS) if a - tol <= v <= b + tol]

    out = []
    for ox, oy, _ in obstacles:
        for i in bands(ox):
            for j in bands(oy):
                if i % 2 == 1 and j % 2 == 1:
                    continue                                   # 黄色区，本来就不能走
                r = (LANE_BANDS[i][0], LANE_BANDS[j][0], LANE_BANDS[i][1], LANE_BANDS[j][1])
                if i % 2 == 0 and j % 2 == 0 and any(r[0] <= sx <= r[2] and r[1] <= sy <= r[3] for sx, sy in stops):
                    continue                                   # 有停车点的路口，不能整块封
                if r not in out:
                    out.append(r)
    return out


def heading_index(yaw):
    h = round(((yaw % 360) + 360) % 360 / 90.0) % 4
    if abs(((yaw - h*90) + 180) % 360 - 180) > 1e-6:
        raise ValueError(f'航向 {yaw} 度不是0/90/180/270，规划只支持横平竖直')
    return h


def wrap180(a):
    """角度换到 (-180, 180]。"""
    a = (a + 180) % 360 - 180
    return 180 if a == -180 else a


def yaw_of(h):
    return wrap180(h*90)


def _rect_point_dist(x, y, r):
    dx = np.maximum(np.maximum(r[0]-x, 0), x-r[2])
    dy = np.maximum(np.maximum(r[1]-y, 0), y-r[3])
    return np.hypot(dx, dy)


class Motion:
    """STM32 实际的运动快慢，用来估每条指令要多少秒。数值都能在配置里改(键名见下面)。
    v17：配置里 stm32_params 有 SCACC/SSACC(STM32 真正用的加减速)时按它算，没有才用 route_acc_rpm_s/strafe_acc_rpm_s。"""
    def __init__(self, cfg):
        g = cfg.get
        sp = g('stm32_params') or {}
        if not isinstance(sp, dict):
            sp = {}
        acc_f = sp.get('SCACC', g('route_acc_rpm_s', 300))
        acc_s = sp.get('SSACC', g('strafe_acc_rpm_s', 200))
        self.v_f = float(g('route_speed_rpm', 220)) / 60.0 * float(g('fwd_mm_per_rev', 249.0))        # 直行最高速 mm/s
        self.a_f = float(acc_f) / 60.0 * float(g('fwd_mm_per_rev', 249.0))                            # 直行加减速 mm/s²
        self.v_s = float(g('strafe_speed_rpm', 170)) / 60.0 * float(g('strafe_mm_per_rev', 235.0))    # 横移最高速
        self.a_s = float(acc_s) / 60.0 * float(g('strafe_mm_per_rev', 235.0))                         # 横移加减速
        self.turn90 = float(g('turn90_s', 1.8))          # 一次 90° 转弯(含起步、对准、等待回复)
        self.turn180 = float(g('turn180_s', 2.5))
        self.overhead = float(g('cmd_overhead_s', 0.25))  # 每条指令：串口往返 + 停稳 + 读陀螺仪

    @staticmethod
    def _seg(d, v, a):
        d = abs(d)
        if d <= 0:
            return 0.0
        if d >= v*v/a:
            return d/v + v/a
        return 2*math.sqrt(d/a)

    def time_of(self, cmd, val):
        if cmd == 'F':
            return self._seg(val, self.v_f, self.a_f) + self.overhead
        if cmd == 'S':
            return self._seg(val, self.v_s, self.a_s) + self.overhead
        if cmd == 'R':
            a = abs(val)
            if a < 1:
                return 0.0
            t = self.turn90*a/90.0 if a <= 90 else self.turn90 + (self.turn180-self.turn90)*(min(a, 180)-90)/90.0
            return max(t, 1.0) + 0.1
        return 0.0

    def route_time(self, cmds):
        return sum(self.time_of(c, v) for c, v in cmds)


# ====================================================================================
# 车身和各区域的几何
# ====================================================================================

class Body:
    """真实车身(车身坐标：x 朝车头，y 朝车左，原点=车中心)：L×W 的矩形 + 左侧雷达圆盘。
    转轴：车中心后 turn_pivot_back_mm、左 turn_pivot_left_mm(默认都是 0，STM32 的 R 绕车中心转)。"""
    def __init__(self, cfg):
        g = cfg.get
        self.hl = float(g('car_length_mm', 290)) / 2
        self.hw = float(g('car_width_mm', 260)) / 2
        ov = float(g('lidar_overhang_mm', 45))       # 雷达比车身左侧多伸出多少
        self.disk = None
        if ov > 0:
            lf = float(g('lidar_forward_mm', 22.5))
            ll = float(g('lidar_left_mm', 134.8))
            side = 1.0 if ll >= 0 else -1.0
            r = self.hw + ov - abs(ll)
            if r < 10:                                # 配置里雷达位置和突出量对不上：按突出量放一个 35mm 的圆盘
                r = min(35.2, self.hw + ov)
                ll = side * (self.hw + ov - r)
            self.disk = (lf, ll, r)
        self.d = float(g('turn_pivot_back_mm', 0))
        self.l = float(g('turn_pivot_left_mm', 0))
        # 转轴到车身最远点的距离：原地转弯扫过的圆
        pts = [(sx*self.hl + self.d, sy*self.hw - self.l) for sx in (-1, 1) for sy in (-1, 1)]
        rr = max(math.hypot(x, y) for x, y in pts)
        if self.disk:
            lf, ll, r = self.disk
            rr = max(rr, math.hypot(lf + self.d, ll - self.l) + r)
        self.r_turn = rr
        left = max(self.hw, (self.disk[1] + self.disk[2]) if self.disk else 0.0)
        right = max(self.hw, (-self.disk[1] + self.disk[2]) if self.disk else 0.0)
        front = max(self.hl, (self.disk[0] + self.disk[2]) if self.disk else 0.0)
        back = max(self.hl, (-self.disk[0] + self.disk[2]) if self.disk else 0.0)
        self.box = (-back, front, -right, left)       # 外框(含雷达)：后、前、右、左

    def key(self):
        return (self.hl, self.hw, self.disk, self.d, self.l)

    def center_off(self, yaw):
        """转轴点 → 车中心(世界坐标的偏移)。"""
        a = math.radians(yaw)
        c, s = math.cos(a), math.sin(a)
        return (self.d*c + self.l*s, self.d*s - self.l*c)

    def corners(self, x, y, yaw):
        a = math.radians(yaw); c, s = math.cos(a), math.sin(a)
        return [(x + bx*c - by*s, y + bx*s + by*c)
                for bx, by in ((-self.hl, -self.hw), (self.hl, -self.hw), (self.hl, self.hw), (-self.hl, self.hw))]

    def disk_at(self, x, y, yaw):
        if not self.disk:
            return None
        a = math.radians(yaw); c, s = math.cos(a), math.sin(a)
        lf, ll, r = self.disk
        return (x + lf*c - ll*s, y + lf*s + ll*c, r)


# 区域：(说明, 种类, x0, y0, x1, y1, 圆角半径, 平移硬余量, 平移希望余量, 转弯硬余量, 转弯希望余量)
# 种类：yellow 黄色区、zone 暂存/粗加工区、raw 原料转盘、edge 场地边、fixed 封死的车道/额外禁区、obs 障碍物
def _mk(what, kind, rect, rr, mh, mp, tm, add_turn=False):
    th = (mh + tm) if add_turn else max(mh, tm)
    tp = (mp + tm) if add_turn else max(mp, tm)
    return (what, kind, float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3]), float(rr),
            float(mh), float(mp), float(th), float(tp))


def raw_area(cfg):
    """原料转盘中心可能在的范围 (x0, y0, x1, y1)：规则说中心 x 在 1100~1300 随机，伸进场内 75~85 → y 2465~2475。
    配置 raw_area_mm 可以直接给；没给就按 raw_center_mm 的 x ± raw_x_range_mm(默认100)、y 取 min(配置y, 2465)~配置y。"""
    a = cfg.get('raw_area_mm')
    if a:
        return tuple(float(v) for v in a[:4])
    rc = cfg.get('raw_center_mm', [1200, 2480])
    cx, cy = float(rc[0]), float(rc[1])
    dx = float(cfg.get('raw_x_range_mm', 100))
    return (cx - dx, min(cy, 2465.0), cx + dx, max(cy, 2465.0))


def _margins(cfg):
    g = cfg.get
    ym = max(0.0, float(g('yellow_margin_mm', YELLOW_MARGIN_MM)))
    ymin = float(g('yellow_min_margin_mm', YELLOW_MIN_MARGIN_MM))
    ymin = max(1.0, min(ymin, ym))          # 最低档不会是 0(压到黄色区本轮就结束)
    ym = max(ym, ymin)
    return dict(ym=ym, ymin=ymin,
                zm=max(0.0, float(g('zone_margin_mm', ZONE_MARGIN_MM))),
                rm=max(0.0, float(g('raw_margin_mm', RAW_MARGIN_MM))),
                em=max(0.0, float(g('edge_margin_mm', EDGE_MARGIN_MM))),
                cor=max(0.0, float(g('start_corridor_mm', START_CORRIDOR_MM))),
                tm=max(0.0, float(g('turn_margin_mm', 20))),
                om=max(0.0, float(g('margin_mm', 20))))


def _edge_items(em, cor, tm, soft=True):
    """场地边：场地外面按几块大矩形算。启停区那一段余量 0(车要停进角上)；启停区旁边 cor 这一段：
    soft=True 时硬余量 0、希望余量 em(开进启停区时可以贴边，别的时候尽量不贴)。"""
    out = []
    lo, hi = 300.0, 2100.0                # 启停区在东边 y<300 和 y>2100，x>2100
    c_em = 0.0 if soft else em
    W = '场地边'
    # 西边：整条都按 em
    out.append(_mk(W, 'edge', (-_BIG, -_BIG, 0, _BIG), 0, em, em, tm))
    # 南边、北边：x<2100-cor 按 em；2100-cor~2100 是启停区旁边；x>2100 是启停区
    for y0, y1 in ((-_BIG, 0.0), (FIELD, _BIG)):
        out.append(_mk(W, 'edge', (-_BIG, y0, hi - cor, y1), 0, em, em, tm))
        if cor > 0:
            out.append(_mk(W, 'edge', (hi - cor, y0, hi, y1), 0, c_em, em, tm))
        out.append(_mk(W, 'edge', (hi, y0, _BIG, y1), 0, 0.0, 0.0, tm))
    # 东边
    out.append(_mk(W, 'edge', (FIELD, -_BIG, _BIG, lo), 0, 0.0, 0.0, tm))
    out.append(_mk(W, 'edge', (FIELD, hi, _BIG, _BIG), 0, 0.0, 0.0, tm))
    if cor > 0:
        out.append(_mk(W, 'edge', (FIELD, lo, _BIG, min(lo + cor, hi)), 0, c_em, em, tm))
        out.append(_mk(W, 'edge', (FIELD, max(hi - cor, lo), _BIG, hi), 0, c_em, em, tm))
    if lo + cor < hi - cor:
        out.append(_mk(W, 'edge', (FIELD, lo + cor, _BIG, hi - cor), 0, em, em, tm))
    return out


def fixed_items(cfg, margin_mm=None):
    """固定区域(黄色区、暂存/粗加工区、原料转盘、场地边、extra_blocked_rects)。
    margin_mm 给定时：所有区域都按这一个余量(场地边也是，启停区旁边也不例外；只有启停区本身那一段场地边还是 0)。"""
    m = _margins(cfg)
    if margin_mm is not None:
        v = max(0.0, float(margin_mm))
        ymh = ymp = zm = rm = em = v
        soft = False
    else:
        ymh, ymp, zm, rm, em = m['ymin'], m['ym'], m['zm'], m['rm'], m['em']
        soft = True
    tm = m['tm']
    out = []
    for r in YELLOW:
        out.append(_mk('黄色区', 'yellow', r, 0, ymh, ymp, tm))
    out.append(_mk('暂存区', 'zone', TEMP_ZONE, 0, zm, zm, tm))
    out.append(_mk('粗加工区', 'zone', ROUGH_ZONE, 0, zm, zm, tm))
    out.append(_mk('原料转盘', 'raw', raw_area(cfg), float(cfg.get('raw_radius_mm', 150)), rm, rm, tm))
    out += _edge_items(em, m['cor'], tm, soft=soft)
    for r in (cfg.get('extra_blocked_rects') or []):
        out.append(_mk(f'禁区{tuple(r)}', 'fixed', r, 0, 0.0, 0.0, tm))
    return out


def obstacle_items(cfg, obstacles, margin_mm=None):
    m = _margins(cfg)
    om = m['om'] if margin_mm is None else max(0.0, float(margin_mm))
    return [_mk(f'障碍物({float(x):.0f},{float(y):.0f})', 'obs', (x, y, x, y), r, om, om, m['tm'], add_turn=True)
            for x, y, r in obstacles]


def sealed_items(cfg, sealed):
    tm = _margins(cfg)['tm']
    return [_mk(f'有障碍物、封死的车道 x{r[0]}~{r[2]} y{r[1]}~{r[3]}', 'fixed', r, 0, 0.0, 0.0, tm) for r in sealed]


# ---------- 标量几何(任意角度)：凸多边形 / 线段 / 点 到轴对齐矩形的有符号距离 ----------

def _hull(pts):
    pts = sorted(set((round(p[0], 6), round(p[1], 6)) for p in pts))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _seg_pt(px, py, ax, ay, bx, by):
    vx, vy = bx-ax, by-ay
    L2 = vx*vx + vy*vy
    t = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((px-ax)*vx + (py-ay)*vy) / L2))
    return math.hypot(px - (ax + t*vx), py - (ay + t*vy))


def _poly_gap(poly, r):
    """凸多边形(1~n 个点)到矩形 r=(x0,y0,x1,y1) 的有符号距离：分开时是真实最近距离，重叠时是负的穿入深度。"""
    x0, y0, x1, y1 = r
    rc = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    axes = [(1.0, 0.0), (0.0, 1.0)]
    n = len(poly)
    if n >= 2:
        for k in range(n if n > 2 else 1):
            ax_, ay_ = poly[k]; bx_, by_ = poly[(k+1) % n]
            ex, ey = bx_-ax_, by_-ay_
            L = math.hypot(ex, ey)
            if L > 1e-9:
                axes.append((-ey/L, ex/L))
    best = -math.inf
    for ux, uy in axes:
        pp = [p[0]*ux + p[1]*uy for p in poly]
        rp = [c[0]*ux + c[1]*uy for c in rc]
        sep = max(min(rp) - max(pp), min(pp) - max(rp))
        if sep > best:
            best = sep
    if best <= 0:
        return best
    d = math.inf
    for px, py in poly:
        dx = max(x0 - px, 0.0, px - x1); dy = max(y0 - py, 0.0, py - y1)
        d = min(d, math.hypot(dx, dy))
    if n >= 2:
        for cx, cy in rc:
            for k in range(n if n > 2 else 1):
                ax_, ay_ = poly[k]; bx_, by_ = poly[(k+1) % n]
                d = min(d, _seg_pt(cx, cy, ax_, ay_, bx_, by_))
    return d


def _shape_gaps(items, hull, seg, rdisk, sag=0.0):
    """一个形状(车身凸包 hull + 雷达扫过的线段 seg、半径 rdisk)到每个区域的距离(已减去区域圆角)。返回列表。"""
    xs = [p[0] for p in hull] + ([p[0] - rdisk for p in seg] + [p[0] + rdisk for p in seg] if seg else [])
    ys = [p[1] for p in hull] + ([p[1] - rdisk for p in seg] + [p[1] + rdisk for p in seg] if seg else [])
    bx0, bx1, by0, by1 = min(xs), max(xs), min(ys), max(ys)
    out = []
    for it in items:
        rect = (it[2], it[3], it[4], it[5]); rr = it[6]
        gx = max(rect[0] - bx1, bx0 - rect[2]); gy = max(rect[1] - by1, by0 - rect[3])
        lb = math.hypot(max(gx, 0.0), max(gy, 0.0)) - rr
        if lb > max(it[8], it[10]) + 100:          # 离得很远：不用细算
            out.append(lb)
            continue
        g = _poly_gap(hull, rect) - rr
        if seg:
            g = min(g, _poly_gap(seg, rect) - rr - rdisk)
        out.append(g - sag)
    return out


class _Sweep:
    """按指令一条条推算车位，算每条指令扫过的范围离各区域多远。"""
    def __init__(self, body, items):
        self.b = body
        self.items = items

    def pose_gaps(self, pose):
        x, y, yaw = pose
        dk = self.b.disk_at(x, y, yaw)
        return _shape_gaps(self.items, _hull(self.b.corners(x, y, yaw)),
                           [(dk[0], dk[1])] if dk else None, dk[2] if dk else 0.0)

    def move_gaps(self, pose, cmd, val, pivot=None):
        """返回 (新位姿, 每个区域的距离, 是否转弯)。pivot=(后, 左) 转轴相对车中心的位置。"""
        x, y, yaw = pose
        b = self.b
        if cmd in ('F', 'S'):
            a = math.radians(yaw)
            if cmd == 'F':
                nx, ny = x + val*math.cos(a), y + val*math.sin(a)
            else:
                nx, ny = x - val*math.sin(a), y + val*math.cos(a)
            hull = _hull(b.corners(x, y, yaw) + b.corners(nx, ny, yaw))
            d0, d1 = b.disk_at(x, y, yaw), b.disk_at(nx, ny, yaw)
            seg = [(d0[0], d0[1]), (d1[0], d1[1])] if d0 else None
            return (nx, ny, yaw), _shape_gaps(self.items, hull, seg, d0[2] if d0 else 0.0), False
        if cmd == 'R':
            pb, pl = (b.d, b.l) if pivot is None else (float(pivot[0]), float(pivot[1]))
            a = math.radians(yaw)
            px = x - pb*math.cos(a) - pl*math.sin(a)
            py = y - pb*math.sin(a) + pl*math.cos(a)
            n = max(1, int(math.ceil(abs(val) / 5.0)))
            st = float(val) / n
            # 转轴到车身最远点的距离(按这次的转轴)
            pts = [(sx*b.hl + pb, sy*b.hw - pl) for sx in (-1, 1) for sy in (-1, 1)]
            R0 = max(math.hypot(u, v) for u, v in pts)
            if b.disk:
                R0 = max(R0, math.hypot(b.disk[0] + pb, b.disk[1] - pl) + b.disk[2])
            sag = R0 * (1 - math.cos(math.radians(abs(st)) / 2))
            gaps = None
            cur = (x, y, yaw)

            def rot(p, ang):
                ca, sa = math.cos(math.radians(ang)), math.sin(math.radians(ang))
                dx, dy = p[0]-px, p[1]-py
                return (px + dx*ca - dy*sa, py + dx*sa + dy*ca, p[2] + ang)
            for k in range(n):
                nxt = rot(cur, st)
                hull = _hull(b.corners(*cur) + b.corners(*nxt))
                d0, d1 = b.disk_at(*cur), b.disk_at(*nxt)
                seg = [(d0[0], d0[1]), (d1[0], d1[1])] if d0 else None
                g = _shape_gaps(self.items, hull, seg, d0[2] if d0 else 0.0, sag)
                gaps = g if gaps is None else [min(u, v) for u, v in zip(gaps, g)]
                cur = nxt
            return (cur[0], cur[1], wrap180(yaw + val)), gaps, True
        raise ValueError(cmd)


def _worst(items, gaps, turn):
    """最紧的那个区域：返回 (余量差 = 距离-要求, 距离, 下标)。"""
    best = None
    for k, (it, g) in enumerate(zip(items, gaps)):
        need = it[9] if turn else it[7]
        s = g - need
        if best is None or s < best[0]:
            best = (s, g, k)
    return best


def _what(it, gap, turn):
    need = it[9] if turn else it[7]
    if it[1] == 'edge':
        if gap < 0:
            return '车身(含雷达)会出场地'
        return f'车身(含雷达)离场地边只有 {gap:.0f}mm(要 {need:.0f}mm)'
    if gap < 0:
        return f'车身(含雷达)会压到{it[0]}'
    return f'车身(含雷达)离{it[0]}只有 {gap:.0f}mm(要 {need:.0f}mm)'


def pose_clear(cfg, obstacles, pose, margin_mm=None):
    """某个车位(车中心 x, y, 车头度数，任意角度)放得下吗？
    按真实车身(矩形+雷达圆盘)算到黄色区、暂存/粗加工区、原料转盘(整个可能范围)、场地边、障碍物的距离。
    margin_mm=None：各区域按配置的硬余量(黄色区按 yellow_min_margin_mm，启停区和它旁边那段场地边允许贴边)；
    给定时所有区域都按这一个余量(只有启停区本身那一段场地边还是 0)。
    返回 (ok, 最近距离mm, 说明)：不 ok 时是最差的那个区域，ok 时是离得最近的那个区域。"""
    body = Body(cfg)
    items = fixed_items(cfg, margin_mm) + obstacle_items(cfg, obstacles or [], margin_mm)
    gaps = _Sweep(body, items).pose_gaps(tuple(float(v) for v in pose[:3]))
    s, g, k = _worst(items, gaps, False)
    if s < -EPS_MM:
        return False, g, _what(items[k], g, False)
    k2 = min(range(len(items)), key=lambda i: gaps[i])
    return True, gaps[k2], _what(items[k2], gaps[k2], False)


def _moves_eval(body, items, start_pose, cmds, pivot=None, first_pivot_world=None):
    """逐条推算并检查。返回 (ok, 最紧的距离, 下标, 说明, 每条指令后的位姿, 每个区域全程最近距离)。"""
    sw = _Sweep(body, items)
    pose = tuple(float(v) for v in start_pose[:3])
    mins = list(sw.pose_gaps(pose))
    worst = None
    poses = []
    for idx, (c, v) in enumerate(cmds):
        pv = pivot
        if idx == 0 and c == 'R' and first_pivot_world is not None:
            a = math.radians(pose[2])
            dx, dy = pose[0] - first_pivot_world[0], pose[1] - first_pivot_world[1]
            pv = (dx*math.cos(a) + dy*math.sin(a), dx*math.sin(a) - dy*math.cos(a))
        pose, gaps, turn = sw.move_gaps(pose, c, float(v), pv)
        poses.append(pose)
        mins = [min(a, b) for a, b in zip(mins, gaps)]
        s, g, k = _worst(items, gaps, turn)
        if worst is None or s < worst[0]:
            worst = (s, g, idx, k, turn)
        if s < -EPS_MM:
            return False, g, idx, f'第 {idx+1} 条 {c} {v}：' + _what(items[k], g, turn), poses, mins
    if worst is None:
        s, g, k = _worst(items, mins, False)
        if s < -EPS_MM:
            return False, g, None, '起点：' + _what(items[k], g, False), poses, mins
        return True, g, None, _what(items[k], g, False), poses, mins
    s, g, idx, k, turn = worst
    return True, g, idx, _what(items[k], g, turn), poses, mins


def moves_clear(cfg, obstacles, start_pose, cmds, margin_mm=None, pivot=None):
    """检查一串车体指令 [('F',mm)/('S',mm)/('R',度)] 扫过的范围(真实车身+雷达)：
    F/S 按整段平移扫过的范围算，R 绕转轴按每步不超过 5° 一步步扫(转弯离各区域至少 max(该区域余量, turn_margin_mm))。
    pivot=(后mm, 左mm)：转轴相对车中心的位置，None 按配置 turn_pivot_back_mm / turn_pivot_left_mm。
    margin_mm 同 pose_clear。
    返回 (ok, 最近距离mm, 下标, 说明)：不 ok 时下标是第一条出问题的指令；ok 时是最紧的那一条(没有指令时 None)。"""
    body = Body(cfg)
    items = fixed_items(cfg, margin_mm) + obstacle_items(cfg, obstacles or [], margin_mm)
    ok, g, idx, what, _, _ = _moves_eval(body, items, start_pose, list(cmds), pivot)
    return ok, g, idx, what


def apply_body_move(pose, cmd, val):
    """按车体指令推算新位姿（航位推算，不是测量）。"""
    x, y, yaw = pose; a = math.radians(yaw)
    if cmd == 'F':
        return (x+val*math.cos(a), y+val*math.sin(a), yaw)
    if cmd == 'S':
        return (x-val*math.sin(a), y+val*math.cos(a), yaw)
    if cmd == 'R':
        return (x, y, wrap180(yaw+val))
    raise ValueError(cmd)


# ====================================================================================
# 栅格表(numpy，一次算好)
# ====================================================================================

def _bgap(ax0, ay0, ax1, ay1, it):
    """轴对齐矩形到区域的有符号距离(numpy)，减去区域圆角。"""
    gx = np.maximum(it[2] - ax1, ax0 - it[4])
    gy = np.maximum(it[3] - ay1, ay0 - it[5])
    out = np.hypot(np.maximum(gx, 0.0), np.maximum(gy, 0.0))
    inside = (gx <= 0) & (gy <= 0)
    if np.any(inside):
        out = np.where(inside, np.maximum(gx, gy), out)
    return out - it[6]


def _grid_maps(body, items, ox, oy):
    """栅格(原点偏移 ox, oy)上每个转轴点、4 个车头方向：
    node_h/node_p = min(距离-硬余量)/min(距离-希望余量)；sw_* = 往 +x / +y 走一格扫过的范围；turn_* = 原地转弯的圆；
    gap = 车身到最近区域的真实距离(净空代价用)。"""
    xs = ox + np.arange(N) * STEP
    ys = oy + np.arange(N) * STEP
    PX, PY = np.meshgrid(xs, ys, indexing='ij')
    inf = np.full((N, N), np.inf)
    node_h = np.stack([inf.copy() for _ in range(4)]); node_p = node_h.copy(); gap = node_h.copy()
    sw_h = np.full((4, 2, N, N), np.inf); sw_p = sw_h.copy()
    turn_h = inf.copy(); turn_p = inf.copy()
    if not items:
        return dict(node_h=node_h, node_p=node_p, sw_h=sw_h, sw_p=sw_p, turn_h=turn_h, turn_p=turn_p, gap=gap)
    hl, hw = body.hl, body.hw
    for h in range(4):
        yaw = h * 90
        cdx, cdy = body.center_off(yaw)
        cdx, cdy = round(cdx, 6), round(cdy, 6)
        X = PX + cdx; Y = PY + cdy
        fx, fy = DIRS[h]
        # 车身矩形在这个车头方向下往西、东、南、北伸出多少
        ex = abs(fx)*hl + abs(fy)*hw
        ey = abs(fy)*hl + abs(fx)*hw
        if body.disk:
            lf, ll, r = body.disk
            ddx = lf*fx - ll*fy; ddy = lf*fy + ll*fx
        for it in items:
            g = _bgap(X - ex, Y - ey, X + ex, Y + ey, it)
            gxs = _bgap(X - ex, Y - ey, X + STEP + ex, Y + ey, it)
            gys = _bgap(X - ex, Y - ey, X + ex, Y + STEP + ey, it)
            if body.disk:
                DX = X + ddx; DY = Y + ddy
                g = np.minimum(g, _bgap(DX, DY, DX, DY, it) - r)
                gxs = np.minimum(gxs, _bgap(DX, DY, DX + STEP, DY, it) - r)
                gys = np.minimum(gys, _bgap(DX, DY, DX, DY + STEP, it) - r)
            np.minimum(node_h[h], g - it[7], out=node_h[h])
            np.minimum(node_p[h], g - it[8], out=node_p[h])
            if it[1] != 'edge' or it[8] > 0:
                np.minimum(gap[h], g, out=gap[h])
            np.minimum(sw_h[h, 0], gxs - it[7], out=sw_h[h, 0])
            np.minimum(sw_p[h, 0], gxs - it[8], out=sw_p[h, 0])
            np.minimum(sw_h[h, 1], gys - it[7], out=sw_h[h, 1])
            np.minimum(sw_p[h, 1], gys - it[8], out=sw_p[h, 1])
    for it in items:
        g = _bgap(PX, PY, PX, PY, it) - body.r_turn
        np.minimum(turn_h, g - it[9], out=turn_h)
        np.minimum(turn_p, g - it[10], out=turn_p)
    return dict(node_h=node_h, node_p=node_p, sw_h=sw_h, sw_p=sw_p, turn_h=turn_h, turn_p=turn_p, gap=gap)


def _merge_maps(a, b):
    return {k: np.minimum(a[k], b[k]) for k in a}


_FIXED_CACHE = {}


def _fixed_maps(body, items, ox, oy):
    """固定区域的表只和车身、配置、栅格偏移有关：缓存起来，各段、各次规划都用同一份。"""
    key = (body.key(), tuple(items), round(ox, 3), round(oy, 3))
    m = _FIXED_CACHE.get(key)
    if m is None:
        if len(_FIXED_CACHE) >= 8:
            _FIXED_CACHE.pop(next(iter(_FIXED_CACHE)))
        m = _grid_maps(body, items, ox, oy)
        _FIXED_CACHE[key] = m
    return m


# ====================================================================================
# 规划器
# ====================================================================================

class Planner:
    def __init__(self, cfg, obstacles):
        """obstacles: [(x, y, r), ...] 扫描得到的障碍物。栅格节点是“转轴点”的位置(转轴在车中心时就是车中心)。"""
        self.cfg = cfg
        self.body = Body(cfg)
        self.box = self.body.box
        self.ext = [self._ext(h) for h in range(4)]
        self.d = self.body.d
        self.mg = _margins(cfg)
        self.yellow_margin = self.mg['ym']
        self.yellow_min = self.mg['ymin']
        self.turn_r = self.body.r_turn + self.mg['tm']
        self.obs_margin = self.mg['om']
        self.mode = cfg.get('drive_mode', 'mecanum')
        if self.mode not in ('mecanum', 'rotate'):
            raise ValueError('drive_mode 只能是 mecanum 或 rotate')
        self.motion = Motion(cfg)
        m = self.motion
        # ---- A* 代价(秒) ----
        # 每走一格(25mm)的用时；横移再乘一个“不如直行准”的系数(strafe_cost_factor，默认1.5)
        self.sf = max(1.0, float(cfg.get('strafe_cost_factor', 1.5)))
        self.step_f = STEP / m.v_f
        self.step_s = STEP / m.v_s * self.sf
        # 每开始一段新的直行/横移：加减速损失 + 指令开销
        self.seg_f = m.v_f / m.a_f + m.overhead
        self.seg_s = (m.v_s / m.a_s + m.overhead) * (1 + (self.sf - 1)/2)
        self.turn1 = m.time_of('R', 90)
        self.turn2 = m.time_of('R', 180)
        # 每一段路线里横移的总长度上限(mm)。None=不限制(默认)。限制了规划不出来会自动放开再规划一次
        cap = cfg.get('strafe_max_mm_per_leg', None)
        self.strafe_cap = None if cap is None else int(float(cap)//STEP)
        # 离黄色区/墙/障碍物太近的代价：净空小于 clearance_pref_mm 时，每一步的代价最多乘 (1+clearance_penalty)
        self.clr_pref = float(cfg.get('clearance_pref_mm', 120))
        self.clr_pen = float(cfg.get('clearance_penalty', 1.0))
        # 比希望的余量近(黄色区 15~30mm、启停区旁边贴边)：每走一格多算 yellow_soft_cost_s 秒，每差 1mm 再多算
        # soft_cost_per_mm_s 秒。前一项让规划宁可绕一点也不挤；后一项很小，只让挤的时候走中间，不会为了几毫米来回横移(抖)
        self.soft_cost = float(cfg.get('yellow_soft_cost_s', 0.5))
        self.soft_mm = float(cfg.get('soft_cost_per_mm_s', 0.004))
        rc = cfg.get('raw_center_mm', [1200, 2480]); self.raw = (float(rc[0]), float(rc[1]), float(cfg.get('raw_radius_mm', 150)))
        self.obstacles = [(float(x), float(y), float(r)) for x, y, r in obstacles]
        self.sealed = lane_block_rects(cfg, self.obstacles) if cfg.get('lane_block', True) else []
        self.fixed = fixed_items(cfg)
        self.var = obstacle_items(cfg, self.obstacles) + sealed_items(cfg, self.sealed)
        self.items = self.fixed + self.var
        self.fallback = False
        self.lane_fallback = False
        self._var_cache = {}

    def _ext(self, h):
        """车头方向 h 时，车身外框(含雷达)相对车中心往 西、东、南、北 各伸出多少 mm。"""
        xb, xf, yr, yl = self.box
        fx, fy = DIRS[h]
        lx, ly = -fy, fx                                   # 车左边的方向
        xs = [a*fx + b*lx for a in (xb, xf) for b in (yr, yl)]
        ys = [a*fy + b*ly for a in (xb, xf) for b in (yr, yl)]
        return -min(xs), max(xs), -min(ys), max(ys)

    def pivot_of(self, x, y, yaw):
        cx, cy = self.body.center_off(yaw)
        return x - cx, y - cy

    def center_of(self, px, py, yaw):
        cx, cy = self.body.center_off(yaw)
        return px + cx, py + cy

    # ---------- 查询 ----------
    def maps(self, ox, oy, items=None):
        """栅格偏移 (ox, oy) 的表。items 给定时(放宽了某个区域)不用缓存，现算。"""
        if items is not None:
            return _grid_maps(self.body, items, ox, oy)
        key = (round(ox, 3), round(oy, 3))
        m = self._var_cache.get(key)
        if m is None:
            f = _fixed_maps(self.body, self.fixed, ox, oy)
            m = _merge_maps(f, _grid_maps(self.body, self.var, ox, oy)) if self.var else f
            self._var_cache[key] = m
        return m

    def pose_eval(self, pose, items=None):
        items = self.items if items is None else items
        return _Sweep(self.body, items).pose_gaps(pose)

    def why_blocked(self, x, y, h=1):
        """车中心在 (x, y)、车头方向索引 h 时为什么放不下(按硬余量)。放得下返回 ''。"""
        gaps = self.pose_eval((float(x), float(y), yaw_of(h)))
        s, g, k = _worst(self.items, gaps, False)
        if s >= -EPS_MM:
            return ''
        it = self.items[k]
        if it[1] == 'fixed' and it in self.var:
            return f'这段车道有障碍物，已封死({it[0]})'
        if it[1] == 'obs':
            return f'离{it[0]}太近(只有 {g:.0f}mm，要 {it[7]:.0f}mm)'
        if it[1] == 'yellow' and g >= 0:
            return f'车身(含雷达)离黄色区不到 {it[7]:.0f}mm(只有 {g:.0f}mm)'
        return _what(it, g, False)

    def point_free(self, px, py, h):
        """任意位置(转轴点 px,py，车头方向索引 h)车身放得下吗(不要求在栅格节点上)。"""
        c = self.center_of(px, py, yaw_of(h))
        gaps = self.pose_eval((c[0], c[1], yaw_of(h)))
        return _worst(self.items, gaps, False)[0] >= -EPS_MM

    # ---------- 单段A*(栅格) ----------
    def _leg_tables(self, M):
        """把表变成 A* 用的数组/扁平列表：每个车头方向、每个走向一张“这一步要多少秒(走不了=inf)”。"""
        node_ok = M['node_h'] >= 0
        sw_ok = M['sw_h'] >= 0
        turn_ok = M['turn_h'] >= 0
        pref = max(self.clr_pref, 1.0)
        mult = 1.0 + self.clr_pen * np.clip((pref - M['gap']) / pref, 0.0, 1.0)
        sc, smm = self.soft_cost, self.soft_mm

        def pen(slack_p):
            d = np.maximum(-slack_p, 0.0)
            return np.where(d > 0, sc + smm * d, 0.0)
        mec = self.mode == 'mecanum'
        cost = np.full((4, 4, N, N), np.inf)
        for h in range(4):
            for k, (di, dj) in enumerate(DIRS):
                lateral = (k - h) % 2 == 1
                if lateral and not mec:
                    continue
                ax = 0 if di else 1
                base = self.step_s if lateral else self.step_f
                c = base * np.maximum(mult[h], np.roll(mult[h], -1, axis=ax)) + pen(M['sw_p'][h, ax])
                c = np.where(sw_ok[h, ax], c, np.inf)
                # c[i,j] = 从 (i,j) 往 +ax 走一格；往 -ax 走一格 = 从 (i-1,j) 往 +ax 的那一格
                if di > 0 or dj > 0:
                    if ax == 0:
                        cost[h, k, :-1, :] = c[:-1, :]
                    else:
                        cost[h, k, :, :-1] = c[:, :-1]
                else:
                    if ax == 0:
                        cost[h, k, 1:, :] = c[:-1, :]
                    else:
                        cost[h, k, :, 1:] = c[:, :-1]
        turn = np.where(turn_ok, pen(M['turn_p']), np.inf)
        return dict(cost_a=cost, turn_a=turn, node_a=node_ok,
                    cost=[[cost[h, k].ravel().tolist() for k in range(4)] for h in range(4)],
                    turn=turn.ravel().tolist(), node=[node_ok[h].ravel().tolist() for h in range(4)])

    def _heur(self, T, gn, gh):
        """A* 的启发值：除了“每段横移的总长度上限”以外，和 A* 完全一样的代价(含每段起步、转弯)从每个状态到终点最少多少秒。
        用 numpy 一整行一整行地传(每轮多算一段直行/横移或一次转弯)，几十毫秒。到不了 = inf(也用来快速判断走不走得通)。
        返回 hb[h][d]：车头 h、正在往 d 方向走(d=4 停着)时的扁平列表。"""
        BIG = 1.0e6
        if 'prop' not in T:
            # 每个车头、每个走向的前缀和只和这一段的表有关：算一次，各个终点候选都用
            cost, turn, node = T['cost_a'], T['turn_a'], T['node_a']
            prop = []
            for h in range(4):
                for k, (di, dj) in enumerate(DIRS):
                    c = cost[h, k]
                    if not np.any(np.isfinite(c)):
                        continue
                    cb = np.where(np.isfinite(c), c, BIG)
                    ax = 0 if di else 1
                    plus = di > 0 or dj > 0
                    P = (np.cumsum(cb, axis=ax) - cb) if plus else np.cumsum(cb, axis=ax)
                    lateral = (k - h) % 2 == 1
                    seg = self.seg_s if lateral else self.seg_f
                    win = self.strafe_cap if (lateral and self.strafe_cap is not None) else None
                    prop.append((h, k, ax, plus, P, c + seg, win, cb))
            tc = []
            for h in range(4):
                for h2 in range(4):
                    if h2 != h:
                        steps = min((h2-h) % 4, (h-h2) % 4)
                        ok = np.isfinite(turn) & node[h2] & node[h]
                        tc.append((h, h2, np.where(ok, turn + (self.turn1 if steps == 1 else self.turn2), np.inf)))
            T['prop'], T['tc'] = prop, tc
        prop, tc = T['prop'], T['tc']
        gi, gj = divmod(gn, N)
        B = np.full((4, N, N), np.inf)          # 停着(或刚转完弯)时
        B[gh, gi, gj] = 0.0
        Vd = {}
        for _ in range(200):
            nb = B.copy()
            for h, k, ax, plus, P, cs, win, cb in prop:
                b = B[h]
                if win is not None:
                    # 有横移上限：一次横移最多 win 格(启发值离真实代价更近，A* 少搜很多)
                    v = b.copy()
                    for _ in range(win):
                        nv = np.full((N, N), np.inf)
                        if plus and ax == 0:
                            nv[:-1, :] = v[1:, :]
                        elif plus:
                            nv[:, :-1] = v[:, 1:]
                        elif ax == 0:
                            nv[1:, :] = v[:-1, :]
                        else:
                            nv[:, 1:] = v[:, :-1]
                        v = np.minimum(b, cb + nv)
                elif plus:
                    # 往 + 走：V[i] = min_{j>=i} (B[j] + P[j]) - P[i]，P[i] = 前 i 格的代价和
                    v = np.flip(np.minimum.accumulate(np.flip(b + P, axis=ax), axis=ax), axis=ax) - P
                else:
                    # 往 - 走：V[i] = min_{j<=i} (B[j] - Q[j]) + Q[i]，Q[i] = 到第 i 格(含)的代价和
                    v = np.minimum.accumulate(b - P, axis=ax) + P
                v[v >= BIG / 2] = np.inf
                Vd[(h, k)] = v
                # 停着的时候往 k 方向起步：起步代价 + 第一格 + 之后接着走
                nxt = np.full((N, N), np.inf)
                di, dj = DIRS[k]
                if di > 0:
                    nxt[:-1, :] = v[1:, :]
                elif di < 0:
                    nxt[1:, :] = v[:-1, :]
                elif dj > 0:
                    nxt[:, :-1] = v[:, 1:]
                else:
                    nxt[:, 1:] = v[:, :-1]
                np.minimum(nb[h], cs + nxt, out=nb[h])
            for h, h2, t in tc:
                np.minimum(nb[h], B[h2] + t, out=nb[h])
            if np.array_equal(nb, B):
                break
            B = nb
        inf_list = None
        out = []
        for h in range(4):
            row = []
            for d in range(4):
                v = Vd.get((h, d))
                if v is None:
                    if inf_list is None:
                        inf_list = [math.inf]*(N*N)
                    row.append(inf_list)
                else:
                    row.append(v.ravel().tolist())
            row.append(B[h].ravel().tolist())
            out.append(row)
        return out

    def _astar(self, T, sn, sh, gn, gh, hb):
        """A*：状态 = (栅格点, 车头, 上一步方向(4=停着), 这一段已横移的格数)，代价 = 秒。hb = _heur 的启发值。
        返回动作列表 [('M', 方向) / ('T', 新车头)]，走不通返回 None(搜得太多也返回 None，并置 self._exhausted)。"""
        cap = self.strafe_cap
        RS = (cap + 1) if cap is not None else 1
        offs = (N, 1, -N, -1)
        cost_tab, turn_tab, node_tab = T['cost'], T['turn'], T['node']
        turn1, turn2, seg_f, seg_s = self.turn1, self.turn2, self.seg_f, self.seg_s

        def hfun(n, h, d):
            return hb[h][d][n]

        def sid(n, h, d, r):
            return ((n*4 + h)*5 + d)*RS + r
        s0 = sid(sn, sh, 4, 0)
        cost = {s0: 0.0}; parent = {}
        order = itertools.count()
        q = [(hfun(sn, sh, 4), 0.0, next(order), 0.0, s0)]
        budget = int(self.cfg.get('astar_max_expand', 60000))    # 防止横移上限下走不通时把整个状态空间搜完
        self._exhausted = False
        while q:
            _, _, _, g, s = heapq.heappop(q)
            if g != cost.get(s):
                continue
            budget -= 1
            if budget < 0:
                self._exhausted = True
                return None
            r = s % RS; t_ = s // RS; d = t_ % 5; t_ //= 5; h = t_ % 4; n = t_ // 4
            if n == gn and h == gh:
                acts = []
                while s != s0:
                    s, a = parent[s]
                    acts.append(a)
                return acts[::-1]
            row = cost_tab[h]
            for k in range(4):
                tab = row[k]
                if tab is None:
                    continue
                c = tab[n]
                if c == math.inf:
                    continue
                lateral = (k - h) % 2 == 1
                if lateral and cap is not None and r + 1 > cap:
                    continue                                  # 这一段的横移额度用完了
                m = n + offs[k]
                ng = g + c
                if d != k:                                    # 新开一段直行/横移(从静止起步、或掉头)
                    ng += seg_s if lateral else seg_f
                t = sid(m, h, k, r + (1 if (lateral and cap is not None) else 0))
                if ng < cost.get(t, math.inf):
                    cost[t] = ng; parent[t] = (s, ('M', k))
                    hv = hfun(m, h, k)
                    if hv < math.inf:
                        heapq.heappush(q, (ng + hv, -ng, next(order), ng, t))
            tp = turn_tab[n]
            if tp < math.inf:
                for h2 in range(4):
                    if h2 == h or not node_tab[h2][n]:
                        continue
                    steps = min((h2-h) % 4, (h-h2) % 4)
                    ng = g + (turn1 if steps == 1 else turn2) + tp
                    t = sid(n, h2, 4, r)
                    if ng < cost.get(t, math.inf):
                        cost[t] = ng; parent[t] = (s, ('T', h2))
                        heapq.heappush(q, (ng + hfun(n, h2, 4), -ng, next(order), ng, t))
        return None

    def plan_leg(self, start, goal):
        """(兼容旧接口) start/goal: 转轴点 (x, y, yaw)，在同一套 25mm 栅格上(栅格按 start 对齐)。返回 (动作列表, 说明)。"""
        ox, oy = start[0] % STEP, start[1] % STEP
        M = self.maps(ox, oy)
        sh, gh = heading_index(start[2]), heading_index(goal[2])
        si, sj = int(round((start[0]-ox)/STEP)), int(round((start[1]-oy)/STEP))
        gi, gj = (goal[0]-ox)/STEP, (goal[1]-oy)/STEP
        if abs(gi-round(gi)) > 1e-6 or abs(gj-round(gj)) > 1e-6:
            raise ValueError(f'({goal[0]},{goal[1]}) 不在起点的25mm栅格上')
        gi, gj = int(round(gi)), int(round(gj))
        if not (0 <= gi < N and 0 <= gj < N):
            raise ValueError(f'({goal[0]},{goal[1]}) 超出场地')
        if M['node_h'][sh, si, sj] < 0:
            c = self.center_of(start[0], start[1], yaw_of(sh))
            return None, '起点受阻：' + self.why_blocked(c[0], c[1], sh)
        if M['node_h'][gh, gi, gj] < 0:
            c = self.center_of(goal[0], goal[1], yaw_of(gh))
            return None, '终点受阻：' + self.why_blocked(c[0], c[1], gh)
        T = self._leg_tables(M)
        hb = self._heur(T, gi*N + gj, gh)
        if hb[sh][4][si*N + sj] == math.inf:
            return None, NO_ROUTE
        acts = self._astar(T, si*N + sj, sh, gi*N + gj, gh, hb)
        if acts is None:
            return None, NO_ROUTE
        return acts, 'OK'

    # ---------- 单段：准确地从起点走到停车点 ----------
    def leg(self, start_piv, goal_pose, name):
        """start_piv: 起点转轴点 (x, y, yaw)，任意位置(不要求在栅格上)。goal_pose: 停车点车中心 (x, y, yaw)。
        返回 (leg dict, 'OK') 或 (None, 原因)。"""
        sh = heading_index(start_piv[2]); gh = heading_index(goal_pose[2])
        sx, sy = float(start_piv[0]), float(start_piv[1])
        ox, oy = sx % STEP, sy % STEP
        si, sj = int(round((sx-ox)/STEP)), int(round((sy-oy)/STEP))
        if not (0 <= si < N and 0 <= sj < N):
            return None, '起点在场地外'
        sc = self.center_of(sx, sy, yaw_of(sh)); start_c = (sc[0], sc[1], yaw_of(sh))
        gx, gy, gyaw = float(goal_pose[0]), float(goal_pose[1]), float(goal_pose[2])
        gp = self.pivot_of(gx, gy, gyaw)
        notes = []
        items = list(self.items)
        relaxed = False
        # 起点、停车点本身离某个区域不够硬余量：只把这个区域放宽到它自己的距离(这一段)，并写进 note
        sg = self.pose_eval(start_c, items)
        gg = self.pose_eval((gx, gy, gyaw), items)
        park = False
        for k, it in enumerate(items):
            low = None
            if sg[k] < it[7] - EPS_MM:
                low = sg[k]
                notes.append(f'⚠ 起点离{it[0]}只有 {sg[k]:.0f}mm(要 {it[7]:.0f}mm)，离开时按 {max(sg[k], 0):.0f}mm 放宽')
            if gg[k] < it[7] - EPS_MM:
                if it[1] == 'obs':
                    park = True
                    continue
                if gg[k] < 0 or (it[1] == 'edge' and it[7] <= 0):
                    return None, f'停车点 {name}({gx:.0f},{gy:.0f}) 本身' + _what(it, gg[k], False).replace('车身(含雷达)', '车身(含雷达)就')
                low = gg[k] if low is None else min(low, gg[k])
                notes.append(f'⚠ 停车点 {name} 本身离{it[0]}只有 {gg[k]:.0f}mm(要 {it[7]:.0f}mm)，'
                             f'这一段只在{it[0]}这里放宽到 {gg[k]:.0f}mm，请把停车点往外挪')
            if low is not None:
                lst = list(it); lst[7] = min(it[7], low - 0.5); items[k] = tuple(lst)
                relaxed = True
        M = self.maps(ox, oy, items if relaxed else None)
        T = self._leg_tables(M)
        sn = si*N + sj
        if not T['node'][sh][sn]:
            return None, '起点受阻：' + (self.why_blocked(start_c[0], start_c[1], sh) or '车身放不下')
        # 终点：离准确停车点最近、车身放得下的几个栅格点(停车点被障碍物占了时，改停在最近的放得下的点)
        cands = []
        rad = 2 if not park else 12
        ci, cj = (gp[0]-ox)/STEP, (gp[1]-oy)/STEP
        for i in range(int(math.floor(ci)) - rad + 1, int(math.floor(ci)) + rad + 1):
            for j in range(int(math.floor(cj)) - rad + 1, int(math.floor(cj)) + rad + 1):
                if 0 <= i < N and 0 <= j < N and T['node'][gh][i*N + j]:
                    cands.append((math.hypot(ox + i*STEP - gp[0], oy + j*STEP - gp[1]), i, j))
        cands.sort()
        if not cands:
            return None, '终点受阻：' + (self.why_blocked(gx, gy, gh) or '附近没有放得下车身的位置')
        best = None
        why = NO_ROUTE
        tried = 0
        for dist_, gi, gj in cands:
            if tried >= (4 if not park else 3):
                break
            gn = gi*N + gj
            hb = self._heur(T, gn, gh)
            if hb[sh][4][sn] == math.inf:
                tried += 1
                continue
            acts = self._astar(T, sn, sh, gn, gh, hb)
            tried += 1
            if acts is None:
                if self._exhausted:
                    why = '找不到路线（按横移上限 strafe_max_mm_per_leg 走不过去）'
                    break
                continue
            node_piv = (ox + gi*STEP, oy + gj*STEP)
            target = (gp[0], gp[1]) if not park else node_piv
            res = self._finish(items, (sx, sy, yaw_of(sh)), acts, node_piv, target, gyaw)
            if res is None:
                continue
            score = (0 if res['ok'] else 1, res['tiny'], res['time'])
            if best is None or score < best[0]:
                best = (score, res, park, node_piv)
            if res['ok'] and res['tiny'] == 0:
                break
        if best is None:
            return None, why
        _, res, park, node_piv = best
        if not res['ok']:
            return None, '规划出来的指令扫过的范围不够余量：' + res['what']
        cmds, poses = res['cmds'], res['poses']
        if park:
            pc = self.center_of(node_piv[0], node_piv[1], gyaw)
            goal_c = (pc[0], pc[1], gyaw)
            ob = [it for k, it in enumerate(self.items) if it[1] == 'obs' and gg[k] < it[7] - EPS_MM]
            notes.append(f'⚠ {name} 的准确停车点({gx:.0f},{gy:.0f})被{ob[0][0] if ob else "障碍物"}挡住，'
                         f'改停在 ({goal_c[0]:.0f},{goal_c[1]:.0f})，差 {math.hypot(goal_c[0]-gx, goal_c[1]-gy):.0f}mm')
        else:
            goal_c = (gx, gy, gyaw)
        # 黄色区：比希望的余量近 → 提示(只有挤不过去的地方才会这样)
        ym = self.yellow_margin
        ymins = [res['mins'][k] for k, it in enumerate(items) if it[1] == 'yellow']
        ygap = min(ymins) if ymins else math.inf
        if ygap < ym - 0.5:
            notes.insert(0, f'⚠ 去 {name} 这一段按离黄色区 {ym:.0f}mm 规划不出来(车道太窄或停车点离黄色区太近)，'
                            f'这一段只留了 {max(ygap, 0):.0f}mm，走这段要特别注意。')
        real = [(res['mins'][k], it) for k, it in enumerate(items) if not (it[1] == 'edge' and it[8] <= 0)]
        leg = dict(stop=name, start=start_c, goal=goal_c, cmds=cmds, poses=poses, note='；'.join(notes),
                   min_gap_mm=round(min(g for g, _ in real), 1) if real else None,
                   yellow_mm=round(ygap, 1) if ygap < math.inf else None, time_s=round(res['time'], 2))
        return leg, 'OK'

    def _finish(self, items, start_piv, acts, node_piv, target_piv, gyaw):
        """A* 动作 → 指令；终点不在栅格上的差值并进最后一条同方向的移动；再按真实车身扫一遍检查。"""
        sh = heading_index(start_piv[2])
        raw, _ = to_commands(start_piv, acts, self.body.d, self.body.l)
        rx, ry = target_piv[0] - node_piv[0], target_piv[1] - node_piv[1]
        start_c = self.center_of(start_piv[0], start_piv[1], start_piv[2]) + (start_piv[2],)
        variants = _fold_variants([list(c) for c in raw], sh, rx, ry)
        best = None
        for cm in variants:
            cm = _clean(cm)
            ok, g, idx, what, poses, mins = _moves_eval(self.body, items, start_c, cm)
            tiny = sum(1 for c, v in cm if c in ('F', 'S') and abs(v) < MIN_MOVE_MM)
            t = self.motion.route_time(cm)
            r = dict(ok=ok, cmds=cm, poses=[(p[0], p[1], wrap180(p[2])) for p in poses], mins=mins,
                     tiny=tiny, time=t, what=what)
            if ok and tiny == 0:
                return r
            sc = (0 if ok else 1, tiny, t)
            if best is None or sc < best[0]:
                best = (sc, r)
        return best[1] if best else None


NO_ROUTE = '找不到路线（车道被障碍物堵死，或按现在的余量车身过不去/没有能转弯的位置）'


def _fold_variants(cmds, h0, rx, ry, limit=10):
    """终点差值 (rx, ry)(世界坐标，转轴点)并进路线里的移动，不另发小移动。
    先试并进最后一条沿 x / y 方向的移动，再试更早的同方向移动，最后才在末尾补一条。返回若干种指令列表。"""
    h = h0
    along = {0: [], 1: []}
    for idx, (k, v) in enumerate(cmds):
        if k == 'R':
            h = (h + int(round(v / 90.0))) % 4
            continue
        dw = DIRS[h] if k == 'F' else DIRS[(h + 1) % 4]
        ax = 0 if dw[0] else 1
        along[ax].append((idx, dw[ax]))
    hend = h
    opts = []
    for ax, res in ((0, rx), (1, ry)):
        if abs(res) < 0.5:
            opts.append([None])
        else:
            opts.append(list(reversed(along[ax])) + ['add'])
    combos = sorted(itertools.product(range(len(opts[0])), range(len(opts[1]))), key=lambda c: (c[0] + c[1], c))
    out = []
    for cx, cy in combos[:limit]:
        cm = [list(c) for c in cmds]
        add = []
        for ax, res, ch in ((0, rx, opts[0][cx]), (1, ry, opts[1][cy])):
            if ch is None:
                continue
            if ch == 'add':
                if DIRS[hend][ax]:
                    add.append(['F', res * DIRS[hend][ax]])
                else:
                    add.append(['S', res * DIRS[(hend + 1) % 4][ax]])
            else:
                idx, sg = ch
                cm[idx][1] += res * sg
        out.append(cm + add)
    return out


def _clean(cmds):
    """取整到 mm，去掉 0，合并相邻同类的平移。"""
    out = []
    for k, v in cmds:
        v = int(round(v))
        if v == 0:
            continue
        if out and out[-1][0] == k and k != 'R':
            nv = out[-1][1] + v
            out.pop()
            if nv != 0:
                out.append((k, nv))
            continue
        out.append((k, v))
    return out


def to_commands(start, acts, d=0.0, l=0.0):
    """把A*动作变成车体指令。start 是转轴点；返回的位姿是车身中心（用于画图）。
    d/l：车中心在转轴前 d、右 l(即转轴在车中心后 d、左 l)。"""
    x, y = float(start[0]), float(start[1]); h = heading_index(start[2])
    cmds, poses = [], []

    def center():
        fx, fy = DIRS[h]
        return (x + d*fx + l*fy, y + d*fy - l*fx, yaw_of(h))
    for a in acts:
        if a[0] == 'M':
            k = a[1]
            rel = (k - h) % 4
            kind, sign = {0: ('F', 1), 2: ('F', -1), 1: ('S', 1), 3: ('S', -1)}[rel]
            x += DIRS[k][0]*STEP; y += DIRS[k][1]*STEP
            if cmds and cmds[-1][0] == kind and np.sign(cmds[-1][1]) == sign:
                cmds[-1][1] += sign*STEP; poses[-1] = center()
            else:
                cmds.append([kind, sign*STEP]); poses.append(center())
        else:
            h2 = a[1]
            delta = {0: 0, 1: 90, 2: 180, 3: -90}[(h2 - h) % 4]
            h = h2
            if cmds and cmds[-1][0] == 'R':
                cmds[-1][1] = wrap180(cmds[-1][1] + delta)
                poses[-1] = center()
                if cmds[-1][1] == 0:
                    cmds.pop(); poses.pop()
            else:
                cmds.append(['R', delta]); poses.append(center())
    return [(c, int(v)) for c, v in cmds], poses


def check_moves(planner, pose, moves):
    """检查一串车体指令是否会压线/撞已知障碍(真实车身，转弯按角度扫)；返回(是否可行, 说明, 终点位姿)。"""
    for cmd, _ in moves:
        if cmd == 'S' and planner.mode != 'mecanum':
            return False, '差速车不能横移(S)', pose
    ok, g, idx, what, poses, _ = _moves_eval(planner.body, planner.items, pose, list(moves))
    end = poses[-1] if poses else pose
    if not ok:
        end = poses[idx - 1] if idx else pose
        return False, what, end
    return True, 'OK', end


def _plan_mission(cfg, obstacles, start_pose, stop_names, start_pivot=None, planner=None):
    """按停车点顺序逐段规划。start_pose/stops 都是车身中心 (x, y, 车头)。
    start_pivot 给定时直接用它当起点转轴点（例如第二站斜着停、原地转回正后的转轴位置）。
    返回 [dict(stop, start, goal, cmds, poses, note, ...)], 说明, planner。位置都换算回车身中心。
    每一段都从上一段的准确停车点出发、准确停在这一段的停车点上(差值并进同方向的移动，不另发小移动)。"""
    planner = planner or Planner(cfg, obstacles)
    stops = cfg['stops']
    legs = []
    h0 = heading_index(start_pose[2])
    p = start_pivot if start_pivot is not None else planner.pivot_of(*start_pose)
    pose = (float(p[0]), float(p[1]), yaw_of(h0))
    for name in stop_names:
        if name not in stops:
            raise ValueError(f'nav_config/配置 的 stops 里没有 {name}')
        gx, gy, gyaw = (float(v) for v in stops[name][:3])
        heading_index(gyaw)
        leg, why = planner.leg(pose, (gx, gy, gyaw), name)
        if leg is None:
            return legs, f'去 {name} 的路线失败：{why}', planner
        legs.append(leg)
        g = leg['goal']
        pv = planner.pivot_of(g[0], g[1], g[2])
        pose = (pv[0], pv[1], yaw_of(heading_index(g[2])))
    return legs, 'OK', planner


def plan_mission(cfg, obstacles, start_pose, stop_names, start_pivot=None):
    """规划全程。v11 按下面的顺序试，第一个规划出来的就用：
    1) 有障碍物的车道整段封死(障碍物离格子边不到 lane_block_tol_mm 时，旁边一格也封)；
    2) 只封障碍物所在的那一格；
    3) 车道不封死，只绕开障碍物本身(和 v10 一样)。
    1、2 先用车道格子查一遍还连不连得通，不通就直接跳过(省时间)。
    每一档都先按横移上限规划；最后一档规划不出来再放开横移上限。
    planner.fallback = 放开了横移上限；planner.lane_fallback = 没能封死车道。
    v17：黄色区不再整段“先 30 再 15 再 0”重来，而是一次规划里只在挤不过去的地方放宽(最低 yellow_min_margin_mm)。"""
    cap = cfg.get('strafe_max_mm_per_leg', None)
    lane = bool(cfg.get('lane_block', True))
    tol = float(cfg.get('lane_block_tol_mm', 80))
    stops = cfg.get('stops') or {}
    obstacles = [(float(x), float(y), float(r)) for x, y, r in obstacles]
    pts = [tuple(start_pose[:2])] + [tuple(float(v) for v in stops[n][:2]) for n in stop_names if n in stops]
    tries = []
    if lane and obstacles:
        seen = []
        for t in ((tol, 0.0) if tol > 0 else (0.0,)):
            sealed = lane_block_rects(dict(cfg, lane_block_tol_mm=t), obstacles)
            if sealed and sealed not in seen and lanes_connected(sealed, pts):
                seen.append(sealed)
                tries.append(dict(cfg, lane_block=True, lane_block_tol_mm=t))
    tries.append(dict(cfg, lane_block=False))
    if cap is not None:
        tries.append(dict(cfg, lane_block=False, strafe_max_mm_per_leg=None))
    first = None
    for c2 in tries:
        legs, why, planner = _plan_mission(c2, obstacles, start_pose, stop_names, start_pivot)
        if first is None:
            first = (legs, why, planner)
        if why == 'OK':
            planner.fallback = (cap is not None and c2.get('strafe_max_mm_per_leg', cap) is None)
            planner.lane_fallback = (lane and bool(obstacles) and not c2['lane_block'])
            return legs, why, planner
    legs, why, planner = first
    planner.fallback = False
    planner.lane_fallback = False
    return legs, why, planner


def plan_leg(cfg, obstacles, start_pose, goal, start_pivot=None):
    """单段规划(第一段重规划、回家)。start_pose：车中心 (x, y, 车头)，车头要横平竖直(差 3° 以内按最近的算)；
    start_pivot：给定时用它当起点转轴点(世界坐标 x, y)。goal：停车点名(在 cfg['stops'] 里)或车中心位姿 (x, y, 车头)。
    返回和 plan_mission 一样的一段 dict(多一个 'planner')；规划不出来抛 ValueError(原因)。"""
    stops = dict(cfg.get('stops') or {})
    if isinstance(goal, str):
        if goal not in stops:
            raise ValueError(f'配置的 stops 里没有 {goal}')
        name = goal
    else:
        name = 'GOAL'
        stops[name] = [float(goal[0]), float(goal[1]), float(goal[2])]
    yaw = float(start_pose[2])
    h = round(yaw / 90.0)
    if abs(wrap180(yaw - h*90)) > 3.0:
        raise ValueError(f'起点车头 {yaw:.1f}° 不是横平竖直的，先转正再规划')
    sp = (float(start_pose[0]), float(start_pose[1]), yaw_of(h % 4))
    legs, why, pl = plan_mission(dict(cfg, stops=stops), obstacles, sp, [name], start_pivot=start_pivot)
    if why != 'OK' or not legs:
        raise ValueError(why)
    leg = dict(legs[0])
    leg['planner'] = pl
    return leg


def route_stats(legs, cfg):
    """路线统计：前进/后退总mm、横移总mm、转弯次数、转弯累计度数、指令条数、预计用时(秒)。"""
    m = Motion(cfg)
    tot = dict(F=0, S=0, R=0, turns=0, n=0, time=0.0)
    for L in legs:
        for c, v in L['cmds']:
            tot[c] += abs(v)
            tot['n'] += 1
            if c == 'R':
                tot['turns'] += 1
            tot['time'] += m.time_of(c, v)
    return tot
