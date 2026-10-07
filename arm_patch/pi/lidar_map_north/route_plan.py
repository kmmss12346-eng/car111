#!/usr/bin/env python3
"""全流程路线规划（25mm栅格A*，按“用时”算代价）。只算路线和指令，不直接驱动电机。

坐标：场地左下角原点，X向右，Y向上，单位mm。
航向：东0°，北90°，西180°，南-90°；逆时针为正。
车体指令：F=前进(负数后退)，S=横移(正数向左，负数向右，麦轮才有)，R=原地转(正数逆时针)。

规则约束（2027智能搬运）：
- 离开启停区后车身铅垂投影只能在灰色车道上：不能压黄色区、暂存区、粗加工区、原料转盘，不能出场地。
- 平移时车身按 max(长,宽) 的正方形检查；
- 原地转弯时车身扫过一个圆，只在转得开的地方转。

v10 和之前的不同：
1. 现在 STM32 的 R 指令是绕“车中心”转(转的同时叠加平移，车中心基本不动)，所以 turn_pivot_back_mm 默认 0。
   (转轴参数还保留：以后换了车、转轴又不在中心时照样能用。)
2. A* 的代价从“毫米数”改成“预计用时(秒)”：直行、横移各有自己的速度和加减速；每多一次起停、每多转一次弯，都按
   实测的时间加上。横移现在又快又稳，所以不再限制横移长度，规划器会自己权衡“多转弯”和“横着走”哪个更省时间。
3. 离黄色区、墙、障碍物越近代价越高：同样的用时，规划器会选更宽的车道、走在车道中间。
   (车靠陀螺仪和电机脉冲航位推算，没有别的位置传感器，留得越宽，累积误差越不容易压线。)
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

# 车身离黄色区至少留多少 mm(硬性，规划不会再贴着黄色区走)。配置 yellow_margin_mm 可改。
# 某一段按这个余量规划不出来(比如中间那条只有 400mm 宽的车道)，这一段会自动减半再试，还不行才按 0 规划，并给出提示。
YELLOW_MARGIN_MM = 30

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
    """STM32 实际的运动快慢，用来估每条指令要多少秒。数值都能在配置里改(键名见下面)。"""
    def __init__(self, cfg):
        g = cfg.get
        self.v_f = float(g('route_speed_rpm', 220)) / 60.0 * float(g('fwd_mm_per_rev', 249.0))        # 直行最高速 mm/s
        self.a_f = float(g('route_acc_rpm_s', 300)) / 60.0 * float(g('fwd_mm_per_rev', 249.0))        # 直行加减速 mm/s²
        self.v_s = float(g('strafe_speed_rpm', 170)) / 60.0 * float(g('strafe_mm_per_rev', 235.0))    # 横移最高速
        self.a_s = float(g('strafe_acc_rpm_s', 200)) / 60.0 * float(g('strafe_mm_per_rev', 235.0))    # 横移加减速
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


class Planner:
    def __init__(self, cfg, obstacles):
        """obstacles: [(x, y, r), ...] 扫描得到的障碍物。栅格节点是“转轴点”的位置(转轴在车中心时就是车中心)。"""
        L = float(cfg.get('car_length_mm', 290)); W = float(cfg.get('car_width_mm', 260))
        B = float(cfg.get('plan_body_mm', 300) or 0)   # v11：规划时把车当成 B×B 的正方形(默认300，比真车大，留冗余)
        self.half = max(L, W, B)/2
        # v13：车左边装着雷达，比车身左侧突出 lidar_overhang_mm。车身外框(车身坐标 x朝车头、y朝车左)：
        #   前后各 max(L,B)/2；右边 max(W,B)/2；左边 = 车身半宽 + 雷达突出 + 和右边一样的冗余，至少 max(W,B)/2
        ov = float(cfg.get('lidar_overhang_mm', 45))
        spare = max(0.0, (B - W)/2)
        self.box = (-max(L, B)/2, max(L, B)/2, -max(W, B)/2, max(max(W, B)/2, W/2 + ov + spare))
        self.ext = [self._ext(h) for h in range(4)]
        self.d = float(cfg.get('turn_pivot_back_mm', 0))
        self.turn_r = math.hypot(L/2 + abs(self.d), W/2) + float(cfg.get('turn_margin_mm', 20))
        self.obs_margin = float(cfg.get('margin_mm', 20))
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
        rc = cfg.get('raw_center_mm', [1200, 2480]); self.raw = (float(rc[0]), float(rc[1]), float(cfg.get('raw_radius_mm', 150)))
        self.obstacles = [(float(x), float(y), float(r)) for x, y, r in obstacles]
        self.sealed = lane_block_rects(cfg, self.obstacles) if cfg.get('lane_block', True) else []
        self.fixed_base = YELLOW + [TEMP_ZONE, ROUGH_ZONE] + [tuple(r) for r in (cfg.get('extra_blocked_rects') or [])]
        self.fixed = self.fixed_base + [tuple(r) for r in self.sealed]
        # 黄色区外扩 yellow_margin_mm 以后再判断车身放不放得下(只影响"能不能走"，净空代价仍按真实黄色区算)
        ym = max(0.0, float(cfg.get('yellow_margin_mm', YELLOW_MARGIN_MM)))
        self.yellow_margin = ym
        self.yellow_big = [(r[0]-ym, r[1]-ym, r[2]+ym, r[3]+ym) for r in YELLOW]
        self.fixed_free = self.yellow_big + [r for r in self.fixed if r not in YELLOW]
        xs = np.arange(N)*STEP
        X, Y = np.meshgrid(xs, xs, indexing='ij')
        # 4个车头方向各一张“车身放得下”的表：车身中心 = 转轴点 + d×车头方向
        self.free = np.stack([self._center_free(X + self.d*DIRS[h][0], Y + self.d*DIRS[h][1], h) for h in range(4)])
        clear = np.stack([self._center_clear(X + self.d*DIRS[h][0], Y + self.d*DIRS[h][1], h) for h in range(4)])
        pref = max(self.clr_pref, 1.0)
        self.mult = 1.0 + self.clr_pen*np.clip((pref - clear)/pref, 0.0, 1.0)
        R = self.turn_r
        E = R - float(cfg.get('turn_edge_allow_mm', 0))   # 转弯时车角允许超出场地边线多少mm，默认0
        cx, cy, cr = self.raw
        turn = (X >= E) & (X <= FIELD-E) & (Y >= E) & (Y <= FIELD-E)
        for r in self.fixed_base:      # 封死的车道只管“不能开进去”；在旁边路口里原地转、车角稍微扫过路段口没关系(障碍物本身下面单独检查)
            turn &= _rect_point_dist(X, Y, r) >= R
        turn &= np.hypot(X-cx, Y-cy) >= R + cr
        for ox, oy, orr in self.obstacles:
            turn &= np.hypot(X-ox, Y-oy) >= R + orr + self.obs_margin
        self.turn = turn
        self.X, self.Y = X, Y
        self.fallback = False

    def _ext(self, h):
        """车头方向 h 时，车身外框相对车中心往 西、东、南、北 各伸出多少 mm。"""
        xb, xf, yr, yl = self.box
        fx, fy = DIRS[h]
        lx, ly = -fy, fx                                   # 车左边的方向
        xs = [a*fx + b*lx for a in (xb, xf) for b in (yr, yl)]
        ys = [a*fy + b*ly for a in (xb, xf) for b in (yr, yl)]
        return -min(xs), max(xs), -min(ys), max(ys)

    def _center_free(self, X, Y, h):
        w, e, so, no = self.ext[h]
        free = (X - w >= 0) & (X + e <= FIELD) & (Y - so >= 0) & (Y + no <= FIELD)
        for r in self.fixed_free:                  # 与固定区域的开区间相交即禁止（刚好贴边允许）；黄色区已外扩 yellow_margin_mm
            free &= ~((X + e > r[0]) & (X - w < r[2]) & (Y + no > r[1]) & (Y - so < r[3]))
        cx, cy, cr = self.raw                      # 原料转盘（圆）与车身外框
        dx = np.maximum(np.maximum((X - w) - cx, cx - (X + e)), 0)
        dy = np.maximum(np.maximum((Y - so) - cy, cy - (Y + no)), 0)
        free &= np.hypot(dx, dy) >= cr
        for ox, oy, orr in self.obstacles:        # 障碍物按外接正方形再加余量，保守
            m = orr + self.obs_margin
            free &= ~((X + e > ox - m) & (X - w < ox + m) & (Y + no > oy - m) & (Y - so < oy + m))
        return free

    def _center_clear(self, X, Y, h):
        """车身外框到最近的黄色区/暂存区/粗加工区/转盘/墙/障碍物的空隙(mm)，越大越安全。"""
        w, e, so, no = self.ext[h]
        c = np.minimum(np.minimum(X - w, FIELD - X - e), np.minimum(Y - so, FIELD - Y - no))
        for r in self.fixed:
            gx = np.maximum(np.maximum(r[0] - (X + e), (X - w) - r[2]), 0)
            gy = np.maximum(np.maximum(r[1] - (Y + no), (Y - so) - r[3]), 0)
            c = np.minimum(c, np.hypot(gx, gy))
        cx, cy, cr = self.raw
        c = np.minimum(c, np.hypot(np.maximum(np.maximum((X - w) - cx, cx - (X + e)), 0),
                                   np.maximum(np.maximum((Y - so) - cy, cy - (Y + no)), 0)) - cr)
        for ox, oy, orr in self.obstacles:
            c = np.minimum(c, np.hypot(np.maximum(np.maximum((X - w) - ox, ox - (X + e)), 0),
                                       np.maximum(np.maximum((Y - so) - oy, oy - (Y + no)), 0)) - orr)
        return np.maximum(c, 0.0)

    def pivot_of(self, x, y, yaw):
        a = math.radians(yaw)
        return x - self.d*math.cos(a), y - self.d*math.sin(a)

    def center_of(self, px, py, yaw):
        a = math.radians(yaw)
        return px + self.d*math.cos(a), py + self.d*math.sin(a)

    def point_free(self, px, py, h):
        """任意位置(转轴点 px,py，车头方向索引 h)车身放得下吗(不要求在栅格节点上)。"""
        cx, cy = px + self.d*DIRS[h][0], py + self.d*DIRS[h][1]
        return bool(self._center_free(np.array([[cx]], float), np.array([[cy]], float), h)[0, 0])

    def snap(self, px, py, h):
        """转轴点吸附到最近的、车身放得下的25mm节点。"""
        i0, j0 = int(math.floor(px/STEP)), int(math.floor(py/STEP))
        cands = sorted(((i, j) for i in range(i0-1, i0+3) for j in range(j0-1, j0+3) if 0 <= i < N and 0 <= j < N),
                       key=lambda ij: math.hypot(ij[0]*STEP-px, ij[1]*STEP-py))
        for i, j in cands:
            if self.free[h, i, j]:
                return i*STEP, j*STEP
        i, j = cands[0]
        return i*STEP, j*STEP

    # ---------- 查询 ----------
    def node(self, x, y):
        i, j = x/STEP, y/STEP
        if abs(i-round(i)) > 1e-6 or abs(j-round(j)) > 1e-6:
            raise ValueError(f'({x},{y}) 不在25mm节点上，请把停车点改成25的倍数')
        i, j = int(round(i)), int(round(j))
        if not (0 <= i < N and 0 <= j < N):
            raise ValueError(f'({x},{y}) 超出场地')
        return i, j

    def why_blocked(self, x, y, h=1):
        w, e, so, no = self.ext[h]
        if not (x - w >= 0 and x + e <= FIELD and y - so >= 0 and y + no <= FIELD):
            return '车身(含雷达)会出场地'
        for r in self.fixed_free:
            if x + e > r[0] and x - w < r[2] and y + no > r[1] and y - so < r[3]:
                if r in self.yellow_big and self.yellow_margin > 0:
                    return f'车身(含雷达)离黄色区不到 {self.yellow_margin:.0f}mm'
                return (f'这段车道有障碍物，已封死{r}' if r in self.sealed else f'车身(含雷达)会压到固定区域{r}')
        for ox, oy, orr in self.obstacles:
            m = orr + self.obs_margin
            if x + e > ox - m and x - w < ox + m and y + no > oy - m and y - so < oy + m:
                return f'离障碍物({ox:.0f},{oy:.0f})太近'
        return '车身压到原料转盘'

    # ---------- 单段A* ----------
    def plan_leg(self, start, goal):
        """start/goal: 转轴点 (x, y, yaw)，在25mm节点上。返回 (动作列表, 说明)。"""
        si, sj = self.node(start[0], start[1]); gi, gj = self.node(goal[0], goal[1])
        sh, gh = heading_index(start[2]), heading_index(goal[2])
        if not self.free[sh, si, sj]:
            c = self.center_of(start[0], start[1], yaw_of(sh))
            return None, '起点受阻：' + self.why_blocked(c[0], c[1], sh)
        if not self.free[gh, gi, gj]:
            c = self.center_of(goal[0], goal[1], yaw_of(gh))
            return None, '终点受阻：' + self.why_blocked(c[0], c[1], gh)
        mec = self.mode == 'mecanum'
        s0 = (si, sj, 4, sh, 0)                      # (i, j, 上一步方向, 车头, 这段路已横移的格数)
        cost = {s0: 0.0}; parent = {}
        order = itertools.count()
        h_step = min(self.step_f, self.step_s / self.sf)          # 每格最快用时(可采纳的启发值)
        hfun = lambda i, j, h: (abs(i-gi)+abs(j-gj))*h_step + (0 if h == gh else self.turn1)
        q = [(hfun(si, sj, sh), next(order), 0.0, s0)]
        free, mult, turn = self.free, self.mult, self.turn
        while q:
            _, _, g, s = heapq.heappop(q)
            if g != cost.get(s):
                continue
            i, j, d, h, r = s
            if i == gi and j == gj and h == gh:
                acts = []
                while s != s0:
                    s, a = parent[s]
                    acts.append(a)
                return acts[::-1], 'OK'
            for k, (di, dj) in enumerate(DIRS):
                if not mec and k % 2 != h % 2:        # 差速车只能沿车头方向前进/后退
                    continue
                ni, nj = i+di, j+dj
                if not (0 <= ni < N and 0 <= nj < N) or not free[h, ni, nj]:
                    continue
                lateral = (k - h) % 2 == 1                   # 相对车头是左右方向 = 横移
                if lateral and self.strafe_cap is not None and r + 1 > self.strafe_cap:
                    continue                                  # 这一段的横移额度用完了
                ng = g + (self.step_s if lateral else self.step_f) * mult[h, ni, nj]
                if d != k:                                    # 新开一段直行/横移(从静止起步、或掉头)
                    ng += self.seg_s if lateral else self.seg_f
                t = (ni, nj, k, h, r + (1 if (lateral and self.strafe_cap is not None) else 0))
                if ng < cost.get(t, math.inf):
                    cost[t] = ng; parent[t] = (s, ('M', k))
                    heapq.heappush(q, (ng+hfun(ni, nj, h), next(order), ng, t))
            if turn[i, j]:
                for h2 in range(4):
                    if h2 == h or not free[h2, i, j]:
                        continue
                    steps = min((h2-h) % 4, (h-h2) % 4)
                    ng = g + (self.turn1 if steps == 1 else self.turn2)
                    t = (i, j, 4, h2, r)
                    if ng < cost.get(t, math.inf):
                        cost[t] = ng; parent[t] = (s, ('T', h2))
                        heapq.heappush(q, (ng+hfun(i, j, h2), next(order), ng, t))
        return None, '找不到路线（可能被障碍物堵死，或没有能转弯的位置）'


def to_commands(start, acts, d=0.0):
    """把A*动作变成车体指令。start 是转轴点；返回的位姿是车身中心（用于画图）。"""
    x, y = float(start[0]), float(start[1]); h = heading_index(start[2])
    cmds, poses = [], []
    def center():
        return (x + d*DIRS[h][0], y + d*DIRS[h][1], yaw_of(h))
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


def check_moves(planner, pose, moves):
    """检查一串车体指令是否会压线/撞已知障碍；返回(是否可行, 说明, 终点位姿)。"""
    for cmd, val in moves:
        if cmd == 'R':
            i, j = planner.node(round(pose[0]), round(pose[1]))
            if not planner.turn[i, j]:
                return False, f'在({pose[0]:.0f},{pose[1]:.0f})原地转会扫出车道或碰障碍', pose
            pose = apply_body_move(pose, cmd, val)
            continue
        if cmd == 'S' and planner.mode != 'mecanum':
            return False, '差速车不能横移(S)', pose
        n = int(abs(val)//STEP)
        for _ in range(n):
            pose = apply_body_move(pose, cmd, STEP if val > 0 else -STEP)
            i, j = planner.node(round(pose[0]), round(pose[1]))
            if not planner.free[heading_index(round(pose[2]/90)*90), i, j]:
                return False, f'移动到({pose[0]:.0f},{pose[1]:.0f})时{planner.why_blocked(pose[0], pose[1], heading_index(round(pose[2]/90)*90))}', pose
        rest = abs(val) - n*STEP
        if rest:
            pose = apply_body_move(pose, cmd, rest if val > 0 else -rest)
    return True, 'OK', pose


def _plan_mission(cfg, obstacles, start_pose, stop_names, start_pivot=None):
    """按停车点顺序逐段规划。start_pose/stops 都是车身中心 (x, y, 车头)。
    start_pivot 给定时直接用它当起点转轴点（例如第二站斜着停、原地转回正后的转轴位置）。
    返回 [dict(stop, start, goal, cmds, poses)], 说明, planner。位置都换算回车身中心。"""
    # 每一段先按 yellow_margin_mm 规划，不行再减半、再按 0(只有真走不通的那一段放宽，别的段不受影响)
    ym = max(0.0, float(cfg.get('yellow_margin_mm', YELLOW_MARGIN_MM)))
    margins = sorted({ym, round(ym / 2), 0.0}, reverse=True)
    planners = [Planner(dict(cfg, yellow_margin_mm=m), obstacles) for m in margins]
    planner = planners[-1]                             # 余量 0 的那个：吸附栅格、停车点检查和以前完全一样
    d = planner.d
    stops = cfg['stops']
    legs = []
    def body_moves(dx, dy, h):
        """世界坐标里的小位移 → 车头方向h下的前后(F)、左右(S)指令(mm取整)。"""
        u = DIRS[h]; f = dx*u[0] + dy*u[1]; l = -dx*u[1] + dy*u[0]
        return [(k, int(round(v))) for k, v in (('F', f), ('S', l)) if abs(v) >= 1]

    def merge(cmds):
        out = []
        for k, v in cmds:
            if out and out[-1][0] == k and k != 'R':
                out[-1] = (k, out[-1][1] + v)
                if out[-1][1] == 0:
                    out.pop()
            elif v != 0:
                out.append((k, v))
        return out

    h0 = heading_index(start_pose[2])
    p = start_pivot if start_pivot is not None else planner.pivot_of(*start_pose)
    sx, sy = planner.snap(p[0], p[1], h0)
    pre = body_moves(sx - p[0], sy - p[1], h0)       # 实际起点 → 栅格点
    pose = (sx, sy, yaw_of(h0))
    for name in stop_names:
        if name not in stops:
            raise ValueError(f'nav_config/配置 的 stops 里没有 {name}')
        gx, gy, gyaw = (float(v) for v in stops[name])
        gh = heading_index(gyaw)
        gp = planner.pivot_of(gx, gy, gyaw)
        gpx, gpy = planner.snap(gp[0], gp[1], gh)
        goal = (gpx, gpy, yaw_of(gh))
        acts, why, used = None, '', 0.0
        for pl in planners:
            acts, w2 = pl.plan_leg(pose, goal)
            if acts is not None:
                used = pl.yellow_margin
                break
            why = why or w2                            # 记下最严格那一档失败的原因
        if acts is None:
            return legs, f'去 {name} 的路线失败：{w2}', planner
        cmds, poses = to_commands(pose, acts, d)
        note = ''
        if used < ym:
            note = (f'⚠ 去 {name} 这一段按离黄色区 {ym:.0f}mm 规划不出来(车道太窄或停车点离黄色区太近)，'
                    f'这一段只留了 {used:.0f}mm，走这段要特别注意。({why})')
        if planner.point_free(gp[0], gp[1], gh):
            post = body_moves(gp[0] - gpx, gp[1] - gpy, gh)   # 栅格点 → 准确的停车点
            gcenter = (gx, gy, gyaw)
        else:
            # 准确的停车点已经被障碍物占了：不往里走，停在最近的放得下的栅格点上
            post = []
            gcenter = planner.center_of(gpx, gpy, yaw_of(gh)) + (gyaw,)
            gcenter = (gcenter[0], gcenter[1], gyaw)
            note = (note + '；' if note else '') + f'⚠ {name} 的准确停车点({gx:.0f},{gy:.0f})被障碍物挡住，改停在 ({gcenter[0]:.0f},{gcenter[1]:.0f})，差 {math.hypot(gcenter[0]-gx, gcenter[1]-gy):.0f}mm'
        cmds = merge(pre + cmds + post)
        pre = [(k, -v) for k, v in post]                   # 下一段先从准确停车点回到栅格点
        legs.append(dict(stop=name, start=planner.center_of(*pose), goal=gcenter,
                         cmds=cmds, poses=poses + [gcenter], note=note))
        pose = goal
    return legs, 'OK', planner


def plan_mission(cfg, obstacles, start_pose, stop_names, start_pivot=None):
    """规划全程。v11 按下面的顺序试，第一个规划出来的就用：
    1) 有障碍物的车道整段封死(障碍物离格子边不到 lane_block_tol_mm 时，旁边一格也封)；
    2) 只封障碍物所在的那一格；
    3) 车道不封死，只绕开障碍物本身(和 v10 一样)。
    1、2 先用车道格子查一遍还连不连得通，不通就直接跳过(省时间)。
    每一档都先按横移上限规划；最后一档规划不出来再放开横移上限。
    planner.fallback = 放开了横移上限；planner.lane_fallback = 没能封死车道。"""
    cap = cfg.get('strafe_max_mm_per_leg', None)
    lane = bool(cfg.get('lane_block', True))
    tol = float(cfg.get('lane_block_tol_mm', 80))
    stops = cfg.get('stops') or {}
    pts = [tuple(start_pose[:2])] + [tuple(float(v) for v in stops[n][:2]) for n in stop_names if n in stops]
    tries = []
    if lane:
        for t in ((tol, 0.0) if tol > 0 else (0.0,)):
            sealed = lane_block_rects(dict(cfg, lane_block_tol_mm=t), [(float(x), float(y), float(r)) for x, y, r in obstacles])
            if lanes_connected(sealed, pts):
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
            planner.lane_fallback = (lane and not c2['lane_block'])
            return legs, why, planner
    legs, why, planner = first
    planner.fallback = False
    planner.lane_fallback = False
    return legs, why, planner


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
