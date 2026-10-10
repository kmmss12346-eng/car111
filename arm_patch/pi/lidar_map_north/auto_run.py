"""一键流程：scan -> STM32 走第二站 -> second -> 规划 -> 按路线驱动。
v14：到 QR 再扫一次障碍物；回启停区前用雷达定位一次。
1010：每个停车点先用雷达重定位(reloc.py)再修正；小于 20mm 的移动不再丢掉(带到下一条同方向的移动里)；
      时间不够从当前停车点直接回家；回家前定位失败不再照原路线乱走；启停区 1/2 切换；屏幕选区一键启动(race)；
      车长时间不动时手臂轻摆；退出时升降停 60mm。
ctx 由 map_merge_live.py 提供（在后台线程里调用）。"""
import copy
import math
import re
import threading
import time

import route_plan as _rp
from route_plan import Motion
from stm32_link import parse_done


class Abort(Exception):
    pass


MIN_MOVE_MM = 20     # 路线中间小于这个距离的前进/横移不单独发(车本身的精度就在这个量级，发了反而多一次起停)，带到下一条同方向的移动里
STOP_MIN_MM = 5      # 到停车点时还差这么多以上就补一条慢速小移动，保证车真的停在停车点上；更小的带到下一段
FIELD_MM = 2400.0

DEFAULT_SECOND_MOVES = {'left_mm': 42, 'forward_mm': 83, 'cw_deg': 60}

MAX_HEAD_ERR_DEG = 10.0   # 指令做完车头还差这么多度：车没按指令动(正常在 2° 以内)，再往下走只会越错越远
STALL_HINT = ('车轮没转起来(STM32 发了转弯指令，陀螺仪几乎没变；前进/横移的 DONE 是按时间算的，不代表车真的走了)。'
              '先输入 mot 看 5 个电机驱动器的电压/使能/堵转保护；再把车抬起来输入 send R 90 看轮子转不转。'
              '常见原因：电机电池没电或没开、驱动器触发了堵转保护、新接的升降驱动器地址或串口线接错')

# 不能压的区域(和 route_plan 一样；route_plan 是旧版本时用这里的)
YELLOW = [tuple(r) for r in getattr(_rp, 'YELLOW', [(550, 1400, 1000, 1850), (1400, 1400, 1850, 1850),
                                                    (550, 550, 1000, 1000), (1400, 550, 1850, 1000)])]
TEMP_ZONE = tuple(getattr(_rp, 'TEMP_ZONE', (0, 910, 150, 1490)))
ROUGH_ZONE = tuple(getattr(_rp, 'ROUGH_ZONE', (910, 0, 1490, 150)))
START_RECTS = {1: (2100, 2100, 2400, 2400), 2: (2100, 0, 2400, 300)}


def check_move(cmd, val, ok, reply):
    """检查一条运动指令的回复：没问题返回 None，否则返回一句给人看的原因。"""
    reply = reply or ''
    if not ok:
        return f'{cmd} {val} 失败：{reply}' + (f'。{STALL_HINT}' if 'STALL' in reply else '')
    err = parse_done(reply).get('err')
    if err is not None and abs(err) > MAX_HEAD_ERR_DEG:
        return f'{cmd} {val} 做完后车头还差 {err:+.1f}°(正常在 2° 以内)。{STALL_HINT}'
    return None


def wrap180(a):
    """角度换到 (-180, 180]。"""
    a = (float(a) + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def _axis(yaw, cmd):
    """车头 yaw 时，F(前进)/S(左移) 在世界坐标里的单位方向。"""
    a = math.radians(yaw)
    if cmd == 'F':
        return math.cos(a), math.sin(a)
    return -math.sin(a), math.cos(a)


def apply_move(pose, cmd, val):
    """按车体指令推算新位姿(绕车中心转，和规划一致)。"""
    x, y, yaw = (float(v) for v in pose[:3])
    if cmd == 'R':
        return (x, y, wrap180(yaw + float(val)))
    ux, uy = _axis(yaw, cmd)
    return (x + float(val)*ux, y + float(val)*uy, yaw)


def _simulate(pose, cmds):
    """按路线指令推算每一步之后的车位。F 沿车头、S 向车左、R 绕车中心转(和规划一致)。"""
    out = []
    for c, v in cmds:
        pose = apply_move(pose, c, v)
        out.append(pose)
    return out


def _fmt(v):
    return f'{v:+d}' if isinstance(v, int) else f'{v:+.1f}'


# ================================================================ 路线 -> 指令序列
def flatten(turn_back, legs, min_move=MIN_MOVE_MM, stop_min=STOP_MIN_MM):
    """路线 -> [(停车点名或None, 指令, 数值)]。第一步先原地转回起点车头方向。
    小于 min_move 的前进/横移不单独发，按世界方向记下来，加到下一条同方向的移动里(不丢)；
    到停车点时还差 stop_min 以上就补一条小移动(车真的停在停车点上)，更小的记在停车点标记里(rest)，由 drive 带到下一段。
    停车点标记：(名字, None, {'goal': 停车点位姿或None, 'rest': {'F': mm, 'S': mm}})。"""
    seq = []
    if turn_back is not None and abs(turn_back) >= 0.05:
        tb = round(float(turn_back), 1)
        seq.append((None, 'R', int(tb) if tb == int(tb) else tb))
    h = 0.0                                     # 相对出发时的车头方向(度)
    for L in legs:
        cx = cy = 0.0                           # 这一段里还欠着没走的位移(相对坐标)
        last = None
        for cmd, val in L['cmds']:
            if cmd == 'R':
                seq.append((None, 'R', val))
                h += float(val)
                continue
            if cmd not in ('F', 'S'):
                seq.append((None, cmd, val))
                continue
            ux, uy = _axis(h, cmd)
            c = cx*ux + cy*uy
            tot = float(val) + c
            cx -= c*ux
            cy -= c*uy
            if abs(tot) < min_move:
                cx += tot*ux
                cy += tot*uy
                continue
            v = int(round(tot))
            cx += (tot - v)*ux
            cy += (tot - v)*uy
            seq.append((None, cmd, v))
            last = cmd
        rest = {}
        for cmd in ([last] if last else []) + [q for q in ('F', 'S') if q != last]:
            ux, uy = _axis(h, cmd)
            c = cx*ux + cy*uy
            if abs(c) >= stop_min:
                v = int(round(c))
                seq.append((None, cmd, v))      # 到停车点前补一条小移动
                cx -= v*ux
                cy -= v*uy
                c -= v
            rest[cmd] = c
        goal = tuple(float(v) for v in L['goal'][:3]) if L.get('goal') is not None else None
        seq.append((L['stop'], None, dict(goal=goal, rest=rest)))        # 到达停车点的标记
    return seq


def summarize(seq):
    tot = {'F': 0, 'S': 0, 'R': 0}
    n = 0
    for _, c, v in seq:
        if c:
            tot[c] += abs(v)
            n += 1
    return n, tot


def move_speeds(cfg):
    """路线里 F、S 用的最高速(转/分)，R 不用。和 STM32 里的 ROUTE_SPEED / ROUTE_STRAFE_SPEED 一致(默认 220 / 170)。"""
    cfg = cfg or {}
    return {'F': int(cfg.get('route_speed_rpm', 220)), 'S': int(cfg.get('strafe_speed_rpm', 170))}


def _marker_info(val):
    return val if isinstance(val, dict) else {}


def _marker_goal(seq, i, stops=None):
    g = _marker_info(seq[i][2]).get('goal')
    if g is not None:
        return tuple(float(v) for v in g[:3])
    stops = stops or {}
    if seq[i][0] in stops:
        return tuple(float(v) for v in stops[seq[i][0]][:3])
    return None


def has_prehome(seq):
    """回启停区前有 PREHOME 停车点：最后一次定位就在离启停区 ~250mm 的直线上，不用 home_cut。"""
    marks = [i for i, (s, c, _) in enumerate(seq) if c is None]
    fin = [i for i in marks if str(seq[i][0]).upper().startswith('START')]
    if not fin:
        return False
    prev = [i for i in marks if i < fin[-1]]
    return bool(prev) and str(seq[prev[-1]][0]).upper().startswith('PREHOME')


def home_cut(seq, cfg):
    """v14：在最后回启停区那一段里，找“离启停区还剩 home_approach_mm 左右、车头已经转好”的地方。
    车走到这里就停下，用雷达定位，再按实际位置走进启停区，不再照原路线最后几步走。
    必要时把一条长的前进/后退拆成两截。返回 (新的 seq, 信息) 或 (seq, None)。
    1010：前一个停车点的位置用路线里实际的停车点(被障碍物挡住改停别处时不一样)。"""
    stops = (cfg or {}).get('stops') or {}
    marks = [i for i, (s, c, _) in enumerate(seq) if c is None]
    fin = [i for i in marks if str(seq[i][0]).upper().startswith('START')]
    if not fin:
        return seq, None
    m = fin[-1]
    prev = [i for i in marks if i < m]
    if not prev:
        return seq, None
    p0 = _marker_goal(seq, prev[-1], stops)
    goal = _marker_goal(seq, m, stops)
    if p0 is None or goal is None:
        return seq, None
    idx = list(range(prev[-1] + 1, m))
    cmds = [(seq[i][1], seq[i][2]) for i in idx]
    poses = _simulate(p0, cmds)
    lastr = max([k for k, (c, _) in enumerate(cmds) if c == 'R'], default=-1)
    A = float((cfg or {}).get('home_approach_mm', 800))
    dist = lambda p: math.hypot(goal[0] - p[0], goal[1] - p[1])
    pose = poses[lastr] if lastr >= 0 else p0
    k = lastr + 1                       # 从这条开始都是平移
    seq = list(seq)
    while k < len(cmds) and dist(pose) > A:
        c, v = cmds[k]
        nxt = poses[k]
        if dist(nxt) >= A - 1:          # 整条走完还在范围外：照走
            pose = nxt; k += 1; continue
        # 这一条走到一半就进范围：拆成两截，前一截照走
        u = _axis(pose[2], c)
        sg = 1 if v > 0 else -1
        lo, hi = 0.0, float(abs(v))
        for _ in range(40):
            mid = (lo + hi) / 2
            q = (pose[0] + sg*mid*u[0], pose[1] + sg*mid*u[1])
            if math.hypot(goal[0]-q[0], goal[1]-q[1]) > A: lo = mid
            else: hi = mid
        s1 = int(round(lo))
        if s1 >= 20:
            j = idx[k]
            seq[j:j+1] = [(None, c, sg*s1), (None, c, v - sg*s1)]
            pose = (pose[0] + sg*s1*u[0], pose[1] + sg*s1*u[1], pose[2])
            return _cut_info(seq, j + 1, pose, goal)
        break
    if k >= len(cmds):
        return seq, None                # 最后一步才转弯，没有合适的地方停下来定位
    return _cut_info(seq, idx[k], pose, goal)


def _cut_info(seq, j, pose, goal):
    """j = 从这一条开始不照原路线走，改成“定位后按实际位置走”。"""
    rest = []
    k = j
    while k < len(seq) and seq[k][1] is not None:
        rest.append(seq[k]); k += 1
    order = []
    for _, c, _ in rest:
        if c in ('F', 'S') and c not in order:
            order.append(c)
    mx = max([abs(sum(v for _, c, v in rest if c == q)) for q in ('F', 'S')] + [0])
    return seq, dict(index=j, marker=k, pose=pose, goal=goal, order=tuple(order) or ('S', 'F'), max_move=mx + 250)


# ================================================================ 车身几何(真实车身 + 雷达圆盘)
def body_model(cfg):
    """真实车身：car_length × car_width 的矩形，加左边的雷达圆盘(中心在 lidar_forward/lidar_left，比车身左边突出 lidar_overhang_mm)。"""
    g = (cfg or {}).get
    L = float(g('car_length_mm', 290))
    W = float(g('car_width_mm', 260))
    ov = float(g('lidar_overhang_mm', 40))
    lf = float(g('lidar_forward_mm', 22.5))
    ll = float(g('lidar_left_mm', 134.8))
    r = W/2 + ov - ll
    disk = (lf, ll, r) if r > 0 else None
    return dict(L=L, W=W, disk=disk)


def body_poly(pose, model):
    x, y, yaw = (float(v) for v in pose[:3])
    a = math.radians(yaw)
    c, s = math.cos(a), math.sin(a)
    hl, hw = model['L']/2, model['W']/2
    pts = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    poly = [(x + px*c - py*s, y + px*s + py*c) for px, py in pts]
    disk = None
    if model.get('disk'):
        f, l, r = model['disk']
        disk = (x + f*c - l*s, y + f*s + l*c, r)
    return poly, disk


def _pt_seg(px, py, a, b):
    ex, ey = b[0] - a[0], b[1] - a[1]
    ll = ex*ex + ey*ey
    t = 0.0 if ll <= 1e-12 else max(0.0, min(1.0, ((px - a[0])*ex + (py - a[1])*ey) / ll))
    return math.hypot(px - a[0] - t*ex, py - a[1] - t*ey)


def _poly_dist(A, B):
    """两个凸多边形的距离；相交时返回负数(最小穿透深度)。"""
    min_ov = math.inf
    for P in (A, B):
        n = len(P)
        for i in range(n):
            ex, ey = P[(i+1) % n][0] - P[i][0], P[(i+1) % n][1] - P[i][1]
            ln = math.hypot(ex, ey)
            if ln <= 1e-12:
                continue
            nx, ny = -ey/ln, ex/ln
            pa = [x*nx + y*ny for x, y in A]
            pb = [x*nx + y*ny for x, y in B]
            ov = min(max(pa), max(pb)) - max(min(pa), min(pb))
            if ov <= 0:
                d = math.inf
                for P1, Q in ((A, B), (B, A)):
                    m = len(Q)
                    for px, py in P1:
                        for j in range(m):
                            d = min(d, _pt_seg(px, py, Q[j], Q[(j+1) % m]))
                return d
            min_ov = min(min_ov, ov)
    return -min_ov


def _poly_circle(P, cx, cy, r):
    n = len(P)
    d = min(_pt_seg(cx, cy, P[i], P[(i+1) % n]) for i in range(n))
    inside = True
    for i in range(n):
        ax, ay = P[i]
        bx, by = P[(i+1) % n]
        if (bx - ax)*(cy - ay) - (by - ay)*(cx - ax) < 0:
            inside = False
            break
    return (-d if inside else d) - r


def _rect_poly(r):
    return [(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])]


def _pt_rect(px, py, r):
    """点到矩形的有符号距离(在里面是负数)。"""
    dx = max(r[0] - px, 0.0, px - r[2])
    dy = max(r[1] - py, 0.0, py - r[3])
    if dx == 0.0 and dy == 0.0:
        return -min(px - r[0], r[2] - px, py - r[1], r[3] - py)
    return math.hypot(dx, dy)


def regions(cfg, obstacles=()):
    """不能碰的东西：[(名字, 'rect', (x0,y0,x1,y1)) / (名字, 'circle', (x,y,r))]。启停区不算(允许进去)。"""
    out = [('黄色区', 'rect', r) for r in YELLOW]
    out += [('暂存区', 'rect', TEMP_ZONE), ('粗加工区', 'rect', ROUGH_ZONE)]
    out += [('禁行区', 'rect', tuple(r)) for r in ((cfg or {}).get('extra_blocked_rects') or [])]
    rc = (cfg or {}).get('raw_center_mm', [1200, 2480])
    out.append(('原料转盘', 'circle', (float(rc[0]), float(rc[1]), float((cfg or {}).get('raw_radius_mm', 150)))))
    for o in obstacles or ():
        out.append((f'障碍物({float(o[0]):.0f},{float(o[1]):.0f})', 'circle', (float(o[0]), float(o[1]), float(o[2]))))
    return out


def edge_clearance(pose, model):
    """车身(含雷达)离场地边线最近多少 mm(出了场地是负数)。"""
    poly, disk = body_poly(pose, model)
    c = min(min(x, FIELD_MM - x, y, FIELD_MM - y) for x, y in poly)
    if disk:
        x, y, r = disk
        c = min(c, x - r, FIELD_MM - x - r, y - r, FIELD_MM - y - r)
    return c


def pose_clearance(cfg, obstacles, pose, model=None, regs=None):
    """真实车身(含雷达)离场地边线和所有不能碰的区域最近多少 mm、是什么。负数 = 已经压上/出界。"""
    model = model or body_model(cfg)
    regs = regs if regs is not None else regions(cfg, obstacles)
    poly, disk = body_poly(pose, model)
    best, what = edge_clearance(pose, model), '场地边线'
    xs = [p[0] for p in poly] + ([disk[0] - disk[2], disk[0] + disk[2]] if disk else [])
    ys = [p[1] for p in poly] + ([disk[1] - disk[2], disk[1] + disk[2]] if disk else [])
    bb = (min(xs), min(ys), max(xs), max(ys))
    for name, kind, g in regs:
        if kind == 'rect':
            gap = max(g[0] - bb[2], bb[0] - g[2], g[1] - bb[3], bb[1] - g[3])
            if gap > best:
                continue
            d = _poly_dist(poly, _rect_poly(g))
            if disk:
                d = min(d, _pt_rect(disk[0], disk[1], g) - disk[2])
        else:
            cx, cy, r = g
            gap = max(cx - r - bb[2], bb[0] - cx - r, cy - r - bb[3], bb[1] - cy - r)
            if gap > best:
                continue
            d = _poly_circle(poly, cx, cy, r)
            if disk:
                d = min(d, math.hypot(disk[0] - cx, disk[1] - cy) - disk[2] - r)
        if d < best:
            best, what = d, name
    return best, what


def moves_clearance(cfg, obstacles, pose, cmds, step_mm=10.0, step_deg=2.0, model=None):
    """一串指令一路扫过去(平移每 step_mm、转弯每 step_deg 检查一次)，车身离边线/各区域最近多少。
    返回 (最近 mm, 第几条, 是什么, 终点位姿)。"""
    model = model or body_model(cfg)
    regs = regions(cfg, obstacles)
    best, at, what = math.inf, -1, ''
    p = tuple(float(v) for v in pose[:3])
    for i, (c, v) in enumerate(cmds):
        v = float(v)
        n = max(1, int(math.ceil(abs(v) / (step_deg if c == 'R' else step_mm))))
        for k in range(1, n + 1):
            q = apply_move(p, c, v*k/n)
            d, w = pose_clearance(cfg, obstacles, q, model, regs)
            if d < best:
                best, at, what = d, i, w
        p = apply_move(p, c, v)
    if not cmds:
        best, what = pose_clearance(cfg, obstacles, p, model, regs)
    return best, at, what, p


def _plan_check(cfg, obstacles, pose, cmds):
    """route_plan 里有 moves_clear 时也问它(margin 0 = 真的会压上/出界才算不行)。返回 None = 没意见，否则一句原因。
    本来就不够的(车已经离得很近)，只要不比现在更近也放行。"""
    mc = getattr(_rp, 'moves_clear', None)
    if mc is None:
        return None
    try:
        r = mc(cfg, list(obstacles or []), tuple(pose), [(c, v) for c, v in cmds], margin_mm=0)
        if r[0]:
            return None
        pc = getattr(_rp, 'pose_clear', None)
        if pc is not None and r[1] is not None:
            s = pc(cfg, list(obstacles or []), tuple(pose), margin_mm=0)
            if s[1] is not None and float(r[1]) >= float(s[1]) - 0.5:
                return None
        return f'{r[3] if len(r) > 3 else ""}(离 {float(r[1]):.0f}mm)' if r[1] is not None else str(r[-1])
    except Exception:
        return None


def safe_moves(cfg, obstacles, pose, cmds, need_mm):
    """这几条修正动作能不能做：一路上离边线和各区域至少 need_mm；车现在就已经不到 need_mm 时，只要不比现在更近也行。
    返回 (能不能, 最近 mm, 是什么)。"""
    now, _ = pose_clearance(cfg, obstacles, pose)
    best, _, what, _ = moves_clearance(cfg, obstacles, pose, cmds)
    if best < min(need_mm, now) - 0.5:
        return False, best, what
    veto = _plan_check(cfg, obstacles, pose, cmds)
    if veto:
        return False, best, veto
    return True, best, what


def clamp_move(cfg, obstacles, pose, cmd, val, need_mm, edge_mm=None, unit=1.0):
    """把一条修正动作缩到能安全做的最大值(按 unit 取整)。edge_mm：离场地边线另外至少要留多少(回启停区时用)。返回能做的值(可能是 0)。"""
    model = body_model(cfg)

    def ok(v):
        if v == 0:
            return True
        if not safe_moves(cfg, obstacles, pose, [(cmd, v)], need_mm)[0]:
            return False
        if edge_mm is not None:
            e0 = edge_clearance(pose, model)
            n = max(1, int(math.ceil(abs(v) / (2.0 if cmd == 'R' else 10.0))))
            for k in range(1, n + 1):
                e = edge_clearance(apply_move(pose, cmd, v*k/n), model)
                if e < min(edge_mm, e0) - 0.01:
                    return False
        return True
    if ok(val):
        return val
    sg = 1 if val > 0 else -1
    lo, hi = 0, int(abs(val) / unit)
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if ok(sg*mid*unit):
            lo = mid
        else:
            hi = mid - 1
    v = sg*lo*unit
    return int(v) if unit == 1.0 else round(v, 1)


# ================================================================ STM32 固件能力 / 转弯指令
def firmware_caps(link, cfg=None):
    """新固件(GET 里有 PSPD)的 R 支持 0.1° 小数；旧固件只认整数度。配置 r_decimals=true/false 可以强制。"""
    caps = {'rdec': False}
    force = (cfg or {}).get('r_decimals')
    if force is not None:
        caps['rdec'] = bool(force)
        return caps
    try:
        ps, _ = link.get_params()
        caps['rdec'] = bool(ps) and 'PSPD' in ps
    except Exception:
        pass
    return caps


def turn_min_deg(caps):
    """能发的最小转角：新固件 0.05°(按 0.1° 取整后不是 0)；旧固件 1°。"""
    return 0.05 if caps.get('rdec') else 1.0


def send_turn(link, deg, caps, log=None, toward_zero=False):
    """发一条 R。R 0 在 STM32 上是 HOME(把现在的车头记为要保持的方向)，所以绝不发 0。
    新固件发 0.1° 小数(用 request 自己拼指令，不依赖 stm32_link.move 怎么格式化数字)，旧固件四舍五入到整数度。
    toward_zero=True(限过幅的修正)：取整时只往小里取，不会比限幅后的角度转得多。
    返回 (实际发的角度(没发是 0), ok, 回复)。"""
    if caps.get('rdec') and hasattr(link, 'request'):
        v = (math.trunc(float(deg) * 10) / 10.0) if toward_zero else round(float(deg), 1)
        if abs(v) < 0.1:
            return 0, True, '太小，不发'
        if v != int(v):
            try:
                out = link.request(f'R {v:.1f}', 15.0, collect=True)
                ok, reply = bool(out[0]), str(out[1] or '')
            except Exception as e:
                ok, reply = False, repr(e)
            if ok or 'ARG' not in reply:
                return v, ok, reply
            caps['rdec'] = False                        # 固件不认小数：以后都发整数
            if log:
                log(f'    (STM32 不认小数角度 R {v:.1f}：{reply}，以后按整数度发)')
        else:
            v = int(v)
            ok, reply = link.move('R', v, None)
            return v, ok, reply
    vi = math.trunc(float(deg)) if toward_zero else int(round(float(deg)))
    if vi == 0:
        return 0, True, '小于 1°，旧固件不发'
    ok, reply = link.move('R', vi, None)
    return vi, ok, reply


# ================================================================ 位置跟踪
class Nav:
    """drive 里记着两样东西：
    ref = 按路线指令，车"应该"在的位置(规划的位置)；est = 按实际发出的指令(和雷达测到的)，车"实际"在的位置。
    每条前进/横移发出去前，按 ref 和 est 的差把这一条加长/缩短(沿这一条的方向)，前面没走的、雷达测到的偏差就这样补上。
    yaw_off = 车实际车头 - est 的车头(雷达测到、还没修掉的；下一次转弯时一起修)。
    start 给了时(ctx.route_start)用世界坐标；没给就用相对出发点的坐标(只能做按指令推算)。"""

    def __init__(self, start=None):
        self.world = bool(start)
        if start:
            self.est = tuple(float(v) for v in start['est'][:3])
            self.ref = tuple(float(v) for v in start['ref'][:3])
            self.real_yaw0 = start.get('yaw_real')
        else:
            self.est = self.ref = (0.0, 0.0, 0.0)
            self.real_yaw0 = None
        self.yaw_off = 0.0

    def real(self):
        return (self.est[0], self.est[1], wrap180(self.est[2] + self.yaw_off))

    def turn_cmd(self, val):
        ref_next = apply_move(self.ref, 'R', val)
        return ref_next, wrap180(ref_next[2] - self.est[2] - self.yaw_off)

    def turned(self, target_yaw, sent):
        real = self.est[2] + self.yaw_off + float(sent)
        self.est = (self.est[0], self.est[1], wrap180(target_yaw))
        self.yaw_off = wrap180(real - self.est[2])

    def lin_cmd(self, cmd, val):
        ref_next = apply_move(self.ref, cmd, val)
        ux, uy = _axis(self.real()[2], cmd)
        return ref_next, (ref_next[0] - self.est[0])*ux + (ref_next[1] - self.est[1])*uy

    def moved(self, cmd, sent):
        p = apply_move(self.real(), cmd, sent)
        self.est = (p[0], p[1], self.est[2])

    def owed(self, target=None):
        """车还差多少才到 target(默认 ref)：车身坐标 (前后 F, 左右 S)。"""
        t = target or self.ref
        dx, dy = t[0] - self.est[0], t[1] - self.est[1]
        yaw = self.real()[2]
        fx, fy = _axis(yaw, 'F')
        sx, sy = _axis(yaw, 'S')
        return dx*fx + dy*fy, dx*sx + dy*sy

    def sync_rest(self, rest):
        for c in ('F', 'S'):
            if rest and rest.get(c):
                self.ref = apply_move(self.ref, c, rest[c])

    def measured(self, pose, e_last=0.0):
        """雷达测到车在 pose(世界坐标，车头是实际车头)。e_last = 上一条指令 DONE 里的车头误差(目标 - 实际)，
        这部分 STM32 下一条指令自己会修，不算进 yaw_off。"""
        self.est = (float(pose[0]), float(pose[1]), self.est[2])
        self.yaw_off = wrap180(float(pose[2]) + float(e_last or 0.0) - self.est[2])


# ================================================================ 小工具
def _role(stop):
    try:
        from task_plan import role_of
        return role_of(stop)
    except Exception:
        s = str(stop or '').upper()
        for r in ('QR', 'RAW', 'ROUGH', 'TEMP', 'START'):
            if s.startswith(r):
                return r
        return None


def _call(obj, name, *a, **kw):
    """有这个方法才调用(另一块的接口可能还没有)。"""
    fn = getattr(obj, name, None) if obj is not None else None
    if fn is None:
        return None
    return fn(*a, **kw)


def make_tick(hooks, cfg, now=time.monotonic):
    """长时间不动(扫描、规划、定位等待)时每隔 keepalive_s 秒调一次 hooks.keepalive()(手臂轻摆，避开 15 秒规则)。
    只在行驶线程里调用。没有 hooks.keepalive 返回 None。"""
    if hooks is None or not hasattr(hooks, 'keepalive'):
        return None
    # 默认 5.5 秒：hooks.keepalive 离上次调用不到 5 秒时什么都不做，隔 4 秒调会每隔一次被跳过(实际 8 秒才摆一次)
    period = float((cfg or {}).get('keepalive_s', 5.5))
    last = [now()]

    def tick():
        if now() - last[0] >= period:
            last[0] = now()
            try:
                hooks.keepalive()
            except Abort:
                raise
            except Exception:
                pass
    return tick


def run_bg(fn, tick=None, aborted=None, poll=0.1, sleep=time.sleep):
    """fn 放到后台线程里做(规划、扫描)，当前线程一边等一边 tick()(手臂轻摆)、看 abort。返回 fn 的结果，fn 的异常照样抛出。"""
    box = {}

    def work():
        try:
            box['r'] = fn()
        except BaseException as e:
            box['e'] = e
    t = threading.Thread(target=work, daemon=True)
    t.start()
    while t.is_alive():
        if aborted is not None and aborted():
            raise Abort('收到 abort')
        if tick is not None:
            tick()
        t.join(poll)
    if 'e' in box:
        raise box['e']
    return box.get('r')


def leg_time(motion, seq, a, b, turn_extra=0.0):
    """seq[a:b] 里的指令预计要多少秒。turn_extra：每次转弯另外加的秒数(紧的转弯前要停下来雷达定位)。"""
    if motion is None:
        return 0.0
    return sum(motion.time_of(c, v) + (turn_extra if c == 'R' else 0.0) for _, c, v in seq[a:b] if c)


def home_route_legs(cfg, obstacles, start_pose, names):
    """从 start_pose(某个停车点)直接回家的路线(经过 names，例如 ['PREHOME', 'START2'])。
    route_plan 有 plan_leg 就逐段用它，否则(或它失败)用 plan_mission。规划不出来返回 None。"""
    stops = (cfg or {}).get('stops') or {}
    pl = getattr(_rp, 'plan_leg', None)
    if pl is not None:
        try:
            legs, pose = [], tuple(start_pose)
            for n in names:
                goal = tuple(float(v) for v in stops[n][:3])
                L = pl(cfg, list(obstacles), pose, goal)
                if not L:
                    raise ValueError(n)
                L = dict(L)
                L['stop'] = n
                L.setdefault('goal', goal)
                legs.append(L)
                pose = tuple(L['goal'])
            return legs
        except Exception:
            pass
    try:
        legs, why, _ = _rp.plan_mission(cfg, list(obstacles), tuple(start_pose), list(names))
        return legs if why == 'OK' and len(legs) == len(names) else None
    except Exception:
        return None


# ================================================================ 回家：定位 + 小步修到出发位置
def start_rect(goal, cfg=None):
    r = (cfg or {}).get('start_rect')
    if r:
        return tuple(float(v) for v in r)
    return START_RECTS[1] if float(goal[1]) > FIELD_MM/2 else START_RECTS[2]


def in_rect(p, r):
    return r[0] <= p[0] <= r[2] and r[1] <= p[1] <= r[3]


def inset_goal(goal, cfg, edge_mm=None):
    """回家的目标点：出发位置车身离场地边线只有几毫米(启停区在角上，车长 290 放进 300)。雷达定位差几毫米就出场地，
    所以目标点往场内挪到车身(含雷达)离每条边线至少 home_goal_edge_mm(默认 10)。挪的方向是场地内侧，宁可在启停区开着的那边多出一点。"""
    e = float((cfg or {}).get('home_goal_edge_mm', 10) if edge_mm is None else edge_mm)
    poly, disk = body_poly(goal, body_model(cfg))
    xs = [p[0] for p in poly] + ([disk[0] - disk[2], disk[0] + disk[2]] if disk else [])
    ys = [p[1] for p in poly] + ([disk[1] - disk[2], disk[1] + disk[2]] if disk else [])
    dx = max(0.0, e - min(xs)) - max(0.0, max(xs) - (FIELD_MM - e))
    dy = max(0.0, e - min(ys)) - max(0.0, max(ys) - (FIELD_MM - e))
    return (float(goal[0]) + dx, float(goal[1]) + dy, float(goal[2]))


def home_approach(ctx, link, log, nav, goal, cfg, caps, obstacles=(), max_move=None, order=None, label='回启停区对准',
                  aborted=None, e_last=0.0):
    """回到启停区：雷达定位 -> 按实际位置小步(home_fix_rpm，默认 60 转/分，STM32 精确模式)修到出发时手放的位置，最多 2 轮。
    每一步都先限幅：车身(含雷达)不出场地(离边线至少 home_edge_mm)、不碰别的区域，最后要停在启停区里。
    返回 dict(status, pose)。status：
      'ok'           到了(误差在 home_fix_tol_mm / home_fix_tol_deg 以内)
      'near'         修过了但还没进误差范围，停在这里(不再乱动)
      'fail_unmoved' 一开始就定位失败，车没动过(调用的人可以接着按原路线走)
      'fail_moved'   修过以后定位失败：不用推算位置再测一次；还不行就按"最后一次测准的位置 + 之后发过的指令"
                     补完剩下的距离(朝场地边的方向少走 home_short_mm，宁短不长)；连测准过都没有就原地停住
      'off'          没有雷达定位(桌面模拟/没有参考扫描)"""
    g = (cfg or {}).get
    if not nav.world or not hasattr(ctx, 'relocalize'):
        return dict(status='off', pose=None)
    if not g('home_fix_enabled', True):
        log(f'    {label}：配置里关掉了(home_fix_enabled=false)，不做')
        return dict(status='off', pose=None)
    aborted = aborted or (lambda: ctx.aborted())
    rounds = max(1, min(2, int(g('home_fix_rounds', 2))))
    tol = float(g('home_fix_tol_mm', 6))
    tol_a = float(g('home_fix_tol_deg', 1.0))
    frames = int(g('home_fix_frames', 12))
    rpm = int(g('home_fix_rpm', g('reloc_slow_rpm', 60)))
    edge = float(g('home_edge_mm', 3))
    short = float(g('home_short_mm', 15))
    long_mm = float(g('home_long_mm', 100))
    frac = float(g('approach_guard_frac', 0.02))
    need = float(g('home_check_margin_mm', 0))
    limit = float(max_move if max_move is not None else g('home_max_move_mm', 150))
    zone = start_rect(goal, cfg)
    goal0 = tuple(float(v) for v in goal[:3])
    goal = inset_goal(goal0, cfg)
    if math.hypot(goal[0] - goal0[0], goal[1] - goal0[1]) > 0.5:
        log(f'    (回家目标往场内挪 {goal[0]-goal0[0]:+.0f},{goal[1]-goal0[1]:+.0f}mm：车身离场地边线至少 '
            f'{float(g("home_goal_edge_mm", 10)):.0f}mm，雷达定位差几毫米也不出场地)')
    obstacles = list(obstacles or [])
    moved = False
    last_good = None
    e_last = [float(e_last or 0.0)]             # 上一条指令 DONE 里的车头误差(STM32 下一条自己会修)

    def send(cmd, v):
        if cmd == 'R':
            sent, ok, reply = send_turn(link, v, caps, log, toward_zero=True)
            if sent:
                nav.turned(goal[2], sent)               # 目标车头 = 出发时的车头；没转够的记在 yaw_off 里
        else:
            sent = int(round(v))
            if sent == 0:
                return True
            ok, reply = link.move(cmd, sent, rpm)
            if ok:
                nav.moved(cmd, sent)
        if sent:
            log(f"      {({'R': '转', 'S': '横移', 'F': '前后'})[cmd]} {_fmt(sent)} -> {'完成' if ok else reply}")
            err = parse_done(reply).get('err') if ok else None
            e_last[0] = err or 0.0
            bad = check_move(cmd, sent, ok, reply)
            if bad:
                log(f'      ★ {bad}')                   # 车没按指令动：不再修(再修只会越错越远)
                return False
        return ok

    def plan_moves(final):
        """从 nav 的位置到 goal 的修正动作(先转车头，再平移；平移先做让车离边线更远的那个)。"""
        moves = []
        dth = wrap180(goal[2] - nav.real()[2])
        if abs(dth) >= tol_a and abs(dth) >= turn_min_deg(caps):
            v = clamp_move(cfg, obstacles, nav.real(), 'R', dth, need, edge, unit=0.1 if caps.get('rdec') else 1.0)
            if v:
                moves.append(('R', v))
        p = nav.real() if not moves else apply_move(nav.real(), 'R', moves[0][1])
        yaw = p[2]
        dx, dy = goal[0] - p[0], goal[1] - p[1]
        fx, fy = _axis(yaw, 'F')
        sx, sy = _axis(yaw, 'S')
        f, s = dx*fx + dy*fy, dx*sx + dy*sy
        model = body_model(cfg)
        comps = [(c, v) for c, v in (('F', f), ('S', s)) if abs(v) >= (tol if not final else STOP_MIN_MM)]
        if order:
            comps.sort(key=lambda cv: list(order).index(cv[0]) if cv[0] in order else 9)
        else:
            comps.sort(key=lambda cv: -edge_clearance(apply_move(p, cv[0], cv[1]), model))
        for c, v in comps:
            e0 = edge_clearance(p, model)
            toward = edge_clearance(apply_move(p, c, v), model) < e0
            if final and toward:
                v = math.copysign(max(0.0, abs(v) - short), v)     # 朝边线走的少走一点：宁短不长
            elif toward and abs(v) > long_mm:
                # 离得远(没有 PREHOME、从 home_cut 的地方过来，几百 mm)：距离差 1~2% 就会冲出场地。
                # 先少走 approach_guard_frac × 距离 + home_short_mm，下一轮雷达定位后再慢慢修进去
                v = math.copysign(max(0.0, abs(v) - (frac*abs(v) + short)), v)
            v = clamp_move(cfg, obstacles, p, c, int(round(v)), need, edge)
            if v:
                moves.append((c, v))
                p = apply_move(p, c, v)
        return moves, p

    for rnd in range(1, rounds + 2):
        if aborted():
            raise Abort('收到 abort，回启停区对准停止')
        res = _reloc(ctx, nav.real(), frames)
        tag = f'{label}(第{rnd}次)'
        if not res.get('ok'):
            log(f"    {tag}：定位失败：{res.get('why')}")
            if not moved:
                return dict(status='fail_unmoved', pose=None)
            res = _reloc(ctx, nav.real(), frames, use_init=False)
            if res.get('ok'):
                log('      不用推算位置再测一次，测到了。')
            else:
                log(f"      不用推算位置再测一次也失败：{res.get('why')}")
                if last_good is None:
                    log('      ★ 一次都没测准过：原地停住，不再按原路线走(免得冲出场地)。')
                    return dict(status='fail_moved', pose=None)
                moves, end = plan_moves(final=True)
                log(f'      按最后一次测准的位置 + 之后发过的指令推算，补完剩下的距离(朝边线少走 {short:.0f}mm)：'
                    + ('，'.join(f'{c} {_fmt(v)}' for c, v in moves) or '不用动'))
                for c, v in moves:
                    if aborted():
                        raise Abort('收到 abort，回启停区对准停止')
                    if not send(c, v):
                        break
                return dict(status='fail_moved', pose=nav.real())
        last_good = tuple(res['pose'])
        nav.measured(res['pose'], e_last[0])
        p = nav.real()
        dth = wrap180(goal[2] - p[2])
        f, s = nav.owed(goal)
        log(f"    {tag}：离出发位置 前后{-f:+.0f}mm 左右{-s:+.0f}mm 车头{-dth:+.1f}°"
            f"（{res.get('inlier', 0)*100:.0f}%的点对上，残差{res.get('rms', 0):.1f}mm，ICP {res.get('t', 0):.2f}秒）")
        if abs(f) < tol and abs(s) < tol and abs(dth) < tol_a:
            log('      已经在误差范围内。')
            return dict(status='ok', pose=p)
        if rnd > rounds:
            log(f'      修了{rounds}次还没进误差范围，停在这里(不再动)。')
            return dict(status='near', pose=p)
        if math.hypot(f, s) > limit:
            log(f'      要走 {math.hypot(f, s):.0f}mm，超过 {limit:.0f}mm，不像是对的，不按它走。')
            return dict(status='fail_moved' if moved else 'fail_unmoved', pose=p)
        moves, end = plan_moves(final=False)
        if not in_rect(end, zone) and in_rect(p, zone):
            log(f'      修完会停在启停区外面({end[0]:.0f},{end[1]:.0f})，不修。')
            return dict(status='near', pose=p)
        if not moves:
            log('      (限幅以后没有能安全做的修正，停在这里)')
            return dict(status='near', pose=p)
        log('      修正：' + '，'.join({'R': '转', 'S': '横移', 'F': '前后'}[c] + f' {_fmt(v)}' for c, v in moves))
        for c, v in moves:
            if aborted():
                raise Abort('收到 abort，回启停区对准停止')
            moved = True
            if not send(c, v):
                return dict(status='fail_moved', pose=nav.real())
    return dict(status='near', pose=nav.real())


def _reloc(ctx, pose, frames, use_init=True, **kw):
    """ctx.relocalize：旧的 ctx 不认识 use_init 等参数时去掉再调。"""
    try:
        if use_init and not kw:
            return ctx.relocalize(pose, frames=frames) or {}
        return ctx.relocalize(pose, frames=frames, use_init=use_init, **kw) or {}
    except TypeError:
        if not use_init:
            return dict(ok=False, why='这个版本不支持不用推算位置测')
        return ctx.relocalize(pose, frames=frames) or {}
    except Abort:
        raise
    except Exception as e:
        return dict(ok=False, why=f'定位出错：{e!r}')


# ================================================================ 按路线行驶
class _Drive:
    def __init__(self, ctx, link, seq, log, stop_wait, hooks, speeds, motion, cfg, start, caps):
        self.ctx, self.link, self.log = ctx, link, log
        self.stop_wait, self.hooks, self.motion = stop_wait, hooks, motion
        self.cfg = cfg or {}
        self.speeds = speeds or {}
        self.caps = caps if caps is not None else {'rdec': False}
        self.nav = Nav(start)
        g = self.cfg.get
        self.stops = g('stops') or {}
        self.rescan = {str(s).upper() for s in g('rescan_stops', ['QR'])}
        rs = g('reloc_stops')
        self.reloc_stops = None if rs is None else {str(s).upper() for s in rs}
        self.reloc_on = bool(g('reloc_enabled', True)) and hasattr(ctx, 'relocalize')
        self.reloc_frames = int(g('reloc_frames', 8))
        self.fine = int(g('reloc_slow_rpm', 60))
        self.slow_below = float(g('slow_below_mm', 60))
        self.home_short = float(g('home_short_mm', 15))
        self.use_approach = bool(g('home_approach', True)) and hasattr(ctx, 'relocalize')
        self.seq = list(seq)
        self.cut = None
        self._recut()
        self.total = sum(1 for _, c, _ in self.seq if c)
        self.i = 0
        self.k = 0
        self.e_last = 0.0
        self.approach = None
        self.homing = False
        self.final_leg = False                  # 正在走进启停区的最后一段：朝场地边线的那一条先少走 home_short_mm
        self.t_start = time.monotonic()
        self.t_move = 0.0
        self.travel = 0.0                       # 上次雷达定位以后走了多少(mm，转一次弯按 200mm 算)
        self.min_travel = float(g('reloc_min_travel_mm', 0))
        self.yaw_real0 = (start or {}).get('yaw_real')
        self.leg_cuts = []                      # 这一段里因为离边线/区域太近少走的方向(世界坐标单位向量)
        self.n_fix = [0, 0, 0]                  # 雷达定位：次数、没成功的次数；按定位修正的小动作条数
        self.turn_extra = float(g('turn_extra_s', 0.8))

    # ------------------------------------------------------------ 工具
    def _recut(self):
        self.cut = None
        if self.use_approach and not has_prehome(self.seq) and self.nav.world:
            self.seq, self.cut = home_cut(self.seq, self.cfg)

    def _aborted(self):
        if self.ctx.aborted():
            try:
                self.link.abort()
            except Exception:
                pass
            raise Abort('收到 abort，已停止发送')

    def _obstacles(self):
        try:
            return list(self.ctx.obstacles()) if hasattr(self.ctx, 'obstacles') else []
        except Exception:
            return []

    def _next_marker(self, k):
        for j in range(k + 1, len(self.seq)):
            if self.seq[j][1] is None:
                return j
        return None

    def _splice(self, legs, why):
        new_seq = flatten(0, legs)
        self.seq = self.seq[:self.k + 1] + new_seq
        self._recut()
        self.total = self.i + sum(1 for _, c, _ in new_seq if c)
        n2, tot2 = summarize(new_seq)
        self.log(f'    {why}：剩下 {n2} 条指令，前进/后退 {tot2["F"]}mm，横移 {tot2["S"]}mm，转弯 {tot2["R"]}°')

    def _send(self, cmd, v, speed, label=''):
        self.i += 1
        self.total = max(self.total, self.i)
        est = f'，预计{self.motion.time_of(cmd, v):.1f}秒' if self.motion is not None else ''
        t0 = time.monotonic()
        ok, reply = self.link.move(cmd, v, speed)
        dt = time.monotonic() - t0
        self.t_move += dt
        info = parse_done(reply)
        extra = f'，车头误差 {info["err"]:+.2f}°' if 'err' in info else ''
        self.e_last = info.get('err') or 0.0
        self.log(f'  [{self.i}/{self.total}] {cmd} {_fmt(v)}{label} -> {"完成" if ok else reply}  用时{dt:.1f}秒{est}{extra}')
        bad = check_move(cmd, v, ok, reply)
        if bad:
            raise Abort(f'第{self.i}条指令 {bad}')
        self.travel += abs(v)

    def _turn(self, deg, target_yaw, label='', toward_zero=False):
        """发一条转弯(route 或修正)。target_yaw = 转完以后 est 的车头。"""
        t0 = time.monotonic()
        sent, ok, reply = send_turn(self.link, deg, self.caps, self.log, toward_zero)
        if not sent and ok:
            self.nav.turned(target_yaw, 0.0)
            return
        self.i += 1
        self.total = max(self.total, self.i)
        dt = time.monotonic() - t0
        self.t_move += dt
        info = parse_done(reply)
        extra = f'，车头误差 {info["err"]:+.2f}°' if 'err' in info else ''
        self.e_last = info.get('err') or 0.0
        self.log(f'  [{self.i}/{self.total}] R {_fmt(sent)}{label} -> {"完成" if ok else reply}  用时{dt:.1f}秒{extra}')
        bad = check_move('R', sent, ok, reply)
        if bad:
            raise Abort(f'第{self.i}条指令 {bad}')
        self.nav.turned(target_yaw, sent)
        if abs(sent) >= 45:
            self.travel += 200.0

    # ------------------------------------------------------------ 主循环
    def _check_turn_back(self):
        """出发时第一条 R(从第二站转回出发时的车头)：规划不检查它扫过的范围。车角扫出场地/碰到东西就提醒(还在启停区里，规则不管)。"""
        if not self.nav.world or not self.seq or self.seq[0][1] != 'R' or self.yaw_real0 is None:
            return
        _, want = self.nav.turn_cmd(self.seq[0][2])
        p = (self.nav.est[0], self.nav.est[1], float(self.yaw_real0))
        best, _, what, _ = moves_clearance(self.cfg, self._obstacles(), p, [('R', want)])
        if best < 0:
            self.log(f'  ⚠ 出发时原地转回 {want:+.0f}° 时车角离{what} {best:.0f}mm(会扫出一点；还在启停区里，规则不管)。'
                     '车放在启停区里时尽量离东边线远一点')

    def run(self):
        self._check_turn_back()
        while self.k < len(self.seq):
            stop, cmd, val = self.seq[self.k]
            self._aborted()
            if self.cut is not None and self.k == self.cut['index'] and self.approach is None:
                if self._cut_approach():
                    self.k = self.cut['marker']           # 不再照原路线剩下的几步走，直接到"到达启停区"
                    continue
            if cmd is None:
                self._stop(stop, _marker_info(val))
                self.k += 1
                continue
            self._command(cmd, val)
            self.k += 1
        self.log(f'  路线行驶共用 {time.monotonic()-self.t_start:.1f} 秒（其中指令执行 {self.t_move:.1f} 秒）')
        if self.n_fix[0]:
            self.log(f'  路上雷达定位 {self.n_fix[0]} 次(没成功 {self.n_fix[1]} 次)，按定位修正 {self.n_fix[2]} 条小动作')

    def _command(self, cmd, val):
        if cmd == 'R':
            self._before_turn(val)
            ref_next, want = self.nav.turn_cmd(val)
            self._turn(want, ref_next[2])
            self.nav.ref = ref_next
            self._after_turn()
            return
        ref_next, want = self.nav.lin_cmd(cmd, val)
        label = ''
        if self.nav.world and abs(want) >= 1:
            cut, label = self._approach_cut(cmd, want)
            if cut > 0:
                want = math.copysign(max(0.0, abs(want) - cut), want)
                ux, uy = _axis(self.nav.real()[2], cmd)
                sg = 1.0 if want >= 0 else -1.0
                self.leg_cuts.append((sg*ux, sg*uy))
        v = int(round(want))
        if abs(v) < STOP_MIN_MM:
            self.nav.ref = ref_next                       # 差得太少：这一条不发，带到后面
            if v:
                self.log(f'    ({cmd} {_fmt(val)} 加上前面的偏差只剩 {v:+d}mm，不单独发，带到后面)')
            return
        if v != val and not label:
            label = f'(原 {_fmt(val)}，补上偏差)'
        speed = self.fine if abs(v) <= self.slow_below else self.speeds.get(cmd)
        self._send(cmd, v, speed, label)
        self.nav.moved(cmd, v)
        self.nav.ref = ref_next

    def _count(self, meas):
        self.n_fix[0] += 1
        if not (meas or {}).get('ok'):
            self.n_fix[1] += 1
        return meas

    def _uncert(self):
        """上次雷达定位以后，按推算的位置可能差多少(mm)：approach_guard_travel_frac × 走过的距离。"""
        return float(self.cfg.get('approach_guard_travel_frac', 0.015)) * self.travel

    def _before_turn(self, val):
        """路线里原地转弯：规划按"车在计划的位置"检查过转弯扫过的范围。车按推算走了一段，实际位置可能差几厘米，
        转的时候车角就可能扫到黄区/工位区/转盘/场地边。按推算的位置算，扫过的地方离它们比"可能差的距离"还近：先用雷达定位，
        把车修回计划的转弯点再转。"""
        g = self.cfg.get
        if not (self.nav.world and self.reloc_on and g('reloc_before_turn', True)):
            return
        if self.travel < float(g('reloc_before_turn_min_mm', 300)):
            return
        _, want = self.nav.turn_cmd(val)
        if abs(want) < 1:
            return
        obs = self._obstacles()
        best, _, what, _ = moves_clearance(self.cfg, obs, self.nav.real(), [('R', want)], step_deg=3.0)
        u = self._uncert() + float(g('approach_guard_mm', 10))
        if best >= u:
            return
        self.log(f'    (要原地转 {want:+.0f}°：按推算的位置转，车角离{what}只有 {best:.0f}mm，可能已经差了 {self._uncert():.0f}mm：先雷达定位)')
        meas = self._count(_reloc(self.ctx, self.nav.real(), int(g('reloc_turn_frames', 4))))   # 路线中间：少拍几帧，快
        if not meas.get('ok'):
            self.log(f"    ◆ 转弯前雷达定位没成功：{meas.get('why')}。照推算的转。")
            return
        self.travel = 0.0
        p = meas['pose']
        self.log(f"    ◆ 转弯前雷达定位：车在 ({p[0]:.0f},{p[1]:.0f}) 车头{p[2]:.1f}°"
                 f"（{meas.get('inlier', 0)*100:.0f}%的点对上，残差{meas.get('rms', 0):.1f}mm，ICP {meas.get('t', 0):.2f}秒）")
        self.nav.measured(p, self.e_last)
        need = float(g('reloc_check_margin_mm', 10))
        f, s = self.nav.owed()
        moves = [(c, v) for c, v in (('S', s), ('F', f)) if abs(v) >= float(g('reloc_turn_tol_mm', 8))]
        model = body_model(self.cfg)
        q = self.nav.real()
        moves.sort(key=lambda cv: -pose_clearance(self.cfg, obs, apply_move(q, cv[0], cv[1]), model)[0])
        for c, v in moves:
            v2 = clamp_move(self.cfg, obs, self.nav.real(), c, int(round(v)), need)
            if abs(v2) >= STOP_MIN_MM:
                self._send(c, v2, self.fine, '(转弯前定位修正)')
                self.n_fix[2] += 1
                self.nav.moved(c, v2)

    def _after_turn(self):
        """路线中间转完弯、接下来是一条长直行(>= reloc_split_mm)、上次定位以后又走了 >= reloc_min_travel_mm：先用雷达定位一次。
        前一条长直行走多走少的误差，转弯以后就变成了横向偏差，一路带着往黄区/工位区那边偏；在这里修掉
        (车头和横向偏差马上修，沿着下一条方向的偏差并进下一条)。"""
        g = self.cfg.get
        if not (self.nav.world and self.reloc_on and g('reloc_after_turn', True)):
            return
        nxt = None
        for j in range(self.k + 1, len(self.seq)):
            c = self.seq[j][1]
            if c is None or c == 'R':
                return
            if c in ('F', 'S'):
                nxt = (c, self.seq[j][2])
                break
        if nxt is None or abs(nxt[1]) < float(g('reloc_split_mm', 1200)) or self.travel < max(self.min_travel, 1.0):
            return
        meas = self._count(_reloc(self.ctx, self.nav.real(), int(g('reloc_turn_frames', 4))))   # 路线中间：少拍几帧，快
        if not meas.get('ok'):
            self.log(f"    ◆ 转弯后雷达定位没成功：{meas.get('why')}。照推算的走。")
            return
        self.travel = 0.0
        p = meas['pose']
        self.log(f"    ◆ 转弯后、长直行前雷达定位：车在 ({p[0]:.0f},{p[1]:.0f}) 车头{p[2]:.1f}°"
                 f"（{meas.get('inlier', 0)*100:.0f}%的点对上，残差{meas.get('rms', 0):.1f}mm，ICP {meas.get('t', 0):.2f}秒）")
        self.nav.measured(p, self.e_last)
        need = float(g('reloc_check_margin_mm', 10))
        obs = self._obstacles()
        off = self.nav.yaw_off
        yt = max(float(g('reloc_yaw_tol_deg', 1.0)) if not self.caps.get('rdec') else min(float(g('reloc_yaw_tol_deg', 1.0)),
                 float(g('reloc_yaw_min_deg', 0.3))), turn_min_deg(self.caps))
        if abs(off) >= yt:
            v = clamp_move(self.cfg, obs, self.nav.real(), 'R', round(-off, 1), need, unit=0.1 if self.caps.get('rdec') else 1.0)
            if v and abs(v) >= turn_min_deg(self.caps):
                self._turn(v, self.nav.est[2], '(修车头)', toward_zero=True)
        f, s = self.nav.owed()
        side = ('F', f) if nxt[0] == 'S' else ('S', s)       # 和下一条直行垂直的那个方向
        if abs(side[1]) >= float(g('reloc_lateral_tol_mm', 15)):
            c, v = side[0], int(round(side[1]))
            v2 = clamp_move(self.cfg, obs, self.nav.real(), c, v, need)
            if abs(v2) >= STOP_MIN_MM:
                self._send(c, v2, self.fine, '(转弯后定位修正)')
                self.n_fix[2] += 1
                self.nav.moved(c, v2)

    def _approach_cut(self, cmd, want):
        """朝场地边线/黄区/工位区/转盘/障碍物走的一条，按推算走满时离它很近：距离差 1~2%(长横移更多)就会压上。
        先少走一点(终点至少留 approach_guard_frac(横移 approach_guard_frac_s) × 距离 + approach_guard_mm
        + 上次定位以后可能已经差的 approach_guard_travel_frac × 走过的距离)，到停车点用雷达定位后再慢速修过去。
        走进启停区的最后一条(场地角上，车身离边线只有几毫米)另外先少走 home_short_mm。返回 (少走多少 mm, 日志)。"""
        g = self.cfg.get
        model = body_model(self.cfg)
        obs = self._obstacles()
        regs = regions(self.cfg, obs)
        p = self.nav.real()
        end = apply_move(p, cmd, want)
        c0, _ = pose_clearance(self.cfg, obs, p, model, regs)
        c1, what = pose_clearance(self.cfg, obs, end, model, regs)
        frac = float(g('approach_guard_frac_s', 0.03)) if cmd == 'S' else float(g('approach_guard_frac', 0.02))   # 麦轮横移误差大一些
        guard = frac * abs(want) + float(g('approach_guard_mm', 10)) + self._uncert()     # 加上上次定位以后已经可能差的
        cut, label = 0.0, ''
        if c1 < c0 - 1.0 and c1 < guard:                  # 只管"越走越近"的(沿着边线/区域平行走的不管)
            cut = guard - max(c1, 0.0) if c1 >= 0 else guard
            label = f'(终点离{what}只有 {c1:.0f}mm：先少走 {cut:.0f}mm，到停车点再慢速修)'
        if self.final_leg:
            e0, e1 = edge_clearance(p, model), edge_clearance(end, model)
            if e1 < e0 - 1.0 and e1 < self.home_short + 10 and self.home_short > cut:
                cut = self.home_short
                label = f'(进启停区先少走 {self.home_short:.0f}mm，到了再慢速修进去)'
        return min(cut, abs(want)), label

    # ------------------------------------------------------------ 停车点
    def _stop(self, stop, info):
        self.nav.sync_rest(info.get('rest'))
        g = info.get('goal')
        if self.nav.world and g is not None:
            d = math.hypot(g[0] - self.nav.ref[0], g[1] - self.nav.ref[1])
            if d > 30:
                self.log(f'    (路线推算的停车点和规划的差 {d:.0f}mm，按规划的停车点算)')
            self.nav.ref = tuple(float(v) for v in g[:3])
        up = str(stop).upper()
        is_start = up.startswith('START')
        is_pre = up.startswith('PREHOME')
        role = _role(stop)
        self.log(f'  ★ 到达 {stop}   (已用 {time.monotonic()-self.t_start:.1f} 秒)')
        goal = self.nav.ref
        if is_start:
            ap = self.approach or {}
            done = ap.get('status') == 'ok' or (ap.get('status') == 'near' and ap.get('pose') is not None
                                                 and in_rect(ap['pose'], start_rect(goal, self.cfg)))
            if not done:                # 回家前定位'near'但还在启停区外(修正被限幅了)：到这里再对准一次
                t_h = time.monotonic()
                if self.nav.world and hasattr(self.ctx, 'relocalize'):
                    res = home_approach(self.ctx, self.link, self.log, self.nav, goal, self.cfg, self.caps, self._obstacles(),
                                        aborted=self.ctx.aborted, e_last=self.e_last)
                    self.log(f"    回家对准：{res['status']}，用时 {time.monotonic()-t_h:.1f} 秒")
                elif hasattr(self.ctx, 'home_fix'):
                    self.ctx.home_fix(self.link, self.log)     # 旧 ctx：回到启停区，和出发时的扫描对齐后小步修回去
                    self.log(f'    回家对准用时 {time.monotonic()-t_h:.1f} 秒')
        else:
            meas = self._measure(stop)
            self._correct(stop, meas, is_pre)
            self.leg_cuts = []
            nxt = self._next_marker(self.k)
            if nxt is not None and str(self.seq[nxt][0]).upper().startswith('START') and not self.cut:
                self.final_leg = True
        if self.hooks is not None and hasattr(self.hooks, 'adjust'):
            self.hooks.adjust(stop, self.link, self.log)        # 以后接摄像头：看地面标记
        if is_pre:
            pass                                                # PREHOME 只是回家前定位用的点，没有任务
        elif self.homing and not is_start and role is not None:
            self.log(f'    时间不够，{stop} 不做任务，直接往回走')
        elif self.hooks is not None and hasattr(self.hooks, 'task'):
            self.hooks.task(stop, self.link, self.log)
        elif self.stop_wait > 0:
            self.log(f'    执行任务：先停 {self.stop_wait:.1f} 秒代替')
            t_end = time.monotonic() + self.stop_wait
            while time.monotonic() < t_end:
                if self.ctx.aborted():
                    raise Abort('收到 abort，已停止发送')
                time.sleep(0.05)
        if not is_start:
            self._time_check(stop, goal)

    def _measure(self, stop):
        """停车点上雷达定位。QR 这类要再扫一次障碍物的停车点，直接用那次扫描的帧定位。"""
        up = str(stop).upper()
        if not self.nav.world:
            return None
        meas = None
        remaining = [s for s, c, _ in self.seq[self.k+1:] if c is None]
        if up in self.rescan and hasattr(self.ctx, 'station_scan') and remaining:
            t_s = time.monotonic()
            try:
                res = self.ctx.station_scan(stop, self.nav.ref, remaining, est=self.nav.real())
            except TypeError:
                res = self.ctx.station_scan(stop, self.nav.ref, remaining)
            if res and res.get('replanned'):
                if res.get('reason') != 'ROUTE OK' or not res.get('legs'):
                    raise Abort(f'{stop} 扫到新障碍物后规划不出剩下的路线：{res.get("reason")}，车停在这里')
                self._splice(res['legs'], '已换成新路线')
            meas = (res or {}).get('reloc')
            if meas is not None:
                self._count(meas)
            self.log(f'    站点扫描用时 {time.monotonic()-t_s:.1f} 秒')
        elif self.reloc_on and (self.reloc_stops is None or up in self.reloc_stops or
                                any(up.startswith(s) for s in self.reloc_stops)):
            if self.travel < self.min_travel and not up.startswith('PREHOME'):
                self.log(f'    (上次定位以后才走了 {self.travel:.0f}mm，不到 reloc_min_travel_mm={self.min_travel:.0f}，这里不定位)')
            else:
                meas = self._count(_reloc(self.ctx, self.nav.real(), self.reloc_frames))
        if meas is not None:
            if meas.get('ok'):
                self.travel = 0.0
                p = meas['pose']
                self.log(f"    ◆ 雷达定位：车在 ({p[0]:.0f},{p[1]:.0f}) 车头{p[2]:.1f}°，比推算的偏 "
                         f"x{meas.get('dx', 0):+.0f} y{meas.get('dy', 0):+.0f}mm 车头{meas.get('dyaw', 0):+.1f}°"
                         f"（{meas.get('inlier', 0)*100:.0f}%的点对上，残差{meas.get('rms', 0):.1f}mm，ICP {meas.get('t', 0):.2f}秒）")
            else:
                self.log(f"    ◆ 雷达定位没成功：{meas.get('why')}。按推算的位置走。")
        return meas

    def _next_axis(self):
        """离开这个停车点后第一条平移是 F 还是 S(中间先转弯就算 None：车头变了，方向对不上)。"""
        for j in range(self.k + 1, len(self.seq)):
            c = self.seq[j][1]
            if c is None or c == 'R':
                return None
            if c in ('F', 'S'):
                return c
        return None

    def _correct(self, stop, meas, is_pre):
        """按测到的车位修：车头偏了用 R 修(下一次转弯也会顺便修)；左右偏 ≥ reloc_lateral_tol_mm 用慢速 S 修；
        前后偏差并进下一条前进(下一条不是沿这个方向的，偏 ≥ reloc_pos_tol_mm 就在这里修掉)。"""
        g = self.cfg.get
        ok = bool(meas and meas.get('ok'))
        if ok:
            self.nav.measured(meas['pose'], self.e_last)
        if not self.nav.world and not ok:
            tol_s = tol_f = float(STOP_MIN_MM)
        elif ok:
            tol_s = float(g('home_fix_tol_mm', 6) if is_pre else g('reloc_lateral_tol_mm', 15))
            tol_f = float(g('home_fix_tol_mm', 6) if is_pre else g('reloc_pos_tol_mm', 15))
        else:
            tol_s = tol_f = float(STOP_MIN_MM)
        need = float(g('reloc_check_margin_mm', 10))
        obs = self._obstacles() if self.nav.world else []
        if ok:
            off = self.nav.yaw_off
            yt = float(g('home_fix_tol_deg', 1.0) if is_pre else g('reloc_yaw_tol_deg', 1.0))
            yt = max(yt if not self.caps.get('rdec') else min(yt, float(g('reloc_yaw_min_deg', 0.3))), turn_min_deg(self.caps))
            if abs(off) >= yt:
                v = -off
                if self.nav.world:
                    v = clamp_move(self.cfg, obs, self.nav.real(), 'R', round(v, 1), need, unit=0.1 if self.caps.get('rdec') else 1.0)
                if v and abs(v) >= turn_min_deg(self.caps):
                    self._turn(v, self.nav.est[2], '(修车头)', toward_zero=self.nav.world)
                elif abs(off) >= 1.0:
                    self.log(f'    ⚠ 车头偏 {off:+.1f}°，原地修会碰到东西，留到下一次转弯再修')
        f, s = self.nav.owed()
        nxt = self._next_axis()
        moves = []
        if abs(s) >= tol_s:                               # 左右(离工位/黄区的远近)：做任务前就修掉
            moves.append(('S', s))
        if abs(f) >= tol_f and not (nxt == 'F' and ok):
            moves.append(('F', f))
        if not moves:
            return
        if self.nav.world:
            model = body_model(self.cfg)
            p = self.nav.real()
            moves.sort(key=lambda cv: -pose_clearance(self.cfg, obs, apply_move(p, cv[0], cv[1]), model)[0])
        for c, v in moves:
            v = int(round(v))
            if not ok and self.nav.world and self.leg_cuts:
                ux, uy = _axis(self.nav.real()[2], c)
                sg = 1.0 if v >= 0 else -1.0
                if any(sg*(ux*wx + uy*wy) > 0.5 for wx, wy in self.leg_cuts):
                    self.log(f'    (没定位成功：刚才离边线/区域太近少走的 {c} {v:+d} 不按推算补，宁短不长)')
                    continue
            if self.nav.world:
                v2 = clamp_move(self.cfg, obs, self.nav.real(), c, v, need)
                if v2 != v:
                    self.log(f'    ⚠ {c} {v:+d} 会离东西太近，只走 {v2:+d}')
                v = v2
            if abs(v) < STOP_MIN_MM:
                continue
            self._send(c, v, self.fine, '(定位修正)' if ok else '(补到停车点)')
            self.n_fix[2] += 1
            self.nav.moved(c, v)

    # ------------------------------------------------------------ 时间
    def _home_legs(self, name, goal):
        if not hasattr(self.ctx, 'home_route'):
            return None
        try:
            return self.ctx.home_route(name, goal)
        except Exception:
            return None

    def _home_time(self, name, goal, from_k):
        fin = float(self.cfg.get('home_finish_s', 10.0))
        legs = self._home_legs(name, goal)
        if legs and self.motion is not None:
            return sum(self.motion.time_of(c, v) + (self.turn_extra if c == 'R' else 0.0) for L in legs for c, v in L['cmds']) + fin
        return leg_time(self.motion, self.seq, from_k + 1, len(self.seq), self.turn_extra) + fin

    def _time_check(self, stop, goal):
        h = self.hooks
        if h is None:
            return
        if hasattr(h, 'set_home_eta'):
            try:
                h.set_home_eta(self._home_time(stop, goal, self.k))
            except Exception:
                pass
        nxt = self._next_marker(self.k)
        if nxt is None or self.homing or not hasattr(h, 'go_home_now'):
            return
        nname = self.seq[nxt][0]
        nrole = _role(nname)
        if nrole is None or nrole == 'START':
            return
        est_leg = leg_time(self.motion, self.seq, self.k + 1, nxt, self.turn_extra) + float(self.cfg.get('stop_overhead_s', 3.0))
        ngoal = _marker_goal(self.seq, nxt, self.stops)
        est_home = self._home_time(nname, ngoal, nxt)
        try:
            go = bool(h.go_home_now(nrole, est_leg, est_home))
        except Abort:
            raise
        except Exception as e:
            self.log(f'    (go_home_now 出错：{e!r}，照原路线走)')
            return
        if not go:
            return
        legs = self._home_legs(stop, goal)
        if legs:
            self.log(f'  ⏱ 时间不够做 {nname} 了：从 {stop} 直接回启停区')
            self._splice(legs, '回家路线')
        else:
            self.log(f'  ⏱ 时间不够做 {nname} 了：没有预先算好的回家路线，照原路线开回去，路上不再做任务')
        self.homing = True

    # ------------------------------------------------------------ 回家前定位(没有 PREHOME 时)
    def _cut_approach(self):
        p, g = self.cut['pose'], self.cut['goal']
        self.log(f'  ◆ 离启停区还有约 {math.hypot(g[0]-p[0], g[1]-p[1]):.0f}mm：停下来用雷达定位，按实际位置走进去（不照原路线最后几步走）')
        res = home_approach(self.ctx, self.link, self.log, self.nav, g, self.cfg, self.caps, self._obstacles(),
                            max_move=self.cut['max_move'], order=self.cut['order'], label='回家前定位', aborted=self.ctx.aborted,
                            e_last=self.e_last)
        self.approach = res
        st = res['status']
        if st in ('fail_unmoved', 'off'):
            self.log('    定位没成功，车没动过：按原路线剩下的几步走，到了再对准一次。')
            self.approach = None
            self.cut = None
            self.final_leg = True
            return False
        if st == 'fail_moved':
            self.log('    ★ 修过以后定位失败：不再照原路线走(原路线是按修之前的位置算的，会冲出去)。')
        return True


def drive(ctx, link, seq, log=print, stop_wait=3.0, hooks=None, speeds=None, motion=None, cfg=None, start=None, caps=None):
    """逐条发给 STM32，等 DONE。任何一条失败就停下，不再往下发。
    到每个停车点：先用雷达定位并修正(没有雷达定位时补到停车点)，再调姿态(hooks.adjust)，再执行任务(hooks.task)。
    v14：到 rescan_stops 里的停车点(默认 QR)再扫一次，发现新障碍物就从这里重新规划剩下的路线；
         回启停区那一段，离启停区还有 home_approach_mm 时先用雷达定位，再按实际位置走进去(路线里有 PREHOME 时不用)。
    1010：start = ctx.route_start()(世界坐标的起点)；caps = firmware_caps(link)。"""
    if hooks is not None:
        try:
            hooks.ctx = ctx                          # 机械臂任务钩子要用 ctx.aborted() 检查 abort
        except Exception:
            pass
    _Drive(ctx, link, seq, log, stop_wait, hooks, speeds, motion, cfg, start, caps).run()


def print_seq(seq, log):
    """把要发的指令按停车点列出来(代替以前的"输入 y 确认")。"""
    names = {'F': ('前进', '后退'), 'S': ('左移', '右移')}
    cur = []
    for stop, c, v in seq:
        if c is None:
            log(f'  到 {stop}：' + ('，'.join(cur) if cur else '原地'))
            cur = []
        elif c == 'R':
            cur.append(('逆时针转' if v > 0 else '顺时针转') + f'{abs(v)}°')
        else:
            cur.append(names[c][0 if v > 0 else 1] + f'{abs(v)}mm')


def run_mission(ctx, link, log=print, first_scan=True, stop_wait=3.0, settle_s=0.8, hooks=None, cfg=None, scanned=False):
    """first_scan=True: 完整流程(scanned=True：第一次扫描开跑前已经做好了，屏幕选区一键启动时)；False: 只按当前已规划的路线行驶。
    1010：开跑(输入 go / 屏上按 START)以后不再问任何问题(规则：开始后不能碰车和电脑)；配置 confirm_route=true 时才要输入 y(只在桌上调试用)。"""
    cfg = cfg or {}
    if first_scan:
        _call(hooks, 'start_clock')                 # 比赛计时从这里开始
    tick = make_tick(hooks, cfg)
    try:
        ctx.tick = tick                            # ctx 等扫描、规划时调用(手臂轻摆，避开 15 秒规则)
    except Exception:
        pass
    if first_scan:
        ok, reply = link.ping()
        if not ok:
            raise Abort(f'STM32 没有回应 PING：{reply}（检查 USART3 接线，和 STM32 程序是不是新版）')
        log('STM32 通信正常。')
        bad = link.sync_params(cfg.get('stm32_params'), log=log)
        if cfg.get('stm32_params'):
            log(f'已把配置里的 {len(cfg["stm32_params"])} 个运动参数发给 STM32' + (f'（{len(bad)} 个失败）' if bad else ''))
        if hooks is not None and hasattr(hooks, 'prepare'):
            try:
                hooks.prepare(link, log)           # 机械臂初始化、收臂、清掉旧任务码、打开摄像头
            except Abort:
                raise
            except Exception as e:
                log(f'  (机械臂准备出错：{e!r}，继续)')
        ok, reply = link.home()
        if not ok:
            raise Abort(f'STM32 不认识 HOME：{reply}（STM32 程序是不是新版）')
        if not scanned:
            log('① 第一次扫描：车保持不动……')
            ctx.scan()
        else:
            log('① 第一次扫描开跑前已经做好了。')
        rel = cfg.get('second_relative_moves', DEFAULT_SECOND_MOVES)
        log(f'② 走第二站：左移{rel.get("left_mm", 0)}、前进{rel.get("forward_mm", 0)}、顺时针转{rel.get("cw_deg", 0)}°（绕车中心）……')
        ok, reply = link.second_move(None if cfg.get('second_use_p2', False) else rel, log=log)
        if not ok:
            raise Abort(f'STM32 第二站移动失败：{reply}' + (f'。{STALL_HINT}' if 'STALL' in str(reply) else ''))
        err = parse_done(reply).get('err')
        if err is not None and abs(err) > MAX_HEAD_ERR_DEG:
            raise Abort(f'第二站没转到位(车头还差 {err:+.1f}°)，接着扫描会把地图建错，所以停下。{STALL_HINT}')
        time.sleep(settle_s)
        log('③ 第二次扫描：车保持不动……')
        ctx.second()
    caps = firmware_caps(link, cfg)
    turn_back, legs, reason = ctx.plan()
    if reason != 'ROUTE OK' or not legs:
        raise Abort(f'没有可用路线：{reason}')
    seq = flatten(turn_back, legs)
    n, tot = summarize(seq)
    motion = Motion(cfg)
    est = motion.route_time([(c, v) for _, c, v in seq if c])
    log(f'④ 路线规划完成：{n} 条指令，前进/后退共 {tot["F"]:.0f} mm，横移 {tot["S"]:.0f} mm，转弯累计 {tot["R"]:.0f}°，预计行驶 {est:.0f} 秒'
        + ('；STM32 支持 0.1° 转角' if caps.get('rdec') else ''))
    if hasattr(ctx, 'print_route'):
        ctx.print_route()                          # 开跑前把路线列出来(不再等人输入 y)
    else:
        print_seq(seq, log)
    if not first_scan:
        bad = link.sync_params(cfg.get('stm32_params'), log=log)
    if cfg.get('confirm_route', False) and hasattr(ctx, 'ask'):
        if not ctx.ask('确认车周围没人、场地清空后，输入 y 开始行驶；输入 n 取消。(比赛时把配置 confirm_route 关掉)'):
            raise Abort('已取消，没有发任何运动指令')
    start = None
    if hasattr(ctx, 'route_start'):
        try:
            start = ctx.route_start()
        except Exception as e:
            log(f'  (没拿到起点位置：{e!r}，只按指令推算)')
    log('⑤ 开始行驶（要紧急停车：输入 abort 立刻停车；或直接断电）')
    drive(ctx, link, seq, log=log, stop_wait=stop_wait, hooks=hooks, speeds=move_speeds(cfg), motion=motion, cfg=cfg,
          start=start, caps=caps)
    log('✔ 路线全部走完。')


# ================================================================ 启停区 1 / 2
ZONE_KEYS = ('start_zone', 'car_x_mm', 'car_y_mm', 'car_yaw_deg', 'second_relative_moves', 'second_pose_mm', 'second_use_p2')
ZONE_STOPS = ('START1', 'START2', 'PREHOME')


def zone_pose(p, zfrom, zto):
    """启停区 1 = 启停区 2 绕场地中心逆时针转 90°：(x, y, 车头) -> (2400 - y, x, 车头 + 90)。"""
    x, y = float(p[0]), float(p[1])
    h = float(p[2]) if len(p) > 2 else 0.0
    zfrom, zto = int(zfrom), int(zto)
    if zfrom == zto:
        return (x, y, wrap180(h))
    if zfrom == 2 and zto == 1:
        return (FIELD_MM - y, x, wrap180(h + 90))
    return (y, FIELD_MM - x, wrap180(h - 90))


def zone_snapshot(raw):
    """配置文件里和启停区有关的值(原样)。切换启停区后写回文件时用它恢复，文件里的内容不变。"""
    snap = {k: copy.deepcopy(raw[k]) for k in ZONE_KEYS if k in raw}
    st = raw.get('stops') or {}
    snap['stops'] = {k: copy.deepcopy(st[k]) for k in ZONE_STOPS if k in st}
    return snap


def zone_restore(out, snap):
    for k in ZONE_KEYS:
        if k in snap:
            out[k] = copy.deepcopy(snap[k])
        else:
            out.pop(k, None)
    st = out.get('stops')
    if isinstance(st, dict):
        for k in ZONE_STOPS:
            if k in snap.get('stops', {}):
                st[k] = copy.deepcopy(snap['stops'][k])
            else:
                st.pop(k, None)
    return out


def _r(p):
    return [round(float(v), 1) for v in p]


def zone_settings(snap, zone, overrides=None, prehome_mm=250.0):
    """按配置文件(snap，写的是 start_zone 那个区)推出 zone 区要用的值：
    起点车位、START 停车点(车头和起点一样)、PREHOME(启停区外 prehome_mm、同一条直线上)、第二站动作。
    另一个区 = 绕场地中心转 90°(第二站动作原样用)；配置里 "zones": {"1": {...}, "2": {...}} 可以逐项指定。"""
    bz = int(snap.get('start_zone', 2) or 2)
    z = int(zone)
    if z not in (1, 2):
        raise ValueError('启停区只能是 1 或 2')
    st = snap.get('stops') or {}
    car0 = (float(snap.get('car_x_mm', 2250)), float(snap.get('car_y_mm', 150)), float(snap.get('car_yaw_deg', 90)))
    car = zone_pose(car0, bz, z)
    sname = f'START{z}'
    bname = f'START{bz}'
    start = zone_pose(st[bname], bz, z) if bname in st else car
    pre = zone_pose(st['PREHOME'], bz, z) if 'PREHOME' in st else None
    sp = snap.get('second_pose_mm')
    out = dict(zone=z, car=car, start_name=sname, start_stop=start, prehome=pre,
               rel=dict(snap.get('second_relative_moves') or DEFAULT_SECOND_MOVES),
               second_pose=zone_pose(sp, bz, z) if sp else None,
               use_p2=bool(snap.get('second_use_p2', False)) if (z == 2 and bz == 2) else False,
               prehome_mm=float(prehome_mm))
    ov = (overrides or {}).get(str(z)) or (overrides or {}).get(z) or {}
    if ov:
        if any(k in ov for k in ('car_x_mm', 'car_y_mm', 'car_yaw_deg')):
            out['car'] = (float(ov.get('car_x_mm', out['car'][0])), float(ov.get('car_y_mm', out['car'][1])),
                          float(ov.get('car_yaw_deg', out['car'][2])))
            if sname not in (ov.get('stops') or {}):
                out['start_stop'] = out['car']
        if 'second_relative_moves' in ov:
            out['rel'] = dict(ov['second_relative_moves'])
        if 'second_pose_mm' in ov:
            out['second_pose'] = tuple(ov['second_pose_mm']) if ov['second_pose_mm'] else None
        if 'second_use_p2' in ov:
            out['use_p2'] = bool(ov['second_use_p2'])
        if 'prehome_mm' in ov:
            out['prehome_mm'] = float(ov['prehome_mm'])
            out['prehome'] = None
        os_ = ov.get('stops') or {}
        if sname in os_:
            out['start_stop'] = tuple(float(v) for v in os_[sname][:3])
        if 'PREHOME' in os_:
            out['prehome'] = tuple(float(v) for v in os_['PREHOME'][:3]) if os_['PREHOME'] else None
            out['prehome_mm'] = 0.0 if not os_['PREHOME'] else out['prehome_mm']
    if out['prehome'] is None and out['prehome_mm'] > 0:
        x, y, h = out['start_stop']
        ux, uy = _axis(h, 'F')
        out['prehome'] = (round(x + out['prehome_mm']*ux, 1), round(y + out['prehome_mm']*uy, 1), h)
    return out


def apply_zone(raw, zone, snap):
    """切换启停区：一次改完所有和启停区有关的(原地改 raw：起点车位、start_zone、第二站动作(同一个 dict 原地改)、
    START 停车点、PREHOME、second_pose_mm、second_use_p2)。返回 zone_settings 的结果。"""
    s = zone_settings(snap, zone, raw.get('zones'), float(raw.get('prehome_mm', 250)))
    raw['start_zone'] = s['zone']
    raw['car_x_mm'], raw['car_y_mm'], raw['car_yaw_deg'] = s['car']
    rel = raw.get('second_relative_moves')
    if isinstance(rel, dict):
        rel.clear()
        rel.update(s['rel'])
    else:
        raw['second_relative_moves'] = dict(s['rel'])
    if s['second_pose']:
        raw['second_pose_mm'] = _r(s['second_pose'])
    else:
        raw.pop('second_pose_mm', None)
    raw['second_use_p2'] = s['use_p2']
    st = raw.setdefault('stops', {})
    st[s['start_name']] = _r(s['start_stop'])
    if s['prehome']:
        st['PREHOME'] = _r(s['prehome'])
    else:
        st.pop('PREHOME', None)
    return s


def mission_names(raw):
    """配置里的 mission，START 换成本区的 START1/START2，最后回家前加 PREHOME(配置里有 PREHOME 停车点时)。"""
    z = int(raw.get('start_zone', 2) or 2)
    names = [f'START{z}' if s == 'START' else s for s in raw.get('mission', [])]
    stops = raw.get('stops') or {}
    if 'PREHOME' in stops and names and str(names[-1]).upper().startswith('START') and 'PREHOME' not in names:
        names = names[:-1] + ['PREHOME', names[-1]]
    return names


# ================================================================ 屏幕选区一键启动(race)
_ZONE_RE = re.compile(r'ZONE (\d) (\d)')


def zone_request(link, text, timeout=3.0):
    """发一条 ZONE 指令。返回 (ok, 回复, 信息行)。旧固件回 ERR CMD。"""
    if link is None or not hasattr(link, 'request'):
        return False, '没有 request', []
    try:
        out = link.request(text, timeout, collect=True)
    except Exception as e:
        return False, repr(e), []
    out = list(out) + [None, None, None]
    return bool(out[0]), str(out[1] or ''), list(out[2] or [])


def zone_query(link):
    """ZONE? -> (选的区 0/1/2, 按了 START 没有 0/1)；问不到返回 None。"""
    ok, rep, info = zone_request(link, 'ZONE?')
    if not ok:
        return None
    for line in list(info) + [rep]:
        m = _ZONE_RE.search(str(line))
        if m:
            return int(m.group(1)), int(m.group(2))
    return None


def zone_msg(link, text):
    t = str(text).encode('ascii', 'replace').decode('ascii').replace('"', "'")[:20]
    return zone_request(link, 'ZONE MSG ' + t)[0]


def race_flow(ctx, link, log, cfg, start_run, now=time.monotonic, sleep=time.sleep):
    """比赛流程(一键启动，开始后不再碰车和电脑)：
    1 发 ZONE ASK：屏上显示"选启停区 1 / 2"；
    2 一直问 ZONE?：选了区就切换到那个区，等车放好、不动了(陀螺仪 ~2 秒稳定)，先做第一次扫描并预先规划，
      屏上状态行显示 SCAN / PLAN / READY / ERR…；车又被挪动(陀螺仪变了 >2° 或雷达看到挪了)或者按了 BACK 重新选，就重新扫描；
    3 屏上按 START：ZONE LOCK，start_run(区, 扫过没有) 开跑，之后不再问任何问题。
    固件不认识 ZONE(回 ERR)：返回 'nozone'，用配置里的 start_zone 和终端的 go 启动。"""
    g = (cfg or {}).get
    poll_s = float(g('race_poll_s', 0.3))
    still_s = float(g('race_still_s', 2.0))
    still_deg = float(g('race_still_deg', 0.3))
    move_deg = float(g('race_move_deg', 2.0))
    check_s = float(g('race_check_s', 5.0))
    ok, rep, _ = zone_request(link, 'ZONE ASK')
    if not ok:
        log(f'STM32 不认识 ZONE 指令({rep})：屏幕不能选启停区。用配置里的 start_zone={g("start_zone", 2)}，'
            '终端输入 zone 1 / zone 2 选区，再输入 go 开始。')
        return 'nozone'
    log('屏幕上选启停区(1 或 2)，把车放好，等屏上显示 READY 后按 START。终端输入 abort 取消。')
    zone = None
    scanned = False
    yaws = []
    t_zone = now()
    yaw_ref = None
    last_check = now()
    plan_seen = None
    last_msg = [None]
    fails = 0

    def msg(t):
        if t != last_msg[0]:
            last_msg[0] = t
            zone_msg(link, t)

    while True:
        if ctx.aborted():
            raise Abort('收到 abort，比赛流程取消(车没动过)')
        q = zone_query(link)
        if q is None:
            fails += 1
            if fails in (10, 100):
                log('  (问不到 ZONE?，STM32 连着吗？)')
            sleep(poll_s)
            continue
        fails = 0
        z, s = q
        if z == 0:
            if zone is not None:
                log('  屏上回到了选区页面：重新选启停区。')
                zone = None
                scanned = False
            sleep(poll_s)
            continue
        if z != zone:
            log(f'  屏上选了启停区 {z}。把车放好别动……')
            ctx.zone_select(z)
            zone = z
            scanned = False
            yaws = []
            t_zone = now()
            plan_seen = None
            msg('PLACE CAR')
        if s == 1:
            log(f'★ 屏上按了 START：启停区 {zone}，开跑！')
            if not scanned:
                log('  (还没扫描就按了 START：开跑后马上扫描)')
            zone_request(link, 'ZONE LOCK')
            start_run(zone, scanned)
            return 'started'
        y = ctx.gyro_yaw() if hasattr(ctx, 'gyro_yaw') else None
        t = now()
        if not scanned:
            if y is not None:
                yaws.append((t, y))
                yaws = [(a, b) for a, b in yaws if t - a <= still_s + 0.5]
                span = max(abs(wrap180(b - yaws[-1][1])) for _, b in yaws)
                still = yaws[0][0] <= t - still_s and span <= still_deg
            else:
                still = t - t_zone >= still_s
            if still:
                msg('SCAN')
                log('  车放好了：第一次扫描(车别动)……')
                try:
                    ctx.scan()
                except Abort as e:
                    if ctx.aborted():
                        raise
                    # 扫描失败(雷达超时等)：开跑前还能重来，不要整个 race 退出(退出了就得再碰电脑)
                    log(f'  第一次扫描失败：{e}。过一会儿重新扫描。')
                    msg('SCAN ERR')
                    ctx.zone_select(zone)
                    yaws = []
                    t_zone = now()
                    sleep(poll_s)
                    continue
                scanned = True
                yaw_ref = ctx.gyro_yaw() if hasattr(ctx, 'gyro_yaw') else None
                msg('PLAN')
                if hasattr(ctx, 'preplan_start'):
                    ctx.preplan_start()
                last_check = now()
            sleep(poll_s)
            continue
        st = ctx.preplan_state() if hasattr(ctx, 'preplan_state') else 'ok'
        if st != plan_seen and st not in (None, 'run'):
            plan_seen = st
            if st == 'ok':
                msg(f'READY Z{zone}')
                log('  预先规划好了：屏上 READY，可以按 START。')
            else:
                msg('PLAN ERR')
                log(f'  预先规划失败：{st}(按 START 以后第二次扫描完再规划一次)')
        moved = None
        if y is not None and yaw_ref is not None and abs(wrap180(y - yaw_ref)) > move_deg:
            moved = f'陀螺仪变了 {wrap180(y - yaw_ref):+.1f}°'
        elif check_s > 0 and now() - last_check >= check_s and hasattr(ctx, 'still_check'):
            last_check = now()
            r = ctx.still_check()
            if r and r.get('moved'):
                moved = r.get('why') or '雷达看到车挪了'
        if moved:
            log(f'  车被挪动了({moved})：重新扫描。')
            msg('MOVED')
            ctx.zone_select(zone)
            scanned = False
            yaws = []
            t_zone = now()
            plan_seen = None
        sleep(poll_s)


# ================================================================ 待机：收臂、升降停 60mm
def park_arm(link, hooks=None, log=print):
    """收臂并把升降停到 60mm(开机自动认高度要求升降在 60 附近)。有 hooks.park 用它；
    否则发 PARK，旧固件(ERR CMD)改成 STOW + LIFT 60。尽力而为：失败只记日志。"""
    if hooks is not None and hasattr(hooks, 'park'):
        try:
            hooks.park()
            return True
        except Exception as e:
            log(f'  收臂停 60mm 出错：{e!r}')
            return False
    if link is None or not hasattr(link, 'request'):
        log('  没有连接 STM32，不能收臂停 60mm')
        return False
    ok, rep, _ = zone_request(link, 'PARK', 40.0)
    if ok:
        log('  已收臂，升降停在 60mm。')
        return True
    if 'NOZERO' in rep:
        log(f'  升降位置不知道(PARK -> {rep})：不自动降。把升降放到最低点输入 arm LIFT ZERO 后再 park。')
        return False
    if 'CMD' not in rep:
        log(f'  PARK 失败：{rep}')
        return False
    ok, rep, _ = zone_request(link, 'STOW', 40.0)       # 旧固件没有 PARK
    if not ok and 'NOCAL' not in rep:
        log(f'  收臂失败(STOW -> {rep})：不降升降。')
        return False
    ok, rep, _ = zone_request(link, 'LIFT 60', 25.0)
    if ok:
        log('  已收臂，升降停在 60mm(旧固件：STOW + LIFT 60)。')
        return True
    log(f'  升降停 60mm 失败：{rep}')
    return False


def quit_park(link, hooks, log, running, aborted, enabled=True):
    """退出程序(q、Ctrl+C、关窗口)时：没在跑、没急停过才收臂停 60mm。急停过不自动动(人手可能在旁边)。
    还在跑：先发急停让车停下，不收臂。返回 True/False/None(没做)。"""
    if not enabled or link is None:
        return None
    if running:
        log('退出时一键流程还在运行：先发急停让车停下，不收臂。')
        try:
            link.abort()
        except Exception:
            pass
        return False
    if aborted:
        log('急停过：退出时不自动收臂(要停 60mm：重新运行后确认车旁没人，输入 park)。')
        return False
    log('退出：收臂、升降停 60mm(下次开机能自动认出高度)……')
    return park_arm(link, hooks, log)
