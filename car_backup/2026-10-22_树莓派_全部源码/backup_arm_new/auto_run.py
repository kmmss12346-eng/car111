"""一键流程：scan -> STM32 走第二站 -> second -> 规划 -> 按路线驱动。
路线执行只靠 STM32 里的陀螺仪(车头)和电机脉冲(距离)。v14：到 QR 再扫一次障碍物；回启停区前用雷达定位一次。
ctx 由 map_merge_live.py 提供（在后台线程里调用）。"""
import time

from route_plan import Motion
from stm32_link import parse_done


class Abort(Exception):
    pass


MIN_MOVE_MM = 20     # 小于这个距离的前进/横移不发：车本身的精度就在这个量级，发了反而多一次起停

DEFAULT_SECOND_MOVES = {'left_mm': 42, 'forward_mm': 83, 'cw_deg': 60}


def flatten(turn_back, legs, min_move=MIN_MOVE_MM):
    """路线 -> [(停车点名或None, 指令, 数值)]。第一步先原地转回起点车头方向。"""
    seq = []
    if turn_back is not None and abs(turn_back) >= 0.5:
        seq.append((None, 'R', round(turn_back)))
    for L in legs:
        for cmd, val in L['cmds']:
            if cmd in ('F', 'S') and abs(val) < min_move:
                continue
            seq.append((None, cmd, val))
        seq.append((L['stop'], None, 0))        # 到达停车点的标记
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


import math


def _simulate(pose, cmds):
    """按路线指令推算每一步之后的车位。F 沿车头、S 向车左、R 绕车中心转(和规划一致)。"""
    x, y, yaw = pose
    out = []
    for c, v in cmds:
        a = math.radians(yaw)
        if c == 'F':
            x += v*math.cos(a); y += v*math.sin(a)
        elif c == 'S':
            x += -v*math.sin(a); y += v*math.cos(a)
        elif c == 'R':
            yaw += v
        out.append((x, y, yaw))
    return out


def home_cut(seq, cfg):
    """v14：在最后回启停区那一段里，找“离启停区还剩 home_approach_mm 左右、车头已经转好”的地方。
    车走到这里就停下，用雷达定位，再按实际位置走进启停区，不再照原路线最后几步走。
    必要时把一条长的前进/后退拆成两截。返回 (新的 seq, 信息) 或 (seq, None)。"""
    stops = (cfg or {}).get('stops') or {}
    marks = [i for i, (s, c, _) in enumerate(seq) if c is None]
    fin = [i for i in marks if str(seq[i][0]).upper().startswith('START')]
    if not fin:
        return seq, None
    m = fin[-1]
    prev = [i for i in marks if i < m]
    if not prev or seq[m][0] not in stops or seq[prev[-1]][0] not in stops:
        return seq, None
    p0 = tuple(float(v) for v in stops[seq[prev[-1]][0]][:3])
    goal = tuple(float(v) for v in stops[seq[m][0]][:3])
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
        a = math.radians(pose[2]); u = (math.cos(a), math.sin(a)) if c == 'F' else (-math.sin(a), math.cos(a))
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


def drive(ctx, link, seq, log=print, stop_wait=3.0, hooks=None, speeds=None, motion=None, cfg=None):
    """逐条发给 STM32，等 DONE。任何一条失败就停下，不再往下发。
    到每个停车点：先调姿态(hooks.adjust)，再执行任务(hooks.task)。现在 task 只是停 stop_wait 秒。
    v14：到 rescan_stops 里的停车点(默认 QR)再扫一次，发现新障碍物就从这里重新规划剩下的路线；
         回启停区那一段，离启停区还有 home_approach_mm 时先用雷达定位，再按实际位置走进去。"""
    cfg = cfg or {}
    speeds = speeds or {}
    if hooks is not None:
        try:
            hooks.ctx = ctx                          # 机械臂任务钩子要用 ctx.aborted() 检查 abort
        except Exception:
            pass
    stops = cfg.get('stops') or {}
    rescan = {str(s).upper() for s in cfg.get('rescan_stops', ['QR'])}
    use_approach = bool(cfg.get('home_approach', True)) and hasattr(ctx, 'home_approach')
    seq = list(seq)
    cut = None
    if use_approach:
        seq, cut = home_cut(seq, cfg)
    total = sum(1 for _, c, _ in seq if c)
    approached = False
    i = 0
    k = 0
    t_start = time.monotonic()
    t_move = 0.0
    while k < len(seq):
        stop, cmd, val = seq[k]
        if ctx.aborted():
            link.abort()
            raise Abort('收到 abort，已停止发送')
        if cut is not None and k == cut['index'] and not approached:
            p, g = cut['pose'], cut['goal']
            log(f'  ◆ 离启停区还有约 {math.hypot(g[0]-p[0], g[1]-p[1]):.0f}mm：停下来用雷达定位，按实际位置走进去（不照原路线最后几步走）')
            approached = ctx.home_approach(link, log, p, cut['order'], cut['max_move'])
            if approached:
                k = cut['marker']               # 跳过原路线剩下的几步，直接到“到达启停区”
                continue
            log('    定位没成功，按原路线剩下的几步走，到了再对准一次。')
            cut = None
        if cmd is None:
            log(f'  ★ 到达 {stop}   (已用 {time.monotonic()-t_start:.1f} 秒)')
            if hooks is not None and hasattr(hooks, 'adjust'):
                hooks.adjust(stop, link, log)        # 以后接摄像头：看地面标记，发 F/S/R 小动作把车摆正
            if str(stop).upper() in rescan and hasattr(ctx, 'station_scan') and stop in stops:
                remaining = [s for s, c, _ in seq[k+1:] if c is None]
                if remaining:
                    t_s = time.monotonic()
                    res = ctx.station_scan(stop, stops[stop], remaining)
                    if res and res.get('replanned'):
                        if res.get('reason') != 'ROUTE OK' or not res.get('legs'):
                            raise Abort(f'{stop} 扫到新障碍物后规划不出剩下的路线：{res.get("reason")}，车停在这里')
                        new_seq = flatten(0, res['legs'])
                        if use_approach:
                            new_seq, cut = home_cut(new_seq, cfg)
                        seq = seq[:k+1] + new_seq
                        if cut is not None:
                            cut = dict(cut, index=cut['index'] + k + 1, marker=cut['marker'] + k + 1)
                        total = i + sum(1 for _, c, _ in new_seq if c)
                        n2, tot2 = summarize(new_seq)
                        log(f'    已换成新路线：剩下 {n2} 条指令，前进/后退 {tot2["F"]}mm，横移 {tot2["S"]}mm，转弯 {tot2["R"]}°')
                    log(f'    站点扫描用时 {time.monotonic()-t_s:.1f} 秒')
            if str(stop).upper().startswith('START') and hasattr(ctx, 'home_fix') and not approached:
                t_h = time.monotonic()
                ctx.home_fix(link, log)              # v12：回到启停区，和出发时的扫描对齐后小步修回去
                log(f'    回家对准用时 {time.monotonic()-t_h:.1f} 秒')
            if hooks is not None and hasattr(hooks, 'task'):
                hooks.task(stop, link, log)           # 以后接抓取/放置/读二维码
            elif stop_wait > 0:
                log(f'    执行任务：先停 {stop_wait:.1f} 秒代替')
                t_end = time.monotonic() + stop_wait
                while time.monotonic() < t_end:
                    if ctx.aborted():
                        raise Abort('收到 abort，已停止发送')
                    time.sleep(0.05)
            k += 1
            continue
        i += 1
        est = f'，预计{motion.time_of(cmd, val):.1f}秒' if motion is not None else ''
        t0 = time.monotonic()
        ok, reply = link.move(cmd, val, speeds.get(cmd))
        dt = time.monotonic() - t0
        t_move += dt
        info = parse_done(reply)
        extra = ''
        if 'err' in info:
            extra = f'，车头误差 {info["err"]:+.2f}°'
        log(f'  [{i}/{total}] {cmd} {val:+d} -> {"完成" if ok else reply}  用时{dt:.1f}秒{est}{extra}')
        if not ok:
            raise Abort(f'第{i}条指令 {cmd} {val} 失败：{reply}')
        k += 1
    log(f'  路线行驶共用 {time.monotonic()-t_start:.1f} 秒（其中指令执行 {t_move:.1f} 秒）')


def run_mission(ctx, link, log=print, first_scan=True, stop_wait=3.0, settle_s=0.8, hooks=None, cfg=None):
    """first_scan=True: 完整流程；False: 只按当前已规划的路线行驶。"""
    cfg = cfg or {}
    if first_scan:
        ok, reply = link.ping()
        if not ok:
            raise Abort(f'STM32 没有回应 PING：{reply}（检查 USART3 接线，和 STM32 程序是不是新版）')
        log('STM32 通信正常。')
        bad = link.sync_params(cfg.get('stm32_params'), log=log)
        if cfg.get('stm32_params'):
            log(f'已把配置里的 {len(cfg["stm32_params"])} 个运动参数发给 STM32' + (f'（{len(bad)} 个失败）' if bad else ''))
        ok, reply = link.home()
        if not ok:
            raise Abort(f'STM32 不认识 HOME：{reply}（STM32 程序是不是新版）')
        log('① 第一次扫描：车保持不动……')
        ctx.scan()
        rel = cfg.get('second_relative_moves', DEFAULT_SECOND_MOVES)
        log(f'② 走第二站：左移{rel.get("left_mm", 0)}、前进{rel.get("forward_mm", 0)}、顺时针转{rel.get("cw_deg", 0)}°（绕车中心）……')
        ok, reply = link.second_move(None if cfg.get('second_use_p2', False) else rel, log=log)
        if not ok:
            raise Abort(f'STM32 第二站移动失败：{reply}')
        time.sleep(settle_s)
        log('③ 第二次扫描：车保持不动……')
        ctx.second()
    turn_back, legs, reason = ctx.plan()
    if reason != 'ROUTE OK' or not legs:
        raise Abort(f'没有可用路线：{reason}')
    seq = flatten(turn_back, legs)
    n, tot = summarize(seq)
    motion = Motion(cfg)
    est = motion.route_time([(c, v) for _, c, v in seq if c])
    log(f'④ 路线规划完成：{n} 条指令，前进/后退共 {tot["F"]} mm，横移 {tot["S"]} mm，转弯累计 {tot["R"]}°，预计行驶 {est:.0f} 秒')
    if not first_scan:
        bad = link.sync_params(cfg.get('stm32_params'), log=log)
    if not ctx.ask('确认车周围没人、场地清空后，输入 y 开始行驶；输入 n 取消。'):
        raise Abort('已取消，没有发任何运动指令')
    log('⑤ 开始行驶（要紧急停车：输入 abort 立刻停车；或直接断电）')
    drive(ctx, link, seq, log=log, stop_wait=stop_wait, hooks=hooks, speeds=move_speeds(cfg), motion=motion, cfg=cfg)
    log('✔ 路线全部走完。')
