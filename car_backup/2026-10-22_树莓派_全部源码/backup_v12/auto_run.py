"""一键流程：scan -> STM32 走第二站 -> second -> 规划 -> 按路线驱动。
路线执行只靠 STM32 里的陀螺仪(车头)和电机脉冲(距离)，树莓派不做任何雷达位置修正。
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


def drive(ctx, link, seq, log=print, stop_wait=3.0, hooks=None, speeds=None, motion=None):
    """逐条发给 STM32，等 DONE。任何一条失败就停下，不再往下发。
    到每个停车点：先调姿态(hooks.adjust)，再执行任务(hooks.task)。现在 task 只是停 stop_wait 秒。"""
    speeds = speeds or {}
    total = sum(1 for _, c, _ in seq if c)
    i = 0
    t_start = time.monotonic()
    t_move = 0.0
    for stop, cmd, val in seq:
        if ctx.aborted():
            link.abort()
            raise Abort('收到 abort，已停止发送')
        if cmd is None:
            log(f'  ★ 到达 {stop}   (已用 {time.monotonic()-t_start:.1f} 秒)')
            if hooks is not None and hasattr(hooks, 'adjust'):
                hooks.adjust(stop, link, log)        # 以后接摄像头：看地面标记，发 F/S/R 小动作把车摆正
            if hooks is not None and hasattr(hooks, 'task'):
                hooks.task(stop, link, log)           # 以后接抓取/放置/读二维码
            elif stop_wait > 0:
                log(f'    执行任务：先停 {stop_wait:.1f} 秒代替')
                t_end = time.monotonic() + stop_wait
                while time.monotonic() < t_end:
                    if ctx.aborted():
                        raise Abort('收到 abort，已停止发送')
                    time.sleep(0.05)
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
    drive(ctx, link, seq, log=log, stop_wait=stop_wait, hooks=hooks, speeds=move_speeds(cfg), motion=motion)
    log('✔ 路线全部走完。')
