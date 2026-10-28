"""(10-28 用户"老师要求我们交两个视频对比一下，一个是写死的，一个是自己规划的")写死路线。主程序里输入：

    fixroute          看写死模式开没开、存的是哪条路线
    fixroute save     把刚才规划好(跑过)的路线和这次的地图存下来(fixed_route.json，和配置文件放在一起)
    fixroute on       打开写死模式(存进配置)
    fixroute off      关掉，race / go 照常自己扫描、规划

写死模式 race 1 / race 2 只等屏上按 START：按了以后原地等 3 秒(不扫描；机械臂准备就在这 3 秒里做)→ 第二站动作
(左移、前进、转)→ 再等 3 秒(不扫描)→ 不规划，按存好的路线跑，二维码那里不做站点扫描。
跑的时候雷达定位照常做：和存下来的地图(存的时候那两次扫描)对位置。所以障碍物要摆在存的时候的位置，车放在同一个启停区。
存的路线是从第二站算的(和自己规划时一样)。"""
import json
import os
import shutil
import time
from pathlib import Path

FILE_NAME = 'fixed_route.json'
VERSION = 1
# 规划出来、跑的时候要用的东西(map_merge_live 的 state 里的键)，原样存、原样放回去
STATE_KEYS = ('turn_back', 'start_pivot', 'route_ref', 'plan_margin', 'plan_obs', 'sealed', 'seal_soft', 'seal_s', 'box',
              'strafe_fallback', 'lane_fallback')
_TUPLE_KEYS = ('plan_start', 'start_pivot', 'route_ref', 'box')
_LEG_TUPLES = ('start', 'goal', 'stop_pose', 'goal_shift')


def path_for(config_path):
    """写死路线文件：和配置文件放在同一个文件夹。"""
    return Path(config_path).with_name(FILE_NAME)


def snapshot(state, views, objects, zone, now=time.localtime):
    """现在的路线和地图 -> 存进文件的内容。没有两次扫描、没有规划好的全程路线就抛 ValueError(说清楚先做什么)。"""
    legs = list(state.get('legs') or [])
    if len(views) < 2:
        raise ValueError('还没有两次扫描：先正常跑一次(race 2 / go)，或者 scan → p2 → second 规划好路线，再 fixroute save')
    if not legs or state.get('reason') != 'ROUTE OK':
        raise ValueError(f'现在没有规划好的路线({state.get("reason")})：先 route 规划全程，再 fixroute save')
    if state.get('legs_partial'):
        raise ValueError('现在的路线是跑到半路重新规划的(只有后半段)：先 route 规划全程，再 fixroute save')
    if state.get('route_ref') is None:
        raise ValueError('不知道这条路线是从哪个车位规划的：先 route 规划全程，再 fixroute save')
    out = dict(version=VERSION, saved_at=time.strftime('%m-%d %H:%M', now()), zone=int(zone), legs=legs,
               views=list(views[:2]), objects=list(objects), path=list(state.get('path') or []),
               plan_start=state.get('plan_start') or legs[0].get('start'))    # 跑完 plan_start 清掉了：第一段的起点就是它
    for k in STATE_KEYS:
        out[k] = state.get(k)
    return out


def _plain(o):
    if hasattr(o, 'tolist'):                     # numpy 的数、数组
        return o.tolist()
    raise TypeError(f'{type(o).__name__} 存不进 JSON')


def save(path, snap):
    """存进 path(先写临时文件再改名，存到一半断电也不会坏)；以前存的留一份 .bak。"""
    p = Path(path)
    data = json.dumps(snap, ensure_ascii=False, separators=(',', ':'), default=_plain)
    tmp = str(p) + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    if p.exists():
        shutil.copyfile(p, p.with_name(p.name + '.bak'))
    os.replace(tmp, p)


def load(path):
    """读写死路线。没存过、读不出来抛 ValueError(说清楚怎么办)。列表换回元组(和规划出来的一样)。"""
    p = Path(path)
    if not p.exists():
        raise ValueError(f'还没存写死路线({p.name} 不存在)：先正常跑一次(race 2)，跑完输入 fixroute save')
    try:
        snap = json.loads(p.read_text(encoding='utf-8'))
    except (OSError, ValueError) as e:
        raise ValueError(f'{p.name} 读不出来({e})：重新跑一次再 fixroute save') from None
    if not isinstance(snap, dict) or snap.get('version') != VERSION or not snap.get('legs') or len(snap.get('views') or []) < 2:
        raise ValueError(f'{p.name} 不是这一版的写死路线：重新跑一次再 fixroute save')
    for L in snap['legs']:
        L['cmds'] = [(str(c), v) for c, v in L.get('cmds') or []]
        for k in _LEG_TUPLES:
            if L.get(k) is not None:
                L[k] = tuple(L[k])
        if L.get('poses') is not None:
            L['poses'] = [tuple(q) for q in L['poses']]
    for k in _TUPLE_KEYS:
        if snap.get(k) is not None:
            snap[k] = tuple(snap[k])
    for k in ('path', 'plan_obs', 'sealed'):
        snap[k] = [tuple(q) for q in snap.get(k) or []]
    return snap


def check(snap, zone):
    """这条写死路线现在能不能跑：启停区要和存的时候一样。不能跑返回原因，能跑返回 None。"""
    z = int(snap.get('zone') or 0)
    if z != int(zone):
        return (f'写死路线是启停区 {z} 的，现在是启停区 {int(zone)}：用 race {z}，'
                f'或者在启停区 {int(zone)} 正常跑一次再 fixroute save')
    return None


def describe(snap):
    """一句话说存的是哪条路线。"""
    legs = snap.get('legs') or []
    t = sum(float(L.get('time_s') or 0.0) for L in legs)
    return (f'启停区 {snap.get("zone")}，{snap.get("saved_at") or "?"} 存的，{len(legs)} 站 ' + '→'.join(str(L.get('stop')) for L in legs)
            + f'，障碍物 {len(snap.get("objects") or [])} 个' + (f'，预计行驶 {t:.0f} 秒' if t > 0 else ''))
