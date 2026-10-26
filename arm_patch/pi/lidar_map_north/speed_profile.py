"""(10-26 用户"整个车运动和转向都太慢了帮我加速……尤其是转向的时候")底盘一键快慢，主程序里输入：

    speed            看现在是哪一档、各参数多少
    speed fast       提速：直行/横移快一点、起步刹车猛一点；转弯最后那几度不再慢慢爬
    speed max        再快一档(直行/横移就是以前 220/170 转/分的速度)；车头偏、停不准就退回 fast
    speed normal     还原成第一次 speed fast/max 之前的样子(原来的值存在配置的 speed_saved 里)
    turntune         车原地转几次 ±90°，自动选转得最快、又停得准的 TCLAG(车周围留出 40cm 空地)

转弯为什么慢：STM32 转弯最后几度按"差几度给几转/分"(TCKP)靠上去，TCKP=3 是按 PPD=25.4(原地转 1° 要的脉冲)调的。
车上标定以后 PPD=47，同样的 TCKP 车身只转一半快：底盘仿真(换成 PPD 47)里 R 90 最后 20° 要爬 1.5 秒，
整个 R 90 用 2.6~3.1 秒，和实车日志一样(R +90 用 2.5 秒、R +60 用 3.2 秒)。fast/max 按现在的 PPD 换算 TCKP(PPD 47 → 6)，
最高转速、加减速提一点：仿真里 R 90 平均 2.6 → 2.0 秒，小角度修车头 1.0 → 0.7 秒，停下误差都在 0.5° 以内。
转得最快的 TCLAG(车身跟上轮子要多久)要看实车：turntune 在实车上试几个值，选最快又准的存进配置。

改的东西都存进配置(树莓派每次运行把 stm32_params 发给 STM32)，第一次改之前把原来的值存在 speed_saved 里。"""
import math

from stm32_link import parse_done

LEVELS = ('fast', 'max')

# 每一档：树莓派配置里的直行/横移速度、用来估时的转弯秒数；STM32 的加减速(直行 SCACC 起步、SCDEC 刹车，横移 SSACC)、
# 转弯最高速 TCV、起步 TCA、刹车 TCD、最低转速 TCMIN(TCKP 按 PPD 换算，见 tckp_for)
PRESETS = {
    'fast': ({'route_speed_rpm': 190, 'strafe_speed_rpm': 160, 'turn90_s': 2.0, 'turn180_s': 2.8},
             {'SCACC': 150.0, 'SCDEC': 110.0, 'SSACC': 180.0, 'TCV': 120.0, 'TCA': 600.0, 'TCD': 300.0, 'TCMIN': 2.0}),
    'max': ({'route_speed_rpm': 220, 'strafe_speed_rpm': 170, 'turn90_s': 1.9, 'turn180_s': 2.6},
            {'SCACC': 200.0, 'SCDEC': 150.0, 'SSACC': 200.0, 'TCV': 150.0, 'TCA': 800.0, 'TCD': 300.0, 'TCMIN': 2.0}),
}
CFG_KEYS = ('route_speed_rpm', 'strafe_speed_rpm', 'turn90_s', 'turn180_s')
STM_KEYS = ('SCACC', 'SCDEC', 'SSACC', 'TCV', 'TCA', 'TCD', 'TCKP', 'TCMIN')
# STM32 程序里编译进去的默认值(配置里没存、也读不到 STM32 现在的值时，还原就按它)
FW_DEFAULTS = {'SCACC': 300.0, 'SCDEC': 0.0, 'SSACC': 200.0, 'TCV': 80.0, 'TCA': 400.0, 'TCD': 250.0, 'TCKP': 3.0, 'TCMIN': 1.0,
               'TCLAG': 0.16, 'PPD': 25.4}
# 代码里的默认值(配置里没写时)
CFG_DEFAULTS = {'route_speed_rpm': 220, 'strafe_speed_rpm': 170, 'turn90_s': 1.8, 'turn180_s': 2.5}


def tckp_for(ppd):
    """转弯末段的 TCKP(转/分 每度)：车身"差 1° 转多快"保持 TCKP=3、PPD=25.4 时那么快(约 6.8 度/秒 每度)。
    PPD 47 → 6.0，PPD 25.4 → 3.25。限在 3~8。"""
    try:
        p = float(ppd)
    except (TypeError, ValueError):
        p = FW_DEFAULTS['PPD']
    if not p > 0:
        p = FW_DEFAULTS['PPD']
    return round(min(8.0, max(3.0, 0.128 * p)), 1)


def level(raw_cfg):
    """现在是哪一档：没用过 speed fast/max(或已经 speed normal)是 'normal'。"""
    return str((raw_cfg.get('speed_saved') or {}).get('level') or 'normal')


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _ppd(raw_cfg, now):
    sp = raw_cfg.get('stm32_params') or {}
    for src in (now or {}, sp):
        v = _num(src.get('PPD'))
        if v:
            return v
    return FW_DEFAULTS['PPD']


def targets(raw_cfg, name, now=None):
    """这一档要改成什么：(树莓派配置的键, STM32 参数)。now = STM32 现在的参数(GET)，用来读 PPD。"""
    cfg_t, stm_t = PRESETS[name]
    stm = dict(stm_t)
    stm['TCKP'] = tckp_for(_ppd(raw_cfg, now))
    return dict(cfg_t), stm


def _fmt(v):
    if v is None:
        return '(没设)'
    f = _num(v)
    if f is None:
        return str(v)
    return f'{f:g}'


def _read_now(link, log):
    if link is None or not hasattr(link, 'get_params'):
        return None
    try:
        ps, rep = link.get_params()
    except Exception as e:                              # noqa: BLE001  读不到就只改配置
        log(f'  (读 STM32 参数失败：{e!r}；只改配置，下次运行自动发过去)')
        return None
    if not ps:
        log(f'  (读 STM32 参数失败：{rep}；只改配置，下次运行自动发过去)')
    return ps or None


def _send(link, params, log):
    """一个个 SET 给 STM32。返回发失败的名字。"""
    bad = []
    if link is None:
        return bad
    for k, v in params.items():
        try:
            ok, rep = link.set_param(k, float(v))
        except Exception as e:                          # noqa: BLE001
            ok, rep = False, repr(e)
        if not ok:
            bad.append(k)
            log(f'  ★ STM32 不收 {k}={_fmt(v)}：{rep}(STM32 程序太旧就重新烧录)')
    return bad


def switch(raw_cfg, name, link=None, log=print):
    """换到 name 档('fast' / 'max' / 'normal')：改 raw_cfg(调用的人存文件)，连着 STM32 就马上 SET。
    返回 (改了哪些 [(名字, 原来, 现在)], 发失败的 STM32 参数名)。"""
    if name not in LEVELS + ('normal',):
        raise ValueError('格式：speed fast / speed max / speed normal / speed')
    sp = dict(raw_cfg.get('stm32_params') or {})
    saved = raw_cfg.get('speed_saved')
    now = _read_now(link, log)
    changes = []
    if name == 'normal':
        if not saved:
            log('现在就是 normal(没用 speed fast/max 改过)，不用还原。')
            return changes, []
        stm_send = {}
        for k in CFG_KEYS:
            old = (saved.get('cfg') or {}).get(k)
            cur = raw_cfg.get(k)
            if old is None:
                raw_cfg.pop(k, None)
            else:
                raw_cfg[k] = old
            if cur != old:
                changes.append((k, cur, old))
        for k in STM_KEYS:
            old_cfg = (saved.get('stm') or {}).get(k)
            back = old_cfg if old_cfg is not None else (saved.get('stm_now') or {}).get(k)
            if back is None:
                back = FW_DEFAULTS[k]
            cur = sp.get(k)
            if old_cfg is None:
                sp.pop(k, None)
            else:
                sp[k] = old_cfg
            stm_send[k] = back
            if _num(cur) != _num(back):
                changes.append((k, cur, back))
        raw_cfg['stm32_params'] = sp
        raw_cfg.pop('speed_saved', None)
        return changes, _send(link, stm_send, log)
    cfg_t, stm_t = targets(raw_cfg, name, now)
    if not saved:
        # 第一次提速：把原来的样子记下来(配置里没有的记 None；STM32 现在的值也记下，还原时配置里没有的就发回它)
        saved = {'cfg': {k: raw_cfg.get(k) for k in CFG_KEYS},
                 'stm': {k: sp.get(k) for k in STM_KEYS},
                 'stm_now': {k: (now or {}).get(k) for k in STM_KEYS}}
    saved = dict(saved, level=name)
    for k, v in cfg_t.items():
        cur = raw_cfg.get(k, CFG_DEFAULTS.get(k))
        if _num(cur) != _num(v):
            changes.append((k, cur, v))
        raw_cfg[k] = v
    for k, v in stm_t.items():
        cur = sp.get(k, (now or {}).get(k, FW_DEFAULTS.get(k)))
        if _num(cur) != _num(v):
            changes.append((k, cur, v))
        sp[k] = v
    raw_cfg['stm32_params'] = sp
    raw_cfg['speed_saved'] = saved
    return changes, _send(link, stm_t, log)


def show(raw_cfg, link=None, log=print):
    """speed 不带参数：现在哪一档、各参数多少。"""
    sp = raw_cfg.get('stm32_params') or {}
    now = _read_now(link, log) if link is not None else None
    lv = level(raw_cfg)
    log(f'现在：speed {lv}' + ('' if lv == 'normal' else '(speed normal 还原成原来的)'))
    log('  直行 route_speed_rpm=' + _fmt(raw_cfg.get('route_speed_rpm', CFG_DEFAULTS['route_speed_rpm'])) +
        '  横移 strafe_speed_rpm=' + _fmt(raw_cfg.get('strafe_speed_rpm', CFG_DEFAULTS['strafe_speed_rpm'])) +
        '  估时 turn90_s=' + _fmt(raw_cfg.get('turn90_s', CFG_DEFAULTS['turn90_s'])))
    keys = STM_KEYS + ('TCLAG', 'PPD')
    log('  STM32：' + '  '.join(f'{k}={_fmt(sp.get(k, (now or {}).get(k, FW_DEFAULTS.get(k))))}' for k in keys))
    log('  换档：speed fast / speed max / speed normal；转弯再快：turntune(车原地转几次 ±90°，自动选 TCLAG)')


def report(changes, bad, name, log=print):
    if not changes and not bad:
        log(f'已经是 speed {name}，没有要改的。')
        return
    for k, a, b in changes:
        log(f'  {k:16s} {_fmt(a):>7s} → {_fmt(b)}')
    if name == 'normal':
        log('已还原成原来的速度(存进配置)。')
    else:
        log(f'已换成 speed {name}(存进配置，下一次 go / race 起作用；STM32 的参数已经发过去)。' +
            ('车头偏、停不准、打滑就 speed fast 或 speed normal 退回。' if name == 'max' else
             '跑一趟看看：车头偏、停不准就 speed normal 退回。转弯再快：turntune'))
    if bad:
        log(f'★ 这几个 STM32 没收：{", ".join(bad)}。下次运行会再发一次')


# ================================================================ turntune
TUNE_LAGS = (0.16, 0.13, 0.10, 0.08)


def turn_tune(link, log=print, lags=TUNE_LAGS, reps=2, max_err=0.6, aborted=None):
    """(10-26)实车原地转 +90、-90 各 reps 次，每个 TCLAG 试一遍(车最后转回原来的车头)。
    选平均用时最短、每次停下误差都 ≤ max_err 度、没报错的；用时差不到 0.05 秒选大的(更稳)。都不行就保持原来的。
    返回 (选的 TCLAG, 原来的 TCLAG, 每个 TCLAG 的结果列表)。"""
    if link is None:
        raise ValueError('没有连接 STM32')
    ps, rep = link.get_params()
    if not ps or 'TCLAG' not in ps:
        raise ValueError(f'读不到 STM32 的 TCLAG：{rep}')
    old = float(ps['TCLAG'])
    ok, rep = link.home()
    if not ok:
        raise ValueError(f'HOME 失败：{rep}')
    rows = []
    try:
        for lag in lags:
            ok, rep = link.set_param('TCLAG', float(lag))
            if not ok:
                log(f'  TCLAG {lag:.2f}：STM32 不收({rep})，跳过')
                continue
            ts, es, bad = [], [], None
            for _ in range(int(reps)):
                for d in (90, -90):
                    if aborted is not None and aborted():
                        raise RuntimeError('收到 abort')
                    ok, reply = link.move('R', d)
                    info = parse_done(reply) if ok else {}
                    if not ok or 't' not in info:
                        bad = str(reply)
                        break
                    ts.append(float(info['t']))
                    es.append(float(info.get('err') or 0.0))
                if bad:
                    break
            row = dict(lag=float(lag), ts=ts, es=es, bad=bad)
            rows.append(row)
            if bad:
                log(f'  TCLAG {lag:.2f}：出错 {bad}')
                break                                   # 卡住/急停：不再转了
            log(f'  TCLAG {lag:.2f}：每次 {" ".join(f"{t:.2f}" for t in ts)} 秒，平均 {sum(ts)/len(ts):.2f} 秒，'
                f'停下误差最大 {max(abs(e) for e in es):.2f}°')
    finally:
        best = pick(rows, old, max_err)
        link.set_param('TCLAG', best)
    return best, old, rows


def pick(rows, old, max_err=0.6):
    """从 turn_tune 的结果里选 TCLAG：都没出错、误差都 ≤ max_err 的里面平均用时最短的；差不到 0.05 秒选大的。没有就用 old。"""
    good = [r for r in rows if not r.get('bad') and r.get('ts') and max(abs(e) for e in r['es']) <= max_err]
    if not good:
        return old
    fastest = min(sum(r['ts']) / len(r['ts']) for r in good)
    near = [r for r in good if sum(r['ts']) / len(r['ts']) <= fastest + 0.05]
    return max(r['lag'] for r in near)


def turn_estimates(rows, lag):
    """turntune 选好以后，用实测的平均用时更新估时(turn90_s)：返回 (turn90_s, turn180_s) 或 None。"""
    for r in rows:
        if math.isclose(r['lag'], lag) and r.get('ts') and not r.get('bad'):
            t90 = round(sum(r['ts']) / len(r['ts']) + 0.1, 1)
            return t90, round(t90 * 1.4, 1)
    return None
