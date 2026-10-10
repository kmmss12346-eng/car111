"""摄像头闭环对准（视觉伺服）。

要解决的问题：摄像头跟着手臂一起转、一起升降，画面的方向和比例会变，而且物料要放到 ±1.5mm(1 环)才拿满分。
所以不"看一次、算一次、动一次"，而是边看边修：

    测偏差 p = 目标像素 - 爪子像素  →  换成动作量  →  动 ID1/ID2(精调) 或底盘(大偏差)  →  等稳  →  再测  →  直到够小

不需要事先知道摄像头朝哪、每毫米多少像素、ID1 的减速比。程序自己测出"动作量 ↔ 画面移动量"的对应关系(雅可比矩阵 J)：
  J_arm  每个单位动作(ID2 转 1°、ID1 转 1°)，目标在画面里移动多少像素
  J_ch   每个单位动作(底盘横移 S 1mm、前进 F 1mm)，目标在画面里移动多少像素
第一次用(或发现对不上)时先做几个小动作测出来(探测)，存进文件，下次直接用；每次修正后还会用实际移动量在线微调(Broyden 更新)。

保护：
  - 误差变大(发散)就丢掉这组 J 重新探测，再不行就放弃，不盲动
  - 手臂相对观察姿态的总偏移有上限，超了就改用底盘
  - 每步动作量有上限；总时间有上限
"""
import json
import math
import os
import time

import numpy as np


class ServoError(Exception):
    pass


DEFAULTS = dict(
    max_iter=6,                 # 最多修正几次
    timeout_s=10.0,             # 一次对准最多用多少秒
    gain_arm=0.85,              # 手臂每次修正掉多少比例的偏差(<1 留余量，防止过冲)
    gain_ch=0.8,                # 底盘
    settle_arm_s=0.20,          # 手臂动完等多久再拍(摄像头在手臂上，要等它不抖)
    settle_ch_s=0.35,           # 底盘动完等多久
    arm_limit_deg=dict(id2=250.0, id1=12.0),    # 手臂相对开始对准时的姿态，最多再偏多少度(ID2 约 0.175mm/度：250° ≈ 伸缩 ±44mm，车离圆环远一点近一点都靠它补)
    step_limit_deg=dict(id2=90.0, id1=4.0),     # 手臂每次最多动多少度
    min_step_deg=dict(id2=0.35, id1=0.35),      # 舵机比这小的一步动不了(STM32 的到位误差 ATOL=0.3°)：小于它的一步要么放大到它，要么不动
    chassis_step_max_mm=60.0,   # 底盘每次最多动多少毫米
    chassis_min_mm=6.0,         # 偏差小于这个就不动底盘(底盘只能到几毫米精度)
    arm_cover_mm=18.0,          # 偏差大于这个，并且允许动底盘，就先用底盘
    probe_min_px=14.0,          # 探测时画面至少要移动这么多像素才算测准
    probe_start_deg=dict(id2=15.0, id1=1.5),    # 探测时手臂先动多少度(不够会自动加大)
    probe_chassis_mm=12.0,      # 探测时底盘先动多少毫米
    probe_max_scale=6.0,
    diverge_ratio=1.4,          # 这次误差比上次大这么多倍就认为发散
    confirm=True,               # 误差够小时再测一次确认
    near_avg=2.0,               # 偏差比容差大、但在容差的这么多倍以内：先再测一次取平均再决定动不动(剩下的偏差和测量噪声差不多大，不去追噪声)
    broyden_min_px=6.0,         # 预计移动超过这么多像素才用来在线修正 J
    broyden_gain=0.5,
    broyden_max_rel=0.5,        # 实际移动和预计差太多(比如原料盘自己在转)就不拿来修正 J
    max_cond=40.0,              # J 的条件数超过这个说明两个轴在画面里几乎平行，没法解
    chassis_axes='SF',          # 底盘能动哪几个方向：'SF' = 横移和前后；'F' = 只前后(粗加工区/暂存区：车不横着挪，不会压进工位)
    chassis_fix_max_mm=None,    # 一次对准里底盘最多累计挪多少毫米，超过就停下(多半是认错了目标)；None = 不限
)


def _merge(base, extra):
    out = {}
    for k, v in base.items():
        out[k] = dict(v) if isinstance(v, dict) else v
    for k, v in (extra or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k].update(v)
        else:
            out[k] = v
    return out


class Result:
    def __init__(self, ok, err_mm, iters, p, reason='', elapsed=0.0, history=None, used_chassis=False, probed=False):
        self.ok = ok
        self.err_mm = err_mm
        self.iters = iters
        self.p = p
        self.reason = reason
        self.elapsed = elapsed
        self.history = history or []
        self.used_chassis = used_chassis
        self.probed = probed

    def __repr__(self):
        return (f'Result(ok={self.ok}, err={self.err_mm:.2f}mm, iters={self.iters}, '
                f'{self.elapsed:.1f}s{", probed" if self.probed else ""}{", chassis" if self.used_chassis else ""}'
                f'{", " + self.reason if self.reason else ""})')


class JacStore:
    """把测出来的 J 存成 json，下次启动直接用。键：(kind, group)，kind='RAW'/'RING'，group='arm'/'ch'。"""

    def __init__(self, path=None):
        self.path = path
        self.data = {}
        self.dirty = False
        if path and os.path.exists(path):
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    self.data = json.load(f)
            except Exception:
                self.data = {}

    def get(self, kind, group):
        v = (self.data.get(kind) or {}).get(group)
        if not v:
            return None
        try:
            a = np.array(v, float)
            return a if a.shape == (2, 2) and np.all(np.isfinite(a)) else None
        except Exception:
            return None

    def put(self, kind, group, J):
        self.data.setdefault(kind, {})[group] = [[float(x) for x in row] for row in np.asarray(J)]
        self.dirty = True

    def drop(self, kind, group):
        if kind in self.data and group in self.data[kind]:
            del self.data[kind][group]
            self.dirty = True
        if group == 'ch':
            self.set_synth(kind, False)

    def synth(self, kind):
        """这个 kind 的底盘 J 只测过前后(F)，横移(S)那一列是补出来的：要横移时得重新测。"""
        return bool((self.data.get(kind) or {}).get('ch_s_synth'))

    def set_synth(self, kind, flag):
        d = self.data.get(kind)
        if flag:
            if d is None:
                d = self.data.setdefault(kind, {})
            if not d.get('ch_s_synth'):
                d['ch_s_synth'] = True
                self.dirty = True
        elif d is not None and 'ch_s_synth' in d:
            del d['ch_s_synth']
            self.dirty = True

    def save(self):
        if not self.path or not self.dirty:
            return
        try:
            tmp = self.path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(self.data, f, indent=2)
            os.replace(tmp, self.path)
            self.dirty = False
        except Exception:
            pass


def _norm(v):
    return float(np.linalg.norm(v))


class VisualServo:
    """act 要提供：
         arm_move(d_id2_deg, d_id1_deg) -> (实际 ID2 动了多少度, 实际 ID1 动了多少度) 或 None(=按给的动了)
         chassis_move(s_mm, f_mm)       S 向左为正，F 向前为正
    """

    def __init__(self, act, store=None, cfg=None, log=print, sleep=time.sleep, clock=time.monotonic):
        self.act = act
        self.store = store if store is not None else JacStore(None)
        self.cfg = _merge(DEFAULTS, cfg)
        self.log = log
        self.sleep = sleep
        self.clock = clock
        self.dev = np.zeros(2)          # 手臂 [ID2, ID1] 相对开始对准时的偏移(度)
        self._axes = None               # 这一次对准底盘能动的方向(run 里设，结束清掉)
        self._ch_synth = False          # 现在的底盘 J 的横移那一列是不是补出来的(只测过前后)

    # ------------------------------------------------------------------ 测量
    def _measure(self, measure, tries=3):
        for _ in range(tries):
            p = measure()
            if p is not None:
                return np.array(p, float)
            self.sleep(0.1)
        raise ServoError('看不到目标')

    # ------------------------------------------------------------------ 手臂 / 底盘动作
    def _clip_arm(self, du):
        """按每步上限和总偏移上限裁剪。返回 (裁剪后的 du, 是否被明显裁剪)。"""
        c = self.cfg
        out = np.array(du, float)
        sat = False
        for i, key in enumerate(('id2', 'id1')):
            want = out[i]
            step = c['step_limit_deg'][key]
            lim = c['arm_limit_deg'][key]
            v = max(-step, min(step, want))
            v = max(-lim - self.dev[i], min(lim - self.dev[i], v))
            if abs(want) > 1e-9 and abs(want - v) > 0.3 * abs(want):
                sat = True
            out[i] = v
        return out, sat

    def _min_step(self, J, p, du):
        """太小的一步舵机动不了：放大到 min_step 能让偏差更小就放大，否则这个轴这次不动。"""
        out = np.array(du, float)
        for i, key in enumerate(('id2', 'id1')):
            m = float(self.cfg['min_step_deg'].get(key, 0.0))
            if m <= 0 or abs(out[i]) < 1e-9 or abs(out[i]) >= m:
                continue
            big = out.copy()
            big[i] = math.copysign(m, out[i])
            zero = out.copy()
            zero[i] = 0.0
            out = big if _norm(p + J @ big) < _norm(p + J @ zero) else zero
        return out

    def _apply_arm(self, du):
        du, sat = self._clip_arm(du)
        if _norm(du) < 1e-6:
            return du, sat
        got = self.act.arm_move(float(du[0]), float(du[1]))
        applied = np.array(got if got is not None else du, float)
        self.dev += applied
        self.sleep(self.cfg['settle_arm_s'])
        return applied, sat

    def _f_only(self):
        return str(getattr(self, '_axes', None) or self.cfg.get('chassis_axes') or 'SF').upper() == 'F'

    def _ch_cmd(self, Jc, p):
        """底盘这一步怎么动 (S, F)。只许前后动时：只用 J 的 F 那一列，按最小二乘算前后挪多少(横向的偏差留给手臂)。"""
        c = self.cfg
        if self._f_only():
            jf = np.asarray(Jc, float)[:, 1]
            n2 = float(jf @ jf)
            if n2 < 1e-12:
                return 0.0, 0.0
            return 0.0, float(-c['gain_ch'] * float(jf @ p) / n2)
        sf = -c['gain_ch'] * np.linalg.solve(Jc, p)
        return float(sf[0]), float(sf[1])

    def _apply_chassis(self, s_mm, f_mm):
        c = self.cfg
        if self._f_only():
            s_mm = 0.0                                   # 不横移
        s_mm = max(-c['chassis_step_max_mm'], min(c['chassis_step_max_mm'], s_mm))
        f_mm = max(-c['chassis_step_max_mm'], min(c['chassis_step_max_mm'], f_mm))
        s_mm, f_mm = float(round(s_mm)), float(round(f_mm))      # STM32 的 S/F 只收整数毫米
        if s_mm == 0 and f_mm == 0:
            return np.zeros(2)
        self.act.chassis_move(s_mm, f_mm)
        self.sleep(c['settle_ch_s'])
        return np.array([s_mm, f_mm])

    # ------------------------------------------------------------------ 探测雅可比
    def _margin(self, p, bounds):
        """目标离画面边缘还有多远(像素)。bounds = ((p 的下限 u, v), (p 的上限 u, v))；没给返回 None。"""
        if bounds is None:
            return None
        (u0, v0), (u1, v1) = bounds
        return float(min(p[0] - u0, u1 - p[0], p[1] - v0, v1 - p[1]))

    def _probe(self, measure, group, bounds=None):
        """小动作测出 J。返回 2x2：第 0 列 = ID2(或 S)，第 1 列 = ID1(或 F)。
        目标离画面边缘近时用更小的探测步长(只要画面移动够测准就行)，保证探测时目标不会出画面。"""
        c = self.cfg
        last = self._measure(measure)           # 每个轴的基准都是"上一次测到的位置"，不会把前一个轴造成的位移算进去
        min_px = c['probe_min_px']
        margin = self._margin(last, bounds)
        if margin is not None:
            min_px = max(5.0, min(min_px, 0.45 * margin))
        shrink = min_px / c['probe_min_px']
        J = np.zeros((2, 2))
        names = ('ID2', 'ID1') if group == 'arm' else ('S', 'F')
        f_only = group != 'arm' and self._f_only()
        for axis in ((1,) if f_only else (0, 1)):
            if group == 'arm':
                key = 'id2' if axis == 0 else 'id1'
                step = c['probe_start_deg'][key] * shrink
                sign = -1.0 if self.dev[axis] > 0.5 * c['arm_limit_deg'][key] else 1.0
            else:
                step = c['probe_chassis_mm'] * shrink
                sign = 1.0
            base = last
            moved = 0.0
            cur = base
            flipped = False
            for _ in range(5):
                if group == 'arm':
                    du = np.zeros(2)
                    du[axis] = sign * step
                    applied, _sat = self._apply_arm(du)
                    got = float(applied[axis])
                else:
                    ap = self._apply_chassis(sign * step if axis == 0 else 0.0, sign * step if axis == 1 else 0.0)
                    got = float(ap[axis])
                if abs(got) < 1e-9:
                    break
                moved += got
                try:
                    cur = self._measure(measure)
                except ServoError:
                    # 动了以后目标跑出了画面：撤销这一步，换相反方向、一半步长再试(只试一次)
                    if flipped:
                        raise
                    flipped = True
                    if group == 'arm':
                        back = np.zeros(2)
                        back[axis] = -moved
                        self._apply_arm(back)
                    else:
                        self._apply_chassis(-moved if axis == 0 else 0.0, -moved if axis == 1 else 0.0)
                    moved = 0.0
                    sign = -sign
                    step *= 0.5
                    try:
                        cur = self._measure(measure)
                    except ServoError:
                        raise ServoError('探测时目标出了画面，撤销后也找不回来：目标太靠近画面边缘，先手动把目标放到画面中间附近再 vcal')
                    continue
                if _norm(cur - base) >= min_px:
                    break
                scale = min(c['probe_max_scale'], max(1.5, min_px * 1.6 / max(_norm(cur - base), 1.0)))
                step *= scale
            dp = cur - base
            if abs(moved) < 1e-9 or _norm(dp) < 0.4 * min_px:
                raise ServoError(f'探测失败：动 {names[axis]} 画面几乎没变化(移动 {_norm(dp):.1f}px)，'
                                 '检查摄像头有没有拍到目标、手臂/底盘是否真的动了')
            J[:, axis] = dp / moved
            last = cur
        if f_only:
            J[:, 0] = (-J[1, 1], J[0, 1])                 # 不横移：S 那一列用不到，按"和 F 垂直、一样大"补上(只为了 J 能求逆)
        if group != 'arm':
            self._ch_synth = f_only
        cond = float(np.linalg.cond(J))
        if not np.isfinite(cond) or cond > c['max_cond']:
            raise ServoError(f'探测失败：{names[0]} 和 {names[1]} 在画面里的移动方向几乎平行(条件数 {cond:.0f})')
        self.log(f'    探测 {group}：J=[{J[0, 0]:+.3f} {J[0, 1]:+.3f}; {J[1, 0]:+.3f} {J[1, 1]:+.3f}] 像素/单位')
        return J

    # ------------------------------------------------------------------ 主循环
    def _decide(self, J, p, e, allow_chassis):
        """这一次用手臂还是底盘。偏差大、或者手臂这一步够不着(超出行程)就用底盘；其余用手臂(精度高)。"""
        c = self.cfg
        if not allow_chassis or e < c['chassis_min_mm']:
            return 'arm'
        if self._f_only() and J['ch'] is not None:
            jf = np.asarray(J['ch'], float)[:, 1]
            along = e * abs(float(jf @ p)) / max(_norm(jf) * _norm(p), 1e-9)   # 偏差里沿着"底盘前后"方向的那一段(毫米)
            if along < c['chassis_min_mm']:
                return 'arm'                             # 主要是横向的偏差：底盘不能横移，只能靠手臂伸缩
            if along > c['arm_cover_mm']:
                return 'ch'                              # 沿着圆环那一排差得多：底盘前后挪
            if J['arm'] is not None:                     # 差得不多：沿这一排动的那个舵机够得着就用手臂(底盘尽量少动)
                Ja = np.asarray(J['arm'], float)
                want = -c['gain_arm'] * np.linalg.solve(Ja, p)
                got, _sat = self._clip_arm(want)
                ju = jf / max(_norm(jf), 1e-9)
                ia = int(np.argmax([abs(float(Ja[:, i] @ ju)) / max(_norm(Ja[:, i]), 1e-9) for i in (0, 1)]))
                # 只看沿这一排的那个舵机(一般是 ID1)够不够得着：横向(ID2 伸缩)够不着，底盘前后挪也帮不上
                sat = abs(want[ia]) > 1e-9 and abs(want[ia] - got[ia]) > 0.3 * abs(want[ia])
                return 'ch' if sat else 'arm'
            return 'arm'
        if e > c['arm_cover_mm']:
            return 'ch'
        if J['arm'] is not None:
            _du, sat = self._clip_arm(-c['gain_arm'] * np.linalg.solve(J['arm'], p))
            if sat:
                return 'ch'
        return 'arm'

    def run(self, kind, measure, scale_px_per_mm, tol_mm, allow_chassis=True, max_iter=None, timeout_s=None, label='',
            bounds=None, confirm=None, chassis_axes=None, chassis_fix_max_mm=None):
        """对准(见 _run)。chassis_axes 只管这一次：结束后恢复成配置里的(vcal 之类单独探测时不受影响)。"""
        self._axes = chassis_axes or self.cfg.get('chassis_axes') or 'SF'
        try:
            return self._run(kind, measure, scale_px_per_mm, tol_mm, allow_chassis, max_iter, timeout_s, label, bounds, confirm,
                             chassis_fix_max_mm)
        finally:
            self._axes = None

    def _run(self, kind, measure, scale_px_per_mm, tol_mm, allow_chassis=True, max_iter=None, timeout_s=None, label='',
             bounds=None, confirm=None, chassis_fix_max_mm=None):
        """对准。measure() 返回 (du, dv) = 目标像素 - 爪子像素；看不到返回 None。返回 Result。

        kind          'RAW'(原料盘上的物料) 或 'RING'(地上的圆环)；J 按它分开存
        scale_px_per_mm  这个高度下每毫米多少像素(只用来把像素换成毫米判断容差)
        tol_mm        偏差小于它就算对准
        allow_chassis 偏差大、手臂够不着时允许动底盘
        bounds        画面范围换算成 p 的上下限 ((u 下限, v 下限), (u 上限, v 上限))，用来在目标靠近画面边缘时缩小探测步长
        confirm       达标后是否再测一次确认；None=用配置里的 confirm。对精度要求不高的(夹原料、取回)可以关掉省时间
        chassis_axes  这次底盘能动的方向('SF' / 'F')；None = 用配置里的
        chassis_fix_max_mm  这次对准底盘最多累计挪多少毫米；None = 用配置里的
        """
        c = self.cfg
        fix_max = chassis_fix_max_mm if chassis_fix_max_mm is not None else c.get('chassis_fix_max_mm')
        ch_moved = [0.0]                # 这次对准底盘累计挪了多少毫米
        max_iter = int(max_iter or c['max_iter'])
        timeout = float(timeout_s or c['timeout_s'])
        do_confirm = c['confirm'] if confirm is None else bool(confirm)
        t0 = self.clock()
        self.dev = np.zeros(2)
        J = {g: self.store.get(kind, g) for g in ('arm', 'ch')}
        self._ch_synth = J['ch'] is not None and self.store.synth(kind)
        if self._ch_synth and allow_chassis and not self._f_only():
            J['ch'] = None                  # 存着的底盘 J 只测过前后，这次要横移：重新探测
            self._ch_synth = False
        used_ch = False
        probed = False
        hist = []
        moves = 0
        passes = 0
        bad = 0
        reprobes = 0
        prev_e = None
        confirmed = False
        tag = f'[{label or kind}]'
        p = np.zeros(2)
        e = float('inf')
        fresh = [False]                 # p 是不是最后一次动作以后测的(动了以后没测到 = 不知道现在偏多少)
        avg = [False]                   # 这个位置已经测过两次取了平均
        stalls = 0

        def meas():
            v = self._measure(measure)
            fresh[0] = True
            return v

        def moved():
            fresh[0] = False
            avg[0] = False

        def over_budget():
            return fix_max is not None and ch_moved[0] >= float(fix_max)

        def finish(ok, reason=''):
            if ok or probed:
                for g in ('arm', 'ch'):
                    if J[g] is not None:
                        self.store.put(kind, g, J[g])
                if J['ch'] is not None:
                    self.store.set_synth(kind, self._ch_synth)
                self.store.save()
            # 动了以后没测到：不能拿动之前的偏差当结果(会按一个没人测过的位置去夹)
            return Result(ok, e if (ok or fresh[0]) else float('inf'), moves, p, reason, self.clock() - t0, hist, used_ch, probed)

        try:
            p = meas()
            while True:
                passes += 1
                if passes > 4 * max_iter + 6:
                    return finish(False, '循环次数过多')
                e = _norm(p) / scale_px_per_mm
                hist.append(e)
                self.log(f'  {tag} 偏差 {e:.2f}mm  (像素 {p[0]:+.1f},{p[1]:+.1f})')

                near = float(c.get('near_avg') or 0.0)
                if tol_mm < e <= near * tol_mm and fresh[0] and not avg[0]:
                    avg[0] = True                            # 离目标很近：再测一次取平均，免得按一次测量的噪声去动
                    p = (p + meas()) / 2.0
                    e = _norm(p) / scale_px_per_mm
                    self.log(f'  {tag} 再测一次取平均 {e:.2f}mm')
                    if e <= tol_mm:
                        self.log(f'  {tag} 对准完成')
                        return finish(True)                  # 两次的平均已经在容差内(相当于复测过了)

                if e <= tol_mm:
                    if do_confirm and not confirmed:
                        confirmed = True
                        p = meas()
                        e2 = _norm(p) / scale_px_per_mm
                        if e2 <= tol_mm * 1.15:
                            e = e2
                            hist.append(e)
                            self.log(f'  {tag} 复测 {e:.2f}mm，对准完成')
                            return finish(True)
                        continue
                    return finish(True)
                confirmed = False
                if moves >= max_iter:
                    return finish(False, f'{max_iter}次修正后仍有 {e:.2f}mm')
                if self.clock() - t0 > timeout:
                    return finish(False, f'超时({timeout:.0f}秒)，还差 {e:.2f}mm')

                # 发散：这次比上次明显更差。第一次发散丢掉 J 重新探测，再发散就放弃
                if prev_e is not None and e > prev_e * c['diverge_ratio'] + 0.6:
                    bad += 1
                    self.log(f'  {tag} 误差变大了({prev_e:.2f}→{e:.2f}mm)，J 可能不对')
                    if bad >= 2 or reprobes >= 1:
                        for g in ('arm', 'ch'):
                            self.store.drop(kind, g)
                        self.store.save()
                        return finish(False, '发散：动作方向和画面对不上，需要重新校准(vcal)')
                    for g in ('arm', 'ch'):
                        J[g] = None
                        self.store.drop(kind, g)
                    reprobes += 1
                else:
                    bad = 0

                group = self._decide(J, p, e, allow_chassis)
                if J[group] is None:
                    moved()
                    J[group] = self._probe(measure, group, bounds)
                    probed = True
                    p = meas()                               # 探测动过了，重新测，下一圈再决定
                    prev_e = None
                    continue

                cmd = None
                if group == 'arm':
                    du = self._min_step(J['arm'], p, -c['gain_arm'] * np.linalg.solve(J['arm'], p))
                    cmd, _s = self._clip_arm(du)
                    if _norm(cmd) < 1e-9 and _norm(du) < 1e-9:
                        return finish(False, f'剩下的偏差 {e:.2f}mm 比舵机能动的最小一步还小，没法再修')
                    moved()
                    applied, sat = self._apply_arm(du)
                else:
                    if over_budget():
                        return finish(False, f'底盘已经为对准挪了 {ch_moved[0]:.0f}mm 还差 {e:.2f}mm，停下(多半认错了目标，或者车停得太偏)')
                    sf = self._ch_cmd(J['ch'], p)
                    moved()
                    applied = self._apply_chassis(sf[0], sf[1])
                    used_ch = True
                    ch_moved[0] += _norm(applied)
                if _norm(applied) < 1e-9:
                    fresh[0] = True                          # 回读说一点没动：上次测的偏差还是现在的
                else:
                    stalls = 0                               # 只算连着两次没动
                if group == 'arm' and cmd is not None and _norm(cmd) > 1e-6 and _norm(applied) < 1e-9:
                    stalls += 1                              # 指令发了，角度读回来没变
                    if stalls >= 2:
                        return finish(False, '手臂指令发了但没动(ID1/ID2 卡住、或者这一步太小舵机动不了)')
                    p = meas()
                    continue
                if _norm(applied) < 1e-9:
                    if group == 'arm' and allow_chassis and e >= c['chassis_min_mm']:
                        # 手臂一点都动不了(到头了)：这一圈改用底盘
                        if J['ch'] is None:
                            moved()
                            J['ch'] = self._probe(measure, 'ch', bounds)
                            probed = True
                            p = meas()
                            prev_e = None
                            continue
                        if over_budget():
                            return finish(False, f'底盘已经为对准挪了 {ch_moved[0]:.0f}mm 还差 {e:.2f}mm，停下(多半认错了目标，或者车停得太偏)')
                        sf = self._ch_cmd(J['ch'], p)
                        moved()
                        applied = self._apply_chassis(sf[0], sf[1])
                        used_ch = True
                        ch_moved[0] += _norm(applied)
                        group = 'ch'
                    if _norm(applied) < 1e-9:
                        fresh[0] = True
                        return finish(False, f'手臂已到行程极限、底盘也不用动，还差 {e:.2f}mm')
                prev_e = e
                moves += 1

                p_new = meas()
                # 在线修正 J：用这次实际的"动了多少 → 画面变了多少"(差得太多说明是别的原因，比如原料盘在转，不拿来修)
                dp = p_new - p
                pred = J[group] @ applied
                ratio = _norm(dp) / max(_norm(pred), 1e-9)
                cosang = float(dp @ pred) / max(_norm(dp) * _norm(pred), 1e-9)
                # 方向对、只是大小差几倍(J 的比例不对，比如观察高度改过)也修；方向不对的(原料盘在转)不修
                if _norm(pred) >= c['broyden_min_px'] and (_norm(dp - pred) <= c['broyden_max_rel'] * _norm(pred)
                                                            or (cosang >= 0.9 and 0.3 <= ratio <= 3.0)):
                    # 按"每个轴让画面动了多少像素"分摊修正量：ID2 每度只动零点几像素、ID1 每度动好几像素，
                    # 直接按角度分摊的话，测量噪声几乎全算到 ID2 那一列上，几次以后 ID2 那一列就错了(来回晃、越晃越大)
                    sc = np.maximum(np.linalg.norm(J[group], axis=0), 1e-6)
                    x = applied * sc
                    J[group] = J[group] + c['broyden_gain'] * np.outer(dp - pred, x * sc) / float(x @ x)
                p = p_new
        except ServoError as ex:
            return finish(False, str(ex))
