"""雷达重定位(只用 numpy)：把现在扫到的一圈点，和出发时的参考点云对齐(2D ICP)，算出车现在实际在哪。

参考点云 = 第一次扫描(启停区里) + 第二次扫描(第二站)，都换到"出发时的车身坐标"里，第二次扫描之后建一次。
参考点云上预先算好一张最近点表(20mm 一格)：对齐时每个点直接查表，不用和几千个参考点逐个比距离，树莓派上也快。

坐标：
  车身坐标  x 朝车头，y 朝车左(mm)，原点在车中心；
  世界坐标  和 route_plan 一样：场地左下角原点，x 朝东，y 朝北，航向东 0°、北 90°，逆时针为正。
ICP 的结果 (theta_deg, tx, ty)：现在的车(车中心、车头)在"出发时的车身坐标"里的位置和朝向，和 home_fix.icp_align 的含义一样。
"""
import math
import time

import numpy as np


# ---------------------------------------------------------------- 坐标换算
def rotate(xy, deg):
    """逆时针转 deg 度(和 lidar_map_live.rotate 一样)。"""
    a = math.radians(deg)
    co, si = math.cos(a), math.sin(a)
    return np.asarray(xy, dtype=float).reshape(-1, 2) @ np.array([[co, si], [-si, co]])


def wrap180(a):
    return (float(a) + 180.0) % 360.0 - 180.0


def rel_of(world, origin):
    """世界坐标的位姿 world=(x,y,航向) -> 在 origin 车身坐标里的 (theta_deg, tx, ty)。"""
    x0, y0, p0 = (float(v) for v in origin[:3])
    dx, dy = float(world[0]) - x0, float(world[1]) - y0
    a = math.radians(-p0)
    return (wrap180(float(world[2]) - p0), dx*math.cos(a) - dy*math.sin(a), dx*math.sin(a) + dy*math.cos(a))


def world_of(rel, origin):
    """(theta_deg, tx, ty)(在 origin 车身坐标里) -> 世界坐标 (x, y, 航向)。"""
    x0, y0, p0 = (float(v) for v in origin[:3])
    th, tx, ty = (float(v) for v in rel)
    a = math.radians(p0)
    return (x0 + tx*math.cos(a) - ty*math.sin(a), y0 + tx*math.sin(a) + ty*math.cos(a), wrap180(p0 + th))


def body_to_frame(P, pose, origin):
    """车在 pose(世界)时的车身坐标点 P -> origin(世界)车身坐标里的点。"""
    th, tx, ty = rel_of(pose, origin)
    return rotate(P, th) + [tx, ty]


# ---------------------------------------------------------------- 点
def body_points(frames, c, cartesian=None, rmin=150.0, rmax=4500.0, voxel=20.0, max_n=1500, seed=0):
    """多帧原始极坐标点 -> 车身坐标点(和 home_fix.body_points 一样的换算)：去掉车身自己的回波、太近太远的点，
    按 voxel 毫米的格子去重，最多留 max_n 个点。cartesian 默认用车上的 ld14p_scan.cartesian。"""
    if cartesian is None:
        from ld14p_scan import cartesian
    pol = [p for f in frames for p in f.get('points', []) if rmin <= p[1] <= rmax]
    if not pol:
        return np.empty((0, 2))
    P = np.asarray(cartesian(pol), float).reshape(-1, 2)
    P[:, 1] *= c['sdk_y_sign']
    B = rotate(P, c['lidar_yaw_deg']) + [c['lidar_forward_mm'], c['lidar_left_mm']]
    pad = float(c.get('self_echo_pad_mm', 120))
    own = (np.abs(B[:, 0]) <= c.get('car_length_mm', 290)/2 + pad) & (np.abs(B[:, 1]) <= c.get('car_width_mm', 260)/2 + pad)
    B = B[~own]
    return thin(B, voxel, max_n, seed)


def thin(P, voxel=20.0, max_n=None, seed=0):
    """按 voxel 毫米的格子去重(每格留一个点)，再随机(固定种子)留最多 max_n 个。"""
    P = np.asarray(P, float).reshape(-1, 2)
    if len(P) == 0:
        return P
    key = np.round(P / voxel).astype(np.int64)
    _, idx = np.unique(key[:, 0] * 1000003 + key[:, 1], return_index=True)
    P = P[np.sort(idx)]
    if max_n is not None and len(P) > max_n:
        P = P[np.sort(np.random.default_rng(seed).choice(len(P), int(max_n), replace=False))]
    return P


def merge_reference(clouds, voxel=20.0, max_n=2500, max_r=None):
    """几次扫描的点(都已经在出发时的车身坐标里)合成一个参考点云。max_r：离原点更远的点不要(太远的点少、不稳)。"""
    P = np.vstack([np.asarray(c, float).reshape(-1, 2) for c in clouds if len(c)]) if any(len(c) for c in clouds) else np.empty((0, 2))
    if max_r is not None and len(P):
        P = P[np.hypot(P[:, 0], P[:, 1]) <= max_r]
    return thin(P, voxel, max_n)


# ---------------------------------------------------------------- 最近点表
class RefGrid:
    """参考点云 + 最近点表。每一格存"离格子中心最近的参考点"的序号(只算到 reach 毫米以内，更远的格子是 -1)。
    查询时看这个点所在格子和周围 8 格存的参考点，取真正最近的一个。用跳跃泛洪(jump flooding)建表，全部是 numpy 数组运算。"""

    def __init__(self, ref, cell=20.0, reach=320.0):
        t0 = time.monotonic()
        self.ref = np.asarray(ref, float).reshape(-1, 2)
        self.cell = float(cell)
        self.reach = float(reach)
        n = len(self.ref)
        if n == 0:
            self.x0 = self.y0 = 0.0
            self.nx = self.ny = 1
            self.idx = np.full((1, 1), -1, np.int32)
            self.build_s = 0.0
            return
        lo = self.ref.min(0) - reach - cell
        hi = self.ref.max(0) + reach + cell
        self.x0, self.y0 = float(lo[0]), float(lo[1])
        self.nx = int(math.ceil((hi[0] - lo[0]) / cell)) + 1
        self.ny = int(math.ceil((hi[1] - lo[1]) / cell)) + 1
        cx = (self.x0 + (np.arange(self.nx) + 0.5) * cell).astype(np.float32)
        cy = (self.y0 + (np.arange(self.ny) + 0.5) * cell).astype(np.float32)
        rx = self.ref[:, 0].astype(np.float32)
        ry = self.ref[:, 1].astype(np.float32)
        idx = np.full((self.nx, self.ny), -1, np.int32)
        bd = np.full((self.nx, self.ny), np.inf, np.float32)
        # 种子：每个参考点放进它所在的格子；一格里有好几个点时留离格子中心最近的
        ci = ((self.ref[:, 0] - self.x0) / cell).astype(np.int64)
        cj = ((self.ref[:, 1] - self.y0) / cell).astype(np.int64)
        d0 = (cx[ci] - rx)**2 + (cy[cj] - ry)**2
        cid = ci * self.ny + cj
        o = np.lexsort((d0, cid))
        first = np.r_[True, cid[o][1:] != cid[o][:-1]]
        sel = o[first]
        idx.flat[cid[sel]] = sel.astype(np.int32)
        bd.flat[cid[sel]] = d0[sel]
        steps = []
        k = 1
        while k * cell <= reach:
            steps.append(k)
            k *= 2
        steps = steps[::-1] + [1]                        # 1+JFA：最后多走一遍步长 1，修掉少数错的格子
        lim = np.float32((reach + cell) ** 2)
        for k in steps:
            for dx in (-k, 0, k):
                for dy in (-k, 0, k):
                    if dx == 0 and dy == 0:
                        continue
                    ia = slice(max(0, -dx), self.nx - max(0, dx))
                    ib = slice(max(0, dx), self.nx - max(0, -dx))
                    ja = slice(max(0, -dy), self.ny - max(0, dy))
                    jb = slice(max(0, dy), self.ny - max(0, -dy))
                    cand = idx[ib, jb]
                    ok = cand >= 0
                    if not ok.any():
                        continue
                    c = np.where(ok, cand, 0)
                    dd = (cx[ia][:, None] - rx[c])**2 + (cy[ja][None, :] - ry[c])**2
                    better = ok & (dd < bd[ia, ja]) & (dd <= lim)
                    if better.any():
                        sub_i = idx[ia, ja]
                        sub_d = bd[ia, ja]
                        sub_i[better] = cand[better]
                        sub_d[better] = dd[better]
        self.idx = idx
        self.build_s = time.monotonic() - t0

    def nearest(self, Q):
        """Q(n×2) 每个点最近的参考点：返回 (序号, 距离mm)。找不到(reach 以外)的距离是 inf、序号 0。"""
        Q = np.asarray(Q, float).reshape(-1, 2)
        n = len(Q)
        if n == 0 or len(self.ref) == 0:
            return np.zeros(n, np.int64), np.full(n, np.inf)
        ci = np.floor((Q[:, 0] - self.x0) / self.cell).astype(np.int64)
        cj = np.floor((Q[:, 1] - self.y0) / self.cell).astype(np.int64)
        best_j = np.zeros(n, np.int64)
        best_d = np.full(n, np.inf)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                i = ci + dx
                j = cj + dy
                inside = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
                cand = np.full(n, -1, np.int64)
                cand[inside] = self.idx[i[inside], j[inside]]
                ok = cand >= 0
                if not ok.any():
                    continue
                c = np.where(ok, cand, 0)
                d = np.hypot(Q[:, 0] - self.ref[c, 0], Q[:, 1] - self.ref[c, 1])
                better = ok & (d < best_d)
                best_d[better] = d[better]
                best_j[better] = c[better]
        return best_j, best_d


# ---------------------------------------------------------------- ICP
def _rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def icp(cur, grid, init=None, iters=40, d_start=250.0, d_end=30.0, inlier_mm=30.0, min_pts=80, angles=None):
    """点对点 ICP：求 ref ≈ R(θ)·cur + t。cur 是现在的车身坐标点，grid 是 RefGrid(出发时的车身坐标)。
    init=(θ度, tx, ty)：按路线推算的位置(给了就只从这一个角度开始，快)；没给从 0°、±3° 三个角度各试一次取最好的。
    返回 dict(theta_deg, tx, ty, rms, inlier, n, t)；还没判断能不能信，见 judge()。"""
    t0 = time.monotonic()
    cur = np.asarray(cur, float).reshape(-1, 2)
    if len(cur) < min_pts or len(grid.ref) < min_pts:
        return dict(theta_deg=0.0, tx=0.0, ty=0.0, rms=1e9, inlier=0.0, n=0, t=time.monotonic() - t0,
                    few=f'点太少({len(cur)}/{len(grid.ref)})')
    th_i = math.radians(init[0]) if init is not None else 0.0
    t_i = np.array([init[1], init[2]], float) if init is not None else np.zeros(2)
    if angles is None:
        angles = (0.0,) if init is not None else (0.0, 3.0, -3.0)
    best = None
    for dth in angles:
        th, t = th_i + math.radians(dth), t_i.copy()
        for k in range(iters):
            S = cur @ _rot(th).T + t
            j, d = grid.nearest(S)
            thr = d_start + (d_end - d_start) * min(1.0, k / (0.6 * iters))
            keep = d < thr
            if keep.sum() < min_pts // 2:
                break
            A, B = cur[keep], grid.ref[j[keep]]
            ca, cb = A.mean(0), B.mean(0)
            H = (A - ca).T @ (B - cb)
            th2 = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
            t2 = cb - _rot(th2) @ ca
            done = abs(th2 - th) < 1e-5 and np.linalg.norm(t2 - t) < 0.05 and k > 0.6 * iters
            th, t = th2, t2
            if done:
                break
        S = cur @ _rot(th).T + t
        j, d = grid.nearest(S)
        inl = d < inlier_mm
        frac = float(inl.mean())
        rms = float(np.sqrt((d[inl]**2).mean())) if inl.any() else 1e9
        cand = dict(theta_deg=math.degrees(th), tx=float(t[0]), ty=float(t[1]), rms=rms, inlier=frac, n=int(inl.sum()))
        if best is None or (cand['inlier'], -cand['rms']) > (best['inlier'], -best['rms']):
            best = cand
    best['t'] = time.monotonic() - t0
    return best


def judge(res, init=None, min_inlier=0.45, max_rms=15.0, max_shift=150.0, max_deg=5.0):
    """ICP 结果能不能信：返回 '' (能信) 或 一句原因。init 给了时，max_shift/max_deg 比的是离推算位置差多少。"""
    if res.get('few'):
        return res['few']
    if res['inlier'] < min_inlier:
        return f"对上的点只有{res['inlier']*100:.0f}%"
    if res['rms'] > max_rms:
        return f"残差{res['rms']:.1f}mm太大"
    if init is not None:
        ds = math.hypot(res['tx'] - init[1], res['ty'] - init[2])
        da = wrap180(res['theta_deg'] - init[0])
        if ds > max_shift or abs(da) > max_deg:
            return f'离推算位置差太多(位置{ds:.0f}mm 角度{da:+.1f}°)，可能对错了'
    return ''


def gates(cfg):
    """配置里的重定位门槛。"""
    g = (cfg or {}).get
    return dict(min_inlier=float(g('reloc_min_inlier', 0.45)), max_rms=float(g('reloc_max_rms_mm', 15.0)),
                max_shift=float(g('reloc_max_shift_mm', 150.0)), max_deg=float(g('reloc_max_yaw_deg', 5.0)))


def measure(cur, grid, origin, plan_pose, cfg=None, use_init=True, max_shift=None, max_deg=None):
    """一次测量：cur(现在的车身坐标点)对齐到 grid(origin 车身坐标的参考点云)，换回世界坐标，并按门槛判断。
    返回 dict(ok, why, pose, rms, inlier, n, t)。"""
    gt = gates(cfg)
    if max_shift is not None:
        gt['max_shift'] = float(max_shift)
    if max_deg is not None:
        gt['max_deg'] = float(max_deg)
    init = rel_of(plan_pose, origin) if (use_init and plan_pose is not None) else None
    res = icp(cur, grid, init=init)
    why = judge(res, init, **gt)
    if not why and init is None and plan_pose is not None:
        # 没用推算位置做初值(怕初值太偏)：结果还是要和推算位置差不多才信
        why = judge(res, rel_of(plan_pose, origin), **gt)
    pose = world_of((res['theta_deg'], res['tx'], res['ty']), origin)
    return dict(ok=not why, why=why, pose=pose, rms=res['rms'], inlier=res['inlier'], n=res.get('n', 0), t=res['t'])


def relocalize(measure_fn, plan_pose, cfg=None, log=None):
    """停车点上的雷达定位：measure_fn(plan_pose) 返回 measure() 那样的 dict(拍一次、对齐一次)。
    修正量超过 reloc_confirm_mm(50mm)时再测一次，两次要一致(差不超过 reloc_agree_mm / reloc_agree_deg)才信，取平均。
    返回 dict(ok, why, pose, dx, dy, dyaw, rms, inlier, t, n_meas)；dx/dy/dyaw = 测到的 - 推算的(世界坐标)。"""
    g = (cfg or {}).get
    confirm = float(g('reloc_confirm_mm', 50.0))
    agree_mm = float(g('reloc_agree_mm', 15.0))
    agree_deg = float(g('reloc_agree_deg', 1.0))
    px, py, pa = (float(v) for v in plan_pose[:3])
    t_all = 0.0
    r1 = measure_fn(plan_pose)
    t_all += float(r1.get('t', 0.0))
    out = dict(ok=False, why=r1.get('why', ''), pose=r1.get('pose'), rms=r1.get('rms'), inlier=r1.get('inlier'), t=t_all, n_meas=1)
    if not r1.get('ok'):
        return _with_delta(out, plan_pose)
    pose = r1['pose']
    if math.hypot(pose[0] - px, pose[1] - py) > confirm:
        r2 = measure_fn(plan_pose)
        t_all += float(r2.get('t', 0.0))
        out.update(t=t_all, n_meas=2)
        if not r2.get('ok'):
            out.update(why=f"修正量大({math.hypot(pose[0]-px, pose[1]-py):.0f}mm)，再测一次没对上：{r2.get('why')}")
            return _with_delta(out, plan_pose)
        q = r2['pose']
        dd = math.hypot(q[0] - pose[0], q[1] - pose[1])
        da = wrap180(q[2] - pose[2])
        if dd > agree_mm or abs(da) > agree_deg:
            out.update(why=f'修正量大，两次测量不一致(差{dd:.0f}mm、{da:+.1f}°)，不信')
            return _with_delta(out, plan_pose)
        pose = ((pose[0] + q[0]) / 2, (pose[1] + q[1]) / 2, wrap180(pose[2] + da / 2))
        out.update(rms=max(r1['rms'], r2['rms']), inlier=min(r1['inlier'], r2['inlier']))
    out.update(ok=True, why='', pose=pose)
    return _with_delta(out, plan_pose)


def _with_delta(out, plan_pose):
    p = out.get('pose')
    if p is None:
        out.update(dx=0.0, dy=0.0, dyaw=0.0)
    else:
        out.update(dx=p[0] - float(plan_pose[0]), dy=p[1] - float(plan_pose[1]), dyaw=wrap180(p[2] - float(plan_pose[2])))
    return out


class Reference:
    """出发时的参考点云(第一次 + 第二次扫描，在出发时的车身坐标里) + 最近点表。origin = 出发位姿(世界)。"""

    def __init__(self, origin, clouds, cell=20.0, reach=320.0, max_n=2500, max_r=4500.0):
        t0 = time.monotonic()
        self.origin = tuple(float(v) for v in origin[:3])
        self.points = merge_reference(clouds, max_n=max_n, max_r=max_r)
        self.grid = RefGrid(self.points, cell=cell, reach=reach)
        self.build_s = time.monotonic() - t0

    def measure(self, cur, plan_pose, cfg=None, use_init=True, max_shift=None, max_deg=None):
        return measure(cur, self.grid, self.origin, plan_pose, cfg, use_init, max_shift, max_deg)


def build_reference(views, body_points_fn, max_n_view=2000):
    """views：[(扫描的原始帧, 当时的车位(世界))…]，第一项是出发时那次。返回 Reference。"""
    origin = views[0][1]
    clouds = []
    for frames, pose in views:
        B = body_points_fn(frames, max_n_view)
        if len(B):
            clouds.append(body_to_frame(B, pose, origin))
    return Reference(origin, clouds)
