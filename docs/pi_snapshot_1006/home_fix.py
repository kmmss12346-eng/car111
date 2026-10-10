"""v12 回家对准：回到启停区后，用雷达把现在看到的一整圈点，和出发时第一次扫描的点对齐(2D ICP)，
算出车离出发位置差多少(前后、左右、车头角度)，再发小步指令修回去。

为什么准：出发时车就停在启停区里扫过一次，现在回到几乎同一个位置再扫，看到的是同一批东西
(场边的桌椅墙、二维码板、障碍物)，视角几乎一样，对齐误差一般只有几毫米。
场地里被搬动过的东西、走动的人对不上，会被当成离群点丢掉；对上的点太少就不修，保持原样。"""
import math
import numpy as np
from lidar_map_live import rotate
from ld14p_scan import cartesian


def body_points(frames, c, rmin=150.0, rmax=4500.0, voxel=20.0, max_n=1500):
    """多帧原始极坐标点 → 车身坐标(车中心为原点，x 朝车头，y 朝车左，mm)。去掉车身自己的回波、太近太远的点，
    按 voxel 毫米的格子去重，最多留 max_n 个点。"""
    pol = [p for f in frames for p in f.get('points', []) if rmin <= p[1] <= rmax]
    if not pol:
        return np.empty((0, 2))
    P = np.asarray(cartesian(pol), float).reshape(-1, 2)
    P[:, 1] *= c['sdk_y_sign']
    B = rotate(P, c['lidar_yaw_deg']) + [c['lidar_forward_mm'], c['lidar_left_mm']]
    pad = float(c.get('self_echo_pad_mm', 120))
    own = (np.abs(B[:, 0]) <= c.get('car_length_mm', 290)/2 + pad) & (np.abs(B[:, 1]) <= c.get('car_width_mm', 260)/2 + pad)
    B = B[~own]
    if len(B) == 0:
        return B
    key = np.round(B / voxel).astype(np.int64)
    _, idx = np.unique(key[:, 0] * 1000003 + key[:, 1], return_index=True)
    B = B[np.sort(idx)]
    if len(B) > max_n:
        B = B[np.random.default_rng(0).choice(len(B), max_n, replace=False)]
    return B


def _rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def _nn(S, D, DD):
    d2 = (S*S).sum(1)[:, None] + DD[None, :] - 2.0 * S @ D.T
    j = d2.argmin(1)
    return j, np.sqrt(np.maximum(d2[np.arange(len(S)), j], 0.0))


def icp_align(cur, ref, iters=45, d_start=300.0, d_end=30.0, inlier_mm=30.0,
              min_pts=80, min_inlier=0.45, max_rms=15.0, max_shift=200.0, max_deg=8.0, init=None):
    """求 ref ≈ R(θ)·cur + t。含义：现在的车(车中心、车头)在“出发时的车身坐标”里的位置是 t=(tx 前, ty 左)、车头转了 θ。
    init=(θ度, tx, ty)：按路线推算的大概位置，从这里开始对齐(车离出发点还有几十厘米时用)；
    这时 max_shift/max_deg 检查的是“对齐结果离推算位置差多少”。
    返回 dict(ok, why, theta_deg, tx, ty, rms, inlier, n)。"""
    if len(cur) < min_pts or len(ref) < min_pts:
        return dict(ok=False, why=f'点太少({len(cur)}/{len(ref)})', theta_deg=0.0, tx=0.0, ty=0.0, rms=0.0, inlier=0.0, n=0)
    DD = (ref*ref).sum(1)
    best = None
    th_i = math.radians(init[0]) if init is not None else 0.0
    t_i = np.array([init[1], init[2]], float) if init is not None else np.zeros(2)
    for dth in (0.0, math.radians(3), math.radians(-3)):          # 车头可能歪几度：从三个初值各试一次，取最好的
        th, t = th_i + dth, t_i.copy()
        for k in range(iters):
            S = cur @ _rot(th).T + t
            j, d = _nn(S, ref, DD)
            thr = d_start + (d_end - d_start) * min(1.0, k / (0.6 * iters))
            keep = d < thr
            if keep.sum() < min_pts // 2:
                break
            A, B = cur[keep], ref[j[keep]]
            ca, cb = A.mean(0), B.mean(0)
            H = (A - ca).T @ (B - cb)
            th2 = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
            t2 = cb - _rot(th2) @ ca
            done = abs(th2 - th) < 1e-6 and np.linalg.norm(t2 - t) < 0.02 and k > 0.6 * iters
            th, t = th2, t2
            if done:
                break
        S = cur @ _rot(th).T + t
        j, d = _nn(S, ref, DD)
        inl = d < inlier_mm
        frac = float(inl.mean())
        rms = float(np.sqrt((d[inl]**2).mean())) if inl.any() else 1e9
        cand = dict(theta_deg=math.degrees(th), tx=float(t[0]), ty=float(t[1]), rms=rms, inlier=frac, n=int(inl.sum()))
        if best is None or (cand['inlier'], -cand['rms']) > (best['inlier'], -best['rms']):
            best = cand
    why = ''
    if best['inlier'] < min_inlier:
        why = f"对上的点只有{best['inlier']*100:.0f}%"
    elif best['rms'] > max_rms:
        why = f"残差{best['rms']:.1f}mm太大"
    else:
        ds = math.hypot(best['tx'] - t_i[0], best['ty'] - t_i[1])
        da = (best['theta_deg'] - math.degrees(th_i) + 180) % 360 - 180
        if ds > max_shift or abs(da) > max_deg:
            why = f"算出来{'离推算位置' if init is not None else ''}差太多(位置{ds:.0f}mm 角度{da:+.1f}°)，可能对错了"
    best.update(ok=not why, why=why)
    return best
