"""地上圆环(同心圆靶)识别，抗爪子遮挡版：返回圆环中心(亚像素)。

原来的 ring_detect.detect_rings / vision.detect_rings_subpixel 的问题：
  只认"闭合的圆形轮廓"(圆度 >= 0.75)。爪子张开时占了画面下面 40% 左右，圆环一靠近爪子，
  外面几圈就被挡掉一截、不再闭合 -> 整个圆环认不到(偏一点点圆心就没了)。
这里的做法：
  1. Canny 边缘，去掉爪子区域(往外扩几像素)和画面四边的边缘；
  2. 每段边缘单独用 RANSAC 拟合圆(只要看到一段圆弧就行，一段边缘里混着别的线也能分出来)；
  3. 圆心靠得很近的圆弧归成一组(同心圆)；一组里至少有 2 个不同半径的圆才算圆环
     (被爪子切成两段的单个圆、物料顶面的圆都只有一个半径，不算)；
  4. 一组里所有圆弧的点一起做"同心圆"最小二乘：共用一个圆心、每个圆自己的半径，圆心精确到零点几像素。
被爪子挡住一半时圆心误差也在 0.5 像素以内。

接口：
  det = RingDetector(cfg)
  rings = det(frame, claw_mask)   -> [(x, y, 平均半径, 最大半径), ...]，从左到右(和原来的一样)
  det.last                        -> 最近一次的详细结果 [dict(center, radii, r_max, n_circles, coverage, rms, ...)]
"""
import math

import cv2
import numpy as np

from matdet import _refine, _coverage

DEFAULTS = dict(
    canny=[50, 150],             # Canny 阈值(和原来的 ring_detect 一样)
    blur_sigma=1.2,
    r_px=[8.0, 300.0],           # 单个圆的半径范围(像素)
    min_arc_pts=24,              # 一段圆弧至少多少个边缘点
    inlier_px=1.5,               # 边缘点离圆多近算在圆上
    ransac_iters=80,
    min_arc_cov=0.12,            # 一段圆弧至少占圆周多少(比例)
    max_fits_per_contour=3,      # 一段边缘里最多分出几个圆
    max_contours=160,            # 最多看多少段边缘(最长的优先)
    group_px=8.0,                # 圆心离得这么近的圆归成一组
    min_r_gap=2.0,               # 半径差这么多以上才算不同的圆
    min_circles=2,               # 一个圆环至少几个不同半径的圆
    min_cov=0.35,                # 一个圆环所有圆加起来至少看到圆周多少(比例)
    min_r_ratio=0.45,            # 一组里比最大的圆小这么多倍的圆不算(真圆环最里圈/最外圈 >= 0.53)
    claw_margin_px=5,            # 爪子区域往外扩几像素(爪子的边不是圆环的边)
)


def _ransac_circle(pts, rmin, rmax, thr, iters, rng):
    """一次算 iters 组"随机三点定圆"(向量化，比一个一个算快十几倍)，选落在圆上的点最多的，再几何精修。
    返回 (cx, cy, r, 内点数, 覆盖比例, 内点) 或 None。"""
    n = len(pts)
    if n < 6:
        return None
    idx = rng.integers(0, n, size=(iters, 3))
    a, b, c = pts[idx[:, 0]], pts[idx[:, 1]], pts[idx[:, 2]]
    d = 2.0 * (a[:, 0] * (b[:, 1] - c[:, 1]) + b[:, 0] * (c[:, 1] - a[:, 1]) + c[:, 0] * (a[:, 1] - b[:, 1]))
    ok = np.abs(d) > 1e-6
    if not ok.any():
        return None
    a, b, c, d = a[ok], b[ok], c[ok], d[ok]
    a2, b2, c2 = (a * a).sum(1), (b * b).sum(1), (c * c).sum(1)
    ux = (a2 * (b[:, 1] - c[:, 1]) + b2 * (c[:, 1] - a[:, 1]) + c2 * (a[:, 1] - b[:, 1])) / d
    uy = (a2 * (c[:, 0] - b[:, 0]) + b2 * (a[:, 0] - c[:, 0]) + c2 * (b[:, 0] - a[:, 0])) / d
    rr = np.hypot(a[:, 0] - ux, a[:, 1] - uy)
    ok = (rr >= rmin) & (rr <= rmax)
    if not ok.any():
        return None
    ux, uy, rr = ux[ok], uy[ok], rr[ok]
    dist = np.abs(np.hypot(pts[None, :, 0] - ux[:, None], pts[None, :, 1] - uy[:, None]) - rr[:, None])
    k = np.count_nonzero(dist < thr, axis=1)
    j = int(np.argmax(k))
    cx, cy, r = float(ux[j]), float(uy[j]), float(rr[j])
    for _ in range(2):                                   # 内点 -> 精修 -> 再选内点
        dd = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
        inl = pts[dd < thr * 1.25]
        if len(inl) < 6:
            return None
        cx, cy, r = _refine(inl, cx, cy, r, iters=10)
    if not (rmin <= r <= rmax):
        return None
    return cx, cy, r, len(inl), _coverage(inl, cx, cy), inl


def _concentric_refine(groups_pts, cx, cy, radii, iters=12):
    """同心圆最小二乘：groups_pts[k] 是第 k 个圆的点，共用圆心 (cx, cy)，半径各自 radii[k]。"""
    m = len(groups_pts)
    pts = np.vstack(groups_pts)
    idx = np.concatenate([np.full(len(p), k) for k, p in enumerate(groups_pts)])
    r = np.array(radii, float)
    for _ in range(iters):
        dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
        d = np.sqrt(dx * dx + dy * dy) + 1e-9
        res = d - r[idx]
        J = np.zeros((len(pts), 2 + m))
        J[:, 0] = -dx / d
        J[:, 1] = -dy / d
        J[np.arange(len(pts)), 2 + idx] = -1.0
        step = np.linalg.lstsq(J, -res, rcond=None)[0]
        cx += float(step[0])
        cy += float(step[1])
        r += step[2:]
        if abs(step[0]) < 1e-4 and abs(step[1]) < 1e-4:
            break
    dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
    res = np.sqrt(dx * dx + dy * dy) - r[idx]
    return cx, cy, r, res, idx


class RingDetector:
    wants_claw = True                    # vision.Vision 会把爪子区域传进来

    def __init__(self, cfg=None):
        c = dict(DEFAULTS)
        c.update(cfg or {})
        self.cfg = c
        self.last = []
        self.last_arcs = []

    # ---------------------------------------------------------- 第 1、2 步：圆弧
    def arcs(self, frame, claw_mask=None):
        cfg = self.cfg
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), float(cfg['blur_sigma']))
        lo, hi = cfg['canny']
        edge = cv2.Canny(blur, int(lo), int(hi))
        h, w = edge.shape
        if claw_mask is not None and claw_mask.any():
            if claw_mask.shape[:2] != (h, w):
                claw_mask = cv2.resize(claw_mask, (w, h), interpolation=cv2.INTER_NEAREST)
            k = 2 * int(cfg['claw_margin_px']) + 1
            edge[cv2.dilate(claw_mask, np.ones((k, k), np.uint8)) > 0] = 0
        edge[:3, :] = 0
        edge[-3:, :] = 0
        edge[:, :3] = 0
        edge[:, -3:] = 0
        contours, _ = cv2.findContours(edge, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        contours = [c for c in contours if len(c) >= cfg['min_arc_pts']]
        contours.sort(key=len, reverse=True)
        rmin, rmax = cfg['r_px']
        thr = float(cfg['inlier_px'])
        out = []
        rng = np.random.default_rng(11)
        for c in contours[:int(cfg['max_contours'])]:
            pts = np.unique(c[:, 0, :], axis=0).astype(np.float64)
            for _ in range(int(cfg['max_fits_per_contour'])):
                if len(pts) < cfg['min_arc_pts']:
                    break
                f = _ransac_circle(pts, rmin, rmax, thr, int(cfg['ransac_iters']), rng)
                if f is None:
                    break
                cx, cy, r, k, cov, inl = f
                if k < cfg['min_arc_pts']:
                    break
                d = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
                used = d < thr * 1.25
                # 圆弧要"连续地"占够一段圆周：k 个点最多能铺 k 像素长的弧，太短的弧(直线、小噪声)不要
                if cov >= cfg['min_arc_cov'] and k >= 0.6 * cov * 2 * math.pi * r * 0.5:
                    out.append(dict(center=(cx, cy), r=r, n=int(k), cov=cov, pts=inl))
                pts = pts[~used]
        self.last_arcs = out
        return out

    # ---------------------------------------------------------- 第 3、4 步：同心圆
    def detect(self, frame, claw_mask=None):
        cfg = self.cfg
        arcs = self.arcs(frame, claw_mask)
        gpx = float(cfg['group_px'])
        groups = []
        for a in sorted(arcs, key=lambda t: -t['n'] * t['r']):          # 大圆、长弧的圆心最准，先放
            ax, ay = a['center']
            for g in groups:
                if (ax - g['cx']) ** 2 + (ay - g['cy']) ** 2 < gpx * gpx:
                    g['arcs'].append(a)
                    wsum = sum(q['n'] * q['r'] for q in g['arcs'])
                    g['cx'] = sum(q['center'][0] * q['n'] * q['r'] for q in g['arcs']) / wsum
                    g['cy'] = sum(q['center'][1] * q['n'] * q['r'] for q in g['arcs']) / wsum
                    break
            else:
                groups.append(dict(cx=ax, cy=ay, arcs=[a]))
        rings = []
        for g in groups:
            res = self._fit_group(g)
            if res is not None:
                rings.append(res)
        # 两组精修以后圆心重合了(同一个圆环被分成了两组)：留圆多的那个
        rings.sort(key=lambda t: -t['n_circles'])
        keep = []
        for r in rings:
            if all(math.hypot(r['center'][0] - q['center'][0], r['center'][1] - q['center'][1]) > gpx for q in keep):
                keep.append(r)
        keep.sort(key=lambda t: t['center'][0])
        self.last = keep
        return keep

    def _fit_group(self, g):
        cfg = self.cfg
        rbig = max(a['r'] for a in g['arcs'])
        arcs = sorted((a for a in g['arcs'] if a['r'] >= cfg['min_r_ratio'] * rbig), key=lambda q: q['r'])
        # 半径相近的圆弧是同一个圆(比如被爪子切成两段)
        circles = []
        for a in arcs:
            if circles and a['r'] - circles[-1]['r'] < cfg['min_r_gap']:
                c = circles[-1]
                c['pts'].append(a['pts'])
                c['r'] = (c['r'] * c['n'] + a['r'] * a['n']) / (c['n'] + a['n'])
                c['n'] += a['n']
            else:
                circles.append(dict(r=a['r'], n=a['n'], pts=[a['pts']]))
        if len(circles) < cfg['min_circles']:
            return None
        pts = [np.vstack(c['pts']) for c in circles]
        cx, cy, r, res, idx = _concentric_refine(pts, g['cx'], g['cy'], [c['r'] for c in circles])
        # 去掉离得远的点(混进来的别的边)再精修一次
        thr = 2.5 * float(cfg['inlier_px'])
        pts2, r2 = [], []
        for k in range(len(circles)):
            p = pts[k][np.abs(res[idx == k]) < thr]
            if len(p) >= max(8, 0.5 * len(pts[k])):
                pts2.append(p)
                r2.append(r[k])
        if len(pts2) < cfg['min_circles']:
            return None
        cx, cy, r, res, idx = _concentric_refine(pts2, cx, cy, r2)
        if math.hypot(cx - g['cx'], cy - g['cy']) > 2.0 * float(cfg['group_px']):
            return None
        rs = np.sort(r)
        n_circles = 1 + int(np.count_nonzero(np.diff(rs) >= cfg['min_r_gap']))
        if n_circles < cfg['min_circles']:
            return None
        covs = [_coverage(p, cx, cy) for p in pts2]
        cov = max(covs)
        if cov < cfg['min_cov']:
            return None
        rms = float(np.sqrt(np.mean(res * res)))
        return dict(center=(float(cx), float(cy)), radii=[float(v) for v in rs], r_mean=float(np.mean(rs)),
                    r_max=float(rs[-1]), n_circles=n_circles, coverage=float(cov), rms=rms,
                    n_pts=int(sum(len(p) for p in pts2)))

    def __call__(self, frame, claw_mask=None):
        return [(r['center'][0], r['center'][1], r['r_mean'], r['r_max']) for r in self.detect(frame, claw_mask)]
