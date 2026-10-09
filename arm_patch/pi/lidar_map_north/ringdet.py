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
  5. 校赛那种"粗黑环 + 中间白色满分区"的靶：中间被物料盖住(取回、码垛)或者被爪子挡住时只剩最外圈一个圆。
     知道圆环大概多大(r_expect，vclaw RING 量过)时，外圈大小对、而且圈里面紧挨着的一圈是黑的、外面是白的，也算圆环。
被爪子挡住一半时圆心误差也在 0.5 像素以内。

接口：
  det = RingDetector(cfg)
  rings = det(frame, claw_mask, r_expect=None)   -> [(x, y, 平均半径, 最大半径), ...]，从左到右(和原来的一样)
                                  r_expect = (最小, 最大) 最外圈半径像素，给了才允许"只看到最外圈"的情况
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
    min_r_ratio=0.3,             # 一组里比最大的圆小这么多倍的圆不算(校赛靶白心/外圈 约 0.5；6 环靶最里圈/最外圈 0.53)
    group_rel=0.06,              # 大圆弧的圆心没那么准：归组距离放宽到 半径 x 这个(不小于 group_px)
    single_ok=True,              # 只看到最外圈一个圆时，大小对(r_expect)、圈里一圈黑外面白，也算圆环
    single_min_cov=0.22,         # 只看到一个圆时，这个圆至少看到多少圆周(圆心在爪子下面时只露出上面一小段)
    dark_min=40.0,               # 外圈里面紧挨着的一圈要比外面暗这么多(灰度)
    dark_frac=0.7,               # 至少这么多方向满足上面这条(物料盖住一部分也没关系)
    dark_max_sat=90.0,           # 那一圈黑环颜色要淡(饱和度低)：红/蓝/绿色物料的边不算
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
        self._gray, self._bad = blur, None
        self._sat = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 1] if frame.ndim == 3 else None
        if claw_mask is not None and claw_mask.any():
            if claw_mask.shape[:2] != (h, w):
                claw_mask = cv2.resize(claw_mask, (w, h), interpolation=cv2.INTER_NEAREST)
            k = 2 * int(cfg['claw_margin_px']) + 1
            self._bad = cv2.dilate(claw_mask, np.ones((k, k), np.uint8))
            edge[self._bad > 0] = 0
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
    def _tol(self, r):
        return max(float(self.cfg['group_px']), float(self.cfg['group_rel']) * float(r))

    def detect(self, frame, claw_mask=None, r_expect=None):
        cfg = self.cfg
        arcs = self.arcs(frame, claw_mask)
        groups = []
        for a in sorted(arcs, key=lambda t: -t['n'] * t['r']):          # 大圆、长弧的圆心最准，先放
            ax, ay = a['center']
            for g in groups:
                tol = self._tol(max(a['r'], max(q['r'] for q in g['arcs'])))
                if (ax - g['cx']) ** 2 + (ay - g['cy']) ** 2 < tol * tol:
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
            if res is None and r_expect is not None and cfg.get('single_ok', True):
                res = self._fit_single(g, r_expect)
            if res is not None:
                rings.append(res)
        # 两组精修以后圆心重合了(同一个圆环被分成了两组)：留圆多的那个
        rings.sort(key=lambda t: (-t['n_circles'], -t['coverage']))
        keep = []
        for r in rings:
            if all(math.hypot(r['center'][0] - q['center'][0], r['center'][1] - q['center'][1]) > self._tol(max(r['r_max'], q['r_max']))
                   for q in keep):
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
        if math.hypot(cx - g['cx'], cy - g['cy']) > 2.0 * self._tol(max(r2)):
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

    def _fit_single(self, g, r_expect):
        """只剩最外圈一个圆(中间被物料盖住/被爪子挡住)：大小在 r_expect 里，而且圈里黑外面白，才算圆环。"""
        cfg = self.cfg
        lo, hi = float(r_expect[0]), float(r_expect[1])
        big = [a for a in g['arcs'] if lo * 0.95 <= a['r'] <= hi * 1.05]
        if not big:
            return None
        rb = max(a['r'] for a in big)
        same = [a for a in big if abs(a['r'] - rb) <= max(float(cfg['min_r_gap']), 0.04 * rb)]
        pts = np.vstack([a['pts'] for a in same])
        cx, cy, r = _refine(pts, g['cx'], g['cy'], rb, iters=15)
        d = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
        pts = pts[d < 2.5 * float(cfg['inlier_px'])]
        if len(pts) < cfg['min_arc_pts']:
            return None
        cx, cy, r = _refine(pts, cx, cy, r, iters=15)
        if not (lo <= r <= hi):
            return None
        cov = _coverage(pts, cx, cy)
        if cov < cfg['single_min_cov'] or not self._dark_inside(cx, cy, r):
            return None
        res = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r
        return dict(center=(float(cx), float(cy)), radii=[float(r)], r_mean=float(r), r_max=float(r), n_circles=1,
                    coverage=float(cov), rms=float(np.sqrt(np.mean(res * res))), n_pts=int(len(pts)), single=True)

    def _dark_inside(self, cx, cy, r):
        """圆周里面紧挨着的一圈(0.88r~0.94r)比外面(1.06r~1.12r)暗：校赛靶的黑环外边。爪子区域、画面外的方向不算。"""
        g = getattr(self, '_gray', None)
        if g is None:
            return False
        h, w = g.shape[:2]
        bad = getattr(self, '_bad', None)
        ang = np.linspace(0.0, 2.0 * math.pi, 72, endpoint=False)
        ca, sa = np.cos(ang), np.sin(ang)
        ks_in, ks_out = (0.88, 0.94), (1.06, 1.12)
        xs = np.stack([cx + k * r * ca for k in ks_in + ks_out])
        ys = np.stack([cy + k * r * sa for k in ks_in + ks_out])
        xi, yi = np.round(xs).astype(int), np.round(ys).astype(int)
        ok = ((xi >= 0) & (xi < w) & (yi >= 0) & (yi < h)).all(axis=0)
        xi, yi = np.clip(xi, 0, w - 1), np.clip(yi, 0, h - 1)
        if bad is not None:
            ok &= ~(bad[yi, xi] > 0).any(axis=0)
        if int(ok.sum()) < 10:
            return False
        v = g[yi, xi].astype(np.float32)
        inside, outside = v[:2].mean(axis=0), v[2:].mean(axis=0)
        good = (outside - inside) >= float(self.cfg['dark_min'])
        if float(good[ok].mean()) < float(self.cfg['dark_frac']):
            return False
        sat = getattr(self, '_sat', None)
        if sat is not None:                                   # 黑环是灰黑色的；彩色物料(红、蓝、绿…)的边不是圆环
            s_in = sat[yi[:2], xi[:2]].astype(np.float32).mean(axis=0)
            if float(np.median(s_in[ok & good])) > float(self.cfg['dark_max_sat']):
                return False
        return True

    def __call__(self, frame, claw_mask=None, r_expect=None):
        return [(r['center'][0], r['center'][1], r['r_mean'], r['r_max']) for r in self.detect(frame, claw_mask, r_expect)]
