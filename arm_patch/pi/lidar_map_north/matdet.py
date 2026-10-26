"""物料识别(抗爪子遮挡版)：找某种颜色的圆柱物料，返回它顶面圆的圆心。

原来的 wuliao.find_best_target 的问题：
  - 物料被爪子挡住一部分以后，剩下的色块不圆了，会被"圆度/面积"过滤掉 -> 一偏一点圆心就没了；
  - 圆心用的是可见部分的边缘/重心，挡住一半时圆心会往没挡住的那边偏几十像素；
  - 爪子是蓝色的，找蓝色/浅蓝物料时会把爪子当成物料。
这里的做法：
  1. 颜色掩码(颜色范围和 wuliao 一样；wuliao.py 在的话直接用它里面调好的 COLOR_RANGES)；
  2. 去掉爪子区域(claw_mask.png 标定过就用它；没有就每帧自动找画面下方连着底边的蓝色大块)；
  3. 只拿"真的物料边缘"(不挨着爪子、不挨着画面边)的轮廓点，用 RANSAC 拟合圆(会自动丢掉错误的点)，
     再用几何最小二乘精修到亚像素；可见的圆弧太短时用学到的物料半径固定半径拟合；
  4. 用"圆里面是不是这个颜色""圆周有多少被看到""边缘点有多少落在圆上"打分，选最像的那个。
被挡住 80% 时圆心误差也只有 1~2 像素(用 wuliao 的办法会偏 20~40 像素)。

接口：
  det = MaterialDetector(cfg)
  res = det.detect(frame, color_id)   -> None 或 dict(center=(u, v), radius, coverage, occluded, fill, score, ...)
  res, mask = det.find_best_target(frame, color_id)   和 wuliao.find_best_target 一样的返回格式
颜色号：1 红 2 黄 3 蓝 4 绿 5 黑 6 浅蓝
"""
import math
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent

# 和 chengxu/wuliao.py 一样的 HSV 范围(wuliao.py 在同一个文件夹时会用它里面的，以它为准)
DEFAULT_RANGES = {
    1: [((0, 70, 40), (12, 255, 255)), ((168, 70, 40), (180, 255, 255))],
    2: [((15, 60, 50), (40, 255, 255))],
    3: [((85, 130, 40), (125, 255, 240))],
    4: [((38, 40, 30), (95, 255, 255))],
    5: [((0, 0, 0), (180, 255, 105))],
    6: [((85, 50, 160), (115, 180, 255))],
}
COLOR_NAMES = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝'}

DEFAULTS = dict(
    r_px=[15.0, 170.0],          # 物料顶面圆半径的允许范围(像素)
    min_area=150,                # 色块最少多少像素(被挡住只剩一小块也要能认)
    min_edge_pts=14,             # 能用来拟合圆的边缘点最少多少个
    inlier_px=2.0,               # 边缘点离圆多近算在圆上
    ransac_iters=160,
    min_fill=0.55,               # 圆里面(爪子以外)至少多少是这个颜色
    min_inlier_ratio=0.45,
    min_coverage=0.22,           # 圆周至少看到多少(比例)
    claw_mask='claw_mask.png',   # 标定过的爪子区域(白色 = 爪子)；没有就自动找
    claw_hsv=[[90, 90, 40], [130, 255, 255]],   # 爪子的颜色(蓝色)，自动找爪子用
    claw_min_area_frac=0.03,     # 自动找爪子：连着画面底边、面积至少占画面这么多的蓝色块
    claw_margin_px=4,            # 爪子区域往外扩几像素(挨着爪子的边缘点不用)
    learn_r=True,                # 没被挡住时记下物料半径，被挡得多时用它
    pp=None,                     # 摄像头光心在画面里的位置(None = 画面中心)。物料离它越远，越能看到物料侧面
    far_side=True,               # 优先用"离光心远的那半圈"边缘拟合：那半圈一定是顶面的边，近的那半圈可能是侧面/底边
    no_lblue=True,               # (10-19 用户：省赛没有浅蓝、黑)浅蓝不单独认：浅蓝的颜色范围并进蓝色，蓝色不再按饱和度和浅蓝分开
                                 #   (实车：蓝色物料在爪子正下方显得淡，被判成了浅蓝，蓝色认不出来)。要分浅蓝就设 False
)


def _ranges():
    try:
        import wuliao                                   # 用户在 wuliao.py 里调过颜色就用调过的
        rg = getattr(wuliao, 'COLOR_RANGES', None)
        if isinstance(rg, dict) and all(k in rg for k in DEFAULT_RANGES):
            return rg
    except Exception:
        pass
    return DEFAULT_RANGES


# ---------------------------------------------------------------- 圆拟合
def _circle3(p):
    (ax, ay), (bx, by), (cx, cy) = p
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-6:
        return None
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    return ux, uy, math.hypot(ax - ux, ay - uy)


def _refine(pts, cx, cy, r, fixed_r=None, iters=20):
    """几何最小二乘(Gauss-Newton)：让点到圆的距离平方和最小。fixed_r 给了就只调圆心。"""
    for _ in range(iters):
        dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
        d = np.sqrt(dx * dx + dy * dy) + 1e-9
        if fixed_r is None:
            J = np.column_stack((-dx / d, -dy / d, -np.ones_like(d)))
            res = d - r
        else:
            J = np.column_stack((-dx / d, -dy / d))
            res = d - fixed_r
        step = np.linalg.lstsq(J, -res, rcond=None)[0]
        cx += float(step[0])
        cy += float(step[1])
        if fixed_r is None:
            r += float(step[2])
        if abs(step[0]) < 1e-4 and abs(step[1]) < 1e-4:
            break
    return cx, cy, (r if fixed_r is None else float(fixed_r))


def _coverage(pts, cx, cy, bins=36):
    ang = np.arctan2(pts[:, 1] - cy, pts[:, 0] - cx)
    idx = ((ang + math.pi) / (2 * math.pi) * bins).astype(int) % bins
    return len(np.unique(idx)) / float(bins)


_TRIPLES_OK = None          # 这台电脑的 numpy：一次抽完(_triples_fast)和一组一组 rng.choice 抽到的一样不一样(第一次用时查一次)


def _triples_fast(n, iters, rng):
    """(10-26)和 [rng.choice(n, 3, replace=False) for _ in range(iters)] 抽到的完全一样，一次抽完(快几十倍)：
    numpy 从 n 个里不放回抽 3 个用的是 Floyd 抽法再打乱，每组用 5 个 32 位随机数(Lemire 有界整数)。
    万一碰到 Lemire 要重抽的(概率约千万分之一)返回 None(调用的地方退回一组一组抽)。"""
    u = rng.integers(0, 1 << 32, size=(iters, 5), dtype=np.uint32).astype(np.uint64)
    js = np.array([n - 3, n - 2, n - 1, 2, 1], np.uint64)
    ex = js + np.uint64(1)
    m = u * ex
    if np.any((m & np.uint64(0xFFFFFFFF)) < (np.uint64(0xFFFFFFFF) - js) % ex):
        return None
    v = (m >> np.uint64(32)).astype(np.int64)
    a0 = v[:, 0]
    a1 = np.where(v[:, 1] == a0, n - 2, v[:, 1])
    a2 = np.where((v[:, 2] == a0) | (v[:, 2] == a1), n - 1, v[:, 2])
    D = np.stack([a0, a1, a2], axis=1)
    rows = np.arange(iters)
    for col, j in ((2, v[:, 3]), (1, v[:, 4])):         # 打乱：第 2 个和 [0, 2] 里抽的那个换，第 1 个和 [0, 1] 里抽的那个换
        t = D[rows, j].copy()
        D[rows, j] = D[rows, col]
        D[rows, col] = t
    return D.astype(np.intp)


def _triples(n, iters, rng):
    """RANSAC 每次用的三个点(下标)：和以前一组一组 rng.choice 抽到的一样。"""
    global _TRIPLES_OK
    if _TRIPLES_OK is None:
        try:
            _TRIPLES_OK = all(np.array_equal(_triples_fast(k, 40, np.random.default_rng(s + k)),
                                             _triples_slow(k, 40, np.random.default_rng(s + k)))
                              for k in (6, 37, 500, 4096) for s in (7, 13))
        except Exception:
            _TRIPLES_OK = False
    if _TRIPLES_OK:
        st = rng.bit_generator.state
        t = _triples_fast(n, iters, rng)
        if t is not None:
            return t
        rng.bit_generator.state = st                   # 要重抽：退回一组一组抽(从同一个状态开始，抽到的还是一样)
    return _triples_slow(n, iters, rng)


def _triples_slow(n, iters, rng):
    return np.array([rng.choice(n, 3, replace=False) for _ in range(iters)], np.intp)


def _ransac_best(pts, rmin, rmax, thr, iters, rng):
    """(10-26 用户"识别……太慢了")RANSAC 一次算完：三个点一组还是按原来的办法抽(同样的随机数，抽到的完全一样)，
    圆心、半径、每个圆上有多少点一起算(以前一组一组算，Python 循环占了认一个颜色的一半时间)。
    挑的规则也一样：内点最多的、一样多取先抽到的。返回 (cx, cy, r) 或 None。"""
    n = len(pts)
    tri = _triples(n, iters, rng)
    P = pts[tri]
    ax, ay, bx, by, cx, cy = P[:, 0, 0], P[:, 0, 1], P[:, 1, 0], P[:, 1, 1], P[:, 2, 0], P[:, 2, 1]
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    ok = np.abs(d) >= 1e-6
    ds = np.where(ok, d, 1.0)
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / ds
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / ds
    r = np.array([math.hypot(x, y) for x, y in zip((ax - ux).tolist(), (ay - uy).tolist())])   # 和 _circle3 一样用 math.hypot
    ok &= (rmin <= r) & (r <= rmax)
    if not ok.any():
        return None
    idx = np.nonzero(ok)[0]
    D = np.abs(np.hypot(pts[None, :, 0] - ux[idx, None], pts[None, :, 1] - uy[idx, None]) - r[idx, None])
    j = int(idx[int(np.argmax(np.count_nonzero(D < thr, axis=1)))])
    return float(ux[j]), float(uy[j]), float(r[j])


def fit_circle(pts, rmin, rmax, r_ref=None, thr=2.0, iters=160, seed=7):
    """RANSAC 找圆，再精修。返回 (cx, cy, r, 内点数, 覆盖比例, 内点) 或 None。"""
    n = len(pts)
    if n < 6:
        return None
    rng = np.random.default_rng(seed + n)
    best = _ransac_best(pts, rmin, rmax, thr, iters, rng)
    if best is None:
        return None
    cx, cy, r = best
    for _ in range(2):                                   # 精修两轮：内点 -> 拟合 -> 再选内点
        d = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
        inl = pts[d < thr * 1.25]
        if len(inl) < 6:
            return None
        cx, cy, r = _refine(inl, cx, cy, r)
    cov = _coverage(inl, cx, cy)
    if r_ref:
        # 看到的圆弧短(被挡得多)半径就不准：固定成学到的半径，只拟合圆心
        if cov < 0.5 or (abs(r - r_ref) > 0.12 * r_ref and cov < 0.8):
            cx, cy, r = _refine(inl, cx, cy, r_ref, fixed_r=r_ref)
            d = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
            inl = pts[d < thr * 1.25]
            if len(inl) < 6:
                return None
            cov = _coverage(inl, cx, cy)
    if not (rmin <= r <= rmax):
        return None
    return cx, cy, r, len(inl), cov, inl


# ---------------------------------------------------------------- (10-24)现场标定颜色
def _in_ranges(hsv, ranges):
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        m |= cv2.inRange(hsv, tuple(int(v) for v in lo), tuple(int(v) for v in hi))
    return m


def calibrate_color(frame, color_id, center, r_px, claw=None):
    """(10-24 用户"黄色的物料识别不太准确，半天识别不出来")现场标定一种颜色的 HSV 范围。
    物料放在 center(爪子点 / 认到的圆心)正下方：取 center 周围半径 r_px 以内(去掉爪子)的像素，按它们实际的色相、饱和度、亮度
    定范围(顶面反光发白的地方饱和度低，固定的"饱和度 >= 60"就漏掉了)；再看画面别处(转盘、地面)有多少也落进这个范围，
    多了就把饱和度下限往上提(物料本身至少还留 85%)。
    返回 dict(ranges=[[lo, hi], …], keep=物料像素落在范围里的比例, fp=别处落在范围里的比例, text=说明)。像素太少抛 ValueError。"""
    color_id = int(color_id)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    cx, cy, r = int(round(center[0])), int(round(center[1])), max(4, int(round(r_px)))
    disk = np.zeros((h, w), np.uint8)
    cv2.circle(disk, (cx, cy), r, 255, -1)
    if claw is not None and claw.any():
        disk[claw > 0] = 0
    px = hsv[disk > 0].astype(np.float64)
    if len(px) < 80:
        raise ValueError(f'绿十字附近能用的像素太少({len(px)} 个)：物料放到绿十字正下方，别让爪子挡住')
    H, S, V = px[:, 0], px[:, 1], px[:, 2]
    bg = np.full((h, w), 255, np.uint8)                   # 别处：去掉物料(取样圈的 3 倍以内)、爪子
    cv2.circle(bg, (cx, cy), int(3.0 * r), 0, -1)
    if claw is not None and claw.any():
        bg[claw > 0] = 0
    n_bg = max(1, int(np.count_nonzero(bg)))
    n_in = max(1, int(np.count_nonzero(disk)))

    def score(rg):
        m = _in_ranges(hsv, rg)
        return (np.count_nonzero(cv2.bitwise_and(m, disk)) / float(n_in),
                np.count_nonzero(cv2.bitwise_and(m, bg)) / float(n_bg))

    def p(a, q):
        return float(np.percentile(a, q))

    if color_id == 5:                                     # 黑：只看亮度(多暗算黑)
        vhi = int(min(200, p(V, 97) + 25))
        ranges = [[[0, 0, 0], [180, 255, vhi]]]
        while True:
            keep, fp = score(ranges)
            if fp <= 0.02 or vhi <= 40 or keep < 0.9:
                break
            vhi -= 5
            ranges = [[[0, 0, 0], [180, 255, vhi]]]
        text = f'亮度 V {p(V, 5):.0f}~{p(V, 95):.0f}：V <= {vhi} 算黑'
    else:
        if color_id == 1:                                 # 红色跨过 0/180：转半圈再算
            hs = (H + 90.0) % 180.0
            lo, hi = p(hs, 3) - 6.0 - 90.0, p(hs, 97) + 6.0 - 90.0
        else:
            lo, hi = max(0.0, p(H, 3) - 6.0), min(180.0, p(H, 97) + 6.0)
        s_lo = int(max(25.0, p(S, 5) - 25.0))
        v_lo = int(max(35.0, p(V, 5) - 45.0))

        def make(s_lo_):
            if lo < 0:
                return [[[int(round(lo + 180)), s_lo_, v_lo], [180, 255, 255]], [[0, s_lo_, v_lo], [int(round(hi)), 255, 255]]]
            if hi > 180:
                return [[[int(round(lo)), s_lo_, v_lo], [180, 255, 255]], [[0, s_lo_, v_lo], [int(round(hi - 180)), 255, 255]]]
            return [[[int(round(lo)), s_lo_, v_lo], [int(round(hi)), 255, 255]]]
        ranges = make(s_lo)
        keep, fp = score(ranges)
        while fp > 0.02 and s_lo < 200:                     # 转盘、地面也被认成这个颜色：饱和度下限往上提
            k2, f2 = score(make(s_lo + 5))
            if k2 < 0.85:
                break
            s_lo += 5
            ranges, keep, fp = make(s_lo), k2, f2
        text = (f'色相 H {p(H, 5):.0f}~{p(H, 95):.0f}，饱和度 S {p(S, 5):.0f}~{p(S, 95):.0f}，亮度 V {p(V, 5):.0f}~{p(V, 95):.0f}')
    keep, fp = score(ranges)
    return dict(ranges=ranges, keep=float(keep), fp=float(fp), text=text)


# ---------------------------------------------------------------- 检测器
class MaterialDetector:
    def __init__(self, cfg=None, folder=None):
        c = dict(DEFAULTS)
        c.update(cfg or {})
        self.cfg = c
        self.folder = Path(folder) if folder else ROOT
        self.set_ranges(c.get('ranges'))
        self.r_ref = {}                 # 颜色号 -> 学到的物料半径(像素)
        self._r_hist = {}
        self._claw_static = None
        self._claw_loaded = False
        self._auto_cache = None         # 找非蓝色物料时自动找到的爪子区域(爪子相对摄像头不动)，找蓝色物料时拿来用
        self.last = None                # 最近一次 detect 的结果(画图用)
        self.last_claw = None           # 最近一次用的爪子区域

    def set_ranges(self, overrides=None):
        """颜色范围：wuliao.py / 默认的，再用 overrides({颜色号: [[lo, hi], …]}，vlive 里按 k 现场标定、存在 vision_cal.json 的
        color_ranges)盖掉那几种颜色。不认浅蓝(no_lblue)时浅蓝的范围并进蓝色。"""
        rg = dict(_ranges())
        for k, v in (overrides or {}).items():
            try:
                rg[int(k)] = [(tuple(int(x) for x in lo), tuple(int(x) for x in hi)) for lo, hi in v]
            except (TypeError, ValueError):
                continue
        if self.cfg.get('no_lblue') and 3 in rg and 6 in rg:
            rg[3] = list(rg[3]) + [r for r in rg[6] if r not in rg[3]]
        self.ranges = rg

    def seed_radius(self, r_px, colors=None):
        """用标定时量到的物料半径(像素)当初始值(所有物料一样大)。被挡得多时用它固定半径拟合。"""
        if not r_px or r_px <= 0:
            return
        for c in (colors or DEFAULT_RANGES.keys()):
            self.r_ref.setdefault(int(c), float(r_px))
            self._r_hist.setdefault(int(c), [float(r_px)])

    # ---------------------------------------------------------- 爪子区域
    def claw_path(self):
        p = Path(self.cfg.get('claw_mask') or 'claw_mask.png')
        return p if p.is_absolute() else self.folder / p

    def _load_static(self, shape=None):
        if not self._claw_loaded:
            self._claw_loaded = True
            p = self.claw_path()
            if p.exists():
                m = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
                if m is not None:
                    m = (m > 127).astype(np.uint8) * 255
                    if m.mean() / 255.0 >= self.cfg['claw_min_area_frac']:       # 空的/几乎没有爪子的文件不用
                        self._claw_static = m
        m = self._claw_static
        if m is not None and shape is not None and m.shape[:2] != tuple(shape[:2]):
            m = cv2.resize(m, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
            self._claw_static = m
        return m

    def auto_claw_mask(self, hsv):
        """自动找爪子：画面下方连着底边的大块爪子颜色，再把每一列爪子最上面那点以下都算爪子(盖住螺丝、白色圆片)。"""
        lo, hi = self.cfg['claw_hsv']
        blue = cv2.inRange(hsv, tuple(int(v) for v in lo), tuple(int(v) for v in hi))
        h, w = blue.shape
        n, lab, st, _ = cv2.connectedComponentsWithStats(blue, connectivity=8)
        m = np.zeros_like(blue)
        min_a = self.cfg['claw_min_area_frac'] * h * w
        box = None
        for i in range(1, n):
            x, y, bw, bh, a = st[i]
            if y + bh >= h - 2 and a >= min_a:
                m[lab == i] = 255
                box = (x, y, x + bw, y + bh) if box is None else (min(box[0], x), min(box[1], y), max(box[2], x + bw), max(box[3], y + bh))
        if box is None:
            return m
        # (10-26)只在爪子那一块(往外多留两个核半径)做闭运算：和整幅画面做完全一样(外面全是 0，核碰不到)，省一半多时间
        x0, y0, x1, y1 = max(0, box[0] - 32), max(0, box[1] - 32), min(w, box[2] + 32), min(h, box[3] + 32)
        sub = cv2.morphologyEx(m[y0:y1, x0:x1], cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31)))
        m = np.zeros_like(blue)
        m[y0:y1, x0:x1] = sub
        cols = m.max(axis=0) > 0
        top = np.argmax(m > 0, axis=0)
        rows = np.arange(h)[:, None]
        m = ((rows >= top[None, :]) & cols[None, :]).astype(np.uint8) * 255
        return m

    def claw_mask(self, frame=None, hsv=None, color_id=None):
        """爪子区域：标定过的 claw_mask.png 优先；没有就自动找蓝色爪子。
        找蓝色/浅蓝物料(3、6)时不能现找：物料挨着爪子会和爪子连成一块，被当成爪子去掉 ->
        用之前找别的颜色时记下的爪子区域；一次都没有就只能现找(可能认错，所以蓝色物料一定要先 vmask)。"""
        if hsv is None:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m = self._load_static(hsv.shape)
        if m is not None:
            return m
        cache = self._auto_cache
        if color_id in (3, 6):
            if cache is not None and cache.shape[:2] == hsv.shape[:2]:
                return cache
            return self.auto_claw_mask(hsv)
        m = self.auto_claw_mask(hsv)
        if color_id is not None and m.any():
            # 蓝色物料挨着爪子会连成一块，爪子区域只会变大：明显变大的不记(更小的总是记，记错了也能自己恢复)
            if cache is None or cache.shape != m.shape or np.count_nonzero(m) <= 1.03 * np.count_nonzero(cache):
                self._auto_cache = m
        return m

    def has_static_mask(self):
        return self._load_static() is not None

    def need_vmask(self, color_id):
        """找蓝色/浅蓝物料，但既没有 claw_mask.png 也没有记下的爪子区域：可能把爪子当物料，要先 vmask。"""
        return int(color_id) in (3, 6) and self._auto_cache is None and not self.has_static_mask()

    def save_claw_mask(self, frames, keep_clear=None):
        """标定爪子区域：手臂抬高、爪子张开、画面里爪子附近没有物料时拍几帧，取"大多数帧都是爪子"的像素。
        keep_clear = 爪子点(夹物料的位置)：它落在爪子区域里说明爪子没张开或者爪子里有蓝色物料，不保存(抛 ValueError)。"""
        acc = None
        n = 0
        for fr in frames:
            if fr is None:
                continue
            hsv = cv2.cvtColor(fr, cv2.COLOR_BGR2HSV)
            m = (self.auto_claw_mask(hsv) > 0).astype(np.float32)
            acc = m if acc is None else acc + m
            n += 1
        if not n:
            return None
        mask = ((acc / n) >= 0.5).astype(np.uint8) * 255
        frac = float(mask.mean() / 255.0)
        if frac < self.cfg['claw_min_area_frac']:
            raise ValueError(f'画面里几乎没找到爪子(占 {frac * 100:.0f}%)：没有保存。手臂要在 OBS RAW 姿态、爪子在画面下方')
        if keep_clear is not None:
            u, v = int(round(keep_clear[0])), int(round(keep_clear[1]))
            if mask[max(0, v - 6):v + 7, max(0, u - 6):u + 7].any():
                raise ValueError('爪子点被算成了爪子区域(爪子没张开，或者爪子里/旁边有蓝色物料)：没有保存。'
                                 'arm CLAW O 张开、把爪子附近的物料拿开再做')
        p = self.claw_path()
        if not cv2.imwrite(str(p), mask):
            raise ValueError(f'写不了 {p}')
        self._claw_static, self._claw_loaded = mask, True
        return p, frac

    # ---------------------------------------------------------- 颜色
    def color_mask(self, hsv, color_id):
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.ranges.get(int(color_id), []):
            m |= cv2.inRange(hsv, tuple(int(v) for v in lo), tuple(int(v) for v in hi))
        return m

    def _blue_ok(self, hsv_px, color_id):
        """蓝(3)和浅蓝(6)的范围有重叠：浅蓝更亮、颜色更淡。"""
        if color_id not in (3, 6) or len(hsv_px) == 0:
            return True
        if color_id == 3 and self.cfg.get('no_lblue'):
            return True                                       # 不认浅蓝：淡一点的蓝也是蓝
        s = float(np.median(hsv_px[:, 1]))
        try:
            s_light = float(self.ranges[6][0][1][1])          # 浅蓝范围的饱和度上限(wuliao 里是 180)
        except Exception:
            s_light = 180.0
        light = s <= s_light                                  # 只看饱和度(亮度随光线变，饱和度不怎么变)
        return light if color_id == 6 else not light

    def _claw_shared(self, hsv, color_id, shared):
        """detect_many 用：和 claw_mask(hsv=, color_id=) 一样的结果，但同一帧自动找爪子只找一次(找一次要十几毫秒)。"""
        m = self._load_static(hsv.shape)
        if m is not None:
            return m
        cache = self._auto_cache
        if color_id in (3, 6) and cache is not None and cache.shape[:2] == hsv.shape[:2]:
            return cache
        if 'auto' not in shared:
            shared['auto'] = self.auto_claw_mask(hsv)
        m = shared['auto']
        if color_id not in (3, 6) and m.any() and not shared.get('cached'):
            shared['cached'] = True                       # 同一帧同一份：记一次和每个颜色都记一次结果一样
            if cache is None or cache.shape != m.shape or np.count_nonzero(m) <= 1.03 * np.count_nonzero(cache):
                self._auto_cache = m
        return m

    # ---------------------------------------------------------- 主函数
    def detect_many(self, frame, color_ids, claw=None):
        """(10-26 用户"识别……太慢了")同一帧认好几个颜色：{颜色号: 结果 或 None}。HSV、爪子区域、不能用的边缘这一帧只算一次
        (一个一个 detect 时每个颜色都重算)；结果和按 color_ids 的顺序一个一个 detect 完全一样(last、last_claw 也是最后一个颜色的)。"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        shared = {}
        out = {}
        for c in color_ids:
            out[int(c)] = self._detect_hsv(hsv, int(c), claw, shared)
        return out

    def detect(self, frame, color_id, claw=None):
        return self._detect_hsv(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), int(color_id), claw, None)

    def _detect_hsv(self, hsv, color_id, claw, shared):
        cfg = self.cfg
        h, w = hsv.shape[:2]
        if claw is not None:
            cm = claw
        elif shared is not None:
            cm = self._claw_shared(hsv, color_id, shared)
        else:
            cm = self.claw_mask(hsv=hsv, color_id=color_id)
        self.last_claw = cm
        mask = self.color_mask(hsv, color_id)
        if cm is not None and cm.any():
            mask[cm > 0] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours = [c for c in contours if cv2.contourArea(c) >= cfg['min_area']]
        if not contours:
            self.last = None
            return None
        contours.sort(key=cv2.contourArea, reverse=True)

        # 不能用的边缘：挨着爪子、挨着画面边(那里不是物料真正的边)。detect_many：同一个爪子区域只算一次
        got = next((b for m_, b in (shared or {}).get('bad', ()) if m_ is cm), None)
        if got is not None:
            bad = got
        else:
            bad = np.zeros((h, w), np.uint8)
            if cm is not None and cm.any():
                k = 2 * int(cfg['claw_margin_px']) + 1
                bad = cv2.dilate(cm, np.ones((k, k), np.uint8))
            bad[:3, :] = 255
            bad[-3:, :] = 255
            bad[:, :3] = 255
            bad[:, -3:] = 255
            if shared is not None:
                shared.setdefault('bad', []).append((cm, bad))

        rmin, rmax = cfg['r_px']
        r_ref = self.r_ref.get(color_id)
        best = None
        ppx, ppy = cfg['pp'] if cfg.get('pp') else (w / 2.0, h / 2.0)
        for c in contours[:5]:
            pts = c[:, 0, :]
            free = pts[bad[pts[:, 1], pts[:, 0]] == 0].astype(np.float64)
            if len(free) < cfg['min_edge_pts']:
                continue
            fits = []
            if cfg.get('far_side'):
                # 物料离光心远时会露出侧面(在靠光心那边)，那边的轮廓是侧面/底边，不是顶面的圆：另外只用远的那半圈拟合一次
                mo = cv2.moments(c)
                if mo['m00'] > 0:
                    gx, gy = mo['m10'] / mo['m00'], mo['m01'] / mo['m00']
                    ux, uy = gx - ppx, gy - ppy
                    dist = math.hypot(ux, uy)
                    if dist > 25.0:
                        ux, uy = ux / dist, uy / dist
                        vx, vy = free[:, 0] - gx, free[:, 1] - gy
                        far = free[(vx * ux + vy * uy) > -0.25 * np.hypot(vx, vy)]
                        if len(far) >= max(cfg['min_edge_pts'], 0.3 * len(free)):
                            f = fit_circle(far, rmin, rmax, r_ref=r_ref, thr=cfg['inlier_px'], iters=cfg['ransac_iters'])
                            if f is not None:
                                fx, fy, fr = f[0], f[1], f[2]
                                d_all = np.abs(np.hypot(free[:, 0] - fx, free[:, 1] - fy) - fr)
                                inl_all = free[d_all < cfg['inlier_px'] * 1.25]
                                fits.append(((fx, fy, fr, max(f[3], len(inl_all)), max(f[4], _coverage(inl_all, fx, fy))), 0.15))
            f = fit_circle(free, rmin, rmax, r_ref=r_ref, thr=cfg['inlier_px'], iters=cfg['ransac_iters'])
            if f is not None:
                fits.append((f[:5], 0.0))
            for (cx, cy, r, k, cov), bonus in fits:      # 远半圈的拟合加分：露出侧面时，整圈轮廓拟合出来的是顶面和底面中间的圆，偏几个像素
                cand = self._evaluate(c, cx, cy, r, k, cov, len(free), mask, cm, bad, hsv, color_id, r_ref, h, w)
                if cand is None:
                    continue
                cand['score'] += bonus
                if best is None or cand['score'] > best['score']:
                    best = cand
        if best is not None and cfg.get('learn_r') and best['occluded'] < 0.03 and best['coverage'] > 0.8:
            hist = self._r_hist.setdefault(color_id, [])
            hist.append(best['radius'])
            del hist[:-15]
            self.r_ref[color_id] = float(np.median(hist))
        self.last = best
        return best

    def _evaluate(self, c, cx, cy, r, k, cov, n_free, mask, cm, bad, hsv, color_id, r_ref, h, w):
        """给一个拟合出来的圆打分；不像物料返回 None。"""
        cfg = self.cfg
        inl_ratio = min(1.0, k / float(max(n_free, 1)))
        disk = np.zeros((h, w), np.uint8)                 # 圆里面(去掉爪子)有多少是这个颜色
        cv2.circle(disk, (int(round(cx)), int(round(cy))), max(1, int(r * 0.9)), 255, -1)
        vis_disk = cv2.bitwise_and(disk, cv2.bitwise_not(cm)) if (cm is not None and cm.any()) else disk
        nvis = int(np.count_nonzero(vis_disk))
        if nvis < cfg['min_area'] * 0.5:
            return None
        fill = int(np.count_nonzero(cv2.bitwise_and(vis_disk, mask))) / float(nvis)
        ring = np.zeros((h, w), np.uint8)
        cv2.circle(ring, (int(round(cx)), int(round(cy))), int(round(r)), 255, 2)
        nring = int(np.count_nonzero(ring))
        occl = int(np.count_nonzero(cv2.bitwise_and(ring, bad))) / float(max(nring, 1))
        if fill < cfg['min_fill'] or inl_ratio < cfg['min_inlier_ratio'] or cov < cfg['min_coverage']:
            return None
        if not self._blue_ok(hsv[vis_disk > 0], color_id):
            return None
        if color_id == 5:                                  # 黑色：光线暗时深色的有色物料也落进黑色范围，看彩度(S*V)区分
            px = hsv[vis_disk > 0].astype(np.float32)
            if float(np.median(px[:, 1] * px[:, 2] / 255.0)) > 35.0:
                return None
        if math.pi * r * r > 0 and np.count_nonzero(disk) < 0.45 * math.pi * (0.9 * r) ** 2:
            return None                                    # 圆心跑到画面外、圆大半在画面外：不可信
        score = 0.35 * fill + 0.25 * inl_ratio + 0.25 * min(1.0, cov / 0.8) + 0.15 * min(1.0, k / 120.0)
        if r_ref:
            score -= 0.5 * min(1.0, abs(r - r_ref) / r_ref)
        return dict(id=color_id, color=COLOR_NAMES.get(color_id, str(color_id)), center=(float(cx), float(cy)),
                    radius=float(r), coverage=float(cov), occluded=float(occl), fill=float(fill),
                    inlier_ratio=float(inl_ratio), n_inliers=int(k), score=float(score),
                    bbox=tuple(int(v) for v in cv2.boundingRect(c)), area=float(cv2.contourArea(c)))

    def find_best_target(self, frame, color_id):
        """和 wuliao.find_best_target 一样的返回：(结果 dict 或 None, 颜色掩码)。"""
        res = self.detect(frame, color_id)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return res, self.color_mask(hsv, color_id)
