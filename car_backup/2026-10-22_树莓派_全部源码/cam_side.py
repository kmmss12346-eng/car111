"""侧边摄像头(右前角)：测车右侧到"和车身平行的分界线"(灰色场地/白色边框)的距离。
标定(2026-10-04，1280×720，车头和分界线平行)：
  在画面中间那一行，找那条斜着的分界线的左右位置 x(像素)，
  距离(mm) = A2*x² + A1*x + A0   —— 从右前角蓝色摄像头座底边量到分界线，横向。
  标定范围 10～130mm，超出这个范围是外推，误差会变大。"""
import math
import cv2
import numpy as np

# 标定(2026-10-04 第二次，摄像头调过角度后)：在第 y 行，分界线的左右位置 x(像素)，
# 距离mm = c2*x² + c1*x + c0 （从右前角蓝色摄像头座底边横向量到灰/白分界线，车头和分界线平行）
ROW_FITS = [   # (行号y, (c2, c1, c0))
    (360, (2.077271332890736e-05, -0.15139118353417427, 143.5743843589403)),
    (540, (3.5307684780036175e-05, -0.1963560007392011, 192.3360043716526)),
    (690, (2.742623529695094e-05, -0.20907165694217608, 232.8374870468701)),
]
CAL_MIN_MM, CAL_MAX_MM = 30.0, 120.0

# 如果跑过 cam_side_cal.py(摄像头又调过角度后重新标定)，用它写的标定文件覆盖上面的值
try:
    import json as _json
    import os as _os
    _p = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), 'cam_side_cal.json')
    if _os.path.exists(_p):
        _d = _json.load(open(_p, encoding='utf-8'))
        if _d.get('method') == 'row_profile':
            ROW_FITS = [(float(y), tuple(float(v) for v in c)) for y, c in _d['row_fits']]
            CAL_MIN_MM, CAL_MAX_MM = float(_d['cal_min_mm']), float(_d['cal_max_mm'])
except Exception as _e:
    print('cam_side_cal.json 读取失败，用默认标定：', _e)


def _candidates(g, step=6, half=12, min_drop=8.0, per_row=3, dark=90):
    """每一行找最多 3 个"左边亮→右边暗"的位置；旁边有黑色(字、黑线)的不要。"""
    h, w = g.shape
    k = np.ones(15, np.float32) / 15
    pts = []
    for y in range(0, h, step):
        r = np.convolve(g[y], k, mode='same')
        d = np.zeros(w, np.float32)
        d[half:w - half] = r[2 * half:] - r[:w - 2 * half]     # 右边减左边：白→灰是负数
        d[:half + 2] = 0
        d[-40:] = 0                                            # 画面最右边有一条黑边，不算
        mn = np.minimum.reduce([np.roll(g[y], s) for s in range(-30, 31, 5)])
        d[mn < dark] = 0
        for _ in range(per_row):
            x = int(np.argmin(d))
            if -d[x] < min_drop:
                break
            pts.append((y, x))
            d[max(0, x - 60):x + 60] = 0
    return np.array(pts, float)


def edge_fit(img, amin=0.7, amax=2.5, iters=500, tol=12.0):
    """找画面里那条"和车身平行"的灰/白分界线(在画面里从左上往右下斜，斜率 0.7～2.5)。
    逐行找亮→暗的位置，再用随机抽样拟合直线(RANSAC)，只认斜率对的那条，字、横线、竖边都排除。
    返回 (a, b, 最上行, 最下行, 点数)：分界线 x = a*y + b；找不到返回 None。"""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    g = cv2.GaussianBlur(g, (9, 9), 0)
    P = _candidates(g)
    if len(P) < 15:
        return None
    rng = np.random.default_rng(0)          # 固定随机种子：同一张图每次结果一样
    best = None
    for _ in range(iters):
        i, j = rng.choice(len(P), 2, replace=False)
        (y1, x1), (y2, x2) = P[i], P[j]
        if abs(y2 - y1) < 30:
            continue
        a = (x2 - x1) / (y2 - y1)
        if not (amin <= a <= amax):
            continue
        b = x1 - a * y1
        inl = np.abs(a * P[:, 0] + b - P[:, 1]) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 15:
        return None
    a, b = np.polyfit(P[best, 0], P[best, 1], 1)
    return float(a), float(b), float(P[best, 0].min()), float(P[best, 0].max()), int(best.sum())


def edge_x(img, row_frac=0.5, **_):
    """兼容旧接口：返回 (第 row_frac 行上分界线的 x, 斜率)。"""
    r = edge_fit(img)
    if r is None:
        return None
    a, b, y0, y1, n = r
    y = img.shape[0] * row_frac
    return a * y + b, a


def side_distance(img):
    """返回 (距离mm, 斜率, 是否在标定范围内)；找不到分界线返回 None。图像缩放到 1280×720 再算。"""
    h, w = img.shape[:2]
    if (w, h) != (1280, 720):
        img = cv2.resize(img, (1280, 720))
    r = edge_fit(img)
    if r is None:
        return None
    a, b, y0, y1, n = r
    for y, (c2, c1, c0) in ROW_FITS:
        if not (y0 - 60 <= y <= y1 + 60):
            continue
        x = a * y + b
        if not (10 <= x <= 1240):
            continue
        d = c2 * x * x + c1 * x + c0
        return d, a, (CAL_MIN_MM - 15 <= d <= CAL_MAX_MM + 15)
    return None


def measure(device=0, n=7, warmup=6):
    """打开侧边摄像头，拍 n 张取中位数。返回 dict(mm, ang, inside, ok_frames) ；一张都没找到分界线返回 None。
    每次用时约 1～2 秒(打开摄像头 + 拍照)，用完就关，不占着摄像头。"""
    import statistics
    import time
    import os
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        return None
    last = None
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        for _ in range(warmup):
            cap.read()
        vals = []
        for _ in range(n):
            ok, f = cap.read()
            if ok:
                last = f
                r = side_distance(f)
                if r is not None:
                    vals.append(r)
            time.sleep(0.03)
    finally:
        cap.release()
    if last is not None:
        try:
            cv2.imwrite(os.path.expanduser('~/cam_last.jpg'), last)     # 最后一张存下来，出问题时发给我看
        except Exception:
            pass
    if len(vals) < max(3, n // 2):
        return None
    d = statistics.median(v[0] for v in vals)
    a = statistics.median(v[1] for v in vals)
    spread = max(v[0] for v in vals) - min(v[0] for v in vals)
    return dict(mm=float(d), ang=float(a), inside=all(v[2] for v in vals), ok_frames=len(vals), spread=float(spread))
