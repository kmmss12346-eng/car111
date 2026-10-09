"""摄像头与识别。

物料：默认用 matdet.py(抗爪子遮挡：只用没被挡住的那段圆边拟合整个圆，被挡住大半也能算准圆心，
      颜色范围和 chengxu/wuliao.py 一样，wuliao.py 在的话直接用它里面调好的)。
      配置 detector='wuliao' 可以换回原来的 wuliao.find_best_target(frame, color_id)。
圆环：默认用 ringdet.py(抗爪子遮挡：每段边缘单独拟合圆弧，同心的圆弧合成一个圆环，被爪子挡掉一截也能认)。
      配置 ring_detector='contour' 可以换回只认闭合圆的 detect_rings_subpixel(和 ring_detect.detect_rings 一样的思路)。

做的事：
  - 只开一次摄像头(/dev/video0)，每次读之前先丢掉缓冲里的旧帧，保证拿到的是"现在"的画面
  - 多帧取中值，再剔除离群点后取平均，比单帧准
  - 输出"偏差像素" = 目标像素 - 爪子像素，交给 visual_servo
爪子像素 claw_px 是爪子的轴线在画面里的位置：RAW 用于夹原料盘上的物料，RING 用于对准地上圆环。
改 claw_px.RING 1 个像素，放置位置大约移动 0.34mm(按 2.96 像素/毫米算)；以后实际放偏了，就调这个数。
"""
import json
import os
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CAL_FILE = 'vision_cal.json'        # vclaw 实测的爪子像素存在这里(优先于配置文件里的 claw_px)


class VisionError(Exception):
    pass


def _cal_path(path=None):
    p = Path(path or CAL_FILE)
    return p if p.is_absolute() else ROOT / p


def load_vision_cal(path=None):
    """读 vision_cal.json：{"claw_px": {"RAW": [u, v], "RING": [u, v]}}。没有或读不了返回 {}。"""
    p = _cal_path(path)
    try:
        with open(p, 'r', encoding='utf-8') as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except Exception:
        return {}


def save_vision_cal(kind, uv, path=None, extra=None):
    """把实测的爪子像素写进 vision_cal.json(只改这一项；extra 里的键一起存，比如 r_px、ZOBRAW)。返回文件路径。"""
    p = _cal_path(path)
    d = load_vision_cal(p)
    d.setdefault('claw_px', {})[kind] = [round(float(uv[0]), 2), round(float(uv[1]), 2)]
    for k, v in (extra or {}).items():
        d[k] = v
    tmp = str(p) + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(d, f, indent=2, ensure_ascii=False)
    os.replace(tmp, p)
    return p


def apply_vision_cal(cfg, path=None):
    """配置(dict)里的 claw_px 用 vision_cal.json 里实测的覆盖。返回同一个 dict。"""
    cal = load_vision_cal(path).get('claw_px') or {}
    cp = dict(cfg.get('claw_px') or {})
    for k, v in cal.items():
        if isinstance(v, (list, tuple)) and len(v) == 2:
            cp[k] = [float(v[0]), float(v[1])]
    cfg['claw_px'] = cp
    cal = load_vision_cal(path)
    if cal.get('r_px') and not cfg.get('r_px_seed'):
        cfg['r_px_seed'] = float(cal['r_px'])                 # vclaw 量到的物料半径(像素)
    if cal.get('ring_rmax_px') and not cfg.get('ring_rmax_cal'):
        cfg['ring_rmax_cal'] = float(cal['ring_rmax_px'])     # vclaw RING 量到的圆环最外圈半径(像素)
    return cfg


class LatestFrame:
    """后台线程一直读摄像头，只留最新的一帧和它读完的时间。
    这样要一帧时不用先扔掉缓冲里的旧帧(每扔一帧要等 33ms)，也保证拿到的是手臂动完以后才拍的画面。"""

    def __init__(self, cap, period_s=1 / 30.0):
        import threading
        self.cap, self.period = cap, float(period_s)
        self.cv = threading.Condition()
        self.frame, self.t = None, 0.0
        self.run = True
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    def _loop(self):
        while self.run:
            try:
                ok = self.cap.grab()
            except Exception:
                ok = False
            if not ok:
                time.sleep(0.01)
                continue
            t = time.monotonic()
            ok, fr = self.cap.retrieve()
            if not ok or fr is None:
                continue
            with self.cv:
                self.frame, self.t = fr, t
                self.cv.notify_all()

    def read(self, after=None, fresh=True, timeout=1.0):
        """最新的一帧。fresh=True：要在 after(默认=现在)之后才开始拍的帧(手臂动完再拍)；
        fresh=False：只要比 after 新的帧。返回 (帧, 时间) 或 (None, None)。"""
        t_req = time.monotonic() if after is None else after
        need = t_req + (self.period if fresh else 1e-4)
        end = time.monotonic() + timeout
        with self.cv:
            while self.frame is None or self.t < need:
                left = end - time.monotonic()
                if left <= 0 or not self.run:
                    return None, None
                self.cv.wait(left)
            return self.frame.copy(), self.t

    def close(self):
        self.run = False
        self.th.join(timeout=1.0)


class Camera:
    supports_after = True               # read(after=, fresh=) 可用

    def __init__(self, device='/dev/video0', width=640, height=480, fps=30, flip=None, flush=2, log=print, threaded=True):
        self.device, self.width, self.height, self.fps = device, int(width), int(height), int(fps)
        self.flip = flip            # None 不翻；cv2.flip 的参数：0 上下，1 左右，-1 两个都翻
        self.flush = int(flush)
        self.log = log
        self.cap = None
        self.threaded = bool(threaded)  # True：后台线程一直读，要帧时直接拿最新的(快，而且一定是新拍的)
        self._lf = None
        self.last_t = None

    def open(self):
        import cv2
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise VisionError(f'打不开摄像头 {self.device}')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        time.sleep(0.4)
        good = 0
        for _ in range(10):
            ok, fr = cap.read()
            good += 1 if (ok and fr is not None) else 0
            time.sleep(0.02)
        if good == 0:
            cap.release()
            raise VisionError('摄像头打开了但读不到画面')
        self.cap = cap
        if self.threaded:
            self._lf = LatestFrame(cap, 1.0 / max(1, self.fps))
        self.log(f'  摄像头 {self.device} 已打开 {self.width}x{self.height}')
        return self

    def read(self, after=None, fresh=True):
        """最新的一帧，读不到返回 None。后台线程模式：fresh=True 保证是 after(默认=现在)之后才拍的。"""
        if self.cap is None:
            return None
        if self._lf is not None:
            fr, t = self._lf.read(after=after, fresh=fresh)
            self.last_t = t
            if fr is not None and self.flip in (-1, 0, 1):
                import cv2
                fr = cv2.flip(fr, self.flip)
            return fr
        for _ in range(self.flush):
            self.cap.grab()
        ok, fr = self.cap.read()
        if not ok or fr is None:
            return None
        if self.flip in (-1, 0, 1):
            import cv2
            fr = cv2.flip(fr, self.flip)
        return fr

    def close(self):
        if self._lf is not None:
            self._lf.close()                # 先停线程再关摄像头
            self._lf = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def circle_material_detector(cfg=None, seed_r=None):
    """matdet.MaterialDetector：抗遮挡的圆拟合。返回 det(frame, color_id) -> (u, v) 或 None；det.detector.last 是完整结果(半径等)。"""
    try:
        from matdet import MaterialDetector
    except ImportError as ex:
        raise VisionError('找不到 matdet.py(或者没装 numpy/opencv)：把 matdet.py 复制到 lidar_map_north 文件夹') from ex
    md = MaterialDetector(cfg)
    if seed_r:
        md.seed_radius(seed_r)

    def det(frame, color_id):
        res = md.detect(frame, int(color_id))
        if res is None:
            return None
        c = res['center']
        return (float(c[0]), float(c[1]))
    det.detector = md
    return det


def default_material_detector(kind='circle', cfg=None, seed_r=None):
    if kind != 'wuliao':
        return circle_material_detector(cfg, seed_r)
    try:
        import wuliao
    except ImportError as ex:
        raise VisionError('找不到 wuliao.py：把 chengxu 里的 wuliao.py 复制到 lidar_map_north 文件夹') from ex

    def det(frame, color_id):
        res, _mask = wuliao.find_best_target(frame, int(color_id))
        if res is None:
            return None
        c = res['center']
        return (float(c[0]), float(c[1]))
    return det


def detect_rings_subpixel(frame):
    """和 chengxu/ring_detect.detect_rings 同一套参数和思路(Canny 边缘 -> 圆形轮廓 -> 同心圆合并成一个圆环)，
    区别：中心不取整(亚像素)；中心按半径加权平均(大圆的中心更准)；多返回一个"这组圆里最大的半径"，
    用来区分真圆环(最外圈很大)和圆环上放着的物料(只有小圆)。返回 [(x, y, 平均半径, 最大半径), ...]，从左到右。"""
    import cv2
    import numpy as np
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 1.2)
    edge = cv2.Canny(blur, 50, 150)
    contours, _ = cv2.findContours(edge, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    circles = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        per = cv2.arcLength(cnt, True)
        if per == 0 or area < 100:
            continue
        if 4 * np.pi * area / (per * per) < 0.75:
            continue
        (x, y), r = cv2.minEnclosingCircle(cnt)
        if r < 10:
            continue
        circles.append((float(x), float(y), float(r)))
    groups = []
    for c in circles:
        for g in groups:
            gx = sum(q[0] * q[2] for q in g) / sum(q[2] for q in g)
            gy = sum(q[1] * q[2] for q in g) / sum(q[2] for q in g)
            if (c[0] - gx) ** 2 + (c[1] - gy) ** 2 < 8.0 ** 2:
                g.append(c)
                break
        else:
            groups.append([c])
    rings = []
    for g in groups:
        if len(g) < 2:
            continue
        w = sum(q[2] for q in g)
        rings.append((sum(q[0] * q[2] for q in g) / w, sum(q[1] * q[2] for q in g) / w,
                      w / len(g), max(q[2] for q in g)))
    rings.sort(key=lambda t: t[0])
    return rings


def default_ring_detector(kind='arcs', cfg=None):
    """'arcs' = ringdet.RingDetector(抗遮挡，要传爪子区域)；'contour' = detect_rings_subpixel(只认闭合的圆)。"""
    if kind == 'contour':
        return detect_rings_subpixel
    try:
        from ringdet import RingDetector
    except ImportError as ex:
        raise VisionError('找不到 ringdet.py(或者没装 numpy/opencv)：把 ringdet.py 复制到 lidar_map_north 文件夹') from ex
    return RingDetector(cfg)


def _robust_mean(points, reject_px=3.0):
    """点列表 -> (均值 u, 均值 v)：先取中值，丢掉离中值超过 reject_px 的，再对剩下的取平均。"""
    n = len(points)
    mu = sorted(p[0] for p in points)[n // 2]
    mv = sorted(p[1] for p in points)[n // 2]
    keep = [p for p in points if abs(p[0] - mu) <= reject_px and abs(p[1] - mv) <= reject_px] or points
    return (sum(p[0] for p in keep) / len(keep), sum(p[1] for p in keep) / len(keep))


class Vision:
    DEFAULT = dict(
        camera=dict(device='/dev/video0', width=640, height=480, fps=30, flip=None),
        claw_px=dict(RAW=[336.8, 282.9], RING=[336.8, 282.9]),
        px_per_mm=dict(RAW=1.97, RING=2.96),   # RAW：现场画面物料半径 49 像素 / 25mm；认到物料以后按 material_diam_mm 现算
        material_diam_mm=50.0,     # 物料顶面直径(毫米)：用认到的物料半径算 RAW 的每毫米像素数；0/None = 用 px_per_mm.RAW
        detector='circle',         # 物料识别：'circle' = matdet.py(抗遮挡圆拟合)；'wuliao' = 原来的 wuliao.find_best_target
        matdet=dict(),             # 给 matdet.MaterialDetector 的参数(r_px 半径范围、claw_mask 爪子区域文件 等)
        frames=5,                  # 每次测量取几帧
        frame_gap_s=0.03,
        reject_px=3.0,
        still_px=2.0,              # 原料盘上的物料连续几次测量移动小于这么多像素才算停稳
        ring_detector='arcs',      # 圆环识别：'arcs' = ringdet.py(抗爪子遮挡)；'contour' = 原来只认闭合圆的办法
        ringdet=dict(),            # 给 ringdet.RingDetector 的参数
        ring_r_px=None,            # [最小, 最大] 圆环(组)的平均半径像素，None=不限制
        ring_rmax_px=None,         # [最小, 最大] 圆环(组)里最大的圆的半径像素；None=vclaw RING 量过就用量到的 0.85~1.2 倍，
                                   #   没量过按 px_per_mm.RING 估(最外圈直径 ring_outer_diam_mm 的 0.6~1.6 倍，放得比较宽)
        ring_outer_diam_mm=95.0,   # 圆环最外圈直径(毫米)：vclaw RING 量到最外圈半径以后，用它算 RING 的每毫米像素数
    )

    def __init__(self, cfg=None, camera=None, material_detector=None, ring_detector=None, log=print, sleep=time.sleep):
        c = {}
        for k, v in self.DEFAULT.items():
            c[k] = dict(v) if isinstance(v, dict) else v
        for k, v in (cfg or {}).items():
            if isinstance(v, dict) and isinstance(c.get(k), dict):
                c[k].update(v)
            else:
                c[k] = v
        self.cfg = c
        self.log = log
        self.sleep = sleep
        self.camera = camera
        self._material = material_detector
        self._ring = ring_detector
        self._mask_md = None            # 只用来找爪子区域的 matdet(物料识别不是 matdet 时)
        self.last_claw = None           # 最近一次认圆环时用的爪子区域(画图用)
        self.last_ring_rmax = None      # 最近一次 ring_px 认到的圆环最外圈半径(像素)

    # ------------------------------------------------------------ 基础
    def open(self):
        if self.camera is None:
            cc = self.cfg['camera']
            self.camera = Camera(cc.get('device', '/dev/video0'), cc.get('width', 640), cc.get('height', 480),
                                 cc.get('fps', 30), cc.get('flip'), flush=cc.get('flush', 2), log=self.log,
                                 threaded=cc.get('threaded', True)).open()
        return self

    def close(self):
        if self.camera is not None and hasattr(self.camera, 'close'):
            self.camera.close()

    def scale(self, kind):
        """每毫米多少像素。RAW：配置了物料直径(material_diam_mm)并且已经认到过没被挡住的物料时，
        用认到的物料半径现算(观察高度变了也准)；RING：vclaw RING 量过圆环最外圈半径时用它和 ring_outer_diam_mm 算；
        否则用配置里的 px_per_mm。"""
        if kind == 'RING' and self.cfg.get('ring_rmax_cal') and self.cfg.get('ring_outer_diam_mm'):
            return 2.0 * float(self.cfg['ring_rmax_cal']) / float(self.cfg['ring_outer_diam_mm'])
        if kind == 'RAW' and self.cfg.get('material_diam_mm'):
            md = self.material_detector_obj() if (self._material is not None or self.cfg.get('detector', 'circle') != 'wuliao') else None
            rr = list(getattr(md, 'r_ref', {}).values()) if md is not None else []
            if rr:
                rr.sort()
                return 2.0 * rr[len(rr) // 2] / float(self.cfg['material_diam_mm'])
        return float(self.cfg['px_per_mm'][kind])

    def bounds(self, kind, inset=8.0):
        """画面范围换算成"偏差像素 p = 目标 - 爪子"的上下限(四边各缩进 inset 像素)，给 visual_servo 判断目标离边缘多远。"""
        cc = self.cfg['camera']
        W, H = float(cc.get('width', 640)), float(cc.get('height', 480))
        cu, cv_ = self.claw(kind)
        return ((inset - cu, inset - cv_), (W - inset - cu, H - inset - cv_))

    def claw(self, kind):
        v = self.cfg['claw_px'][kind]
        return (float(v[0]), float(v[1]))

    def _material_det(self):
        if self._material is None:
            self._material = default_material_detector(self.cfg.get('detector', 'circle'), self.cfg.get('matdet'),
                                                       self.cfg.get('r_px_seed'))
        return self._material

    def material_detector_obj(self):
        """matdet.MaterialDetector(用的是它时)，否则 None。画图、标定爪子区域用。"""
        return getattr(self._material_det(), 'detector', None)

    def _ring_det(self):
        if self._ring is None:
            self._ring = default_ring_detector(self.cfg.get('ring_detector', 'arcs'), self.cfg.get('ringdet'))
        return self._ring

    def claw_mask(self, fr):
        """这一帧里爪子占的区域(白 = 爪子)：标定过的 claw_mask.png 优先，没有就自动找蓝色爪子。认圆环时这块的边缘不用。"""
        md = None
        try:
            md = self.material_detector_obj()
        except VisionError:
            pass
        if md is None:
            if getattr(self, '_mask_md', None) is None:
                try:
                    from matdet import MaterialDetector
                except ImportError:
                    return None
                self._mask_md = MaterialDetector(self.cfg.get('matdet'))
            md = self._mask_md
        return md.claw_mask(frame=fr)

    def _frame(self, after=None, fresh=True):
        if self.camera is None:
            self.open()
        if getattr(self.camera, 'supports_after', False) and getattr(self.camera, '_lf', None) is not None:
            return self.camera.read(after=after, fresh=fresh)
        return self.camera.read()

    def _threaded(self):
        return getattr(self.camera, '_lf', None) is not None

    def _frames(self, n):
        """连续 n 帧：第一帧一定是现在以后才拍的(手臂刚动完)，后面每帧都比前一帧新。"""
        t = None
        for i in range(n):
            fr = self._frame(after=t, fresh=(i == 0))
            t = getattr(self.camera, 'last_t', None)
            yield fr
            if i < n - 1 and not self._threaded():
                self.sleep(self.cfg['frame_gap_s'])

    # ------------------------------------------------------------ 物料(原料盘上)
    def material_px(self, color_id, n=None):
        """物料(顶面中心)的像素位置，多帧稳健平均；检测到的帧太少返回 None。"""
        n = int(n or self.cfg['frames'])
        det = self._material_det()
        pts = []
        for fr in self._frames(n):
            if fr is not None:
                p = det(fr, color_id)
                if p is not None:
                    pts.append(p)
        if len(pts) < max(2, n // 2 + 1):
            return None
        return _robust_mean(pts, self.cfg['reject_px'])

    def material_error(self, color_id, n=None):
        """物料偏差像素 = 物料 - 爪子(RAW)。看不到返回 None。"""
        p = self.material_px(color_id, n)
        if p is None:
            return None
        cu, cv_ = self.claw('RAW')
        return (p[0] - cu, p[1] - cv_)

    def wait_still(self, color_id, timeout_s=10.0, period_s=0.12, need=4):
        """等原料盘停稳：物料位置连续 need 次测量移动都小于 still_px。返回 (是否停稳, 最后的像素位置或 None)。"""
        t_end = time.monotonic() + timeout_s
        hist = []
        last = None
        while time.monotonic() < t_end:
            p = self.material_px(color_id, n=2)
            if p is None:
                hist = []
            else:
                last = p
                hist.append(p)
                hist = hist[-need:]
                if len(hist) >= need:
                    span = max(max(abs(a[0] - b[0]), abs(a[1] - b[1])) for a in hist for b in hist)
                    if span <= self.cfg['still_px']:
                        return True, last
            self.sleep(period_s)
        return False, last

    # ------------------------------------------------------------ 圆环(地上)
    def _ring_rmax_range(self):
        rm = self.cfg.get('ring_rmax_px')
        if rm:
            return float(rm[0]), float(rm[1])
        if self.cfg.get('ring_rmax_cal'):                     # vclaw RING 实测过最外圈半径
            r = float(self.cfg['ring_rmax_cal'])
            return 0.85 * r, 1.2 * r
        outer = float(self.cfg.get('ring_outer_diam_mm') or 95.0) / 2.0 * float(self.cfg['px_per_mm']['RING'])   # 最外圈半径，像素(估计)
        return 0.6 * outer, 1.6 * outer

    def _rings_in_frame(self, fr, any_size=False):
        """检测到的圆环列表 [(x, y, r, ...)]。只保留"最外圈够大"的，物料自己的圆(小)和别的杂圆不算圆环。
        any_size=True：不按大小过滤(只去掉最外圈小于 20 像素的)，vclaw RING 第一次量圆环大小用。"""
        det = self._ring_det()
        if getattr(det, 'wants_claw', False):
            self.last_claw = self.claw_mask(fr)
            rings = det(fr, self.last_claw)
        else:
            rings = det(fr)
        rr = self.cfg.get('ring_r_px')
        if rr:
            rings = [r for r in rings if rr[0] <= r[2] <= rr[1]]
        lo, hi = (20.0, 1e9) if any_size else self._ring_rmax_range()
        return [r for r in rings if len(r) < 4 or lo <= r[3] <= hi]

    def ring_px(self, n=None, expect=None, any_size=False):
        """离 expect(默认=爪子像素)最近的那个圆环的中心像素，多帧稳健平均；看不到返回 None。
        self.last_ring_rmax = 这几帧里那个圆环最外圈半径(像素)的中值。"""
        n = int(n or self.cfg['frames'])
        ex = expect or self.claw('RING')
        pts = []
        rmax = []
        for fr in self._frames(n):
            if fr is not None:
                rings = self._rings_in_frame(fr, any_size)
                if rings:
                    best = min(rings, key=lambda r: (r[0] - ex[0]) ** 2 + (r[1] - ex[1]) ** 2)
                    pts.append((best[0], best[1]))
                    if len(best) >= 4:
                        rmax.append(float(best[3]))
        self.last_ring_rmax = sorted(rmax)[len(rmax) // 2] if rmax else None
        if len(pts) < max(2, n // 2 + 1):
            return None
        return _robust_mean(pts, self.cfg['reject_px'])

    def ring_error(self, n=None):
        """圆环中心偏差像素 = 圆环中心 - 爪子(RING)。看不到返回 None。"""
        p = self.ring_px(n)
        if p is None:
            return None
        cu, cv_ = self.claw('RING')
        return (p[0] - cu, p[1] - cv_)

    # ------------------------------------------------------------ 调试
    def save_debug(self, path, kind='RING', color_id=None):
        """存一张带标注的图：爪子位置(绿十字)、检测到的圆环(黄)/物料(红十字)。"""
        import cv2
        fr = self._frame()
        if fr is None:
            return False
        out = fr.copy()
        cu, cv_ = self.claw(kind)
        cv2.drawMarker(out, (int(cu), int(cv_)), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        try:
            for ring in self._rings_in_frame(fr):
                x, y, r = ring[0], ring[1], ring[2]
                cv2.circle(out, (int(x), int(y)), int(r), (0, 255, 255), 2)
                cv2.drawMarker(out, (int(x), int(y)), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
        except VisionError:
            pass
        if color_id is not None:
            try:
                p = self._material_det()(fr, color_id)
                if p is not None:
                    md = self.material_detector_obj()
                    res = getattr(md, 'last', None) if md is not None else None
                    if res is not None:
                        cv2.circle(out, (int(round(p[0])), int(round(p[1]))), int(round(res['radius'])), (255, 255, 0), 2)
                    cv2.drawMarker(out, (int(p[0]), int(p[1])), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 30, 2)
            except VisionError:
                pass
        return bool(cv2.imwrite(path, out))
