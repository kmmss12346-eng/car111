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
claw_px.PICK：圆环上方观察时，物料正好在爪子下面，它顶面圆心在画面里的位置(物料顶面比圆环高，和 RING 不是同一个点)。
粗加工区第一次放下物料以后自动量一次存起来；取回、码垛时圆环白心被物料盖住，就直接把物料顶面对到这个点。
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
    """读 vision_cal.json：{"claw_px": {"RAW": [u, v], "RING": [u, v]}}。没有或读不了返回 {}。path='' = 不用文件(模拟测试)。"""
    if path == '':
        return {}
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


def forget_vision_cal(kinds=(), keys=(), path=None):
    """从 vision_cal.json 里删掉几个爪子点(claw_px 里的 kind)和键。返回是否改了文件。"""
    p = _cal_path(path)
    d = load_vision_cal(p)
    cp = d.get('claw_px') or {}
    gone = [k for k in kinds if k in cp] + [k for k in keys if k in d]
    if not gone:
        return False
    for k in kinds:
        cp.pop(k, None)
    for k in keys:
        d.pop(k, None)
    tmp = str(p) + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(d, f, indent=2, ensure_ascii=False)
    os.replace(tmp, p)
    return True


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
    if cal.get('ring_outer_diam_mm'):
        cfg['ring_outer_diam_mm'] = float(cal['ring_outer_diam_mm'])   # vclaw RING 时输入的圆环外径(毫米)
    if cal.get('pick_r_px') and not cfg.get('pick_r_px'):
        cfg['pick_r_px'] = float(cal['pick_r_px'])            # 圆环上方观察时物料顶面的半径(像素)，和 claw_px.PICK 一起记下
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
                if not self.run:
                    break
                t = time.monotonic()
                ok, fr = self.cap.retrieve() if ok else (False, None)
            except Exception:                       # 读一帧出错不能让线程死掉(死了以后就再也没有画面)
                ok, fr = False, None
            if not ok or fr is None:
                time.sleep(0.01)
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
        """停线程；返回线程是否已经停了(没停就不能关摄像头，线程还在用它)。"""
        self.run = False
        with self.cv:
            self.cv.notify_all()                    # 正在等帧的马上返回
        self.th.join(timeout=3.0)
        return not self.th.is_alive()


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
        stopped = True
        if self._lf is not None:
            stopped = self._lf.close()      # 先停线程再关摄像头
            self._lf = None
        if self.cap is not None:
            if stopped:
                self.cap.release()
            else:
                self.log('  摄像头读帧卡住了，线程没停下来：先不关摄像头(程序退出时会关)')
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
        ring_outer_diam_mm=95.0,   # 圆环最外圈(黑环外边)直径(毫米)：vclaw RING 100 这样输入实测值；用它算 RING 的每毫米像素数
        ring_line_mm=0.0,          # 6 条细线那种靶填线宽 1.5(识别出来的最外圈半径是线的外边)；校赛粗黑环靶是 0
        live_view='/tmp/vision_live.jpg',   # 识别时把带标注的画面写到这里，另开终端 python3 vview.py 实时看；''=不写
        live_fps=8.0,              # 最多每秒写几张
        pick_r_px=None,            # 圆环上方观察时物料顶面半径(像素)：和 claw_px.PICK 一起自动量出来存进 vision_cal.json
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
        self._pick = None               # 认圆环上物料用的 matdet(和原料盘的分开)
        self.last_pick_r = None         # 最近一次 pick_px 认到的物料顶面半径(像素)
        self._live_t = 0.0

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
        PICK(圆环上放着的物料的顶面)：用量到的物料顶面半径和物料直径算；
        否则用配置里的 px_per_mm。"""
        if kind == 'PICK':
            r = self.cfg.get('pick_r_px') or self.last_pick_r
            if r and self.cfg.get('material_diam_mm'):
                return 2.0 * float(r) / float(self.cfg['material_diam_mm'])
            pp = (self.cfg.get('px_per_mm') or {}).get('PICK')
            return float(pp) if pp else 1.3 * self.scale('RING')         # 物料顶面比地面近，像素/毫米大一些(估计)
        if kind == 'RING' and self.cfg.get('ring_rmax_cal') and self.cfg.get('ring_outer_diam_mm'):
            d = float(self.cfg['ring_outer_diam_mm']) + float(self.cfg.get('ring_line_mm') or 0.0)   # 量到的是最外圈线的外边
            return 2.0 * float(self.cfg['ring_rmax_cal']) / d
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
        cp = self.cfg['claw_px']
        v = cp.get(kind)
        if v is None and kind == 'PICK':
            v = cp['RING']              # PICK 还没量过：画图、算画面范围先用圆环的点(对准前要先 has_pick())
        return (float(v[0]), float(v[1]))

    def has_pick(self):
        """claw_px.PICK 量过没有(没量过就不能按物料对准取回/码垛)。"""
        return (self.cfg.get('claw_px') or {}).get('PICK') is not None

    def set_pick(self, uv, r=None):
        """记下 claw_px.PICK 和物料顶面半径；认圆环上物料的 matdet 按新的半径重建。uv=None：忘掉(下次放下物料时重新量)。"""
        if uv is None:
            self.cfg.get('claw_px', {}).pop('PICK', None)
            self.cfg.pop('pick_r_px', None)
        else:
            self.cfg.setdefault('claw_px', {})['PICK'] = [float(uv[0]), float(uv[1])]
            if r:
                self.cfg['pick_r_px'] = float(r)
        self._pick = None

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
        cache = getattr(md, '_auto_cache', None)
        if cache is not None and not md.has_static_mask() and cache.shape[:2] == fr.shape[:2]:
            return cache            # 找物料时记下的爪子区域(蓝色圆环/蓝色物料挨着爪子时，现找会把它们也当成爪子)
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
                self._publish(fr, 'RAW', p, color_id=color_id)
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
        return 0.35 * outer, 2.0 * outer             # 还没量过：放宽(要看到两圈同心圆才算，不会把别的圆当成圆环)

    def _rings_in_frame(self, fr, any_size=False):
        """检测到的圆环列表 [(x, y, r, ...)]。只保留"最外圈够大"的，物料自己的圆(小)和别的杂圆不算圆环。
        any_size=True：不按大小过滤(只去掉最外圈小于 20 像素的)，vclaw RING 第一次量圆环大小用。"""
        det = self._ring_det()
        if getattr(det, 'wants_claw', False):
            self.last_claw = self.claw_mask(fr)
            rexp = None
            if not any_size and (self.cfg.get('ring_rmax_px') or self.cfg.get('ring_rmax_cal')):
                rexp = self._ring_rmax_range()          # 圆环大小已知：中间被物料盖住、只剩最外圈时也能认
            elif not any_size and self.cfg.get('ring_rmax_seen'):
                r = float(self.cfg['ring_rmax_seen'])   # 没量过，但这次放物料时认到过空圆环：按那个大小认
                rexp = (0.85 * r, 1.2 * r)
            rings = det(fr, self.last_claw, r_expect=rexp)
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
                best = min(rings, key=lambda r: (r[0] - ex[0]) ** 2 + (r[1] - ex[1]) ** 2) if rings else None
                self._publish(fr, 'RING', None if best is None else (best[0], best[1]), rings=rings)
                if best is not None:
                    pts.append((best[0], best[1]))
                    if len(best) >= 4:
                        rmax.append(float(best[3]))
        self.last_ring_rmax = sorted(rmax)[len(rmax) // 2] if rmax else None
        if len(pts) < max(2, n // 2 + 1):
            return None
        return _robust_mean(pts, self.cfg['reject_px'])

    def note_ring_size(self, r):
        """对准空圆环时认到的最外圈半径(像素)。没做 vclaw RING 量圆环大小时记下来(只在这次运行里有效)：
        取回、码垛时白心被物料盖住、只剩黑环外圈，也能按这个大小认出来。"""
        if not r:
            return
        seen = self._ring_seen = (getattr(self, '_ring_seen', []) + [float(r)])[-5:]
        self.cfg['ring_rmax_seen'] = sorted(seen)[len(seen) // 2]

    def note_claw_clear(self):
        """刚对准过空圆环(爪子是空的，附近只有黑白圆环)：这时找到的爪子区域是干净的，记给认圆环上物料的 matdet 用。
        这样取回蓝色/浅蓝物料时，挨着蓝色爪子的物料不会被当成爪子去掉。"""
        import numpy as np
        m = self.last_claw
        if m is None or not getattr(m, 'any', lambda: False)():
            return
        try:
            md = self._pick_det()
        except VisionError:
            return
        if md._auto_cache is None or (md._auto_cache.shape == m.shape and
                                      int(np.count_nonzero(m)) < int(np.count_nonzero(md._auto_cache))):
            md._auto_cache = m          # 原料盘那边记下的爪子区域可能连着蓝色物料(只会偏大)：空圆环上方这份更小就用这份

    def _clean_claw(self, shape):
        """不会把物料算进去的爪子区域：标定过的 claw_mask.png 优先；没有就用之前记下的(蓝色物料连进去只会偏大，取最小的那份)。
        都没有返回 None。爪子在画面里的位置不随姿态变，所以哪个工位记下的都能用。"""
        import numpy as np
        cands = []
        for md in (self._pick, getattr(self._material, 'detector', None), self._mask_md):
            if md is None:
                continue
            try:
                st = md._load_static(shape)
            except Exception:
                st = None
            if st is not None and st.shape[:2] == tuple(shape[:2]) and st.any():
                return st                                   # 标定过的最准
            m = getattr(md, '_auto_cache', None)
            if m is not None and m.shape[:2] == tuple(shape[:2]) and m.any():
                cands.append(m)
        if not cands:
            return None
        return min(cands, key=lambda m: int(np.count_nonzero(m)))

    def ring_centre_free(self, n=2):
        """爪子点附近那个圆环的白心是不是空的(没放着物料)：True 空 / False 有东西 / None 看不清(被爪子挡住太多、没认到圆环)。
        白心里白色(亮、颜色淡)的像素占多少：印的数字是黑的，占得不多；放着物料就基本没有白的了。
        "白"按这一帧圆环外面的白纸/白板有多亮来定(光线暗、摄像头曝光低时白心也只有灰白，不能当成"放了东西")；
        爪子区域用干净的那份(蓝色物料挨着爪子时，现找的爪子区域会把它算进去，白心就"看不见"了)。
        几帧意见不一致返回 None(照常放)。"""
        import cv2
        import numpy as np
        votes = []
        cu, cv_ = self.claw('RING')
        for fr in self._frames(int(n)):
            if fr is None:
                continue
            rings = self._rings_in_frame(fr)
            if not rings:
                continue
            best = min(rings, key=lambda r: (r[0] - cu) ** 2 + (r[1] - cv_) ** 2)
            R = float(best[3]) if len(best) >= 4 else float(best[2])
            h, w = fr.shape[:2]
            yy, xx = np.ogrid[:h, :w]
            d2 = (xx - best[0]) ** 2 + (yy - best[1]) ** 2
            inside = d2 <= (0.4 * R) ** 2
            claw = self._clean_claw((h, w))
            if claw is None:
                claw = self.last_claw
            free = np.ones((h, w), bool)
            if claw is not None and claw.shape[:2] == (h, w):
                free = cv2.dilate(claw, np.ones((9, 9), np.uint8)) == 0
            inside &= free
            npx = int(inside.sum())
            if npx < 120:
                continue
            hsv = cv2.cvtColor(fr, cv2.COLOR_BGR2HSV)
            val, sat = hsv[:, :, 2], hsv[:, :, 1]
            pale = sat < 70
            ref_px = val[(d2 >= (1.08 * R) ** 2) & (d2 <= (1.45 * R) ** 2) & free & pale]   # 圆环外面一圈的白纸/白板
            if ref_px.size < 200:
                ref_px = val[free & pale]
            ref = float(np.percentile(ref_px, 75)) if ref_px.size else 255.0
            white = (val > max(60.0, min(140.0, 0.65 * ref))) & pale
            frac = float((white & inside).sum()) / npx
            votes.append(True if frac >= 0.55 else (False if frac < 0.4 else None))
        votes = [v for v in votes if v is not None]
        if not votes or (len(votes) > 1 and sum(votes) * 2 == len(votes)):
            return None
        return sum(votes) * 2 > len(votes) if len(votes) > 1 else votes[0]

    def ring_error(self, n=None):
        """圆环中心偏差像素 = 圆环中心 - 爪子(RING)。看不到返回 None。"""
        p = self.ring_px(n)
        if p is None:
            return None
        cu, cv_ = self.claw('RING')
        return (p[0] - cu, p[1] - cv_)

    # ------------------------------------------------------------ 圆环上放着的物料(取回、码垛)
    def _pick_det(self):
        """认圆环上物料用的 matdet。和原料盘的分开：观察高度不一样，物料看起来大小不一样，原料盘学到的半径不能用。"""
        if self._pick is None:
            try:
                from matdet import MaterialDetector
            except ImportError as ex:
                raise VisionError('找不到 matdet.py(或者没装 numpy/opencv)：把 matdet.py 复制到 lidar_map_north 文件夹') from ex
            self._pick = MaterialDetector(self.cfg.get('matdet'))
            if self.cfg.get('pick_r_px'):
                self._pick.seed_radius(float(self.cfg['pick_r_px']))
        md = self._pick
        if md._auto_cache is None:
            # 原料盘那边记下过爪子区域就拿来用(爪子在画面里的位置不随姿态变)：蓝色物料挨着爪子时不会和爪子连成一块
            try:
                main = self.material_detector_obj()
            except VisionError:
                main = None
            if getattr(main, '_auto_cache', None) is not None:
                md._auto_cache = main._auto_cache
        return md

    def pick_px(self, color_id, n=None):
        """圆环上(取回时)或下面一层(码垛时)这个颜色的物料，顶面圆心的像素位置，多帧稳健平均；看不到返回 None。
        量过物料顶面半径(pick_r_px)时，大小差得多的不算。self.last_pick_r = 这几帧里顶面半径的中值。"""
        n = int(n or self.cfg['frames'])
        md = self._pick_det()
        r0 = self.cfg.get('pick_r_px')
        pts, rr = [], []
        for fr in self._frames(n):
            if fr is None:
                continue
            res = md.detect(fr, int(color_id))
            if res is not None and r0 and not 0.8 * float(r0) <= res['radius'] <= 1.25 * float(r0):
                res = None
            p = None if res is None else (float(res['center'][0]), float(res['center'][1]))
            self._publish(fr, 'PICK', p, color_id=color_id)
            if p is not None:
                pts.append(p)
                rr.append(float(res['radius']))
        self.last_pick_r = sorted(rr)[len(rr) // 2] if rr else None
        if len(pts) < max(2, n // 2 + 1):
            return None
        return _robust_mean(pts, self.cfg['reject_px'])

    def pick_error(self, color_id, n=None):
        """物料顶面偏差像素 = 物料顶面圆心 - claw_px.PICK。PICK 还没量过、或者看不到，返回 None。"""
        if not self.has_pick():
            return None
        p = self.pick_px(color_id, n)
        if p is None:
            return None
        cu, cv_ = self.claw('PICK')
        return (p[0] - cu, p[1] - cv_)

    # ------------------------------------------------------------ 调试
    def save_debug(self, path, kind='RING', color_id=None):
        """存一张带标注的图(和 vview 看到的一样)：爪子点(绿十字)、圆环(黄；大小不对被过滤掉的是灰色)/物料(青色圆、红叉)。"""
        import cv2
        fr = self._frame()
        if fr is None:
            return False
        try:
            if color_id is not None:
                out = self.overlay(fr, 'RAW', self._material_det()(fr, color_id), color_id=color_id)
            else:
                rings = self._rings_in_frame(fr)
                cu, cv_ = self.claw('RING')
                best = min(rings, key=lambda r: (r[0] - cu) ** 2 + (r[1] - cv_) ** 2) if rings else None
                out = self.overlay(fr, 'RING', None if best is None else (best[0], best[1]), rings=rings)
        except VisionError:
            out = fr
        return bool(cv2.imwrite(path, out))

    # ------------------------------------------------------------ 实时画面(给 vview.py 看)
    def overlay(self, fr, kind, target=None, color_id=None, rings=None):
        """在一帧上画出识别结果，返回新图。kind='RAW' 物料 / 'RING' 圆环；target = 认到的中心(像素)或 None。"""
        import cv2
        out = fr.copy()
        md = None
        if kind == 'RING':
            claw = self.last_claw
        else:
            try:
                md = self._pick if kind == 'PICK' else self.material_detector_obj()
            except VisionError:
                md = None
            claw = getattr(md, 'last_claw', None) if md is not None else None
        if claw is not None and getattr(claw, 'shape', None) is not None and claw.shape[:2] == out.shape[:2] and claw.any():
            out[claw > 0] = (out[claw > 0] * 0.5).astype(out.dtype)               # 爪子区域变暗(这块不用来识别)
        cu, cv_ = self.claw(kind)
        info = ''
        if kind == 'RING':
            details = getattr(self._ring, 'last', None) or []
            kept = [(r[0], r[1]) for r in (rings or [])]
            for d in details:
                x, y = d['center']
                ok = any(abs(x - kx) < 0.05 and abs(y - ky) < 0.05 for kx, ky in kept)
                col = (0, 255, 255) if ok else (150, 150, 150)
                for r in (d['radii'] if ok else [d['r_max']]):
                    cv2.circle(out, (int(round(x)), int(round(y))), int(round(r)), col, 1)
                if not ok:
                    cv2.putText(out, f'r={d["r_max"]:.0f} size?', (int(x) + 6, int(y) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
                elif target is not None and abs(x - target[0]) < 0.05 and abs(y - target[1]) < 0.05:
                    info = f' rmax={d["r_max"]:.0f}px {d["n_circles"]} circles seen {d["coverage"] * 100:.0f}%'
            if not details:
                for r in rings or []:
                    cv2.circle(out, (int(round(r[0])), int(round(r[1]))), int(round(r[2])), (0, 255, 255), 1)
            what = f'RING ({len(kept)} found' + (f', {len(details) - len(kept)} wrong size' if len(details) > len(kept) else '') + ')'
        else:
            res = getattr(md, 'last', None) if md is not None else None
            if target is not None and res is not None:
                cv2.circle(out, (int(round(target[0])), int(round(target[1]))), int(round(res['radius'])), (255, 255, 0), 2)
                info = f' r={res["radius"]:.0f}px hidden {res["occluded"] * 100:.0f}%'
            what = f'MATERIAL {color_id}'
            if kind == 'PICK':
                what += ' on ring' + ('' if self.has_pick() else ' (PICK point not learned)')
        if target is not None:
            cv2.drawMarker(out, (int(round(target[0])), int(round(target[1]))), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 28, 2)
            cv2.line(out, (int(cu), int(cv_)), (int(round(target[0])), int(round(target[1]))), (255, 0, 255), 1)
            du, dv = target[0] - cu, target[1] - cv_
            try:
                mm = (du * du + dv * dv) ** 0.5 / self.scale(kind)
            except Exception:
                mm = float('nan')
            msg = f'{what}: off {du:+.0f},{dv:+.0f}px ~{mm:.1f}mm{info}'
        else:
            msg = f'{what}: NOT FOUND'
        cv2.drawMarker(out, (int(cu), int(cv_)), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        msg += time.strftime('   %H:%M:%S')
        cv2.putText(out, msg, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
        cv2.putText(out, msg, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        return out

    def _publish(self, fr, kind, target=None, color_id=None, rings=None):
        """把带标注的这一帧写到 live_view 文件(限速)，另一个终端里 python3 vview.py 就能实时看。出错也不影响识别。"""
        path = self.cfg.get('live_view')
        if not path or fr is None:
            return
        now = time.monotonic()
        if now - self._live_t < 1.0 / max(0.5, float(self.cfg.get('live_fps') or 8.0)):
            return
        self._live_t = now
        try:
            import cv2
            ok, buf = cv2.imencode('.jpg', self.overlay(fr, kind, target, color_id, rings), [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                tmp = path + '.tmp'
                with open(tmp, 'wb') as f:
                    f.write(buf.tobytes())
                os.replace(tmp, path)                   # 一次换掉，看的那边不会读到写了一半的图
        except Exception:
            pass
