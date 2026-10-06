"""摄像头与识别。

复用 chengxu 里已经调好的识别(这两个文件要和本文件放在同一个文件夹，即 lidar_map_north)：
    wuliao.find_best_target(frame, color_id)   逐帧找某种颜色的物料，返回 {'center': (u, v), ...}
    ring_detect.detect_rings(frame)            逐帧找圆环，返回 [(x, y, r), ...]

做的事：
  - 只开一次摄像头(/dev/video0)，每次读之前先丢掉缓冲里的旧帧，保证拿到的是"现在"的画面
  - 多帧取中值，再剔除离群点后取平均，比单帧准
  - 输出"偏差像素" = 目标像素 - 爪子像素，交给 visual_servo
爪子像素 claw_px 是爪子的轴线在画面里的位置：RAW 用于夹原料盘上的物料，RING 用于对准地上圆环。
改 claw_px.RING 1 个像素，放置位置大约移动 0.34mm(按 2.96 像素/毫米算)；以后实际放偏了，就调这个数。
"""
import time


class VisionError(Exception):
    pass


class Camera:
    def __init__(self, device='/dev/video0', width=640, height=480, fps=30, flip=None, flush=3, log=print):
        self.device, self.width, self.height, self.fps = device, int(width), int(height), int(fps)
        self.flip = flip            # None 不翻；cv2.flip 的参数：0 上下，1 左右，-1 两个都翻
        self.flush = int(flush)
        self.log = log
        self.cap = None

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
        self.log(f'  摄像头 {self.device} 已打开 {self.width}x{self.height}')
        return self

    def read(self):
        """最新的一帧，读不到返回 None。"""
        if self.cap is None:
            return None
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
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def default_material_detector():
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


def default_ring_detector():
    return detect_rings_subpixel


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
        px_per_mm=dict(RAW=4.36, RING=2.96),
        frames=5,                  # 每次测量取几帧
        frame_gap_s=0.03,
        reject_px=3.0,
        still_px=2.0,              # 原料盘上的物料连续几次测量移动小于这么多像素才算停稳
        ring_r_px=None,            # [最小, 最大] 圆环(组)的平均半径像素，None=不限制
        ring_rmax_px=None,         # [最小, 最大] 圆环(组)里最大的圆的半径像素；None=按 px_per_mm.RING 自动算(最外圈直径 95mm 的 0.8~1.25 倍)
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

    # ------------------------------------------------------------ 基础
    def open(self):
        if self.camera is None:
            cc = self.cfg['camera']
            self.camera = Camera(cc.get('device', '/dev/video0'), cc.get('width', 640), cc.get('height', 480),
                                 cc.get('fps', 30), cc.get('flip'), log=self.log).open()
        return self

    def close(self):
        if self.camera is not None and hasattr(self.camera, 'close'):
            self.camera.close()

    def scale(self, kind):
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
            self._material = default_material_detector()
        return self._material

    def _ring_det(self):
        if self._ring is None:
            self._ring = default_ring_detector()
        return self._ring

    def _frame(self):
        if self.camera is None:
            self.open()
        return self.camera.read()

    # ------------------------------------------------------------ 物料(原料盘上)
    def material_px(self, color_id, n=None):
        """物料(顶面中心)的像素位置，多帧稳健平均；检测到的帧太少返回 None。"""
        n = int(n or self.cfg['frames'])
        det = self._material_det()
        pts = []
        for i in range(n):
            fr = self._frame()
            if fr is not None:
                p = det(fr, color_id)
                if p is not None:
                    pts.append(p)
            if i < n - 1:
                self.sleep(self.cfg['frame_gap_s'])
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
        outer = 95.0 / 2.0 * self.scale('RING')              # 最外圈(6 环)半径，像素
        return 0.8 * outer, 1.25 * outer

    def _rings_in_frame(self, fr):
        """检测到的圆环列表 [(x, y, r, ...)]。只保留"最外圈够大"的，物料自己的圆(小)和别的杂圆不算圆环。"""
        rings = self._ring_det()(fr)
        rr = self.cfg.get('ring_r_px')
        if rr:
            rings = [r for r in rings if rr[0] <= r[2] <= rr[1]]
        lo, hi = self._ring_rmax_range()
        return [r for r in rings if len(r) < 4 or lo <= r[3] <= hi]

    def ring_px(self, n=None, expect=None):
        """离 expect(默认=爪子像素)最近的那个圆环的中心像素，多帧稳健平均；看不到返回 None。"""
        n = int(n or self.cfg['frames'])
        ex = expect or self.claw('RING')
        pts = []
        for i in range(n):
            fr = self._frame()
            if fr is not None:
                rings = self._rings_in_frame(fr)
                if rings:
                    best = min(rings, key=lambda r: (r[0] - ex[0]) ** 2 + (r[1] - ex[1]) ** 2)
                    pts.append((best[0], best[1]))
            if i < n - 1:
                self.sleep(self.cfg['frame_gap_s'])
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
                    cv2.drawMarker(out, (int(p[0]), int(p[1])), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 30, 2)
            except VisionError:
                pass
        return bool(cv2.imwrite(path, out))
