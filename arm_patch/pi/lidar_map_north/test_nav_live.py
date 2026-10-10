"""map_merge_live 整体跑一遍的测试(1010)：车上才有的模块(lidar_map_live、ld14p_scan、merge_stations、pose_fix、
reference_correction、stm32_link)都换成测试替身，雷达和车用模拟的(车按指令走，有距离误差和陀螺仪漂移，雷达按车的真实位置扫)，
界面用 Agg(不开窗口)。检查：屏上选区 -> 扫描、预先规划 -> 按 START -> 一路雷达定位 -> 回到启停区，开跑后不问 y，
车身不出场地；两个启停区都跑；退出时收臂停 60mm。路线规划换成简单的替身(这里测的是流程，规划在 test_route_* 里测)。
python3 -m unittest test_nav_live"""
import builtins
import copy
import importlib
import json
import math
import os
import shutil
import sys
import tempfile
import threading
import time
import types
import unittest

import numpy as np

import test_auto_run  # noqa: F401  (先装好 stm32_link / route_plan 的替身)
import auto_run
import reloc
from test_nav_drive import SimCar, manhattan_leg, edge_min
from test_reloc import scan as cast_scan, CIRC

HERE = os.path.dirname(os.path.abspath(__file__))
USER_CFG = os.path.join(HERE, '..', '..', '..', 'docs', 'pi_snapshot_1006', 'map_config_start2_roi.json')
STUBBED = ('lidar_map_live', 'ld14p_scan', 'reference_correction', 'merge_stations', 'pose_fix', 'stm32_link', 'map_merge_live',
           'matplotlib', 'matplotlib.pyplot', 'matplotlib.patches')

WORLD = {}


class World(SimCar):
    """模拟的车 + 屏：ZONE? 先回"没选"，几次以后回选了 zone 区；屏上显示 READY 以后再过几次就"按了 START"。"""

    def __init__(self, pose, zone, **kw):
        SimCar.__init__(self, pose, **kw)
        self.zone = zone
        self.zq = 0
        self.ready_at = None
        self.lock = threading.RLock()

    def _move(self, cmd, val):
        with self.lock:
            return SimCar._move(self, cmd, val)

    def request(self, text, timeout=None, collect=False):
        if text in ('ZONE 1', 'ZONE 2'):                    # 终端 zone 1|2：和在屏上选一样
            self.requests.append(text)
            self.zone = int(text[-1])
            self.zq = max(self.zq, 3)
            out = (True, 'DONE', [])
            return out if collect else out[:2]
        if text == 'ZONE?':
            self.requests.append(text)
            self.zq += 1
            z = (self.zone or 0) if self.zq > 3 else 0
            if self.ready_at is None and any(r.startswith('ZONE MSG READY') for r in self.requests):
                self.ready_at = self.zq
            s = 1 if (self.ready_at is not None and self.zq >= self.ready_at + 3) else 0
            out = (True, 'DONE', [f'ZONE {z} {s}'])
            return out if collect else out[:2]
        return SimCar.request(self, text, timeout, collect)


# ---------------------------------------------------------------- 车上模块的替身
def _rotate(xy, deg):
    return reloc.rotate(xy, deg)


def _to_map(xy, c):
    P = np.asarray(xy, float).reshape(-1, 2).copy()
    P[:, 1] *= c['sdk_y_sign']
    B = _rotate(P, c['lidar_yaw_deg']) + [c['lidar_forward_mm'], c['lidar_left_mm']]
    return _rotate(B, c['car_yaw_deg']) + [c['car_x_mm'], c['car_y_mm']]


def make_stubs():
    lml = types.ModuleType('lidar_map_live')

    class Mapping:
        def __init__(self, c):
            self.c = c
            self.latched = []
            self.points = np.empty((0, 2))
            self.next_id = 1

        def ingest(self, frame, count):
            return self.latched
    lml.Mapping = Mapping
    lml.validate_config = lambda c: c
    lml.roi_class = lambda x, y, c: 'interior'
    lml.rotate = _rotate
    lml.make_path = lambda *a: ([], '')
    lml.body_corners = lambda c: [(-145, -130), (145, -130), (145, 130), (-145, 130)]
    lml.body_half_bound = lambda c: 145
    lml.FIELD_MM = 2400
    lml.to_map = _to_map
    def accumulated_detections(frames, c, report=None):
        """替身：只认 WORLD['seen'] 里登记的圆柱，离雷达 1000mm 以内、扫到 5 个点以上才算(位置按 c 里的车位换算，车位错了位置就错)。"""
        pol = [p for f in frames for p in f.get('points', []) if p[1] > 0]
        if not pol or not WORLD.get('seen'):
            return [], np.empty((0, 2))
        P = _to_map(lds.cartesian(pol), c)
        L = _to_map([[0, 0]], c)[0]
        out = []
        for cx, cy, r in WORLD['seen']:
            sel = P[np.hypot(P[:, 0] - cx, P[:, 1] - cy) < 150]
            if len(sel) >= 5 and math.hypot(cx - L[0], cy - L[1]) <= 1000:
                m = sel.mean(0)
                u = (m - L) / max(1e-6, float(np.hypot(*(m - L))))
                q = m + u * 0.8 * r
                out.append(dict(x=float(q[0]), y=float(q[1]), count=len(sel), frames=len(frames)))
        return out, P
    lml.accumulated_detections = accumulated_detections
    lml.YELLOW_RECTS = [[550, 1400, 1000, 1850], [1400, 1400, 1850, 1850], [550, 550, 1000, 1000], [1400, 550, 1850, 1000]]
    lml.TEMP_ZONE = [0, 910, 150, 1490]
    lml.ROUGH_ZONE = [910, 0, 1490, 150]
    lml.START_RECTS = {1: (2100, 2100), 2: (2100, 0)}
    lml.POINTS = {'QR_SCAN': (2100, 1200), 'RAW_STOP': (1200, 2100), 'TEMP_STOP': (340, 1200), 'ROUGH_STOP': (1200, 340)}

    lds = types.ModuleType('ld14p_scan')

    class Proc:
        def poll(self):
            return None

    class Stream:
        def __init__(self, port, model):
            self.proc = Proc()
            self.count = 0

        def snapshot(self):
            t = time.monotonic() - 0.001
            car = WORLD['car']
            with car.lock:
                pose = car.pose()
            self.count += 1
            off = (self.count * 0.37) % 1.0                 # 真雷达每一帧的采样角度都不一样
            B = reloc.rotate(cast_scan((pose[0], pose[1], pose[2] + off), n=360, noise=2.0, seed=self.count,
                                       circ=WORLD.get('circ', CIRC)), off)
            c = WORLD['mount']                    # 车身坐标 -> 雷达坐标(和车上的安装位置、安装角一样)
            P = _rotate(B - [c['lidar_forward_mm'], c['lidar_left_mm']], -c['lidar_yaw_deg'])
            P[:, 1] *= c['sdk_y_sign']
            pts = [(math.degrees(math.atan2(y, x)), math.hypot(x, y), 200) for x, y in P]
            return {'points': pts, 'seq': self.count}, t, self.count

        def close(self):
            pass
    lds.Stream = Stream
    lds.cartesian = lambda pol: [(d * math.cos(math.radians(a)), d * math.sin(math.radians(a))) for a, d, *_ in pol]

    rc = types.ModuleType('reference_correction')
    rc.applicable = lambda c: False
    rc.description = lambda c: '关(测试)'
    rc.metadata = lambda: {}

    ms = types.ModuleType('merge_stations')

    class History:
        def __init__(self):
            self.views = []
            self.objects = []
            self.merge_radius_mm = 150.0

        def set_radius(self, r):
            self.merge_radius_mm = r

        def commit(self, m, source, seq):
            self.views.append(dict(config=copy.deepcopy(m.c), raw_frames=list(m.raw_frames), field_returns=[], source=source))
            for d in m.latched:
                if all(math.hypot(d['x'] - o['x'], d['y'] - o['y']) > self.merge_radius_mm for o in self.objects):
                    self.objects.append(dict(id=len(self.objects) + 1, x=d['x'], y=d['y'], radius=25.0, seen=d.get('seen', 9),
                                             count=d.get('count', 9), anchor_view=len(self.views), ambiguous=False))

    def moved_config(cfg, left, fwd, cw):
        c = dict(cfg)
        a = math.radians(cfg['car_yaw_deg'])
        c['car_x_mm'] = cfg['car_x_mm'] + fwd * math.cos(a) - left * math.sin(a)
        c['car_y_mm'] = cfg['car_y_mm'] + fwd * math.sin(a) + left * math.cos(a)
        c['car_yaw_deg'] = reloc.wrap180(cfg['car_yaw_deg'] - cw)
        return c
    ms.History = History
    ms.moved_config = moved_config
    ms.second_scan_config = lambda *a: None

    pf = types.ModuleType('pose_fix')
    pf.calibrate_mount = lambda *a: None
    pf.refine_second = lambda *a, **k: (None, '  (测试替身：第二站不按障碍物校正)')

    sl = types.ModuleType('stm32_link')
    sl.parse_done = sys.modules['stm32_link'].parse_done if 'stm32_link' in sys.modules else None

    class Stm32Link:
        def __new__(cls, port):
            return WORLD['car']
    sl.Stm32Link = Stm32Link
    sl.FakeLink = Stm32Link
    if sl.parse_done is None:
        sl.parse_done = test_auto_run.parse_done if hasattr(test_auto_run, 'parse_done') else (lambda r: {})
    return {'lidar_map_live': lml, 'ld14p_scan': lds, 'reference_correction': rc, 'merge_stations': ms, 'pose_fix': pf,
            'stm32_link': sl}


class Anything:
    """界面替身：什么方法都能调，什么都不画。"""

    def __init__(self, *a, **k):
        pass

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        return Anything()

    def __call__(self, *a, **k):
        return Anything()

    def __iter__(self):
        return iter([Anything()])


def make_mpl(timers, show):
    mpl = types.ModuleType('matplotlib')
    plt = types.ModuleType('matplotlib.pyplot')
    patches = types.ModuleType('matplotlib.patches')

    class Canvas(Anything):
        manager = None

        def new_timer(self, *a, **k):
            t = Timer()
            timers.append(t)
            return t

    class Fig(Anything):
        canvas = Canvas()

    class Timer:
        def __init__(self):
            self.cbs = []

        def add_callback(self, f, *a, **k):
            self.cbs.append((f, a, k))

        def start(self, *a):
            pass

        def stop(self, *a):
            pass
    plt.subplots = lambda *a, **k: (Fig(), Anything())
    plt.show = show
    plt.close = lambda *a, **k: None
    patches.Rectangle = patches.Polygon = patches.Circle = Anything
    mpl.pyplot = plt
    mpl.patches = patches
    return {'matplotlib': mpl, 'matplotlib.pyplot': plt, 'matplotlib.patches': patches}


def fake_plan_mission(cfg, obstacles, start_pose, stop_names, start_pivot=None):
    stops = cfg['stops']
    legs, p = [], tuple(float(v) for v in start_pose)
    for n in stop_names:
        g = tuple(float(v) for v in stops[n][:3])
        legs.append(manhattan_leg(n, p, g, turn_first=True))
        p = g
    pl = types.SimpleNamespace(fallback=False, lane_fallback=False, sealed=[], box=None)
    return legs, 'OK', pl


def fake_home_route_legs(cfg, obstacles, start_pose, names):
    return fake_plan_mission(cfg, obstacles, start_pose, names)[0]


class Out:
    def __init__(self):
        self.lines = []
        self.lock = threading.Lock()

    def write(self, s):
        with self.lock:
            self.lines.append(s)
        return len(s)

    def flush(self):
        pass

    def text(self):
        with self.lock:
            return ''.join(self.lines)


class LiveTests(unittest.TestCase):
    def setUp(self):
        WORLD.clear()

    def run_live(self, zone, car_kw=None, cfg_extra=None, timeout=90.0, inputs=None, screen_zone='same'):
        with open(USER_CFG, encoding='utf-8') as f:
            raw = json.load(f)
        WORLD['mount'] = {k: raw[k] for k in ('lidar_forward_mm', 'lidar_left_mm', 'lidar_yaw_deg', 'sdk_y_sign')}
        raw.update(race_auto=True,
                   scan_frames=10, scan_settle_s=0.02, reloc_frames=4, reloc_settle_s=0.02, home_fix_frames=4,
                   race_still_s=0.4, race_poll_s=0.05, race_check_s=0.5, keepalive_s=1.0, mission_cfg={'enabled': False})
        raw.update(cfg_extra or {})
        tmp = tempfile.mkdtemp(prefix='nav_live_')
        cfg_path = os.path.join(tmp, 'cfg.json')
        with open(cfg_path, 'w', encoding='utf-8') as f:
            json.dump(raw, f)
        open(os.path.join(tmp, 'scan_bridge'), 'w').close()
        start = (2250.0, 150.0, 90.0) if zone == 2 else (2250.0, 2250.0, 180.0)
        car = World(start, zone if screen_zone == 'same' else screen_zone, rdec=True, **(car_kw or {}))
        WORLD['car'] = car
        saved = {k: sys.modules.get(k) for k in STUBBED}
        stubs = make_stubs()
        sys.modules.update(stubs)
        sys.modules.pop('map_merge_live', None)
        out = Out()
        self.out_text = out.text
        timers = []
        state = {'done': False}

        def show(*a, **k):
            t_end = time.monotonic() + timeout
            t_fin = None
            while time.monotonic() < t_end:
                for t in timers:
                    for f, a2, k2 in t.cbs:
                        f(*a2, **k2)
                txt = out.text()
                if t_fin is None and ('路线全部走完' in txt or '★ 已停止' in txt or '★ 出错停止' in txt or '★ 比赛流程' in txt
                                      or (state.get('end_on') and state['end_on'] in txt)):
                    t_fin = time.monotonic()
                if t_fin is not None and time.monotonic() - t_fin > 0.6:
                    state['done'] = True
                    return
                time.sleep(0.02)

        script = list(inputs or [])

        def no_input(*a):
            # 终端输入的替身：按顺序，等条件满足了就"敲"一行；敲完了就当终端关了
            while script:
                cond, line = script[0]
                t_end = time.monotonic() + timeout
                while not cond() and time.monotonic() < t_end:
                    time.sleep(0.05)
                script.pop(0)
                return line
            raise EOFError
        sys.modules.update(make_mpl(timers, show))
        old = (builtins.input, sys.stdout, sys.argv)
        try:
            builtins.input = no_input
            sys.stdout = out
            mml = importlib.import_module('map_merge_live')
            mml.ROOT = type(mml.ROOT)(tmp)
            mml.plan_mission = fake_plan_mission
            mml.home_route_legs = fake_home_route_legs
            sys.argv = ['map_merge_live.py', '--port', os.path.join(tmp, 'scan_bridge'), '--config', cfg_path, '--stm-port', 'sim',
                        '--stop-wait', '0']
            rc = mml.main()
        finally:
            builtins.input, sys.stdout, sys.argv = old
            for k, v in saved.items():
                if v is None:
                    sys.modules.pop(k, None)
                else:
                    sys.modules[k] = v
            with open(cfg_path, encoding='utf-8') as f:
                self.cfg_after = json.load(f)
            shutil.rmtree(tmp, ignore_errors=True)
        self.log = out.text()
        self.assertEqual(rc, 0)
        self.assertTrue(state['done'], '一整轮没跑完：\n' + self.log[-3000:])
        return car

    def check_run(self, car, zone):
        log = self.log
        self.assertIn('路线全部走完', log, log[-3000:])
        self.assertEqual(car.requests[0], 'ZONE ASK')
        self.assertIn('ZONE LOCK', car.requests)
        self.assertNotIn('输入 y', log, '开跑后不能再要人输入 y')
        self.assertIn('直接用开跑前规划好的路线', log)
        self.assertGreaterEqual(log.count('雷达定位：车在'), 6, '每个停车点都要雷达定位')
        goal = auto_run.inset_goal((2250.0, 150.0, 90.0) if zone == 2 else (2250.0, 2250.0, 180.0), self.cfg_after)
        p = car.pose()
        self.assertLess(math.hypot(p[0] - goal[0], p[1] - goal[1]), 10.0, f'没回到出发位置：{p}')
        self.assertLess(abs(reloc.wrap180(p[2] - goal[2])), 1.5)
        # 第二站原地转的时候车角会扫出东边 3mm(还在启停区里，规则不管)；离开启停区以后车身不能出场地
        k0 = next(i for i, q in enumerate(car.trace) if not auto_run.in_rect(q, auto_run.START_RECTS[zone]))
        self.assertGreaterEqual(edge_min(car.trace[k0:], self.cfg_after), 0.0)
        self.assertEqual(car.requests[-1], 'PARK', '退出时收臂、升降停 60mm')
        self.assertEqual(self.cfg_after.get('start_zone'), 2, '配置文件里的启停区不跟着切换')

    def test_race_zone2(self):
        car = self.run_live(2, dict(scale=0.015, drift_deg=0.08, resid_deg=0.3, slip=0.01))
        self.check_run(car, 2)

    def test_new_obstacle_at_qr_registered_at_measured_pose(self):
        # 中间车道上有一根圆柱，只有到了 QR 才看得到(离雷达 1000mm 以内)；车到 QR 时按推算的位置偏了 ~40mm：
        # 用雷达测到的车位登记，圆柱的位置要准；从 QR 重新规划剩下的路线，接着跑完
        WORLD['circ'] = CIRC + [(1650.0, 1250.0, 25.0)]
        WORLD['seen'] = [(1650.0, 1250.0, 25.0)]
        car = self.run_live(2, dict(scale=0.025, drift_deg=0.0, resid_deg=0.1))
        log = self.log
        self.assertIn('路线全部走完', log, log[-3000:])
        self.assertIn('发现 1 个新障碍物', log)
        self.assertIn('已换成新路线', log)
        import re
        m = re.search(r'发现 1 个新障碍物：\((\d+),(\d+)\)', log)
        self.assertTrue(m, log[-3000:])
        x, y = int(m.group(1)), int(m.group(2))
        self.assertLess(math.hypot(x - 1650, y - 1250), 15, f'登记的圆柱位置 ({x},{y})')
        k0 = next(i for i, q in enumerate(car.trace) if not auto_run.in_rect(q, auto_run.START_RECTS[2]))
        self.assertGreaterEqual(edge_min(car.trace[k0:], self.cfg_after), 0.0, '横移多走 2.5% 也不能出场地')

    def test_go_from_terminal(self):
        # 不用屏幕：终端 go(调试用)。开跑后不问 y，一样一路定位、回家
        car = self.run_live(2, dict(scale=0.01, drift_deg=0.05, resid_deg=0.2), cfg_extra=dict(race_auto=False),
                            inputs=[(lambda: True, 'go')], screen_zone=None)
        self.assertIn('路线全部走完', self.log, self.log[-3000:])
        self.assertNotIn('ZONE ASK', car.requests)
        self.assertNotIn('输入 y', self.log)
        self.assertGreaterEqual(self.log.count('雷达定位：车在'), 6)
        self.assertEqual(car.requests[-1], 'PARK')

    def test_go_after_reset_scans_at_start_pose(self):
        # 先在别的车位扫过一次(a X Y 角度)再 reset：state['config'] 还是那个车位。go 的第一次扫描要按本区的起点车位算，
        # 否则整张地图和所有雷达定位都按错的原点，车会开错
        def scanned():
            return '扫描完成，已合并 1 次' in self.out_text()
        car = self.run_live(2, dict(scale=0.01), cfg_extra=dict(race_auto=False),
                            inputs=[(lambda: True, 'a 2000 1000 90'), (scanned, 'reset'), (lambda: True, 'go')], screen_zone=None)
        self.assertIn('路线全部走完', self.log, self.log[-3000:])
        goal = auto_run.inset_goal((2250.0, 150.0, 90.0), self.cfg_after)
        p = car.pose()
        self.assertLess(math.hypot(p[0] - goal[0], p[1] - goal[1]), 10.0, f'没回到出发位置：{p}')

    def test_abort_then_quit_does_not_park(self):
        def moving():
            c = WORLD.get('car')
            return c is not None and len(c.sent) >= 6
        car = self.run_live(2, dict(scale=0.01), cfg_extra=dict(race_auto=False),
                            inputs=[(lambda: True, 'go'), (moving, 'abort')], screen_zone=None)
        self.assertIn('★ 已停止', self.log)
        self.assertGreaterEqual(car.aborted, 1, '急停要发给 STM32')
        self.assertNotIn('PARK', car.requests, '急停以后退出不能自动收臂')
        self.assertIn('急停过', self.log)

    def test_terminal_zone_during_race(self):
        # 屏上没选(触摸不好用)：终端 zone 1 代替屏上选区，接着照常扫描、预先规划；START 还是在屏上按
        def asked():
            c = WORLD.get('car')
            return c is not None and 'ZONE ASK' in c.requests
        car = self.run_live(1, dict(scale=0.0), inputs=[(asked, 'zone 1')], screen_zone=None)
        self.assertIn('ZONE 1', car.requests)
        self.check_run(car, 1)

    def test_race_zone1(self):
        car = self.run_live(1, dict(scale=-0.01, drift_deg=-0.08, resid_deg=0.3, slip=0.01))
        self.check_run(car, 1)
        self.assertIn('启停区 1：起点 (2250,2250) 车头 180°', self.log)


if __name__ == '__main__':
    unittest.main()
