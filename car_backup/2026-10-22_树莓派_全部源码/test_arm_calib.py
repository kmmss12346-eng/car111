"""arm_calib.py(机械臂一次性标定向导)的测试：用假 STM32 把整个流程走一遍。python3 test_arm_calib.py"""
import json
import os
import re
import tempfile
import unittest

import arm_calib
from arm_calib import Wizard, CalError, STEPS, STEP_KEYS


RANGES = dict(LFMAX=(10, 105), ZHI=(0, 400), ZGRAB=(0, 400), ZDROP=(0, 400), ZPLC=(0, 400), ZSTK=(0, 400),
              ZOBRAW=(0, 400), ZOBRNG=(0, 400), A1G=(232, 417.6), A1D=(232, 417.6), A1H=(232, 417.6),
              A1P=(232, 417.6), A2E=(-1220, -503.5), A2R=(-1220, -503.5), A2P=(-1220, -503.5),
              CLWO=(1700, 2910), CLWC=(1700, 2910), TT1=(500, 2608), TT2=(500, 2608), TT3=(500, 2608),
              AEXT=(1, 2), ARMOK=(0, 1), FPPM=(1, 100))
DEFAULTS = dict(LFMAX=100, ZHI=100, ZGRAB=20, ZDROP=60, ZPLC=0, ZSTK=60, ZOBRAW=100, ZOBRNG=100,
                A1G=323, A1D=323, A1H=323, A1P=323, A2E=-862, A2R=-862, A2P=-862,
                CLWO=2090, CLWC=2910, TT1=2608, TT2=1708, TT3=808, AEXT=2, ARMOK=0, FPPM=12.9)
LIM = {1: (232.0, 417.6), 2: (-1220.0, -503.5)}


class FakeStm32:
    """只认标定要用的那些指令。舵机按限位夹住，block1 = ID1 被挡住的角度。"""

    def __init__(self, old=False):
        self.P = dict(DEFAULTS)
        if old:
            for k in ('CLWO', 'CLWC', 'TT1', 'TT2', 'TT3', 'AEXT'):
                del self.P[k]
        self.z = 60.0
        self.known = 1
        self.ang = {1: 323.0, 2: -862.0}
        self.released = {1: False, 2: False}
        self.claw = 2090
        self.tt = 2608
        self.block1 = None
        self.sent = []

    def _servo(self, sid, deg):
        lo, hi = LIM[sid]
        deg = max(lo, min(hi, deg))
        if sid == 1 and self.block1 is not None and deg > self.block1:
            deg = self.block1
        self.ang[sid] = deg
        self.released[sid] = False

    def request(self, text, timeout=5.0):
        self.sent.append(text)
        p = text.split()
        v = p[0]
        if v == 'PING':
            return 'PONG', []
        if v == 'GET':
            return 'DONE', [f'P {k}={x:.4f}' for k, x in self.P.items()]
        if v == 'SET' and len(p) == 3:
            if p[1] not in self.P:
                return 'ERR NAME', []
            x = float(p[2])
            lo, hi = RANGES[p[1]]
            if not lo <= x <= hi:
                return 'ERR RANGE', []
            self.P[p[1]] = x
            return 'DONE', []
        if v == 'LIFT?':
            return 'DONE', [f'LIFT {self.known} {self.z:.4f}']
        if v == 'LIFT' and len(p) == 2:
            self.z = max(0.0, min(self.P['LFMAX'], float(p[1])))
            return 'DONE', []
        if v == 'A?':
            return 'DONE', [f'ANG {p[1]} {self.ang[int(p[1])]:.4f}']
        if v == 'AF':
            self._servo(int(p[1]), float(p[2]))
            return 'DONE', [f'ANG {p[1]} {self.ang[int(p[1])]:.4f}']
        if v == 'AP':
            self._servo(1, float(p[1]))
            self._servo(2, float(p[2]))
            return 'DONE', [f'ANG 1 {self.ang[1]:.4f}', f'ANG 2 {self.ang[2]:.4f}']
        if v == 'U':
            self.released[int(p[1])] = True
            return 'DONE', []
        if v == 'CLAW':
            self.claw = {'O': self.P.get('CLWO', 2090), 'C': self.P.get('CLWC', 2910)}.get(p[1]) or int(p[1])
            return 'DONE', []
        if v == 'TT':
            self.tt = int(p[1]) if int(p[1]) > 3 else self.P[f'TT{p[1]}']
            return 'DONE', []
        return 'ERR CMD', []


class Script:
    """按顺序给向导喂输入。('hand', a1, a2) = 先用手把舵机摆到这两个角度(舵机要先松开)，再回车。"""

    def __init__(self, stm, items):
        self.stm = stm
        self.items = list(items)
        self.prompts = []

    def __call__(self, prompt):
        self.prompts.append(prompt)
        if not self.items:
            raise EOFError
        it = self.items.pop(0)
        if isinstance(it, tuple) and it[0] == 'hand':
            assert self.stm.released[1] and self.stm.released[2], '舵机没松开就用手摆'
            self.stm.ang[1], self.stm.ang[2] = it[1], it[2]
            return ''
        return it


def make_cfg(extra=None):
    d = tempfile.mkdtemp()
    path = os.path.join(d, 'cfg.json')
    cfg = {'stops': {'QR': [1, 2, 90]}, 'stm32_params': {'FPPM': 12.9}}
    if extra:
        cfg['stm32_params'].update(extra)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(cfg, f)
    return path


def load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


FULL = [
    'w2', '2',                         # AEXT：先让 ID2 动一下看看，再选 2
    '-50', '',                         # CLWO 2090 -> 2040
    '2800', '',                        # CLWC 直接 2800
    '',                                # ZHI 保持 100
    '1 +10', '2 -20', '',              # 转盘上方：ID1 333、ID2 -882
    '-8', '',                          # TT1 2608 -> 2600
    '+12', '',                         # TT2 1708 -> 1720
    's',                               # TT3 跳过
    'c', '-30', '-5', '',              # ZDROP：夹住物料，100 -> 65
    'u', ('hand', 350.2, -700.5),      # 原料盘：松开用手摆
    '-80', '',                         # ZGRAB 100 -> 20
    's',                               # ZOBRAW 跳过
    '2 -760', '',                      # 圆环：ID2 直接到 -760(负数大于 200 = 直接转到)
    '5', '',                           # ZPLC 直接 5mm
    '65', '',                          # ZSTK 65
    'c', '1 +0.5', '2 -10', '',        # PL1：第一层放置补偿 ID1 +0.5、ID2 -10
    's',                               # PL2 跳过
    's',                               # ZOBRNG 跳过
    '+20', '',                         # A1H 323 -> 343
    'y',                               # 打开 ARMOK
]


class WizardTests(unittest.TestCase):
    def run_wiz(self, items, stm=None, cfg=None, start=0, **kw):
        self.stm = stm or FakeStm32()
        self.cfg = cfg if cfg is not None else make_cfg()
        self.lines = []
        self.inp = Script(self.stm, items)
        kw.setdefault('vision', False)                                    # 测试里不开真的摄像头
        kw.setdefault('sleep', lambda s: None)
        self.wiz = Wizard(self.stm, self.cfg, inp=self.inp, out=self.lines.append, **kw)
        return self.wiz.run(start)

    # ---------------- 第一层 / 第二层放置位置(PL1、PL2)：补偿相对"摄像头对准圆环的姿态"
    TRUE = (324.2, -880.0)                     # 摄像头里圆环正对爪子点时 ID1、ID2 的角度(车停得和 A1P 那一步不一样)

    def _cam(self):
        import numpy as np
        from visual_servo import JacStore
        stm = FakeStm32()
        J = np.array([[0.0, -3.8], [0.26, 0.0]])                          # 列：ID2 每度、ID1 每度 画面动多少像素

        class FakeVision:
            def ring_error(self, n=None, max_px=None):
                d = np.array([stm.ang[2] - WizardTests.TRUE[1], stm.ang[1] - WizardTests.TRUE[0]])
                p = J @ d
                return float(p[0]), float(p[1])

            def scale(self, kind):
                return 1.46

            def bounds(self, kind, inset=8.0):
                return ((-300.0, -220.0), (300.0, 220.0))

            def close(self):
                pass
        store = JacStore(None)
        store.put('RING', 'arm', J)
        return stm, FakeVision(), store

    def _err_mm_at(self, vis, a1, a2):
        import math
        self.stm.ang[1], self.stm.ang[2] = a1, a2
        p = vis.ring_error()
        return math.hypot(*p) / 1.46

    def test_place_steps_measure_from_the_camera_alignment(self):
        """PL1、PL2：空爪先用摄像头对准圆环(和比赛时一样)，补偿 = 夹着物料对正时比这个姿态多转多少；PL2 沿用 PL1 的对准，
        PL1 放下的物料就是下面那层(不用再问)。车停得和 A1P 那一步不一样也不影响。"""
        stm, vis, store = self._cam()
        saved = self.run_wiz(['c', '1 +0.5', '2 -10', '',              # PL1
                              'c', '1 -0.3', '',                       # PL2
                              'q'], stm=stm, start=STEP_KEYS.index('PL1'), vision=vis, store=store)
        adj = load(os.path.join(os.path.dirname(self.cfg), 'place_adj.json'))
        self.assertEqual(adj, {'RING': [0.5, -10.0], 'STACK': [-0.3, 0.0]}, self.lines)
        self.assertEqual(saved['PL1'], (0.5, -10.0))
        ref = self.wiz._place_ref
        self.assertIsNotNone(ref, self.lines)
        self.assertLess(self._err_mm_at(vis, *ref), 1.0)                 # 对准的姿态就是圆环正对爪子点的地方
        self.assertGreater(abs(ref[0] - 323.0), 0.5)                      # 不是 A1P(车停得不一样)
        self.assertTrue(any('摄像头对准圆环了' in l for l in self.lines), self.lines)
        self.assertTrue(any('沿用上一步(PL1)' in l for l in self.lines), self.lines)
        self.assertFalse(any('当下面那层' in p for p in self.inp.prompts), self.inp.prompts)

    def test_pl2_alone_aligns_first_then_asks_for_the_lower_material(self):
        """单独做 PL2：圆环先空着让摄像头对准，再叫你把下面那层放进圆环正中。"""
        stm, vis, store = self._cam()
        self.run_wiz(['', 'c', '2 +5', '', 'q'], stm=stm, start=STEP_KEYS.index('PL2'), vision=vis, store=store)
        adj = load(os.path.join(os.path.dirname(self.cfg), 'place_adj.json'))
        self.assertEqual(adj, {'STACK': [0.0, 5.0]}, self.lines)
        self.assertTrue(any('当下面那层' in p for p in self.inp.prompts), self.inp.prompts)
        self.assertTrue(any('圆环里先空着' in l for l in self.lines), self.lines)

    def test_place_step_without_camera_falls_back_to_a1p(self):
        """用不了摄像头：按 A1P/A2P 算，并且说清楚这样要注意什么。"""
        self.run_wiz(['c', '1 +0.5', '', 'q'], start=STEP_KEYS.index('PL1'))
        adj = load(os.path.join(os.path.dirname(self.cfg), 'place_adj.json'))
        self.assertEqual(adj, {'RING': [0.5, 0.0]})
        self.assertTrue(any('这次没用摄像头对准' in l for l in self.lines), self.lines)

    def test_camera_failure_falls_back(self):
        """摄像头打不开、认不到圆环：不卡住，退回按 A1P/A2P 算。"""
        class Broken:
            def ring_error(self, n=None, max_px=None):
                return None

            def scale(self, kind):
                return 1.46

            def bounds(self, kind, inset=8.0):
                return ((-300.0, -220.0), (300.0, 220.0))
        self.run_wiz(['c', '2 -4', '', 'q'], start=STEP_KEYS.index('PL1'), vision=Broken())
        adj = load(os.path.join(os.path.dirname(self.cfg), 'place_adj.json'))
        self.assertEqual(adj, {'RING': [0.0, -4.0]})
        self.assertTrue(any('没对准圆环' in l or '没做成' in l for l in self.lines), self.lines)
        self.assertTrue(any('这次没用摄像头对准' in l for l in self.lines), self.lines)

    def test_full_run_saves_everything(self):
        saved = self.run_wiz(FULL)
        want = dict(AEXT=2, CLWO=2040, CLWC=2800, ZHI=100, A1D=333, A2R=-882, TT1=2600, TT2=1720, ZDROP=65,
                    A1G=350.2, A2E=-700.5, ZGRAB=20, A1P=323, A2P=-760, ZPLC=5, ZSTK=65, A1H=343, ARMOK=1)
        for k, v in want.items():
            self.assertAlmostEqual(saved[k], v, places=3, msg=k)
            self.assertAlmostEqual(self.stm.P[k], v, places=3, msg='STM32 ' + k)
        self.assertNotIn('TT3', saved)
        self.assertNotIn('ZOBRAW', saved)
        self.assertEqual(saved['PL1'], (0.5, -10))                        # 放置补偿存进 place_adj.json
        self.assertNotIn('PL2', saved)
        adj = load(os.path.join(os.path.dirname(self.cfg), 'place_adj.json'))
        self.assertEqual(adj, {'RING': [0.5, -10]})
        sp = load(self.cfg)['stm32_params']
        for k, v in want.items():
            self.assertAlmostEqual(sp[k], v, places=3, msg='配置 ' + k)
        self.assertEqual(sp['FPPM'], 12.9)                                # 原来的参数不动
        self.assertTrue(os.path.exists(self.cfg.replace('.json', '.json.bak')))
        self.assertEqual(self.stm.claw, 2040)                             # 最后夹爪是张开的
        self.assertEqual((self.stm.ang[1], self.stm.ang[2]), (343, -882))  # 收臂：A1H + A2R
        self.assertEqual(self.stm.z, 100)

    def test_lift_goes_up_before_servos_move(self):
        self.run_wiz(FULL)
        z = None
        for cmd in self.stm.sent:
            if cmd.startswith('LIFT ') and cmd != 'LIFT?':
                z = float(cmd.split()[1])
            if cmd.startswith('AP '):
                self.assertEqual(z, 100.0, '转舵机(AP)之前升降必须在 ZHI：' + cmd)

    def test_hand_pose_reengages_servos(self):
        self.run_wiz(FULL)
        i = self.stm.sent.index('U 2')
        self.assertIn('AP 350.2 -700.5', self.stm.sent[i:])                # 用手摆好以后重新出力
        self.assertIn('SET A1G 350.2', self.stm.sent)

    def test_down_steps_release_and_lift(self):
        self.run_wiz(FULL)
        sent = self.stm.sent
        i = sent.index('SET ZDROP 65')
        self.assertEqual(sent[i + 1:i + 3], ['CLAW 2040', 'LIFT 100'])     # 放下：先松开，再抬起

    def test_no_repeated_commands(self):
        self.run_wiz(FULL)
        lifts = [c for c in self.stm.sent if c.startswith('LIFT ') and c != 'LIFT?']
        self.assertFalse(any(a == b for a, b in zip(lifts, lifts[1:])), lifts)   # 同一个高度不重复发
        tts = [c for c in self.stm.sent if c.startswith('TT ')]
        self.assertFalse(any(a == b for a, b in zip(tts, tts[1:])), tts)

    def test_read_after_hand(self):
        self.run_wiz(['2', 's', 's', 's', 'u', ('hand', 300.0, -900.0)][:-1] + ['r', 'q'])
        self.assertTrue(any('ID1=323°  ID2=-862°' in l for l in self.lines))

    def test_quit_with_unsaved_change_warns_first(self):
        start = STEP_KEYS.index('ZGRAB')
        saved = self.run_wiz(['-10', 'q', '', 'q'], start=start)          # 调了没回车就 q：先提醒，回车才保存
        self.assertTrue(any('还没保存' in l for l in self.lines), self.lines)
        self.assertIn('ZGRAB', saved)
        saved = self.run_wiz(['-10', 'q', 'q'], start=start)              # 再 q 一次：不保存直接退出
        self.assertNotIn('ZGRAB', saved)
        self.assertIn('退出。', self.lines)

    def test_camera_view_opens_on_camera_steps(self):
        """原料盘/圆环那几步自动打开 vlive.py(看物料 / --ring)，离开这几步就关掉；v 手动开关。"""
        started = []

        class FakeProc:
            def __init__(self, args, **kw):
                self.args, self.alive = args, True
                started.append(self)

            def poll(self):
                return None if self.alive else 0

            def terminate(self):
                self.alive = False

            def wait(self, timeout=None):
                return 0
        import arm_calib
        self.stm = FakeStm32()
        self.cfg = make_cfg()
        self.lines = []
        self.inp = Script(self.stm, ['', 'v', 'v', '', 'q'])               # ZOBRAW 回车 → A1P 里 v 关、v 开 → 回车 → q
        self.wiz = arm_calib.Wizard(self.stm, self.cfg, inp=self.inp, out=self.lines.append, view=True, popen=FakeProc)
        self.wiz.run(STEP_KEYS.index('ZOBRAW'))
        modes = [('--ring' in p.args) for p in started]
        self.assertEqual(modes, [False, True, True], [p.args for p in started])   # 物料 → 圆环 → (关了再开)圆环
        self.assertTrue(all(not p.alive for p in started))                 # 退出时都关了
        self.assertTrue(any('摄像头画面已打开' in l for l in self.lines))
        self.assertTrue(any('摄像头画面已关' in l for l in self.lines))

    def test_no_view_by_default(self):
        self.run_wiz(['', 'q'], start=STEP_KEYS.index('ZOBRNG'))
        self.assertFalse(any('摄像头画面已打开' in l for l in self.lines))

    def test_back_and_quit(self):
        saved = self.run_wiz(['2', '-50', '', 'b', '+10', '', 'q'])
        self.assertEqual(saved['CLWO'], 2050)                              # 回到上一步重新调了
        self.assertNotIn('ARMOK', saved)                                   # 没调完不问 ARMOK
        self.assertIn('退出。', self.lines)

    def test_pose_needs_axis(self):
        self.run_wiz(['2', 's', 's', 's', '+5', '1 500', '', 'q'])
        self.assertTrue(any('说清楚调哪个' in l for l in self.lines))
        self.assertTrue(any('ID1 停在 417.6' in l for l in self.lines))   # 到限位了要提示
        self.assertAlmostEqual(self.wiz.saved['A1D'], 417.6)

    def test_blocked_servo_warns(self):
        stm = FakeStm32()
        stm.block1 = 340.0
        self.run_wiz(['2', 's', 's', 's', '1 +30', 'q'], stm=stm)
        self.assertTrue(any('ID1 停在 340' in l and '被挡住' in l for l in self.lines))

    def test_relative_vs_absolute(self):
        self.run_wiz(['2', 's', 's', 's', '2 -5', '2 -900', '2 +3', '', 'q'])
        self.assertAlmostEqual(self.wiz.saved['A2R'], -897.0)

    def test_claw_and_tt_limits(self):
        self.run_wiz(['2', '+2000', '', 'q'])
        self.assertTrue(any('夹爪到限位了' in l for l in self.lines))
        self.assertEqual(self.wiz.saved['CLWO'], 2910)

    def test_old_firmware(self):
        with self.assertRaises(CalError) as cm:
            self.run_wiz([], stm=FakeStm32(old=True))
        self.assertIn('旧程序', str(cm.exception))

    def test_lift_unknown(self):
        stm = FakeStm32()
        stm.known = 0
        with self.assertRaises(CalError):
            self.run_wiz([], stm=stm)

    def test_config_values_pushed_first(self):
        cfg = make_cfg({'CLWO': 2111, 'ZHI': 95})
        self.run_wiz(['q'], cfg=cfg)
        self.assertIn('SET CLWO 2111', self.stm.sent)
        self.assertEqual(self.stm.P['ZHI'], 95)
        self.assertIn('LIFT 95', self.stm.sent)                            # 用的是配置里的 ZHI

    def test_start_from_step(self):
        self.run_wiz(['', 'q'], start=STEP_KEYS.index('ZGRAB'))
        self.assertTrue(any('ZGRAB' in l for l in self.lines))
        self.assertIn('ZGRAB', self.wiz.saved)

    def test_zplc_high_needs_second_enter(self):
        """升降还在高处(100mm)就按回车：ZPLC 先不存、提醒；再按一次回车才存(10-10 实车误存成 100)。"""
        self.run_wiz(['', 'q'], start=STEP_KEYS.index('ZPLC'))
        self.assertNotIn('ZPLC', self.wiz.saved)
        self.assertTrue(any('掉下去' in l for l in self.lines))
        self.run_wiz(['', '', 'q'], start=STEP_KEYS.index('ZPLC'))
        self.assertIn('ZPLC', self.wiz.saved)
        self.run_wiz(['z 2', '', 'q'], start=STEP_KEYS.index('ZPLC'))
        self.assertEqual(self.wiz.saved.get('ZPLC'), 2)

    def test_aext_choice(self):
        self.run_wiz(['w1', 'x', '1', 'q'])
        self.assertEqual(self.wiz.saved['AEXT'], 1)
        self.assertTrue(any('输入 1 或 2' in l for l in self.lines))
        i = self.stm.sent.index('AF 1 328')
        self.assertEqual(self.stm.sent[i + 2], 'AF 1 323')                 # w1：动 5 度再回来

    def test_fullwidth_input(self):
        self.run_wiz(['2', '＋１０', '', 'q'])
        self.assertEqual(self.wiz.saved['CLWO'], 2100)

    def test_no_config(self):
        stm = FakeStm32()
        lines = []
        wiz = Wizard(stm, None, inp=Script(stm, ['1', 'q']), out=lines.append)
        saved = wiz.run()
        self.assertEqual(saved['AEXT'], 1)
        self.assertEqual(stm.P['AEXT'], 1)


class HelperTests(unittest.TestCase):
    def test_config_from_run_live(self):
        d = tempfile.mkdtemp()
        with open(os.path.join(d, 'run_live.sh'), 'w') as f:
            f.write('cd ~/lidar_map_north\npython3 map_merge_live.py --stm-port /dev/serial0 --config my_cfg.json --x 1\n')
        self.assertEqual(str(arm_calib.config_from_run_live(d)), os.path.join(d, 'my_cfg.json'))
        with open(os.path.join(d, 'run_live.sh'), 'w') as f:
            f.write('python3 map_merge_live.py\n')
        self.assertIsNone(arm_calib.config_from_run_live(d))
        self.assertIsNone(arm_calib.config_from_run_live(tempfile.mkdtemp()))

    def test_steps_cover_all_poses(self):
        keys = set()
        for s in STEPS:
            keys.update([s['key']] if isinstance(s['key'], str) else s['key'])
        for k in ('A1G', 'A1D', 'A1H', 'A1P', 'A2E', 'A2R', 'A2P', 'CLWO', 'CLWC', 'TT1', 'TT2', 'TT3',
                  'ZHI', 'ZGRAB', 'ZDROP', 'ZPLC', 'ZSTK', 'ZOBRAW', 'ZOBRNG', 'AEXT'):
            self.assertIn(k, keys)

    def test_main_bad_from(self):
        self.assertEqual(arm_calib.main(['--from', 'NOPE', '--config', '/nonexistent.json']), 1)


if __name__ == '__main__':
    unittest.main(verbosity=1)
