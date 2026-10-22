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
              CLWO=(500, 2910), CLWC=(500, 2910), CLWOR=(0, 2910), CLWOT=(0, 2910), TT1=(500, 2608), TT2=(500, 2608), TT3=(500, 2608),
              AEXT=(1, 2), ARMOK=(0, 1), FPPM=(1, 100), A2T=(-5000, 5000), A1T1=(-720, 720), A1T2=(-720, 720), A2SPD=(0, 1500),
              R1T1=(-720, 720), R2T=(-5000, 5000), R1T2=(-720, 720), R2SPD=(0, 1500))
DEFAULTS = dict(LFMAX=100, ZHI=100, ZGRAB=20, ZDROP=60, ZPLC=0, ZSTK=60, ZOBRAW=100, ZOBRNG=100,
                A1G=323, A1D=323, A1H=323, A1P=323, A2E=-862, A2R=-862, A2P=-862,
                CLWO=2090, CLWC=2910, CLWOR=0, CLWOT=0, TT1=2608, TT2=1708, TT3=808, AEXT=2, ARMOK=0, FPPM=12.9, A2T=0, A1T1=0, A1T2=0, A2SPD=0,
                R1T1=0, R2T=0, R1T2=0, R2SPD=0)
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
        self.top = None                  # (10-21)升降机构真正能到的最高(超过就顶住，STM32 回 LIFTSLIP)
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
            if self.top is not None and self.z > self.top:
                want, self.z = self.z, self.top
                return 'DONE', [f'LIFTSLIP want={want:.4f} got={self.z:.4f}']
            return 'DONE', []
        if v == 'A?':
            return 'DONE', [f'ANG {p[1]} {self.ang[int(p[1])]:.4f}']
        if v == 'AFS':
            self.afs = getattr(self, 'afs', []) + [(int(p[1]), float(p[2]), float(p[3]))]
            self._servo(int(p[1]), float(p[2]))
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
        if v == 'SIDE':
            return 'DONE', []
        if v in ('TRAY', 'APZ'):                                 # (10-21)按 STM32 的路走到车上转盘上方 / 从车上转盘到 (a1, a2) 高度 z
            if self.P.get('ARMOK', 0) < 0.5:
                return 'ERR NOCAL', []
            a1, a2, z = (self.P['A1D'], self.P['A2R'], self.P['ZHI']) if v == 'TRAY' else (float(p[1]), float(p[2]), float(p[3]))
            self._servo(1, a1)
            self._servo(2, a2)
            self.z = z
            return 'DONE', [f'ANG 1 {self.ang[1]:.4f}', f'ANG 2 {self.ang[2]:.4f}']
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
    '-100', '',                        # CLWOT：车上转盘上方张开，从 CLWO(2040)调到 1940
    'u', ('hand', 350.2, -700.5),      # 原料盘：松开用手摆
    's', 's', 's', 's',                # 绕开电池盒 A1T1/A2T/A1T2/A2SPD 跳过
    '-80', '',                         # ZGRAB 100 -> 20
    's',                               # CLWOR 跳过(和 CLWO 一样)
    's',                               # ZOBRAW 跳过
    '2 =-760', '',                     # 圆环：ID2 直接到 -760(= 开头 = 直接转到)
    's', 's',                          # 1、3 号环 R1/R3 跳过
    's', 's', 's', 's',                # 车上转盘去圆环绕开电池盒 R1T2/R2T/R1T1/R2SPD 跳过
    '5', '',                           # ZPLC 直接 5mm
    '65', '',                          # ZSTK 65
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
        kw.setdefault('sleep', lambda s: None)
        self.wiz = Wizard(self.stm, self.cfg, inp=self.inp, out=self.lines.append, **kw)
        return self.wiz.run(start)

    def test_detour_steps(self):
        """(10-19 用户：电池盒挡着)A1T1 转到开始伸长的角度、A2T 伸多长、A1T2 开始缩回的角度、A2SPD 速度(演示伸缩)，
        做完按原路回到原料盘上方。"""
        self.run_wiz(['1 +20', '', '2 +50', '', '1 +40', '', '400', '', 'q'], start=STEP_KEYS.index('A1T1'))
        P = self.stm.P
        self.assertAlmostEqual(P['A1T1'], P['A1G'] + 20, delta=0.5)
        self.assertAlmostEqual(P['A2T'], P['A2E'] + 50, delta=0.5)
        self.assertAlmostEqual(P['A1T2'], P['A1T1'] + 40, delta=0.5)
        self.assertEqual(P['A2SPD'], 400)
        self.assertEqual([(i, s_) for i, _a, s_ in self.stm.afs], [(2, 400.0), (2, 400.0)])   # 按新速度缩回、伸出给人看
        self.assertAlmostEqual(self.stm.ang[1], P['A1G'], delta=0.5)                         # 原路回到原料盘上方

    def test_a2t_step_skipped_on_old_firmware(self):
        stm = FakeStm32()
        for k in ('A1T1', 'A2T', 'A1T2', 'A2SPD'):
            del stm.P[k]
        self.run_wiz(['q'], stm=stm, start=STEP_KEYS.index('A1T1'))
        self.assertIn('旧固件', '\n'.join(self.lines))

    # ---------------- (10-20 用户)1、3 号环：车不动只转手臂；存比 2 号环(A1P/A2P)多转多少，粗加工区、暂存区共用
    def test_ring_poses_saved_for_both_zones(self):
        cfg = make_cfg()
        path = os.path.join(os.path.dirname(cfg), 'ring_teach.json')
        with open(path, 'w', encoding='utf-8') as f:                       # 以前 rteach 存的：原料区的不动，1 号环摄像头记的作废
            json.dump({'RAW': {'1': [5, 6]}, 'ROUGH': {'1': [1.0, 2.0], 'e1': [10, 20], 'src': 'rteach'}}, f)
        saved = self.run_wiz(['1 +11', '2 -32', '',                         # R1：从以前存的 +1/+2 开始调
                              '1 -12.5', '2 +20', '',                       # R3：没存过，从 2 号环上方开始
                              'q'], cfg=cfg, start=STEP_KEYS.index('R1'))
        d = load(path)
        self.assertEqual(d['RAW'], {'1': [5, 6]})
        for zone in ('ROUGH', 'TEMP'):
            self.assertEqual(d[zone]['1'], [12.0, -30.0], zone)
            self.assertEqual(d[zone]['3'], [-12.5, 20.0], zone)
            self.assertEqual(d[zone]['src'], 'arm_calib')
            self.assertEqual(d[zone]['p2'], [323.0, -862.0])
            self.assertNotIn('e1', d[zone])
        self.assertEqual(saved['R1'], (12.0, -30.0))
        self.assertEqual(saved['R3'], (-12.5, 20.0))
        i = self.stm.sent.index('AF 2 -892')                               # R1 调完，R3 开始前手臂回到 2 号环上方
        self.assertIn('AP 323 -862', self.stm.sent[i:])
        self.assertNotIn('A1P', load(cfg)['stm32_params'])                 # 1、3 号环不改 STM32 参数
        out = '\n'.join(self.lines)
        self.assertIn('还没存过', out)
        self.assertIn('1、3 号环的角度存在', out)
        self.assertIn('AP 324 -860', self.stm.sent)
        saved = self.run_wiz(['q'], cfg=cfg, start=STEP_KEYS.index('R1'))  # 再进来：手臂直接到存过的位置，没改直接 q 不啰嗦
        self.assertIn('AP 335 -892', self.stm.sent)
        self.assertIn('原来存的 +12.00 -30.00', '\n'.join(self.lines))
        self.assertFalse(any('还没保存' in l for l in self.lines), self.lines)
        self.assertEqual(saved, {})

    def test_ring_pose_unsaved_change_warns_on_quit(self):
        saved = self.run_wiz(['1 +5', 'q', 'q'], start=STEP_KEYS.index('R1'))
        self.assertTrue(any('还没保存' in l and 'R1' in l for l in self.lines), self.lines)
        self.assertNotIn('R1', saved)
        self.assertFalse(os.path.exists(os.path.join(os.path.dirname(self.cfg), 'ring_teach.json')))

    def test_ring_detour_steps(self):
        """(10-21 用户"从车上转盘到暂存区先转一个角度再伸长再转一个角度再缩回，让我自由调")按从车上转盘出去的顺序调：
        从车上转盘上方(STM32 按比赛的路走过去)开始，R1T2(只转) -> R2T(只伸) -> R1T1(只转) -> R2SPD，做完缩到 2 号环的长度、转到 2 号环上方。"""
        stm = FakeStm32()
        stm.P['ARMOK'] = 1
        self.run_wiz(['c', '1 -20', '', '2 +50', '', '1 -40', '', '400', '', 'q'], stm=stm, start=STEP_KEYS.index('R1T2'))
        P = stm.P
        self.assertIn('TRAY', stm.sent)                                     # 从车上转盘上方开始(按比赛时去车上转盘的路)
        self.assertAlmostEqual(P['R1T2'], P['A1D'] - 20, delta=0.5)
        self.assertAlmostEqual(P['R2T'], P['A2R'] + 50, delta=0.5)
        self.assertAlmostEqual(P['R1T1'], P['R1T2'] - 40, delta=0.5)
        self.assertEqual(P['R2SPD'], 400)
        self.assertEqual(P['A2T'], 0)                                      # 原料区那组不动
        self.assertEqual([(i, a_, s_) for i, a_, s_ in stm.afs], [(2, -862.0, 400.0), (2, -812.0, 400.0)])   # 缩到 2 号环的长度再伸回来
        sent = stm.sent
        i = sent.index('SET R2SPD 400', sent.index('AFS 2 -812 400'))
        back = [c for c in sent[i:] if c.startswith(('AF ', 'AP '))]
        self.assertEqual(back[:2], ['AF 2 -862', 'AP 323 -862'])           # 只缩到 2 号环的长度，再转到 2 号环上方
        self.assertEqual(stm.claw, 2910)                                   # 物料一直夹着(下一步 ZPLC 要用)
        sp = load(self.cfg)['stm32_params']
        self.assertEqual(sp['R2SPD'], 400)

    def test_try_ring_path_with_t(self):
        """t：按保存的值让 STM32 真走一遍(去车上转盘 -> 去 2 号环上方 -> 回车上转盘)，走完回到这一步开始的地方。"""
        stm = FakeStm32()
        stm.P.update(ARMOK=1, R1T2=303.0, R2T=-812.0, R1T1=263.0)
        self.run_wiz(['t', 'q'], stm=stm, start=STEP_KEYS.index('R1T1'))
        sent = stm.sent
        k = sent.index('APZ 323 -862 100')
        self.assertIn('TRAY', sent[:k])
        self.assertIn('TRAY', sent[k:])
        after = [c for c in sent[sent.index('TRAY', k):] if c.startswith('AF ')]
        self.assertEqual(after[:2], ['AF 1 303', 'AF 2 -812'])               # 从车上转盘按分段走回这一步开始的地方(R1T2、R2T)
        self.assertTrue(any('试走完了' in l for l in self.lines), self.lines)
        stm = FakeStm32()                                                   # ARMOK=0：走不了，说清楚
        self.run_wiz(['t', 'q', 'q'], stm=stm, start=STEP_KEYS.index('R2T'))
        self.assertTrue(any('ARMOK=1' in l for l in self.lines), self.lines)

    def test_ring_detour_speed_zero_demo_uses_a2spd(self):
        stm = FakeStm32()
        stm.P.update(R1T1=263.0, R2T=-812.0, R1T2=303.0, A2SPD=300.0)
        self.run_wiz(['0', 'q', 'q'], stm=stm, start=STEP_KEYS.index('R2SPD'))
        self.assertEqual([(i, s_) for i, _a, s_ in stm.afs], [(2, 300.0), (2, 300.0)])

    def test_ring_detour_skipped_on_old_firmware(self):
        stm = FakeStm32()
        for k in ('R1T1', 'R2T', 'R1T2', 'R2SPD'):
            del stm.P[k]
        self.run_wiz(['q'], stm=stm, start=STEP_KEYS.index('R1T2'))
        out = '\n'.join(self.lines)
        self.assertEqual(out.count('旧固件'), 4)
        self.assertIn('ZPLC', out)

    def test_full_run_saves_everything(self):
        saved = self.run_wiz(FULL)
        want = dict(AEXT=2, CLWO=2040, CLWC=2800, CLWOT=1940, ZHI=100, A1D=333, A2R=-882, TT1=2600, TT2=1720, ZDROP=65,
                    A1G=350.2, A2E=-700.5, ZGRAB=20, A1P=323, A2P=-760, ZPLC=5, ZSTK=65, A1H=343, ARMOK=1)
        for k, v in want.items():
            self.assertAlmostEqual(saved[k], v, places=3, msg=k)
            self.assertAlmostEqual(self.stm.P[k], v, places=3, msg='STM32 ' + k)
        self.assertNotIn('TT3', saved)
        self.assertNotIn('ZOBRAW', saved)
        self.assertNotIn('CLWOR', saved)                                  # 跳过：还是 0(和 CLWO 一样)
        self.assertFalse(os.path.exists(os.path.join(os.path.dirname(self.cfg), 'place_adj.json')))   # (10-20)不再量放置补偿
        sp = load(self.cfg)['stm32_params']
        for k, v in want.items():
            self.assertAlmostEqual(sp[k], v, places=3, msg='配置 ' + k)
        self.assertEqual(sp['FPPM'], 12.9)                                # 原来的参数不动
        self.assertTrue(os.path.exists(self.cfg.replace('.json', '.json.bak')))
        self.assertEqual(self.stm.claw, 1940)                             # 最后夹爪是张开的((10-21)其他地方都用车上转盘上方的 CLWOT)
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

    def test_lift_slip_shown(self):
        """(10-21)升降顶住没到(STM32 回 LIFTSLIP)：向导里用中文说出来，提醒把 ZHI 这些改低。"""
        stm = FakeStm32()
        stm.top = 90.0
        self.run_wiz(FULL, stm=stm)
        slip = [l for l in self.lines if '升降没到 100.0mm' in l]
        self.assertTrue(slip, self.lines[-40:])
        self.assertIn('实际在 90.0mm', slip[0])
        self.assertIn('ZHI', slip[0])

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
        self.run_wiz(['2', 's', 's', 's', '2 -5', '2 =-900', '2 +3', '', 'q'])
        self.assertAlmostEqual(self.wiz.saved['A2R'], -897.0)

    def test_signed_values_are_always_relative(self):
        """10-16 用户：2 -400 以前当成"转到 -400 度"(一会往前一会往后)：现在带符号的一律是加减。"""
        self.run_wiz(['2', 's', 's', 's', '2 =-900', '2 -300', '2 +100', '', 'q'])
        self.assertAlmostEqual(self.wiz.saved['A2R'], -1100.0)

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

    # ---------------- (10-20 用户)夹爪张开三个角度：原料盘上方 CLWOR、车上转盘上方 CLWOT、其他 CLWO
    def test_tray_open_step_tunes_at_zdrop_and_keeps_value_through_c_and_o(self):
        saved = self.run_wiz(['-200', 'c', 'o', '', 'q'], start=STEP_KEYS.index('CLWOT'))
        self.assertEqual(saved['CLWOT'], 1890)                             # 从 CLWO 2090 开始调
        self.assertEqual(self.stm.P['CLWOT'], 1890)
        claws = [c for c in self.stm.sent if c.startswith('CLAW ')]
        self.assertEqual(claws[-3:], ['CLAW 1890', 'CLAW 2910', 'CLAW 1890'])   # 调到 1890，c 夹住看看，o 回到正在调的
        i = self.stm.sent.index('CLAW 1890')
        lifts = [c for c in self.stm.sent[:i] if c.startswith('LIFT ')]
        self.assertEqual(lifts[-1], 'LIFT 60')                             # 降到 ZDROP 调
        self.assertIn('AP 323 -862', self.stm.sent[:i])                    # 转盘上方的姿态
        self.assertEqual(self.stm.z, 100)                                  # 回车以后抬回 ZHI
        self.assertEqual(self.stm.claw, 1890)                              # 下一步到原料盘上方：CLWOR 没调 = 和其他地方一样用 CLWOT(和固件一样)

    def test_raw_open_step_unchanged_stays_zero(self):
        saved = self.run_wiz(['', 'q'], start=STEP_KEYS.index('CLWOR'))
        self.assertNotIn('CLWOR', saved)                                   # 没改：还是 0(和 CLWO 一样)
        self.assertEqual(self.stm.P['CLWOR'], 0)
        self.assertTrue(any('没改' in l for l in self.lines), self.lines)
        self.assertTrue(any('LIFT 20' == c for c in self.stm.sent), self.stm.sent)   # 在原料盘上方降到 ZGRAB 调

    def test_open_angles_follow_the_place(self):
        """(10-21 用户)张开只两种：原料盘那几步用 CLWOR，其他所有地方都用车上转盘上方的 CLWOT；没调(0)就是 CLWO。"""
        stm = FakeStm32()
        stm.P['CLWOR'], stm.P['CLWOT'] = 1500, 1700
        self.run_wiz(['c', 'o', 'q', 'q'], stm=stm, start=STEP_KEYS.index('TT1'))
        self.assertEqual([c for c in stm.sent if c.startswith('CLAW ')][-2:], ['CLAW 2910', 'CLAW 1700'])
        self.run_wiz(['q'], stm=stm, start=STEP_KEYS.index('A1G'))
        self.assertEqual(stm.claw, 1500)                                   # 原料盘上方：CLWOR
        self.run_wiz(['c', 'o', 'q', 'q'], stm=stm, start=STEP_KEYS.index('A1P'))
        self.assertEqual([c for c in stm.sent if c.startswith('CLAW ')][-1], 'CLAW 1700')   # 圆环那边也用 CLWOT
        self.run_wiz(['c', '-5', '', 'q'], stm=stm, start=STEP_KEYS.index('ZDROP'))
        self.assertEqual(stm.claw, 1700)                                   # 放进转盘松开：CLWOT

    def test_open_angle_steps_skipped_on_old_firmware(self):
        stm = FakeStm32()
        for k in ('CLWOR', 'CLWOT'):
            del stm.P[k]
        self.run_wiz(['q'], stm=stm, start=STEP_KEYS.index('CLWOT'))
        self.assertTrue(any('CLWOT' in l and '旧固件' in l for l in self.lines), self.lines)
        self.run_wiz(['', '', 'q'], stm=stm, start=STEP_KEYS.index('ZGRAB'))
        self.assertTrue(any('CLWOR' in l and '旧固件' in l for l in self.lines), self.lines)
        self.assertEqual(stm.claw, 2090)                                   # 没有 CLWOR：原料盘上方照旧 CLWO


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

    def test_place_compensation_steps_are_gone(self):
        """(10-20 用户"把 28、29 步的补偿去掉")没有 PL1/PL2 了；--from PL1 说清楚去掉了、该从哪调。"""
        self.assertNotIn('PL1', STEP_KEYS)
        self.assertNotIn('PL2', STEP_KEYS)
        self.assertFalse(any(s['kind'] == 'place' for s in STEPS))
        import io, contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            self.assertEqual(arm_calib.main(['--from', 'PL1', '--config', '/nonexistent.json']), 1)
        self.assertIn('已经去掉了', buf.getvalue())
        self.assertIn('--from A1P', buf.getvalue())


if __name__ == '__main__':
    unittest.main(verbosity=1)
