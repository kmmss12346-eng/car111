"""(10-24)比赛前自检 check：只问、只看文件，不动车。"""
import json
import os
import tempfile
import unittest

import self_check


class FakeLink:
    def __init__(self, new_fw=True, armok=1.0, lift='LIFT 1 60.0 BOOT=POS', prebg=True):
        self.new_fw, self.armok, self.lift, self.prebg = new_fw, armok, lift, prebg
        self.sent = []

    def request(self, text, timeout, collect=False):
        self.sent.append(text)
        if text == 'PING':
            out = (True, 'PONG', [])
        elif text == 'GET':
            out = (True, 'DONE', [f'P ARMOK={self.armok:.4f}', 'P ZHI=115.0000', 'P ZOBRAW=116.0000']
                   + (['P PREBG=1.0000'] if self.new_fw and self.prebg else []))
        elif text == 'LIFT?':
            out = (True, 'DONE', [self.lift])
        elif text.startswith('PRE'):
            out = (False, 'ERR ARG' if self.new_fw else 'ERR CMD', [])
        else:
            out = (False, 'ERR CMD', [])
        return out if collect else out[:2]


class SelfCheckTests(unittest.TestCase):
    def _files(self, d, teach=True):
        if teach:
            with open(os.path.join(d, 'ring_teach.json'), 'w', encoding='utf-8') as f:
                json.dump({'RAW_CT': {'near': '1', 'slots': {'1': {'px': [1, 2], 'a': [55.3, -946.1]}}},
                           'ROUGH': {'1': [1, 2], '3': [3, 4], 'src': 'arm_calib'},
                           'TEMP': {'1': [1, 2], '3': [3, 4], 'src': 'arm_calib'}}, f)
            with open(os.path.join(d, 'servo_cal.json'), 'w', encoding='utf-8') as f:
                json.dump({'RAW': {'arm': [[1, 0], [0, 1]]}, 'RING': {'arm': [[1, 0], [0, 1]]}}, f)

    def _cfg(self):
        return dict(mission_cfg=dict(enabled=True, full_course=True),
                    stops=dict(QR=[2100, 1350, 90], RAW=[1200, 2160, 180], ROUGH=[1200, 340, 0], TEMP=[340, 1200, -90]))

    def test_only_queries_and_reports(self):
        with tempfile.TemporaryDirectory() as d:
            self._files(d)
            lines, link = [], FakeLink()
            bad = self_check.run(link, self._cfg(), os.path.join(d, 'no_lidar'), lines.append, root=d)
            text = '\n'.join(lines)
            self.assertTrue(set(link.sent) <= {'PING', 'GET', 'LIFT?', 'PRE X'}, link.sent)     # 只问，不发会动的指令
            self.assertIn('固件是新的(10-26：开往工位时手臂在后台转、车不停；PREBG=1)', text)
            self.assertIn('底盘速度：speed normal(直行 220、横移 170 转/分)。提速：speed fast，再 turntune', text)
            self.assertIn('升降高度认准了：60.0mm', text)
            self.assertIn('原料区最近那格示教过', text)
            self.assertIn('ROUGH 的 1、3 号环调过手臂角度', text)
            self.assertTrue(any('雷达设备' in b for b in bad), bad)                   # 雷达设备文件不在
            self.assertIn('码垛躲避(第二层每一个都这样走)：绕过电池盒在 R1T1 往里缩到 A2R', text)
            self.assertIn('自检完', text)
            cfg = self._cfg()
            cfg['mission_cfg'].update(stack_carry_a1=150.5, stack_carry_a2=-1230)
            lines = []
            self_check.run(FakeLink(), cfg, os.path.join(d, 'no_lidar'), lines.append, root=d)
            self.assertIn('在 ID1 150.5° 往里缩到 ID2 -1230°，缩着转到要放的环再伸出去(arm_calib 调过)', '\n'.join(lines))

    def test_old_firmware_and_missing_teach(self):
        with tempfile.TemporaryDirectory() as d:
            self._files(d, teach=False)
            lines = []
            bad = self_check.run(FakeLink(new_fw=False, lift='LIFT 1 60.0 BOOT=NOENC'), self._cfg(), d, lines.append, root=d)
            text = '\n'.join(bad)
            self.assertIn('固件可能是旧的', text)
            self.assertIn('升降高度是猜的', text)
            self.assertIn('原料区最近那格没示教', text)
            self.assertIn('TEMP 的 1、3 号环没调', text)

    def test_firmware_before_background_pre_and_speed_level(self):
        """(10-26)认识 PRE 但没有 PREBG：提前摆臂时车要停一下，★ 让烧 tasks_1026d；speed fast 以后自检写现在的档。"""
        with tempfile.TemporaryDirectory() as d:
            self._files(d)
            cfg = dict(self._cfg(), route_speed_rpm=190, strafe_speed_rpm=160, speed_saved={'level': 'fast'})
            lines = []
            bad = self_check.run(FakeLink(prebg=False), cfg, os.path.join(d, 'no_lidar'), lines.append, root=d)
            self.assertTrue(any('固件是 10-26 以前的' in b and 'tasks_1026d' in b for b in bad), bad)
            self.assertIn('底盘速度：speed fast(直行 190、横移 160 转/分)。车头偏、停不准就 speed normal 还原', '\n'.join(lines))

    def test_lfchk_zero_is_flagged(self):
        """(10-25)LFCHK=0：升降走完不核对，旧固件碰到地面一直顶到过载保护：★ 让改回 1.5。"""
        class Link0(FakeLink):
            def request(self, text, timeout, collect=False):
                out = super().request(text, timeout, collect=True)
                if text == 'GET':
                    out = (True, 'DONE', out[2] + ['P LFCHK=0.0000'])
                return out if collect else out[:2]
        with tempfile.TemporaryDirectory() as d:
            self._files(d)
            bad = self_check.run(Link0(), self._cfg(), os.path.join(d, 'no_lidar'), lambda s: None, root=d)
            self.assertTrue(any('set LFCHK 1.5' in b for b in bad), bad)
            bad = self_check.run(FakeLink(), self._cfg(), os.path.join(d, 'no_lidar'), lambda s: None, root=d)
            self.assertFalse(any('LFCHK' in b for b in bad), bad)

    def test_fixed_route_left_on_is_flagged(self):
        """(10-28)写死模式(给老师的对比视频)开着：★ 提醒比赛前 fixroute off；关着不提。"""
        with tempfile.TemporaryDirectory() as d:
            self._files(d)
            bad = self_check.run(FakeLink(), dict(self._cfg(), fixed_route=True), os.path.join(d, 'no_lidar'), lambda s: None, root=d)
            self.assertTrue(any('写死模式开着' in b and 'fixroute off' in b for b in bad), bad)
            bad = self_check.run(FakeLink(), self._cfg(), os.path.join(d, 'no_lidar'), lambda s: None, root=d)
            self.assertFalse(any('写死模式' in b for b in bad), bad)

    def test_no_link(self):
        lines = []
        bad = self_check.run(None, {}, '/nonexistent', lines.append, root=tempfile.gettempdir())
        self.assertTrue(any('STM32 串口没打开' in b for b in bad))
        self.assertTrue(any('机械臂任务没开' in b for b in bad))
        self.assertTrue(any('少停车点' in b for b in bad))


if __name__ == '__main__':
    unittest.main()
