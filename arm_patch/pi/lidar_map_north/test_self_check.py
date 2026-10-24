"""(10-24)比赛前自检 check：只问、只看文件，不动车。"""
import json
import os
import tempfile
import unittest

import self_check


class FakeLink:
    def __init__(self, new_fw=True, armok=1.0, lift='LIFT 1 60.0 BOOT=POS'):
        self.new_fw, self.armok, self.lift = new_fw, armok, lift
        self.sent = []

    def request(self, text, timeout, collect=False):
        self.sent.append(text)
        if text == 'PING':
            out = (True, 'PONG', [])
        elif text == 'GET':
            out = (True, 'DONE', [f'P ARMOK={self.armok:.4f}', 'P ZHI=115.0000', 'P ZOBRAW=116.0000'])
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
            self.assertIn('固件是新的', text)
            self.assertIn('升降高度认准了：60.0mm', text)
            self.assertIn('原料区最近那格示教过', text)
            self.assertIn('ROUGH 的 1、3 号环调过手臂角度', text)
            self.assertTrue(any('雷达设备' in b for b in bad), bad)                   # 雷达设备文件不在
            self.assertIn('码垛躲避(路上有两层的垛时)：绕过电池盒在 R1T1 往里缩到 A2R', text)
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

    def test_no_link(self):
        lines = []
        bad = self_check.run(None, {}, '/nonexistent', lines.append, root=tempfile.gettempdir())
        self.assertTrue(any('STM32 串口没打开' in b for b in bad))
        self.assertTrue(any('机械臂任务没开' in b for b in bad))
        self.assertTrue(any('少停车点' in b for b in bad))


if __name__ == '__main__':
    unittest.main()
