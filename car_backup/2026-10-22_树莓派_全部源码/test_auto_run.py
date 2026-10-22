"""auto_run.py 的检查：车没按指令转(ERR STALL / 车头还差很多度)时要停下并给出提示。python3 -m unittest test_auto_run"""
import re
import sys
import types
import unittest

# auto_run 要用您 v14 里的 route_plan / stm32_link；电脑上没有时用最小的替身(树莓派上有真的就用真的)
try:
    import stm32_link  # noqa: F401
except Exception:
    _sl = types.ModuleType('stm32_link')

    def parse_done(reply):
        out = {}
        m = re.search(r't=(\d+)', reply or '')
        if m:
            out['t'] = int(m.group(1)) / 1000.0
        m = re.search(r'e=(-?\d+)', reply or '')
        if m:
            out['err'] = int(m.group(1)) / 100.0
        return out
    _sl.parse_done = parse_done
    sys.modules['stm32_link'] = _sl
try:
    import route_plan  # noqa: F401
except Exception:
    _rp = types.ModuleType('route_plan')

    class Motion:
        def __init__(self, cfg=None):
            pass

        def time_of(self, cmd, val):
            return 0.0

        def route_time(self, cmds):
            return 0.0
    _rp.Motion = Motion
    sys.modules['route_plan'] = _rp

import auto_run
from auto_run import Abort, check_move


class FakeCtx:
    def __init__(self):
        self.scans = 0

    def aborted(self):
        return False

    def scan(self):
        self.scans += 1

    def second(self):
        self.scans += 1

    def plan(self):
        return 0, [{'stop': 'QR', 'cmds': [('R', -90), ('F', 300)]}], 'ROUTE OK'

    def ask(self, msg):
        return True


class FakeLink:
    def __init__(self, second=(True, 'DONE t=1200 e=12'), moves=None):
        self._second = second
        self._moves = list(moves or [])
        self.sent = []

    def ping(self):
        return True, 'PONG'

    def sync_params(self, params, log=None):
        return []

    def home(self):
        return True, 'DONE'

    def second_move(self, rel=None, log=None):
        return self._second

    def move(self, cmd, val, speed=None):
        self.sent.append((cmd, val))
        return self._moves.pop(0) if self._moves else (True, 'DONE t=800 e=10')

    def abort(self):
        pass


class CheckMoveTests(unittest.TestCase):
    def test_normal_reply_passes(self):
        self.assertIsNone(check_move('R', -60, True, 'DONE t=900 e=-35'))
        self.assertIsNone(check_move('F', 300, True, 'DONE'))

    def test_turn_left_far_off(self):
        msg = check_move('R', -60, True, 'DONE t=6838 e=-5780')     # 这次日志里的情况(旧 STM32 程序)
        self.assertIn('-57.8', msg)
        self.assertIn('mot', msg)

    def test_stall_reply_gets_hint(self):
        msg = check_move('R', -60, False, 'ERR STALL t=847 e=-6000')
        self.assertIn('ERR STALL', msg)
        self.assertIn('send R 90', msg)

    def test_other_error_no_hint(self):
        msg = check_move('F', 300, False, 'ERR GYRO')
        self.assertIn('ERR GYRO', msg)
        self.assertNotIn('mot', msg)


class RunMissionTests(unittest.TestCase):
    def run_mission(self, link):
        logs = []
        try:
            auto_run.run_mission(FakeCtx(), link, log=logs.append, stop_wait=0.0, settle_s=0.0, cfg={})
        finally:
            self.logs = logs

    def test_second_station_turn_not_done_stops_before_scan(self):
        link = FakeLink(second=(True, 'DONE t=6838 e=-5780'))
        with self.assertRaises(Abort) as cm:
            self.run_mission(link)
        self.assertIn('第二站没转到位', str(cm.exception))
        self.assertEqual(link.sent, [], '第二站没转到位还继续发路线指令')

    def test_second_station_stall_error(self):
        link = FakeLink(second=(False, 'R -60 失败：ERR STALL t=847 e=-6000'))
        with self.assertRaises(Abort) as cm:
            self.run_mission(link)
        self.assertIn('mot', str(cm.exception))

    def test_route_leg_stall_stops(self):
        link = FakeLink(moves=[(False, 'ERR STALL t=900 e=-8800')])
        with self.assertRaises(Abort) as cm:
            self.run_mission(link)
        self.assertIn('第1条指令', str(cm.exception))
        self.assertEqual(link.sent, [('R', -90)], '卡住后不该再发下一条')

    def test_normal_run_completes(self):
        link = FakeLink()
        self.run_mission(link)
        self.assertEqual(link.sent, [('R', -90), ('F', 300)])
        self.assertTrue(any('路线全部走完' in l for l in self.logs))


if __name__ == '__main__':
    unittest.main()
