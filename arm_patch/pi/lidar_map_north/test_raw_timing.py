"""(10-26 用户"有时候下一个已经在摄像头下面了但是还是一直不抓取，错过了又要再等一轮")最近那格等原料盘：
回来时它已经停着、只亲眼看到过一次停下也推得出这一次停了多久；推不出来要说出来；等着的时候量出停多久、转多久。"""
import unittest

from mission_hooks import MissionHooks


class FakePlate:
    """原料盘：segs = [(开始时刻, 结束时刻, 最近那格上的颜色 或 None=在转)]。每认一帧时间走 0.2 秒。
    停着：那个颜色正好在爪子点(320, 240)；转着：上一格的颜色每帧往外挪 30 像素(很快出了最近那格那一块)。"""

    def __init__(self, clk, segs):
        self.clk, self.segs = clk, segs

    def claw(self, kind):
        return (320.0, 240.0)

    def material_errors(self, cols):
        t = self.clk[0]
        self.clk[0] += 0.2
        prev = None
        for t0, t1, c in self.segs:
            if t0 <= t < t1:
                if c is not None:
                    return {int(c): (0.0, 0.0)}
                return {int(prev): (30.0 * (t - t0) / 0.2 + 5.0, 0.0)} if prev is not None else {}
            if c is not None:
                prev = c
        return {}


def _hooks(clk, segs, lines):
    h = MissionHooks({'mission_cfg': dict(servo_cal_file=None, vision_cal_file='', raw_keepalive_s=0)},
                     log=lines.append, vision=FakePlate(clk, segs), now=lambda: clk[0], sleep=lambda s: None)
    h._ct_t_arm = clk[0]
    return h


class RawPredictTests(unittest.TestCase):
    def test_one_stop_is_enough_for_the_next(self):
        lines = []
        h = _hooks([0.0], [], lines)
        h._ct_stops = [100.0]
        self.assertIsNone(h._raw_ct_predict(108.0))                # 还不到最早的下一次停下
        self.assertAlmostEqual(h._raw_ct_predict(109.5), 108.1)    # 按最早停下的算(周期打九折)：多算一点已经停了多久
        self.assertIsNone(h._raw_ct_predict(114.0))                # 按最早停下的算快停完了
        self.assertIsNone(h._raw_ct_predict(115.0))                # 离那一次太远：推不准
        h.raw_stops, h.raw_moves = [7.0], [2.5]                    # 量到过：按量到的
        self.assertAlmostEqual(h._raw_ct_period(), 9.5)
        self.assertAlmostEqual(h._raw_ct_predict(109.5), 100.0 + 0.9 * 9.5)


class RawWatchTests(unittest.TestCase):
    def test_back_while_the_next_stop_is_on(self):
        """刚在 100.0 那一次停下夹了 RED(1)，回到看的姿态是 109.5，原料盘已经停着、最近那格上是 GREEN(4)：
        以前推不了(只看到过一次停下)，一直等到它转一下再停(错过这一次)；现在按周期推出来，马上能用。"""
        clk, lines = [109.5], []
        h = _hooks(clk, [(0.0, 106.0, 1), (106.0, 109.0, None), (109.0, 115.0, 4), (115.0, 118.0, None), (118.0, 999.0, 6)], lines)
        h._ct_stops, h._ct_used = [100.0], 100.0
        ob = h._raw_ct_watch([1, 4, 6], 20.0, after_move=True, focus=((320.0, 240.0), 40.0), known_stop=100.0,
                             known_why=None, known_color=1)
        self.assertIsNotNone(ob)
        t, t_stop, px = ob
        self.assertLess(t, 110.5, lines)                           # 一回来就用上这一次(不等到 118 那一次)
        self.assertAlmostEqual(t_stop, 108.1)
        self.assertIn(4, px)
        self.assertIn('按量到的周期推算已经停了', '\n'.join(lines))

    def test_unknown_stop_is_said_once(self):
        """推不出来(只看到过一次、而且离得太远)：说一次为什么在等，等它转一下再停，按看到的停下算。"""
        clk, lines = [130.0], []
        h = _hooks(clk, [(0.0, 132.0, 4), (132.0, 135.0, None), (135.0, 999.0, 6)], lines)
        h._ct_stops, h._ct_used = [100.0], 100.0
        ob = h._raw_ct_watch([4, 6], 20.0, after_move=True, focus=((320.0, 240.0), 40.0), known_stop=None, say_unknown=True)
        text = '\n'.join(lines)
        self.assertEqual(text.count('不知道这一次已经停了多久'), 1, text)
        self.assertIn('GREEN', text)
        t, t_stop, px = ob
        self.assertTrue(135.0 <= t_stop <= 135.5, t_stop)
        self.assertIn(6, px)

    def test_waiting_measures_stop_and_move(self):
        """亲眼看到 100.0 停下、手臂没动，接着等：看到它 106 转起来、109 又停下 -> 量到停了约 6 秒、转了约 3 秒。"""
        clk, lines = [100.2], []
        h = _hooks(clk, [(0.0, 100.0, None), (100.0, 106.0, 4), (106.0, 109.0, None), (109.0, 999.0, 6)], lines)
        ob = h._raw_ct_watch([4, 6], 20.0, after_move=True, focus=((320.0, 240.0), 40.0), prev_stop=100.0)
        _t, t_stop, _px = ob
        self.assertTrue(h._ct_seen)
        self.assertTrue(108.8 <= t_stop <= 109.5, t_stop)
        self.assertEqual(len(h.raw_stops), 1)
        self.assertEqual(len(h.raw_moves), 1)
        self.assertAlmostEqual(h.raw_stops[0], 6.0, delta=0.4)
        self.assertAlmostEqual(h.raw_moves[0], 3.0, delta=0.4)
        clk2, lines2 = [100.2], []                                 # 中间看不出(没给 prev_stop)：不量
        h2 = _hooks(clk2, [(0.0, 100.0, None), (100.0, 106.0, 4), (106.0, 109.0, None), (109.0, 999.0, 6)], lines2)
        h2._raw_ct_watch([4, 6], 20.0, after_move=True, focus=((320.0, 240.0), 40.0))
        self.assertEqual(h2.raw_stops, [])
        self.assertNotIn('不知道这一次已经停了多久', '\n'.join(lines + lines2))    # 接着等同一次停下：不说


if __name__ == '__main__':
    unittest.main()
