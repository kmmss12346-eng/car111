"""zshift.py(主程序 zup 命令)：升降高度参数一起加减。python3 -m unittest test_zshift"""
import unittest

import zshift

HEIGHTS = dict(ZHI=120.0, ZGRAB=102.0, ZDROP=102.0, ZPLC=2.0, ZSTK=67.0, ZOBRAW=120.0, ZOBRNG=120.0)


class FakeLink:
    def __init__(self, params=None, refuse=()):
        self.P = dict(HEIGHTS, LFMAX=130.0, ATOL=0.3) if params is None else dict(params)
        self.refuse = set(refuse)
        self.sets = []

    def get_params(self):
        return dict(self.P), 'DONE'

    def set_param(self, name, value):
        if name in self.refuse or name not in self.P:
            return False, 'ERR NAME'
        self.P[name] = float(value)
        self.sets.append((name, float(value)))
        return True, 'DONE'


class ZShiftTests(unittest.TestCase):
    def test_parse(self):
        self.assertEqual(zshift.parse(['zup', '4']), (4.0, list(zshift.Z_KEYS)))
        self.assertEqual(zshift.parse(['zup', '-4', 'zhi', 'ZDROP']), (-4.0, ['ZHI', 'ZDROP']))
        for bad in (['zup'], ['zup', 'x'], ['zup', '0'], ['zup', '40'], ['zup', '4', 'ATOL']):
            with self.assertRaises(ValueError):
                zshift.parse(bad)

    def test_all_heights_up_4mm(self):
        """(10-28 用户"把所有的就是高度往上面提高四毫米变成124")按 STM32 现在的值加：120 -> 124，别的也都加 4；不是高度的参数不动。"""
        link, lines = FakeLink(), []
        done = zshift.shift(link, 4.0, list(zshift.Z_KEYS), lines.append)
        self.assertEqual(done, {k: v + 4.0 for k, v in HEIGHTS.items()})
        self.assertEqual(link.P['ATOL'], 0.3)
        text = '\n'.join(lines)
        self.assertIn('ZHI 120→124', text)
        self.assertIn('重做 vclaw', text)                          # 看原料盘/圆环的高度变了：提醒十字
        self.assertNotIn('LFMAX', text)
        back = zshift.shift(link, -4.0, list(zshift.Z_KEYS), lines.append)
        self.assertEqual(back, HEIGHTS)

    def test_only_named_and_lfmax_warning(self):
        link, lines = FakeLink(dict(HEIGHTS, LFMAX=120.0)), []
        done = zshift.shift(link, 4.0, ['ZHI', 'ZDROP'], lines.append)
        self.assertEqual(done, dict(ZHI=124.0, ZDROP=106.0))
        self.assertEqual(link.P['ZOBRNG'], 120.0)
        text = '\n'.join(lines)
        self.assertIn('LFMAX=120', text)                          # 升降最高只到 120：ZHI 124 实际升不到
        self.assertNotIn('重做 vclaw', text)

    def test_nothing_changed_when_a_height_would_go_negative(self):
        link = FakeLink()
        with self.assertRaises(ValueError):
            zshift.shift(link, -5.0, list(zshift.Z_KEYS), lambda m: None)   # ZPLC 2 - 5 < 0
        self.assertEqual(link.sets, [])

    def test_refused_and_missing_are_reported(self):
        link, lines = FakeLink(dict(ZHI=120.0, ZGRAB=102.0), refuse=('ZGRAB',)), []
        done = zshift.shift(link, 4.0, list(zshift.Z_KEYS), lines.append)
        self.assertEqual(done, dict(ZHI=124.0))
        text = '\n'.join(lines)
        self.assertIn('STM32 拒绝 ZGRAB', text)
        self.assertIn('ZDROP：STM32 里没有这个参数', text)


if __name__ == '__main__':
    unittest.main()
