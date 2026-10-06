"""task_plan.py 的单元测试：python3 test_task_plan.py"""
import unittest

from task_plan import parse_code, TaskError, role_of, Stats, clean_code


class ParseTests(unittest.TestCase):
    def test_example_from_rules(self):
        p = parse_code('156+123+516+231')
        self.assertEqual([(i.color, i.ring, i.slot) for i in p.items(1)], [(1, 1, 1), (5, 2, 2), (6, 3, 3)])
        self.assertEqual([(i.color, i.ring, i.slot) for i in p.items(2)], [(5, 2, 1), (1, 3, 2), (6, 1, 3)])
        self.assertEqual(p.warnings, [])

    def test_stack_target_by_color(self):
        p = parse_code('156+123+516+231')
        # 第二批：黑(5) 叠在第一批黑所在的 2 号环；红(1) 叠在 1 号环；浅蓝(6) 叠在 3 号环
        self.assertEqual([p.stack_ring(i) for i in p.items(2)], [2, 1, 3])
        self.assertIsNone(p.stack_ring(p.items(1)[0]))      # 第一批没有码垛目标

    def test_stack_missing_color(self):
        p = parse_code('123+123+456+123')                   # 第二批颜色和第一批完全不同
        self.assertEqual([p.stack_ring(i) for i in p.items(2)], [None, None, None])

    def test_clean(self):
        self.assertEqual(clean_code(' "156＋123+516 +231"\r\n'), '156+123+516+231')
        p = parse_code('\x02156+123+516+231\r\n')
        self.assertEqual(p.code, '156+123+516+231')

    def test_errors(self):
        for bad in ('', '156+123+516', '156+123+516+23', '156+123+516+2a1', '756+123+516+231',
                    '156+423+516+231', '156+123+516+231+111', '106+123+516+231'):
            with self.assertRaises(TaskError, msg=bad):
                parse_code(bad)

    def test_warnings(self):
        p = parse_code('155+123+516+231')
        self.assertTrue(any('重复' in w for w in p.warnings))
        p = parse_code('156+112+516+231')
        self.assertTrue(any('1、2、3' in w for w in p.warnings))

    def test_screen_lines(self):
        p = parse_code('156+123+516+231')
        l1, l2 = p.screen_lines()
        self.assertEqual(l1, 'B1 RED1 BLK2 LBL3')
        self.assertEqual(l2, 'B2 BLK2 RED3 LBL1')
        self.assertTrue(all(ord(c) < 128 for c in l1 + l2))


class RoleTests(unittest.TestCase):
    def test_roles(self):
        for stop, role in (('QR', 'QR'), ('RAW', 'RAW'), ('ROUGH', 'ROUGH'), ('TEMP', 'TEMP'),
                           ('START2', 'START'), ('START1', 'START'), ('start', 'START'), ('FOO', None), (None, None)):
            self.assertEqual(role_of(stop), role, stop)

    def test_alias(self):
        al = {'ROUGH': ['PROC', 'ROUGH'], 'TEMP': ['STAGE']}
        self.assertEqual(role_of('PROC1', al), 'ROUGH')
        self.assertEqual(role_of('STAGE', al), 'TEMP')
        self.assertIsNone(role_of('TEMP', al))              # 配了别名就只认别名


class StatsTests(unittest.TestCase):
    def test_counts(self):
        s = Stats()
        s.grab(True); s.grab(False); s.place(True)
        self.assertEqual((s.grab_text(), s.place_text()), ('GRAB 1/2', 'PLACE 1/1'))


if __name__ == '__main__':
    unittest.main(verbosity=1)
