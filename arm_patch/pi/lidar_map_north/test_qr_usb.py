"""qr_usb.py：扫码器插在树莓派 USB 口上(键盘模式)时读任务码。python3 -m unittest test_qr_usb"""
import struct
import unittest

import qr_usb
from arm_link import ArmLink

KEY = {'1': 2, '2': 3, '3': 4, '4': 5, '5': 6, '6': 7, '7': 8, '8': 9, '9': 10, '0': 11}


def ev(code, value, typ=1, t=0.0):
    return struct.pack(qr_usb._FMT, int(t), int((t % 1) * 1e6), typ, code, value)


def typed(text, plus='shift'):
    """按扫码器的方式"打字"：每个键按下+松开，最后回车。"""
    out = b''
    for ch in text:
        if ch == '+':
            if plus == 'shift':
                out += ev(42, 1) + ev(13, 1) + ev(13, 0) + ev(42, 0)
            else:
                out += ev(78, 1) + ev(78, 0)              # 小键盘 +
        else:
            out += ev(KEY[ch], 1) + ev(KEY[ch], 0) + ev(0, 0, typ=0)
    return out + ev(28, 1) + ev(28, 0)


class FakeLink:
    def __init__(self, code=None):
        self.code = code
        self.sent = []

    def request(self, text, timeout=None, collect=True):
        self.sent.append(text)
        if text == 'QR?':
            return True, 'DONE', [f'QR {self.code}' if self.code else 'QR NONE']
        return True, 'DONE', []


class ReaderTests(unittest.TestCase):
    def make(self):
        logs = []
        r = qr_usb.Reader(log=logs.append, devices=lambda: [], opener=None)
        r._fds['/dev/input/event3'] = (-1, 'HID 0000:0001')
        return r, logs

    def test_shift_plus_and_keypad_plus(self):
        for plus in ('shift', 'keypad'):
            r, _ = self.make()
            r.feed('/dev/input/event3', typed('156+123+516+231', plus))
            self.assertEqual(r.get(), '156+123+516+231')

    def test_events_split_across_reads(self):
        r, _ = self.make()
        data = typed('452+321+254+312')
        for i in range(0, len(data), qr_usb._SIZE * 3):
            r.feed('/dev/input/event3', data[i:i + qr_usb._SIZE * 3])
        self.assertEqual(r.get(), '452+321+254+312')

    def test_not_a_code_and_clear(self):
        r, _ = self.make()
        r.feed('/dev/input/event3', typed('12345'))
        self.assertIsNone(r.get())
        r.feed('/dev/input/event3', typed('156+123+516+231'))
        r.clear()
        self.assertIsNone(r.get())

    def test_unknown_name_becomes_scanner_after_a_code(self):
        r, logs = self.make()
        self.assertIsNone(r.scanner)
        r.feed('/dev/input/event3', typed('156+123+516+231'))
        self.assertEqual(r.scanner, '/dev/input/event3')
        self.assertTrue(any('USB 扫码器' in m for m in logs))

    def timed(self, text, t0=0.0, dt=0.01, end=28):
        """带时间的扫码：每个键隔 dt 秒；end = 结尾键(28 回车、15 Tab、None 不发)。返回 (数据, 最后一个键的时间)。"""
        out, t = b'', t0
        for ch in text:
            if ch == '+':
                out += ev(42, 1, t=t) + ev(13, 1, t=t) + ev(13, 0, t=t) + ev(42, 0, t=t)
            else:
                out += ev(KEY[ch], 1, t=t) + ev(KEY[ch], 0, t=t)
            t += dt
        if end is not None:
            out += ev(end, 1, t=t) + ev(end, 0, t=t)
        return out, t

    def test_slow_scanner_tab_end_and_stale_half_line(self):
        """10-12：慢速键盘模式(一条 1.4 秒)、结尾是 Tab、上一次留下半截没回车：都要读得到。"""
        r, _ = self.make()
        r.feed('/dev/input/event3', self.timed('156+123+516+231', dt=0.09)[0])
        self.assertEqual(r.get(), '156+123+516+231')
        r, _ = self.make()
        r.feed('/dev/input/event3', self.timed('156+123+516+231', end=15)[0])
        self.assertEqual(r.get(), '156+123+516+231')
        r, _ = self.make()
        r.feed('/dev/input/event3', self.timed('156+1', end=None)[0])           # 半截，没回车
        r.feed('/dev/input/event3', self.timed('452+321+254+312', t0=30.0)[0])
        self.assertEqual(r.get(), '452+321+254+312')

    def test_no_enter_flushed_when_idle(self):
        r, _ = self.make()
        data, t = self.timed('156+123+516+231', t0=100.0, end=None)
        r.feed('/dev/input/event3', data)
        self.assertIsNone(r.get())
        r._flush_idle(now=t + 0.5)
        self.assertEqual(r.get(), '156+123+516+231')

    def test_human_typing_is_not_the_scanner(self):
        # 在终端里手打 mcode 156+123+516+231：打得慢，不当成扫码器，不独占键盘
        r, _ = self.make()
        data = b''.join(ev(KEY[c], 1, t=0.4 * i) + ev(KEY[c], 0, t=0.4 * i) if c != '+' else
                        ev(42, 1, t=0.4 * i) + ev(13, 1, t=0.4 * i) + ev(13, 0, t=0.4 * i) + ev(42, 0, t=0.4 * i)
                        for i, c in enumerate('156+123+516+231'))
        r.feed('/dev/input/event3', data + ev(28, 1, t=6.5))
        self.assertIsNone(r.get())
        self.assertIsNone(r.scanner)

    def test_name_hint_grabs_at_once(self):
        logs = []
        r = qr_usb.Reader(log=logs.append, devices=lambda: [('/dev/input/event5', 'GM65 Barcode Scanner'),
                                                            ('/dev/input/event1', 'Logitech Keyboard')],
                          opener=lambda p: -1)
        r._refresh()
        self.assertEqual(r.scanner, '/dev/input/event5')

    def test_permission_hint(self):
        logs = []

        def deny(p):
            raise PermissionError(p)
        r = qr_usb.Reader(log=logs.append, devices=lambda: [('/dev/input/event5', 'X')], opener=deny)
        r._refresh()
        r._refresh()
        self.assertEqual(sum('usermod -aG input' in m for m in logs), 1)


class ArmLinkTests(unittest.TestCase):
    def test_usb_code_first_then_stm32(self):
        a = ArmLink(FakeLink('111+222+333+444'), log=lambda m: None)
        r, _ = ReaderTests().make()
        a.usb_qr = r
        self.assertEqual(a.qr(), '111+222+333+444')          # USB 没码：问 STM32
        r.feed('/dev/input/event3', typed('156+123+516+231'))
        self.assertEqual(a.qr(), '156+123+516+231')          # USB 有码：用它
        a.qr_clear()
        self.assertIsNone(r.get())                            # QR CLR 两边都清
        self.assertIn('QR CLR', a.link.sent)

    def test_no_reader_is_old_behaviour(self):
        a = ArmLink(FakeLink(None), log=lambda m: None)
        a.usb_qr = None
        self.assertIsNone(a.qr())



class ProcReaderProtocolTests(unittest.TestCase):
    def test_parent_parses_child_lines(self):
        """(10-11)子进程写 CODE / SCANNER / LOG 行，父进程记下码、扫码器、转发日志；clear 以后同一个码再读到也算。"""
        logs = []
        r = qr_usb.ProcReader.__new__(qr_usb.ProcReader)
        r.log, r._lock, r.code, r.t, r.scanner = logs.append, __import__('threading').Lock(), None, 0.0, None
        r.handle('LOG   扫码器读到任务码：142+321+142+213\n')
        r.handle('SCANNER /dev/input/event4\n')
        r.handle('CODE 142+321+142+213\n')
        self.assertEqual(r.get(), '142+321+142+213')
        self.assertEqual(r.scanner, '/dev/input/event4')
        self.assertEqual(logs, ['  扫码器读到任务码：142+321+142+213'])
        r.clear()
        self.assertIsNone(r.get())
        r.handle('CODE 142+321+142+213\n')
        self.assertEqual(r.get(), '142+321+142+213')

    def test_syn_dropped_resets_the_line(self):
        """内核丢了按键(SYN_DROPPED)：这一行作废，不拼出错的码。"""
        import struct
        rd = qr_usb.Reader(log=lambda m: None, devices=lambda: [])
        ev = lambda typ, code, val, t=1.0: struct.pack(qr_usb._FMT, int(t), int((t % 1) * 1e6), typ, code, val)
        rd.feed('/dev/input/event9', ev(1, 2, 1) + ev(0, 3, 0) + ev(1, 28, 1))
        self.assertIsNone(rd.get())


if __name__ == '__main__':
    unittest.main()