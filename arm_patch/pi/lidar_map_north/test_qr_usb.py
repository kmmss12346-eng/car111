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


if __name__ == '__main__':
    unittest.main()
