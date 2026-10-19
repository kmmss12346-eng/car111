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


class CapsLockTests(unittest.TestCase):
    """(10-19)车上的扫码器每扫一次先按 CapsLock、打完再按一下，要看到大写灯翻过来才接着打码。
    独占以后系统不翻灯，扫码器就"嘀"一声但不打码：独占着的设备由读码程序替系统翻灯。"""
    P = '/dev/input/event4'

    def make(self, grabbed=True):
        logs, leds = [], []
        r = qr_usb.Reader(log=logs.append, devices=lambda: [], opener=None,
                          led=lambda fd, led: leds.append((fd, led)))
        r._fds[self.P] = (7, 'USBKey Chip USBKey Module')
        r.scanner = self.P
        if grabbed:
            r._grabbed.add(self.P)
        return r, logs, leds

    def scan(self, t=0.0):
        return ev(58, 1, t=t) + ev(58, 0, t=t) + typed('134+123+413+231') + ev(58, 1, t=t) + ev(58, 0, t=t)

    def test_grabbed_scanner_gets_its_caps_led_toggled(self):
        r, _, leds = self.make()
        r.feed(self.P, self.scan())
        self.assertEqual(r.get(), '134+123+413+231')
        self.assertEqual(leds, [(7, 1), (7, 1)])               # 按下才翻(松开不翻)，开头一次、结尾一次
        r.feed(self.P, ev(69, 1) + ev(69, 0))
        self.assertEqual(leds[-1], (7, 0))                     # NumLock -> LED_NUML

    def test_not_grabbed_system_toggles_itself(self):
        r, _, leds = self.make(grabbed=False)
        r.feed(self.P, self.scan())
        self.assertEqual(r.get(), '134+123+413+231')
        self.assertEqual(leds, [])

    def test_grab_success_is_remembered_and_env_can_turn_it_off(self):
        import os
        import fcntl
        from unittest import mock
        for env, want in (('1', True), ('0', False)):
            r = qr_usb.Reader(log=lambda m: None, devices=lambda: [], opener=None)
            r._fds[self.P] = (7, 'USBKey Chip USBKey Module')
            with mock.patch.dict(os.environ, {'QR_USB_GRAB': env}), \
                    mock.patch.object(fcntl, 'ioctl', return_value=0) as io:
                r._grab(self.P)
            self.assertEqual(self.P in r._grabbed, want)
            self.assertEqual(io.called, want)
            self.assertEqual(r.scanner, self.P)                # 不独占也照样当它是扫码器、读它的码

    def test_grab_failure_means_no_led_writes(self):
        r = qr_usb.Reader(log=lambda m: None, devices=lambda: [], opener=None, led=lambda fd, led: 1 / 0)
        r._fds[self.P] = (-1, 'USBKey Chip USBKey Module')
        r._grab(self.P)                                        # fd -1：独占失败
        self.assertNotIn(self.P, r._grabbed)
        r.feed(self.P, self.scan())                            # 不会去翻灯(翻了就会 ZeroDivisionError)
        self.assertEqual(r.get(), '134+123+413+231')

    def test_led_write_error_is_reported_once(self):
        def bad(fd, led):
            raise OSError(9, 'Bad file descriptor')
        r, logs, _ = self.make()
        r._led = bad
        r.feed(self.P, self.scan() + self.scan(t=1.0))
        self.assertEqual(r.get(), '134+123+413+231')
        self.assertEqual(sum('翻大写灯没成功' in m for m in logs), 1)

    def test_only_caps_lock_and_no_code_gives_a_hint(self):
        r, logs, _ = self.make()
        for i in range(3):
            r.feed(self.P, ev(58, 1, t=10.0 * i) + ev(58, 0, t=10.0 * i))
        self.assertFalse(any('CapsLock' in m for m in logs))
        r.feed(self.P, ev(58, 1, t=40.0) + ev(58, 0, t=40.0))
        self.assertTrue(any('4 次 CapsLock' in m for m in logs))
        r2, logs2, _ = self.make()
        for i in range(5):                                     # 正常扫：每次两下 CapsLock，读到码就重新数，不提示
            r2.feed(self.P, self.scan(t=10.0 * i))
        self.assertFalse(any('CapsLock' in m for m in logs2))

    def test_scanner_line_that_is_not_a_code_is_logged(self):
        r, logs, _ = self.make()
        r.feed(self.P, typed('12345'))
        self.assertTrue(any('不是任务码' in m and '12345' in m for m in logs))


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

    def test_child_exit_is_logged_and_restarted(self):
        """读码子进程退出了：日志里说一声并重开(最多 3 次)；自己 stop 的不重开。"""
        import io
        import threading

        class P:
            def __init__(self, lines, rc=1):
                self.stdout, self.rc = io.StringIO(lines), rc

            def wait(self):
                return self.rc

        logs = []
        r = qr_usb.ProcReader.__new__(qr_usb.ProcReader)
        r.log, r._lock, r.code, r.t, r.scanner = logs.append, threading.Lock(), None, 0.0, None
        r._stopping, r._restarts = False, 0
        r._p = P('SCANNER /dev/input/event4\n')
        spawned = []
        r._spawn = lambda: spawned.append(1) or P('CODE 134+123+413+231\n' if len(spawned) == 1 else '')
        r._pump()
        self.assertEqual(r.get(), '134+123+413+231')           # 重开的子进程读到的码照样用
        self.assertEqual(len(spawned), 3)
        self.assertEqual(sum('重新开一个' in m for m in logs), 3)
        self.assertTrue(any('不再重开' in m for m in logs))
        r2 = qr_usb.ProcReader.__new__(qr_usb.ProcReader)
        r2.log, r2._lock, r2.code, r2.t, r2.scanner = logs.append, threading.Lock(), None, 0.0, None
        r2._stopping, r2._restarts, r2._p = True, 0, P('')
        r2._spawn = lambda: 1 / 0
        r2._pump()                                             # stop 了：不重开

    def test_syn_dropped_resets_the_line(self):
        """内核丢了按键(SYN_DROPPED)：这一行作废，不拼出错的码。"""
        import struct
        rd = qr_usb.Reader(log=lambda m: None, devices=lambda: [])
        ev = lambda typ, code, val, t=1.0: struct.pack(qr_usb._FMT, int(t), int((t % 1) * 1e6), typ, code, val)
        rd.feed('/dev/input/event9', ev(1, 2, 1) + ev(0, 3, 0) + ev(1, 28, 1))
        self.assertIsNone(rd.get())


if __name__ == '__main__':
    unittest.main()