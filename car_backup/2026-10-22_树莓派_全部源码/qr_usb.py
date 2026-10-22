"""扫码器插在树莓派 USB 口上(GM65 出厂的 USB 键盘模式)时读任务码。

USB 键盘模式下扫码器就像一个键盘：读到码就"打字"打出来，再按一下回车。这里在后台直接读 /dev/input 里的按键，
认出 "ddd+ddd+ddd+ddd" 就记下来；arm_link.ArmLink.qr() 先问这里，没有再问 STM32(QR?)。
- 哪个是扫码器：名字像扫码器的(scan/barcode/GM65…)直接用；认不出名字的键盘，谁"打"出一整条任务码就是它。
  认出来以后独占它(EVIOCGRAB)，码就不会再打进终端里当成命令。
- (10-19)车上这个扫码器每扫一次先按一下 CapsLock、打完码再按一下，要看到电脑把大写灯翻过来才接着打码。
  独占以后系统收不到这个键、不会去翻灯，扫码器就卡住(会"嘀"，但一个数字都不打)：所以独占着的时候由这里替系统翻灯。
  QR_USB_GRAB=0：不独占(码会打进终端，灯由系统自己翻)。
- 只用标准库，不用装 evdev。读 /dev/input 要在 input 组里(树莓派默认就在)；没有权限时打印怎么加。
- QR_USB=0 环境变量可以关掉(电脑上跑测试时也不会去碰键盘)。
"""
import os
import re
import select
import struct
import threading
import time

_CODE = re.compile(r'(\d{3}\+\d{3}\+\d{3}\+\d{3})')
_EV_KEY = 1
_EV_LED = 0x11
_EVIOCGRAB = 0x40044590
_EVIOCGLED = 0x80084519              # EVIOCGLED(8)：读设备现在的指示灯状态
_LOCKS = {58: 1, 69: 0}              # CapsLock、NumLock 键 -> LED_CAPSL、LED_NUML
_FMT = 'llHHi'                       # struct input_event(32/64 位系统按本机 long 的长度)
_SIZE = struct.calcsize(_FMT)
_SHIFT = (42, 54)
_ENTER = (28, 96, 15)                # 回车、小键盘回车、Tab(扫码器结尾设成 Tab 时)
# 主键盘的数字键 KEY_1..KEY_0，小键盘数字
_KEYS = {2: '1', 3: '2', 4: '3', 5: '4', 6: '5', 7: '6', 8: '7', 9: '8', 10: '9', 11: '0',
         71: '7', 72: '8', 73: '9', 75: '4', 76: '5', 77: '6', 79: '1', 80: '2', 81: '3', 82: '0', 78: '+'}
_SHIFT_KEYS = {13: '+'}              # Shift + '=' = '+'(美式键盘布局)
MAX_LINE_S = 2.5                     # 一整条码最多用多久"打"完才算扫码器(人手打字比这慢得多)
_NAME_HINT = re.compile(r'scan|barcode|bar code|qr|gm65|gm6|usbkey|symbol|honeywell|newland|zebra|datalogic', re.I)   # usbkey：10-11 车上这个扫码器叫 "USBKey Chip USBKey Module"


def _open_dev(path):
    """读写打开(独占以后要往扫码器写大写灯)；没有写权限就只读。"""
    try:
        return os.open(path, os.O_RDWR | os.O_NONBLOCK)
    except PermissionError:
        return os.open(path, os.O_RDONLY | os.O_NONBLOCK)


def _toggle_led(fd, led):
    """把设备的一个指示灯翻过来(和系统收到 CapsLock 键时做的一样)。返回翻过以后亮不亮。"""
    import fcntl
    st = bytearray(8)
    fcntl.ioctl(fd, _EVIOCGLED, st, True)
    on = not (st[led // 8] >> (led % 8)) & 1
    os.write(fd, struct.pack(_FMT, 0, 0, _EV_LED, led, int(on)) + struct.pack(_FMT, 0, 0, 0, 0, 0))
    return on


def _devices():
    """[(事件文件, 名字)]：/proc/bus/input/devices 里带键盘功能(kbd)的设备。"""
    out = []
    try:
        text = open('/proc/bus/input/devices', encoding='utf-8', errors='replace').read()
    except OSError:
        return out
    for block in text.split('\n\n'):
        m = re.search(r'N: Name="([^"]*)"', block)
        h = re.search(r'H: Handlers=([^\n]*)', block)
        if not m or not h or 'kbd' not in h.group(1):
            continue
        ev = re.search(r'\b(event\d+)\b', h.group(1))
        if ev:
            out.append(('/dev/input/' + ev.group(1), m.group(1)))
    return out


class Reader:
    def __init__(self, log=print, devices=_devices, opener=None, led=None):
        self.log = log
        self._devices = devices
        self._open = opener or _open_dev
        self._led = led or _toggle_led
        self._lock = threading.Lock()
        self.code = None                  # 最近读到的任务码
        self.t = 0.0
        self.scanner = None               # 认出来的扫码器(事件文件)
        self._fds = {}                    # 事件文件 -> (fd, 名字)
        self._buf = {}
        self._shift = {}
        self._t0 = {}
        self._warned = {}                 # 打不开的设备 -> 上次试的时间(插上的一瞬间 udev 还没改权限：过几秒再试)
        self._last = {}                   # 每个设备上一个按键的时间
        self._grabbed = set()             # 独占成功的设备(这些设备的大写灯由这里翻)
        self._locks = {}                  # 每个设备上次读到码以后按了几次 CapsLock/NumLock
        self._said = {}                   # 提示上次打印的时间(同一种提示不刷屏)
        self._stop = False
        self._th = None

    # ---------------------------------------------------------------- 给 ArmLink 用
    def get(self):
        with self._lock:
            return self.code

    def clear(self):
        with self._lock:
            self.code = None

    def start(self):
        if self._th is None:
            self._th = threading.Thread(target=self._run, daemon=True)
            self._th.start()

    # ---------------------------------------------------------------- 后台
    def _refresh(self):
        for path, name in self._devices():
            if path in self._fds or time.monotonic() - self._warned.get(path, -1e9) < 10.0:
                continue
            try:
                fd = self._open(path)
            except PermissionError:
                if path not in self._warned:
                    self.log(f'  (USB 扫码器：没有权限读 {path}「{name}」。终端输入 sudo usermod -aG input $USER，重启树莓派后再试)')
                self._warned[path] = time.monotonic()
                continue
            except OSError:
                self._warned[path] = time.monotonic()
                continue
            self._warned.pop(path, None)
            self._fds[path] = (fd, name)
            if _NAME_HINT.search(name):
                self._grab(path)

    def _grab(self, path):
        fd, name = self._fds[path]
        if os.environ.get('QR_USB_GRAB', '1') != '0':
            try:
                import fcntl
                fcntl.ioctl(fd, _EVIOCGRAB, 1)        # 独占：码不再打进终端
                self._grabbed.add(path)
            except Exception:
                pass
        if self.scanner != path:
            self.scanner = path
            self.log(f'  USB 扫码器：用「{name}」({path})读任务码')

    def _run(self):
        last = 0.0
        while not self._stop:
            now = time.monotonic()
            if now - last > 2.0:                      # 插拔：每 2 秒看一次有没有新设备
                last = now
                self._refresh()
            self._flush_idle()
            fds = {v[0]: k for k, v in self._fds.items()}
            if not fds:
                time.sleep(0.5)
                continue
            try:
                ready, _, _ = select.select(list(fds), [], [], 0.5)
            except (OSError, ValueError):
                ready = []
            for fd in ready:
                path = fds[fd]
                try:
                    data = os.read(fd, _SIZE * 64)
                except BlockingIOError:
                    continue
                except OSError:                      # 拔掉了
                    self._fds.pop(path, None)
                    self._grabbed.discard(path)
                    try:
                        os.close(fd)
                    except OSError:
                        pass
                    continue
                self.feed(path, data)

    def feed(self, path, data):
        """处理一段原始事件(测试也直接调它)。"""
        for i in range(0, len(data) - _SIZE + 1, _SIZE):
            sec, usec, typ, code, value = struct.unpack(_FMT, data[i:i + _SIZE])
            t = sec + usec / 1e6
            if typ == 0 and code == 3:                # SYN_DROPPED：内核缓冲满了丢了按键，这一行作废
                self._buf[path] = ''
                self._say(('drop', path), '  (扫码器的按键丢了一些(SYN_DROPPED)：这一次的码作废，再扫一次)')
                continue
            if typ != _EV_KEY:
                continue
            if code in _SHIFT:
                self._shift[path] = value != 0
                continue
            if value != 1:                            # 只看按下
                continue
            if code in _LOCKS:
                self._lock_key(path, code)
            buf = self._buf.setdefault(path, '')
            if buf and t - self._last.get(path, t) > 0.5:
                buf = ''                              # 上一行没收到回车就断了(半截)：扔掉，不然这一行的计时从半截那时算
            self._last[path] = t
            if not buf:
                self._t0[path] = t                     # 这一行第一个键的时间
            if code in _ENTER:
                # 扫码器一整条码不到 1 秒就"打"完；人手打 15 个字要好几秒(比如在终端里输入 mcode …)：不算，也不独占那个键盘
                if t - self._t0.get(path, t) <= MAX_LINE_S:
                    self._line(path, buf)
                buf = ''
            else:
                ch = _SHIFT_KEYS.get(code) if self._shift.get(path) else None
                ch = ch or _KEYS.get(code)
                if ch:
                    buf = (buf + ch)[-40:]
                elif code not in _SHIFT_KEYS:
                    buf += '?'                        # 别的键：照样占一位，免得拼出假的码
            self._buf[path] = buf

    def _lock_key(self, path, code):
        """按了 CapsLock/NumLock：独占着的设备替系统把灯翻过来(见开头说明)；没独占的系统自己会翻，不管。"""
        n = self._locks.get(path, 0) + 1
        self._locks[path] = n
        if n >= 4 and path == self.scanner:
            self._say(('locks', path), f'  (扫码器按了 {n} 次 CapsLock 还没打出任务码。'
                                       '可以试试不独占扫码器：q 退出后 QR_USB_GRAB=0 bash run_live.sh)')
        if path in self._grabbed and path in self._fds:
            try:
                self._led(self._fds[path][0], _LOCKS[code])
            except OSError as ex:
                self._say(('led', path), f'  (替扫码器翻大写灯没成功：{ex!r}。扫码器可能卡住不打码)')

    def _say(self, key, msg, every=30.0):
        now = time.monotonic()
        if now - self._said.get(key, -1e9) >= every:
            self._said[key] = now
            self.log(msg)

    def _flush_idle(self, now=None):
        """结尾不发回车的扫码器：一行里已经有完整的任务码、0.2 秒没有新键，就当这一行打完了。"""
        now = time.time() if now is None else now
        for path, buf in list(self._buf.items()):
            if buf and _CODE.search(buf) and now - self._last.get(path, now) > 0.2 \
                    and self._last.get(path, now) - self._t0.get(path, now) <= MAX_LINE_S:
                self._buf[path] = ''
                self._line(path, buf)

    def _line(self, path, line):
        m = _CODE.search(line or '')
        if not m:
            if line and path == self.scanner:
                self._say(('junk', path), f'  (扫码器打出来的一行不是任务码：{line})', every=5.0)
            return
        self._locks[path] = 0
        with self._lock:
            new = m.group(1) != self.code
            self.code = m.group(1)
            self.t = time.monotonic()
        if new:
            self.log(f'  扫码器读到任务码：{m.group(1)}')
        if self.scanner != path and path in self._fds:
            self._grab(path)


class ProcReader:
    """(10-11)在单独的进程里读扫码器，和 Reader 一样用(get / clear / code / t / scanner)。
    run_live 里雷达、摄像头、画图很忙，后台线程抢不到 Python 的 GIL，扫码器"打"一条码的一百来个按键事件
    内核只缓冲 64 个，读晚了就丢键、拼不出码。单独的进程不受影响。
    (10-19：车上"单独测能读到、run_live 里读不到"，更可能是独占以后大写灯不翻、扫码器卡住，见开头说明)"""

    def __init__(self, log=print):
        self.log = log
        self._lock = threading.Lock()
        self.code = None
        self.t = 0.0
        self.scanner = None
        self._stopping = False
        self._restarts = 0
        self._p = self._spawn()
        threading.Thread(target=self._pump, daemon=True).start()
        import atexit
        atexit.register(self.stop)

    def _spawn(self):
        import subprocess
        import sys
        return subprocess.Popen([sys.executable, '-u', os.path.abspath(__file__), '--serve'],
                                stdout=subprocess.PIPE, stdin=subprocess.DEVNULL, text=True, bufsize=1)

    def _pump(self):
        while True:
            p = self._p
            for line in p.stdout:
                self.handle(line)
            rc = p.wait()
            if self._stopping:
                return
            if self._restarts >= 3:
                self.log(f'  ★ 扫码器读取进程又退出了(返回码 {rc})：不再重开，USB 扫码器读不到码了(STM32 的 QR? 照样问)')
                return
            self._restarts += 1
            self.log(f'  ★ 扫码器读取进程退出了(返回码 {rc})：重新开一个')
            try:
                self._p = self._spawn()
            except Exception as ex:
                self.log(f'  ★ 扫码器读取进程开不起来：{ex!r}')
                return

    def handle(self, line):
        line = line.rstrip('\n')
        if line.startswith('CODE '):
            with self._lock:
                self.code = line[5:].strip()
                self.t = time.monotonic()
        elif line.startswith('SCANNER '):
            self.scanner = line[8:].strip()
        elif line.startswith('LOG '):
            self.log(line[4:])

    def get(self):
        with self._lock:
            return self.code

    def clear(self):
        with self._lock:
            self.code = None

    def start(self):
        pass

    def stop(self):
        self._stopping = True
        try:
            self._p.terminate()
        except Exception:
            pass


def _serve():
    """子进程：读扫码器，每读到一条码就往 stdout 写一行 CODE …(同一个码反复读到也每次都写)。"""
    r = Reader(log=lambda m: print('LOG ' + m, flush=True))
    line0 = r._line
    grab0 = r._grab

    def line(path, text):
        line0(path, text)
        m = _CODE.search(text or '')
        if m:
            print('CODE ' + m.group(1), flush=True)

    def grab(path):
        grab0(path)
        print('SCANNER ' + path, flush=True)

    r._line = line
    r._grab = grab
    try:
        r._run()
    except KeyboardInterrupt:
        pass


_reader = None


def reader(log=print):
    """全程只开一个读扫码器的后台(默认单独的进程；QR_USB_PROC=0 用以前的线程)。QR_USB=0 或者没有 /dev/input 时返回 None。"""
    global _reader
    if os.environ.get('QR_USB', '1') == '0' or not os.path.isdir('/dev/input'):
        return None
    if _reader is None:
        if os.environ.get('QR_USB_PROC', '1') != '0':
            try:
                _reader = ProcReader(log=log)
                return _reader
            except Exception as ex:
                log(f'  (扫码器读取进程开不起来：{ex!r}，改用线程)')
        _reader = Reader(log=log)
        _reader.start()
    return _reader


if __name__ == '__main__':
    import sys as _sys
    if '--serve' in _sys.argv:
        _serve()
