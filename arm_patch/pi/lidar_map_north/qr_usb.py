"""扫码器插在树莓派 USB 口上(GM65 出厂的 USB 键盘模式)时读任务码。

USB 键盘模式下扫码器就像一个键盘：读到码就"打字"打出来，再按一下回车。这里在后台直接读 /dev/input 里的按键，
认出 "ddd+ddd+ddd+ddd" 就记下来；arm_link.ArmLink.qr() 先问这里，没有再问 STM32(QR?)。
- 哪个是扫码器：名字像扫码器的(scan/barcode/GM65…)直接用；认不出名字的键盘，谁"打"出一整条任务码就是它。
  认出来以后独占它(EVIOCGRAB)，码就不会再打进终端里当成命令。
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
_EVIOCGRAB = 0x40044590
_FMT = 'llHHi'                       # struct input_event(32/64 位系统按本机 long 的长度)
_SIZE = struct.calcsize(_FMT)
_SHIFT = (42, 54)
_ENTER = (28, 96)
# 主键盘的数字键 KEY_1..KEY_0，小键盘数字
_KEYS = {2: '1', 3: '2', 4: '3', 5: '4', 6: '5', 7: '6', 8: '7', 9: '8', 10: '9', 11: '0',
         71: '7', 72: '8', 73: '9', 75: '4', 76: '5', 77: '6', 79: '1', 80: '2', 81: '3', 82: '0', 78: '+'}
_SHIFT_KEYS = {13: '+'}              # Shift + '=' = '+'(美式键盘布局)
MAX_LINE_S = 1.0                     # 一整条码最多用多久"打"完才算扫码器(人手打字比这慢得多)
_NAME_HINT = re.compile(r'scan|barcode|bar code|qr|gm65|gm6|symbol|honeywell|newland|zebra|datalogic', re.I)


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
    def __init__(self, log=print, devices=_devices, opener=None):
        self.log = log
        self._devices = devices
        self._open = opener or (lambda p: os.open(p, os.O_RDONLY | os.O_NONBLOCK))
        self._lock = threading.Lock()
        self.code = None                  # 最近读到的任务码
        self.t = 0.0
        self.scanner = None               # 认出来的扫码器(事件文件)
        self._fds = {}                    # 事件文件 -> (fd, 名字)
        self._buf = {}
        self._shift = {}
        self._t0 = {}
        self._warned = set()
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
            if path in self._fds or path in self._warned:
                continue
            try:
                fd = self._open(path)
            except PermissionError:
                self._warned.add(path)
                self.log(f'  (USB 扫码器：没有权限读 {path}「{name}」。终端输入 sudo usermod -aG input $USER，重启树莓派后再试)')
                continue
            except OSError:
                self._warned.add(path)
                continue
            self._fds[path] = (fd, name)
            if _NAME_HINT.search(name):
                self._grab(path)

    def _grab(self, path):
        fd, name = self._fds[path]
        try:
            import fcntl
            fcntl.ioctl(fd, _EVIOCGRAB, 1)            # 独占：码不再打进终端
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
            if typ != _EV_KEY:
                continue
            if code in _SHIFT:
                self._shift[path] = value != 0
                continue
            if value != 1:                            # 只看按下
                continue
            buf = self._buf.setdefault(path, '')
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

    def _line(self, path, line):
        m = _CODE.search(line or '')
        if not m:
            return
        with self._lock:
            self.code = m.group(1)
            self.t = time.monotonic()
        if self.scanner != path and path in self._fds:
            self._grab(path)


_reader = None


def reader(log=print):
    """全程只开一个后台读取线程。QR_USB=0 或者没有 /dev/input 时返回 None。"""
    global _reader
    if os.environ.get('QR_USB', '1') == '0' or not os.path.isdir('/dev/input'):
        return None
    if _reader is None:
        _reader = Reader(log=log)
        _reader.start()
    return _reader
