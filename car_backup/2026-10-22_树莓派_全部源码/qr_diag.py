"""扫码器诊断(先退出 run_live 再跑：run_live 的读码进程会独占扫码器，这里就收不到)。
python3 qr_diag.py               列出键盘类设备、权限，然后 20 秒内把扫码器"打"出来的每个键原样打印
python3 qr_diag.py 60            读 60 秒
python3 qr_diag.py 30 grab       (10-19)像 run_live 那样独占扫码器读(独占着的时候替系统翻大写灯，和新的 run_live 一样)
python3 qr_diag.py 30 grab noled 独占但不翻灯(和 10-19 以前的 run_live 一样)：车上这个扫码器这样就不打码
键后面的 (+25ms) 是离上一个键隔了多久(超过 5ms 才写)。
"""
import os, re, select, struct, sys, time, subprocess
import qr_usb
FMT = qr_usb._FMT; SIZE = qr_usb._SIZE
NAMES = {2:'1',3:'2',4:'3',5:'4',6:'5',7:'6',8:'7',9:'8',10:'9',11:'0',13:'=',12:'-',15:'TAB',28:'ENTER',96:'KPENTER',
         42:'LSHIFT',54:'RSHIFT',56:'LALT',29:'LCTRL',78:'KP+',27:']',39:';',71:'KP7',72:'KP8',73:'KP9',75:'KP4',76:'KP5',
         77:'KP6',79:'KP1',80:'KP2',81:'KP3',82:'KP0',69:'NUMLOCK',58:'CAPSLOCK'}
args = [a.lower() for a in sys.argv[1:]]
dur = next((float(a) for a in args if re.fullmatch(r'\d+(\.\d+)?', a)), 20.0)
grab, led = 'grab' in args, 'noled' not in args
print('用户:', os.environ.get('USER'), ' 组:', subprocess.run(['id'], capture_output=True, text=True).stdout.strip())
ps = subprocess.run(['pgrep', '-af', 'qr_usb.py'], capture_output=True, text=True).stdout.strip()
print('正在跑的读码进程:', ps or '没有')
if ps:
    print('  ★ 有读码进程在跑(可能是 run_live 的，或者上次没退干净的)：它独占扫码器，这里读不到。先退出 run_live / kill 掉再试')
text = open('/proc/bus/input/devices', encoding='utf-8', errors='replace').read()
devs = []
for b in text.split('\n\n'):
    n = re.search(r'N: Name="([^"]*)"', b); h = re.search(r'H: Handlers=([^\n]*)', b)
    if not n or not h:
        continue
    ev = re.search(r'\b(event\d+)\b', h.group(1))
    kbd = 'kbd' in h.group(1)
    p = '/dev/input/' + ev.group(1) if ev else None
    print(f'  {"键盘" if kbd else "    "} {p}  「{n.group(1)}」')
    if kbd and p:
        devs.append((p, n.group(1)))
fds = {}
grabbed = set()
for p, n in devs:
    try:
        fd = qr_usb._open_dev(p)
        fds[fd] = (p, n)
    except PermissionError:
        print(f'  ★ 没有权限读 {p}「{n}」：sudo usermod -aG input $USER 然后重启(或者 sudo python3 qr_diag.py)')
        continue
    except OSError as ex:
        print(f'  ★ 打不开 {p}: {ex}')
        continue
    if grab and qr_usb._NAME_HINT.search(n):
        try:
            import fcntl
            fcntl.ioctl(fd, qr_usb._EVIOCGRAB, 1)
            grabbed.add(fd)
            print(f'  独占了 {p}「{n}」' + ('，它按 CapsLock/NumLock 时这里替系统翻灯' if led else '，不替它翻灯(noled)'))
        except OSError as ex:
            print(f'  ★ 独占 {p} 没成功：{ex}')
    if qr_usb._NAME_HINT.search(n):
        try:
            import fcntl
            st = bytearray(8)
            fcntl.ioctl(fd, qr_usb._EVIOCGLED, st, True)
            print(f'  {p} 现在的指示灯：大写(CapsLock) {"亮" if st[0] & 2 else "灭"}，数字(NumLock) {"亮" if st[0] & 1 else "灭"}')
        except OSError as ex:
            print(f'  ({p} 读不到指示灯状态：{ex})')
if not fds:
    print('没有能读的键盘类设备。扫码器插在树莓派上却没出现：它可能是 USB 虚拟串口模式(看 ls /dev/ttyACM*)或者根本没接 USB(接的是 STM32 的 UART5)')
    sys.exit(1)
if grab and not grabbed:
    print('  ★ grab：没有名字像扫码器的设备可以独占(只会像普通键盘那样读)')
print(f'\n现在把任务码对着扫码器，{dur:.0f} 秒内打印每个按键：')
t_end = time.time() + dur; line = {}; txt = {}; shift = {}; t0 = {}; tl = {}; got = 0
while time.time() < t_end:
    r, _, _ = select.select(list(fds), [], [], 0.2)
    now = time.time()
    for fd in r:
        p, n = fds[fd]
        try:
            data = os.read(fd, SIZE * 64)
        except OSError as ex:
            print(f'  ★ {p} 读不了了({ex})：拔掉了？'); fds.pop(fd); break
        for i in range(0, len(data) - SIZE + 1, SIZE):
            s, us, typ, code, val = struct.unpack(FMT, data[i:i + SIZE])
            t = s + us / 1e6
            if typ == 0 and code == 3:
                print(f'  {p} SYN_DROPPED(丢了按键)')
            if typ != 1:
                continue
            if code in qr_usb._SHIFT:
                shift[fd] = val != 0
            if val != 1:
                continue
            if not line.get(fd):
                t0[fd] = t; txt[fd] = ''
            gap = (t - tl[fd]) * 1000 if line.get(fd) else 0.0
            tl[fd] = t
            word = NAMES.get(code, f'#{code}') + (f'(+{gap:.0f}ms)' if gap > 5 else '')
            if code in qr_usb._LOCKS and fd in grabbed and led:
                try:
                    word += '(灯亮)' if qr_usb._toggle_led(fd, qr_usb._LOCKS[code]) else '(灯灭)'
                except OSError as ex:
                    word += f'(翻灯失败 {ex})'
            line[fd] = line.get(fd, '') + ' ' + word
            ch = qr_usb._SHIFT_KEYS.get(code) if shift.get(fd) else None
            txt[fd] = txt.get(fd, '') + (ch or qr_usb._KEYS.get(code, '?' if code not in qr_usb._SHIFT else ''))
            if code in qr_usb._ENTER:
                m = qr_usb._CODE.search(txt[fd])
                got += bool(m)
                print(f'  {p}「{n}」 {t - t0[fd]:.3f}s:{line[fd]}' + (f'   → 任务码 {m.group(1)}' if m else '   → 拼不出任务码'))
                line[fd] = ''
    for fd, l in list(line.items()):
        if l and now - tl.get(fd, now) > 1.0:
            print(f'  {fds.get(fd, ("?",))[0]} 停了 1 秒还没回车:{l}')
            line[fd] = ''
for fd, l in line.items():
    if l:
        print(f'  {fds.get(fd, ("?",))[0]} 没有回车结尾的一行:{l}')
print(f'\n{dur:.0f} 秒里读到完整任务码 {got} 次。' + ('(独占' + ('+替它翻灯' if led else '、不翻灯') + ')' if grabbed else '(没独占)'))
