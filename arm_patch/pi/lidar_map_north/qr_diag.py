"""扫码器诊断(先退出 run_live 再跑：run_live 的读码进程会独占扫码器，这里就收不到)。
python3 qr_diag.py            列出键盘类设备、权限，然后 20 秒内把扫码器"打"出来的每个键原样打印(带时间)
python3 qr_diag.py 60         读 60 秒
"""
import os, re, select, struct, sys, time, grp, subprocess
FMT = 'llHHi'; SIZE = struct.calcsize(FMT)
NAMES = {2:'1',3:'2',4:'3',5:'4',6:'5',7:'6',8:'7',9:'8',10:'9',11:'0',13:'=',12:'-',15:'TAB',28:'ENTER',96:'KPENTER',
         42:'LSHIFT',54:'RSHIFT',56:'LALT',29:'LCTRL',78:'KP+',27:']',39:';',71:'KP7',72:'KP8',73:'KP9',75:'KP4',76:'KP5',
         77:'KP6',79:'KP1',80:'KP2',81:'KP3',82:'KP0',69:'NUMLOCK',58:'CAPSLOCK'}
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
for p, n in devs:
    try:
        st = os.stat(p)
        fds[os.open(p, os.O_RDONLY | os.O_NONBLOCK)] = (p, n)
    except PermissionError:
        print(f'  ★ 没有权限读 {p}「{n}」：sudo usermod -aG input $USER 然后重启(或者 sudo python3 qr_diag.py)')
    except OSError as ex:
        print(f'  ★ 打不开 {p}: {ex}')
if not fds:
    print('没有能读的键盘类设备。扫码器插在树莓派上却没出现：它可能是 USB 虚拟串口模式(看 ls /dev/ttyACM*)或者根本没接 USB(接的是 STM32 的 UART5)')
    sys.exit(1)
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
print(f'\n现在把任务码对着扫码器，{dur:.0f} 秒内打印每个按键(按下=1 松开=0)：')
t_end = time.time() + dur; line = {}; t0 = {}
while time.time() < t_end:
    r, _, _ = select.select(list(fds), [], [], 0.5)
    for fd in r:
        p, n = fds[fd]
        data = os.read(fd, SIZE * 64)
        for i in range(0, len(data) - SIZE + 1, SIZE):
            s, us, typ, code, val = struct.unpack(FMT, data[i:i + SIZE])
            t = s + us / 1e6
            if typ == 0 and code == 3:
                print(f'  {p} SYN_DROPPED(丢了按键)')
            if typ != 1 or val == 2:
                continue
            if val == 1 and not line.get(p):
                t0[p] = t
            if val == 1:
                line[p] = line.get(p, '') + ' ' + NAMES.get(code, f'#{code}')
            if val == 1 and code in (28, 96, 15):
                print(f'  {p}「{n}」 {t - t0[p]:.3f}s:{line[p]}')
                line[p] = ''
for p, l in line.items():
    if l:
        print(f'  {p} 没有回车结尾的一行:{l}')
