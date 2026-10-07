"""扫二维码 + 夹爪 测试。在树莓派终端运行(先退出 map_merge_live)：

    python3 qr_claw_test.py               夹爪先张开 → 等扫到二维码 → 倒数 3 秒 → 夹紧
    python3 qr_claw_test.py --close 2800  夹紧时用 2800 微秒(默认 2910 最紧；数越小越松，太紧舵机发烫就调小)
    python3 qr_claw_test.py --noqr        不等二维码，直接测夹爪张开/夹紧
    python3 qr_claw_test.py --port /dev/ttyUSB0

需要 STM32 烧的是正式程序(v4，带机械臂 arm.c)，不是 uart4_test 测试程序。
"""
import argparse
import re
import sys
import time


class Link:
    def __init__(self, port):
        import serial
        self.err = serial.SerialException
        self.s = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(0.2)
        self.s.reset_input_buffer()

    def _read(self):
        try:
            return self.s.read(256)
        except self.err:
            time.sleep(0.05)
            return b''

    def request(self, text, timeout=5.0):
        """发一行，等 DONE/ERR/PONG；返回 (回复, 中间的信息行)。"""
        self.s.reset_input_buffer()
        self.s.write((text + '\n').encode('ascii'))
        end = time.time() + timeout
        buf = b''
        info = []
        while time.time() < end:
            buf += self._read()
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                t = line.decode('ascii', 'ignore').strip()
                if not t or t.startswith('YAW100'):
                    continue
                if t.startswith(('DONE', 'ERR', 'PONG')):
                    return t, info
                info.append(t)
        return None, info


def must(link, cmd, what):
    rep, _ = link.request(cmd, 5.0)
    print(f'  {cmd} -> {rep}')
    if rep is None or not rep.startswith('DONE'):
        if rep and rep.startswith('ERR'):
            print(f'  {what}失败。STM32 回 {rep}：如果是 ERR CMD，说明 STM32 里不是带机械臂的正式程序(或者 ARM_MODULES_ON=0)，先烧 v4 正式程序。')
        else:
            print(f'  {what}失败：STM32 没回复。')
        sys.exit(1)


def wait_qr(link, timeout):
    """每 0.3 秒问一次 STM32 有没有读到二维码。返回任务码或 None。"""
    t0 = time.time()
    last_dot = 0
    while time.time() - t0 < timeout:
        rep, info = link.request('QR?', 2.0)
        for line in info:
            m = re.match(r'QR (\d{3}\+\d{3}\+\d{3}\+\d{3})', line)
            if m:
                return m.group(1)
        if time.time() - last_dot > 3:
            print(f'  还没扫到……(已等 {time.time() - t0:.0f} 秒)')
            last_dot = time.time()
        time.sleep(0.3)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/serial0')
    ap.add_argument('--close', type=int, default=None, help='夹紧的脉宽(微秒，1700~2910，越大越紧)，不给就用 CLAW C(2910)')
    ap.add_argument('--noqr', action='store_true')
    ap.add_argument('--wait', type=float, default=60.0, help='最多等二维码多少秒')
    a = ap.parse_args()

    try:
        link = Link(a.port)
    except Exception as ex:
        print(f'打不开串口 {a.port}：{ex}\n(map_merge_live 还开着吗？先 q 退出)')
        return 1

    print('① 树莓派 ↔ STM32 通信')
    rep, _ = link.request('PING', 2.0)
    if rep != 'PONG':
        print(f'  没回 PONG(收到 {rep})：先查树莓派和 STM32 的连线/串口号。')
        return 1
    print('  正常')

    print('② 夹爪张开')
    must(link, 'CLAW O', '张开夹爪')
    print('  把要夹的东西放到两个爪子中间(先别放也行)。')

    if not a.noqr:
        print(f'③ 等二维码：把二维码对着扫码模块(最多等 {a.wait:.0f} 秒，Ctrl+C 退出)')
        link.request('QR CLR', 2.0)                 # 清掉以前读到的码，要重新扫一次
        code = wait_qr(link, a.wait)
        if code is None:
            print('  没扫到。检查：扫码模块有没有亮灯/提示音；UART5 接线(PC12→模块RX，PD2←模块TX)；'
                  '模块是不是 9600 波特率、连续扫描模式；码是不是 ddd+ddd+ddd+ddd 格式。')
            print('  只测夹爪可以加 --noqr')
            return 1
        print(f'  ★ 扫到任务码：{code}   (第一批 {code[:7]}，第二批 {code[8:]})，串口屏 t0/t7 应该也显示了')
    else:
        print('③ 跳过二维码(--noqr)')

    print('④ 3 秒后夹紧，手拿开！')
    for i in (3, 2, 1):
        print(f'  {i}')
        time.sleep(1)
    must(link, f'CLAW {a.close}' if a.close else 'CLAW C', '夹紧')
    print('  夹住了吗？数越大夹得越紧(2090 全开，2910 是程序允许的最紧)。')
    print('  太紧、舵机嗡嗡响发烫 → 用小一点的数，如 --close 2800；2910 还夹不住 → 要改 main.c 里的 CLAW_CLOSE_US 和 CLAW_MAX_US。')

    try:
        input('⑤ 按回车松开夹爪(Ctrl+C 不松开直接退出)……')
    except (KeyboardInterrupt, EOFError):
        print()
        return 0
    must(link, 'CLAW O', '张开夹爪')
    print('完成。')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\n已退出')
