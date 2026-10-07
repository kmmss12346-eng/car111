"""测试 STM32 的 PC10 / PC11(UART4，接 5 个电机驱动器的那路串口)。在树莓派终端运行：

    python3 uart4_test.py              正常测试(车要架空，轮子离地！)
    python3 uart4_test.py loop         短接测试：拔掉驱动器的串口线，用杜邦线把 PC10 和 PC11 直接连起来
    python3 uart4_test.py --port /dev/ttyUSB0    STM32 不在 /dev/serial0 时指定串口

树莓派碰不到 STM32 的引脚，所以是让 STM32 替我们做：
  PC10 = UART4 TX：STM32 → 驱动器。好不好看"发了转动指令，轮子转没转"
  PC11 = UART4 RX：驱动器 → STM32。好不好看"MOT? 能不能读到驱动器的回复"
需要 STM32 烧的是 v4 程序(有 MOT? 指令)。运行前先退出 map_merge_live(两个程序不能同时占串口)。
"""
import argparse
import re
import sys
import time

NAMES = {1: '1号(右前轮)', 2: '2号(左前轮)', 3: '3号(右后轮)', 4: '4号(左后轮)', 5: '5号(升降)'}


class Link:
    def __init__(self, port):
        import serial
        self.s = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(0.2)
        self.s.reset_input_buffer()

    def request(self, text, timeout=5.0):
        """发一行，等 DONE/ERR/PONG；返回 (回复, 中间的信息行)。YAW100 不算。"""
        self.s.reset_input_buffer()
        self.s.write((text + '\n').encode('ascii'))
        end = time.time() + timeout
        buf = b''
        info = []
        while time.time() < end:
            buf += self.s.read(256)
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                t = line.decode('ascii', 'ignore').strip()
                if not t or t.startswith('YAW100'):
                    continue
                if t.startswith(('DONE', 'ERR', 'PONG')):
                    return t, info
                info.append(t)
        return None, info

    def yaw(self, timeout=1.0):
        """读一次陀螺仪角度(度)，读不到返回 None。"""
        self.s.reset_input_buffer()
        end = time.time() + timeout
        buf = b''
        while time.time() < end:
            buf += self.s.read(256)
            m = re.findall(rb'YAW100 (-?\d+)', buf)
            if m:
                return int(m[-1]) / 100.0
        return None


def ask(q):
    while True:
        a = input(q + ' (y/n)：').strip().lower()
        if a in ('y', 'n'):
            return a == 'y'


def mot(link):
    rep, info = link.request('MOT?', 5.0)
    if rep is None or not rep.startswith('DONE'):
        print(f'  STM32 回复：{rep}。STM32 不认识 MOT?，说明烧的不是 v4 程序，先烧新程序。')
        sys.exit(1)
    res = {}
    for line in info:
        m = re.match(r'MOT (\d) (.*)', line)
        if m:
            res[int(m.group(1))] = m.group(2).strip()
            print('  ' + line)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', nargs='?', default='normal', choices=['normal', 'loop'])
    ap.add_argument('--port', default='/dev/serial0')
    a = ap.parse_args()

    try:
        link = Link(a.port)
    except Exception as ex:
        print(f'打不开串口 {a.port}：{ex}\n(map_merge_live 还开着吗？先 q 退出。STM32 不在这个口就用 --port 指定)')
        return 1

    print('① 树莓派 ↔ STM32 通信(USART3)')
    rep, _ = link.request('PING', 2.0)
    if rep != 'PONG':
        print(f'  没回 PONG(收到 {rep})：树莓派和 STM32 之间就不通，先查这根线和串口号，PC10/PC11 没法测。')
        return 1
    print('  正常')

    if a.mode == 'loop':
        print('② 短接测试：PC10 发出去的字节应该直接被 PC11 收到')
        print('  请确认：驱动器的串口线已经拔掉，PC10 和 PC11 用一根杜邦线连在一起。')
        input('  准备好按回车……')
        res = mot(link)
        got = [n for n, r in res.items() if 'RX=' in r]
        if got:
            print('\n结论：PC11 收到了 PC10 发出的字节 → PC10、PC11 两个引脚和 UART4 都是好的。')
            print('      问题在驱动器那一侧：驱动器的线、电源、地址。')
        else:
            print('\n结论：短接了却什么都没收到 → PC10 或 PC11 坏了/没焊好，或者短接线没接对(要接在 STM32 板子的 PC10、PC11 上)。')
        return 0

    print('② 解除驱动器堵转保护并使能(MOT EN)')
    rep, _ = link.request('MOT EN', 3.0)
    print(f'  {rep}')

    print('③ 读 5 个驱动器的回复(测 PC11：驱动器 → STM32)')
    res = mot(link)
    answered = [n for n in (1, 2, 3, 4) if n in res and not res[n].startswith('NOREPLY')]
    garbage = [n for n in (1, 2, 3, 4) if n in res and 'RX=' in res[n]]
    for n in (1, 2, 3, 4, 5):
        r = res.get(n, 'NOREPLY')
        if r.startswith('NOREPLY'):
            print(f'  {NAMES[n]}：没回复' + ('(收到了乱码)' if 'RX=' in r else ''))
        else:
            v = re.search(r'V=([\d.?]+)', r)
            print(f'  {NAMES[n]}：有回复，电压 {v.group(1) if v else "?"}V' +
                  ('，★堵转保护' if 'PROT=1' in r else '') + ('，★没使能' if 'EN=0' in r else ''))

    print('④ 发转动指令(测 PC10：STM32 → 驱动器)')
    print('  ★ 车必须架空，4 个轮子离地！车会慢速转动一下。')
    if not ask('  车已经架空了吗'):
        print('  先架空再运行。')
        return 1
    y0 = link.yaw()
    rep, _ = link.request('F 100 30', 15.0)
    print(f'  F 100 30(慢速前进 100mm) -> {rep}')
    turned = ask('  刚才 4 个轮子都转了吗')
    some = False
    if not turned:
        some = ask('  有部分轮子转了吗')
    y1 = link.yaw()

    print('\n========== 结论 ==========')
    if turned or some:
        print('PC10(STM32 → 驱动器)：好的，驱动器收到了指令' + ('' if turned else '(但只有部分轮子转：查不转的那几个的电源线/电机线/地址)'))
    else:
        print('PC10(STM32 → 驱动器)：驱动器没有动作。可能是 ① 驱动器没电(看驱动器屏幕亮不亮) ② PC10 到驱动器 RX 的线断了/接反了 '
              '③ 升降驱动器的 TX 错接在 PC10 上把线顶住了(拔掉升降驱动器的串口线再测) ④ 驱动器地址不是 1~4')
    if answered:
        print('PC11(驱动器 → STM32)：好的，收到了 ' + '、'.join(NAMES[n] for n in answered) + ' 的回复')
        if len(answered) < 4:
            print('  没回复的那几个：查它们的电源和串口线、地址')
    elif garbage:
        print('PC11(驱动器 → STM32)：收到了字节但格式不对：波特率不对、或者线上有干扰/两个驱动器同时回复(地址重复)')
    elif turned or some:
        print('PC11(驱动器 → STM32)：收不到回复，但轮子能转 → 驱动器的 TX 没接到 PC11(以前的程序只发不收，这根线可能本来就没接)。'
              '不影响跑车，只是 mot 读不到状态。想用就把驱动器的 TX 接到 PC11。')
    else:
        print('PC11：也收不到回复。轮子不转 + 没回复，最可能是驱动器根本没电，或者整根串口线没接好。可以再做短接测试：python3 uart4_test.py loop')
    if y0 is not None and y1 is not None:
        print(f'(陀螺仪读数正常：{y0:.1f}° → {y1:.1f}°)')
    else:
        print('(没读到陀螺仪 YAW100)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
