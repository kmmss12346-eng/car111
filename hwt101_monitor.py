#!/usr/bin/env python3
"""树莓派上实时监测 HWT101（维特智能单轴陀螺仪，Z 轴偏航）旋转角度。

协议：WIT 标准 11 字节帧  0x55 <类型> D0..D7 SUM
  0x52 角速度  D0..D1 = Z 轴角速度 (原始 int16 -> 2000 dps / 32768)，HWT101 只有 Z 轴有效
  0x53 角度    D4..D5 = Z 轴偏航角 (原始 int16 -> 180 deg / 32768)
串口：USB-TTL 通常是 /dev/ttyUSB0；接树莓派 GPIO 串口是 /dev/serial0（需关闭串口登录 shell）。
HWT101 出厂波特率常见为 9600 或 115200；不确定就用 --scan 自动探测。

用法：
  python3 hwt101_monitor.py --list            # 列出串口
  python3 hwt101_monitor.py --scan            # 自动探测波特率
  python3 hwt101_monitor.py -p /dev/ttyUSB0 -b 115200
  运行中在终端输入 z 回车 = 把当前角度清零；q 回车 = 退出
  --log yaw.csv 保存记录

注意：这个程序只读取数据，不会向传感器发送任何指令，也不修改传感器内部设置。
"""
import argparse
import select
import struct
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit('缺少 pyserial：pip3 install pyserial（或 sudo apt install python3-serial）')

BAUDS = (115200, 9600, 230400, 57600, 38400, 19200, 4800, 2400)


class Parser:
    """字节流 -> (类型, 原始数据) ，自动找帧头并校验和，出错时逐字节重新同步。"""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf += data
        out = []
        while len(self.buf) >= 11:
            if self.buf[0] != 0x55:
                del self.buf[0]
                continue
            frame = self.buf[:11]
            if sum(frame[:10]) & 0xFF != frame[10]:
                self.bad += 1
                del self.buf[0]
                continue
            out.append((frame[1], bytes(frame[2:10])))
            del self.buf[:11]
        return out


def decode(kind, d):
    """返回 ('yaw', 度) / ('gyro', 度每秒) / None"""
    if kind == 0x53:
        return 'yaw', struct.unpack('<h', d[4:6])[0] / 32768.0 * 180.0
    if kind == 0x52:
        return 'gyro', struct.unpack('<h', d[4:6])[0] / 32768.0 * 2000.0
    return None


class Tracker:
    """把 -180..180 的偏航角展开成累计角度，并支持清零。"""

    def __init__(self):
        self.last = None
        self.total = 0.0
        self.zero = 0.0

    def update(self, yaw):
        if self.last is None:
            self.total = yaw
        else:
            d = yaw - self.last
            if d > 180:
                d -= 360
            elif d < -180:
                d += 360
            self.total += d
        self.last = yaw
        return self.total - self.zero

    def rezero(self):
        self.zero = self.total


def scan(port):
    for baud in BAUDS:
        try:
            with serial.Serial(port, baud, timeout=0.1) as s:
                s.reset_input_buffer()
                p = Parser()
                t0 = time.time()
                ok = 0
                while time.time() - t0 < 1.2:
                    ok += len(p.feed(s.read(256)))
                print(f'{baud:>7} baud: 有效帧 {ok}, 校验错误 {p.bad}')
                if ok >= 3:
                    return baud
        except serial.SerialException as e:
            sys.exit(f'打开 {port} 失败：{e}')
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-p', '--port', default='/dev/ttyUSB0')
    ap.add_argument('-b', '--baud', type=int, default=115200)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--scan', action='store_true')
    ap.add_argument('--log')
    a = ap.parse_args()

    if a.list:
        for p in list_ports.comports():
            print(p.device, p.description)
        return
    if a.scan:
        b = scan(a.port)
        if b is None:
            sys.exit('所有波特率都没收到有效帧：检查接线(TX↔RX 交叉、共地)、供电、传感器是否在输出。')
        print(f'推荐波特率：{b}')
        a.baud = b

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f'打开 {a.port} 失败：{e}')
    log = open(a.log, 'w') if a.log else None
    if log:
        log.write('t_s,yaw_deg,cumulative_deg,gyro_dps\n')

    parser, trk = Parser(), Tracker()
    gyro = float('nan')
    cum = None
    t_start = t_last_frame = t_print = time.time()
    print('开始监测。z+回车=清零，q+回车=退出，Ctrl+C 也可退出。')
    try:
        while True:
            for kind, d in parser.feed(ser.read(256)):
                r = decode(kind, d)
                if r is None:
                    continue
                t_last_frame = time.time()
                if r[0] == 'gyro':
                    gyro = r[1]
                else:
                    cum = trk.update(r[1])
                    if log:
                        log.write(f'{t_last_frame - t_start:.3f},{r[1]:.3f},{cum:.3f},{gyro:.3f}\n')
            now = time.time()
            if now - t_print >= 0.1:
                t_print = now
                if now - t_last_frame > 1.0:
                    msg = '无数据（>1s）：检查接线/波特率/供电'
                elif cum is None:
                    msg = '等待角度帧(0x53)…'
                else:
                    msg = f'偏航(累计) {cum:9.2f}°  折算 {((cum + 180) % 360) - 180:8.2f}°  角速度 {gyro:8.2f}°/s  坏帧 {parser.bad}'
                print('\r' + msg + ' ' * 6, end='', flush=True)
            if select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.readline().strip().lower()
                if c == 'q':
                    break
                if c == 'z':
                    trk.rezero()
    except KeyboardInterrupt:
        pass
    finally:
        print()
        ser.close()
        if log:
            log.close()


if __name__ == '__main__':
    main()
