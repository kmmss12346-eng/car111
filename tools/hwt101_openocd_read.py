#!/usr/bin/env python3
"""在树莓派上通过 ST-Link + OpenOCD 读取 STM32 里的 hwt101_dbg，不需要额外接线。

步骤：
  1) 终端 A 保持运行（读内存不会暂停 MCU）：
       openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
  2) 终端 B：
       python3 hwt101_openocd_read.py --map /home/zstu/car_2027.map
     或直接给地址：--addr 0x2000xxxx
  .map 文件在 Keil 工程 MDK-ARM/car_2027/ 下（和 .hex 同目录），一起 scp 过来；
  每次重新编译地址可能变，所以要用同一次编译的 .map。

hwt101_dbg 结构(小端, 8 个 32 位)：
  magic('HWT1'), frames_ok, frames_bad, yaw(float), yaw_cont(float), gyro(float), last_tick, now_tick
"""
import argparse
import re
import socket
import struct
import sys
import time

MAGIC = 0x48575431


def find_addr(map_path):
    pat = re.compile(r'^\s*hwt101_dbg\s+(0x[0-9a-fA-F]+)', re.M)
    for enc in ('utf-8', 'gbk', 'latin-1'):
        try:
            text = open(map_path, encoding=enc).read()
            break
        except UnicodeDecodeError:
            continue
    m = pat.search(text)
    if not m:
        sys.exit('.map 里没找到 hwt101_dbg：确认用的是新固件编译出的 .map')
    return int(m.group(1), 16)


class Ocd:
    def __init__(self, host, port):
        self.s = socket.create_connection((host, port), timeout=3)
        self._read_until_prompt()

    def _read_until_prompt(self):
        buf = b''
        while not buf.endswith(b'> '):
            d = self.s.recv(4096)
            if not d:
                break
            buf += d
        return buf.decode(errors='replace')

    def cmd(self, c):
        self.s.sendall(c.encode() + b'\x1a')   # OpenOCD telnet 用 0x1a 结束命令
        return self._read_until_prompt()

    def mdw(self, addr, n):
        out = self.cmd(f'mdw 0x{addr:08x} {n}')
        words = []
        for line in out.splitlines():
            m = re.match(r'\s*0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{8}\s*)+)', line)
            if m:
                words += [int(x, 16) for x in m.group(1).split()]
        if len(words) < n:
            raise RuntimeError('读内存失败：' + out.strip())
        return words[:n]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map')
    ap.add_argument('--addr', type=lambda x: int(x, 0))
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=4444)
    ap.add_argument('--hz', type=float, default=5)
    a = ap.parse_args()
    if a.addr is None and not a.map:
        ap.error('需要 --map 或 --addr')
    addr = a.addr if a.addr is not None else find_addr(a.map)
    print(f'hwt101_dbg @ 0x{addr:08x}')

    try:
        ocd = Ocd(a.host, a.port)
    except OSError as e:
        sys.exit(f'连不上 OpenOCD({a.host}:{a.port})：{e}\n先在另一个终端运行 openocd -f interface/stlink.cfg -f target/stm32f4x.cfg')

    prev_ok = None
    try:
        while True:
            w = ocd.mdw(addr, 8)
            if w[0] != MAGIC:
                print(f'magic=0x{w[0]:08x} 不对：地址/固件版本不匹配，或固件还没跑到 HWT101_Init')
            else:
                yaw, cont, gyro = struct.unpack('<fff', struct.pack('<III', w[3], w[4], w[5]))
                age = (w[7] - w[6]) & 0xFFFFFFFF
                rate = '' if prev_ok is None else f'  +{w[1] - prev_ok}帧'
                state = 'OK' if w[1] and age < 500 else ('无数据' if not w[1] else f'数据已停 {age}ms')
                print(f'[{state}] yaw {yaw:8.2f}°  累计 {cont:9.2f}°  角速度 {gyro:8.2f}°/s  好帧 {w[1]} 坏帧 {w[2]}{rate}')
                prev_ok = w[1]
            time.sleep(1 / a.hz)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
