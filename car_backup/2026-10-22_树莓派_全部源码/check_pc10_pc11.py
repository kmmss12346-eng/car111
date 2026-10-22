#!/usr/bin/env python3
"""在树莓派上通过 openocd(就是平时烧录用的那条线)直接读 STM32 的寄存器，检查 PC10/PC11(UART4，接 4 个轮子电机和升降电机)。
不改 STM32 里的任何东西、不停 STM32、不烧录，只读。

用法(树莓派终端)：
    python3 check_pc10_pc11.py            检查一次：时钟、引脚模式、UART4 设置、两根线现在的电平
    python3 check_pc10_pc11.py watch [次数] 连续采 PC10 的电平几秒：这期间在另一个终端里发 send S 600，
                                          看 STM32 是不是真的在 PC10 上往电机发数据
openocd 的参数自动从你烧录时用过的命令里找(~/.bash_history)；找不到就按提示把平时烧录的命令贴给它。
用之前先关掉其它正在运行的 openocd。
"""
import os
import re
import shlex
import subprocess
import sys

REG = {
    'RCC_AHB1ENR': 0x40023830,
    'RCC_APB1ENR': 0x40023840,
    'GPIOC_MODER': 0x40020800,
    'GPIOC_PUPDR': 0x4002080C,
    'GPIOC_IDR':   0x40020810,
    'GPIOC_AFRH':  0x40020824,
    'UART4_SR':    0x40004C00,
    'UART4_BRR':   0x40004C08,
    'UART4_CR1':   0x40004C0C,
}
DROP = ('program', 'reset', 'exit', 'shutdown', 'init', 'verify', 'halt', 'flash', 'resume')


def find_openocd_args():
    """从 bash 历史里找最近一次 openocd 命令，只留下“怎么连上 STM32”的部分(-f / -s / adapter / transport 等)。"""
    hist = os.path.expanduser('~/.bash_history')
    line = None
    if os.path.exists(hist):
        for l in open(hist, encoding='utf-8', errors='ignore'):
            if 'openocd' in l and '-f' in l:
                line = l.strip()
    if not line:
        print('没在 ~/.bash_history 里找到你烧录用的 openocd 命令。')
        line = input('请把你平时烧录 STM32 的那条 openocd 命令整行粘贴进来，回车：').strip()
    try:
        tok = shlex.split(line)
    except ValueError:
        tok = line.split()
    i = tok.index('openocd') if 'openocd' in tok else next(k for k, t in enumerate(tok) if t.endswith('openocd'))
    pre = tok[:i + 1]                         # 可能带 sudo
    keep = []
    rest = tok[i + 1:]
    k = 0
    while k < len(rest):
        t = rest[k]
        if t in ('-f', '-s', '-d') and k + 1 < len(rest):
            keep += [t, rest[k + 1]]; k += 2; continue
        if t == '-c' and k + 1 < len(rest):
            c = rest[k + 1]
            if not any(w in c.lower() for w in DROP):
                keep += [t, c]
            k += 2; continue
        if t.startswith('-f') and len(t) > 2:
            keep.append(t)
        k += 1
    return pre, keep, line


def run_openocd(pre, keep, cmds, timeout=60):
    args = pre + keep + ['-c', 'init']
    for c in cmds:
        args += ['-c', c]
    args += ['-c', 'shutdown']
    p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    out = p.stdout + p.stderr
    vals = []
    for m in re.finditer(r'0x([0-9a-fA-F]{8}):\s+([0-9a-fA-F]{8})', out):
        vals.append((int(m.group(1), 16), int(m.group(2), 16)))
    return vals, out


def bits(v, lo, n):
    return (v >> lo) & ((1 << n) - 1)


def check(pre, keep):
    names = list(REG)
    vals, out = run_openocd(pre, keep, [f'mdw 0x{REG[n]:08x}' for n in names])
    if len(vals) < len(names):
        print('openocd 没读到寄存器。它的输出最后几行是：')
        print('\n'.join(out.strip().splitlines()[-15:]))
        print('\n常见原因：1) 另一个 openocd 还开着(先关掉)；2) STM32 没上电；3) 烧录线没接好。')
        return
    r = {n: v for n, (_, v) in zip(names, vals)}
    ok_all = True

    def line(ok, text):
        nonlocal ok_all
        ok_all = ok_all and ok
        print(('  [对]  ' if ok else '  [错]  ') + text)

    print('\n=== PC10(UART4_TX，STM32 → 电机) / PC11(UART4_RX，电机 → STM32) ===')
    line(bool(r['RCC_AHB1ENR'] & (1 << 2)), 'GPIOC 时钟打开')
    line(bool(r['RCC_APB1ENR'] & (1 << 19)), 'UART4 时钟打开')
    m10, m11 = bits(r['GPIOC_MODER'], 20, 2), bits(r['GPIOC_MODER'], 22, 2)
    mode = {0: '普通输入', 1: '普通输出', 2: '复用功能(串口)', 3: '模拟'}
    line(m10 == 2, f'PC10 模式 = {mode[m10]}（应该是 复用功能）')
    line(m11 == 2, f'PC11 模式 = {mode[m11]}（应该是 复用功能）')
    a10, a11 = bits(r['GPIOC_AFRH'], 8, 4), bits(r['GPIOC_AFRH'], 12, 4)
    line(a10 == 8, f'PC10 复用编号 = AF{a10}（应该是 AF8 = UART4；AF7 是 USART3，AF12 是 SDIO）')
    line(a11 == 8, f'PC11 复用编号 = AF{a11}（应该是 AF8 = UART4）')
    cr1 = r['UART4_CR1']
    line(bool(cr1 & (1 << 13)), 'UART4 已启用(UE)')
    line(bool(cr1 & (1 << 3)), 'UART4 发送已打开(TE)')
    brr = r['UART4_BRR']
    baud = 42_000_000 / brr if brr else 0
    line(abs(baud - 115200) < 3000, f'UART4 波特率 ≈ {baud:.0f}（应该是 115200，和电机驱动一致）')
    idr = r['GPIOC_IDR']
    l10, l11 = bits(idr, 10, 1), bits(idr, 11, 1)
    line(l10 == 1, f'PC10 现在的电平 = {"高" if l10 else "低"}（不发数据时应该是高；一直低说明这根线被拉到地/短路）')
    line(l11 == 1, f'PC11 现在的电平 = {"高" if l11 else "低"}（电机不回话时应该是高；一直低说明电机那边把线拉低了/接错）')
    sr = r['UART4_SR']
    if sr & 0x0F:
        print(f'  [注意] UART4 有接收错误标志 SR=0x{sr:04x}(帧错误/噪声/溢出)：PC11 上的数据不干净，检查电机回线和地线')
    print('\n结论：' + ('PC10/PC11 的设置全部正常。下一步用 watch 看 STM32 走车时是不是真的在发数据。' if ok_all
                     else '上面有 [错] 的项，把这个输出截图发给我。'))


def watch(pre, keep, n=1500):
    import time
    print(f'开始连续读 PC10 电平 {n} 次……')
    t0 = time.monotonic()
    vals, out = run_openocd(pre, keep, ['mdw 0x40020810'] * n, timeout=300)
    print(f'采样用时 {time.monotonic() - t0:.1f} 秒(含 openocd 启动约 1 秒)')
    if not vals:
        print('openocd 没读到数据：\n' + '\n'.join(out.strip().splitlines()[-10:]))
        return
    low = sum(1 for _, v in vals if not (v >> 10) & 1)
    low11 = sum(1 for _, v in vals if not (v >> 11) & 1)
    print(f'采了 {len(vals)} 次：PC10 读到低电平 {low} 次，PC11 读到低电平 {low11} 次')
    if low == 0:
        print('PC10 一直是高：这段时间里 STM32 没有往电机发任何数据(或者 send 没在采样期间发出)。再试一次，确认采样时车在执行 send。')
    elif low == len(vals):
        print('PC10 一直是低：TX 线被拉到地了(短路，或者接到了别的设备的输出上)。')
    else:
        print('PC10 有高有低：STM32 确实在往电机发数据。轮子还不动的话，问题在线路/电机驱动那一侧(地址、波特率、使能、供电)。')
    if 0 < low11 < len(vals):
        print('PC11 也有低电平：电机在回话，说明至少有一个电机收到了指令。')


def main():
    pre, keep, src = find_openocd_args()
    print('用这条命令连 STM32：', ' '.join(shlex.quote(x) for x in pre + keep))
    print('(取自：' + src + ')')
    if len(sys.argv) > 1 and sys.argv[1] == 'watch':
        watch(pre, keep, int(sys.argv[2]) if len(sys.argv) > 2 else 1500)
    else:
        check(pre, keep)


if __name__ == '__main__':
    main()
