"""(10-28 用户"把所有的就是高度往上面提高四毫米变成124")升降的高度参数一起往上/往下挪，主程序里输入：

    zup 4            所有高度(ZHI ZGRAB ZDROP ZPLC ZSTK ZOBRAW ZOBRNG)都加 4mm(ZHI 120 -> 124 …)
    zup -4           都减 4mm(还原)
    zup 4 ZHI ZDROP  只改列出来的几个

按 STM32 现在的值加减，改完存进配置的 stm32_params(树莓派每次运行把它们发给 STM32)。"""

Z_KEYS = ('ZHI', 'ZGRAB', 'ZDROP', 'ZPLC', 'ZSTK', 'ZOBRAW', 'ZOBRNG')
MAX_STEP_MM = 30.0


def parse(parts):
    """zup 后面的参数 -> (毫米, [参数名])。格式不对抛 ValueError(说清楚怎么写)。"""
    usage = '格式：zup 毫米 [参数名…]，例如 zup 4(所有高度都加 4mm)、zup -4(还原)、zup 4 ZHI(只改 ZHI)'
    if len(parts) < 2:
        raise ValueError(usage)
    try:
        dz = float(parts[1])
    except ValueError:
        raise ValueError(usage) from None
    if abs(dz) < 0.05 or abs(dz) > MAX_STEP_MM:
        raise ValueError(f'一次加减 0.1~{MAX_STEP_MM:g}mm')
    names = [str(n).upper() for n in parts[2:]]
    bad = [n for n in names if n not in Z_KEYS]
    if bad:
        raise ValueError('不是高度参数：' + '、'.join(bad) + '(高度参数有 ' + ' '.join(Z_KEYS) + ')')
    return dz, names or list(Z_KEYS)


def shift(link, dz, names=None, log=print):
    """names 里的高度参数(默认全部)都加 dz 毫米：先读 STM32 现在的值，再一个一个 SET。返回改成了的 {名字: 新值}(存配置用)。"""
    ps, rep = link.get_params()
    if ps is None:
        raise ValueError(f'读 STM32 参数失败：{rep}')
    want = {}
    for n in names or Z_KEYS:
        if n not in ps:
            log(f'  ({n}：STM32 里没有这个参数，跳过)')
            continue
        v = round(float(ps[n]) + float(dz), 2)
        if v < 0.0:
            raise ValueError(f'{n} 现在 {float(ps[n]):g}mm，减 {abs(dz):g} 就小于 0 了：一个都没改')
        want[n] = (float(ps[n]), v)
    done = {}
    for n, (_old, v) in want.items():
        ok, rep = link.set_param(n, v)
        if ok:
            done[n] = v
        else:
            log(f'  ★ STM32 拒绝 {n}={v:g}：{rep}')
    if not done:
        log('  一个都没改。')
        return done
    log('已改好，存进配置(以后每次启动自动发给 STM32)：' + '  '.join(f'{n} {want[n][0]:g}→{v:g}' for n, v in done.items()))
    lmax = ps.get('LFMAX')
    over = [n for n, v in done.items() if lmax is not None and v > float(lmax) + 1e-6]
    if over:
        log(f'  ★ 升降最高只到 LFMAX={float(lmax):g}mm：{"、".join(over)} 实际只升到 {float(lmax):g}。'
            f'机构确实还能往上，量好最高能到多少再 set LFMAX 数值(不要超过机构顶住的地方)')
    cam = [n for n in ('ZOBRAW', 'ZOBRNG') if n in done]
    if cam:
        log(f'  注意：{"、".join(cam)} 是看原料盘/看圆环时摄像头的高度，画面里的十字(爪子点)可能差一两个像素。'
            '开跑时会提示"重做 vclaw"；放得不准就重做 vclaw RAW / vclaw RING(原料区再 rteach RAW near)，或者 g 挪十字')
    return done
