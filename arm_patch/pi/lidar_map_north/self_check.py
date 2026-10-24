"""(10-24)比赛前自检：终端输入 check。只问、只看文件，车和手臂都不动。
每一项打一行：✓ 没问题，★ 要处理(后面写怎么办)。最后说一共几项要处理。

查的东西：STM32 连着没有、固件新不新(PRE、升降开机认高度)、机械臂参数、升降高度认没认准；
配置里机械臂任务开没开、full on/off、开机自动比赛(bootrace)开没开、四个停车点；
示教文件(原料区最近那格、1、3 号环、摄像头和手臂的对应关系、爪子点)；摄像头、雷达的设备在不在；开机自启动装没装；SD 卡剩多少空间。"""
import glob
import json
import os
import re
import shutil


def _load(path):
    try:
        with open(path, encoding='utf-8') as f:
            d = json.load(f)
        return d if isinstance(d, dict) else None
    except Exception:
        return None


def run(link, raw_cfg, lidar_port, log, root='.'):
    bad = []

    def ok(text):
        log('  ✓ ' + text)

    def no(text):
        bad.append(text)
        log('  ★ ' + text)

    log('自检(车和手臂都不动)：')
    mc = dict((raw_cfg or {}).get('mission_cfg') or {})

    # ---- STM32
    if link is None:
        no('STM32 串口没打开：查 STM32 和树莓派之间的线、STM32 有没有电，重开程序')
    else:
        try:
            r = link.request('PING', 2.0)
            (ok if r[0] else no)('STM32 回话了(PING)' if r[0] else f'STM32 没回 PING({r[1]})：查线、电源，按一下 STM32 复位再试')
        except Exception as ex:
            no(f'问 STM32 出错：{ex!r}')
        params = {}
        try:
            r = link.request('GET', 4.0, collect=True)
            for line in (r[2] if len(r) > 2 else []):
                m = re.match(r'P (\w+)=(-?[\d.]+)', line)
                if m:
                    params[m.group(1)] = float(m.group(2))
        except Exception as ex:
            log(f'    (GET 出错：{ex!r})')
        if 'ARMOK' not in params:
            no('STM32 里没有机械臂参数(GET 没有 ARMOK)：固件是不是旧的？重新烧 arm.c 那一版')
        elif params['ARMOK'] < 0.5:
            no('机械臂参数还没标定(ARMOK=0)：标定好以后 set ARMOK 1')
        else:
            ok(f'机械臂参数在(ARMOK=1，ZHI={params.get("ZHI", 0):g}，ZOBRAW={params.get("ZOBRAW", 0):g})')
        try:
            r = link.request('LIFT?', 3.0, collect=True)
            line = next((l for l in (r[2] if len(r) > 2 else []) if l.startswith('LIFT ')), '')
            m = re.match(r'LIFT (\d) (-?[\d.]+)', line)
            boot = re.search(r'BOOT=(\w+)', line)
            if not m:
                no(f'LIFT? 没回高度({r[1]})')
            elif m.group(1) != '1':
                no('升降高度不知道(没回零)：把升降放到最低点，输入 arm LIFT ZERO')
            elif boot and boot.group(1) in ('NOCAL', 'NOENC'):
                no(f'升降高度是猜的(BOOT={boot.group(1)}：开机没读到编码器)：查 5 号驱动器的线；现在 {float(m.group(2)):.1f}mm 对不对要看一眼')
            else:
                ok(f'升降高度认准了：{float(m.group(2)):.1f}mm' + (f'(开机 BOOT={boot.group(1)})' if boot else ''))
        except Exception as ex:
            no(f'问升降高度出错：{ex!r}')
        try:
            r = link.request('PRE X', 2.0)                     # 参数不对：新固件回 ERR ARG(不动)，旧固件回 ERR CMD
            if 'ERR ARG' in str(r[1]):
                ok('固件是新的(认识 PRE：开往工位时提前摆臂)')
            else:
                no(f'固件可能是旧的(PRE 回 {r[1]})：开往工位时不会提前摆臂、开机保活也是旧的。重新烧最新的 arm.c')
        except Exception as ex:
            no(f'问固件版本出错：{ex!r}')

    # ---- 配置
    if mc.get('enabled'):
        ok('机械臂任务开着(mission_cfg.enabled=true)')
    else:
        no('机械臂任务没开(mission_cfg.enabled=false)：跑的时候不读码、不夹放')
    full = bool(mc.get('full_course', True))
    log('  · 跑法：' + ('full on(一直跑完全程)' if full else f'full off(3 分钟内回家，{float(mc.get("time_limit_s", 150)):g} 秒以后不再开始新的夹放)')
        + '。要改：full on / full off')
    if (raw_cfg or {}).get('boot_race'):
        log(f'  · 开机自动比赛开着：下次开电闸按启停区 {raw_cfg.get("start_zone")} 直接开跑(只用一次)')
    else:
        log('  · 开机自动比赛没开(调试时这样)。比赛：剩 30 秒时输入 bootrace 1 或 bootrace 2，q，关电闸')
    stops = (raw_cfg or {}).get('stops') or {}
    miss = [s for s in ('QR', 'RAW', 'ROUGH', 'TEMP') if s not in stops]
    if miss:
        no('配置里少停车点：' + '、'.join(miss))
    else:
        ok('四个停车点都在：' + '  '.join(f'{s}({stops[s][0]:.0f},{stops[s][1]:.0f})' for s in ('QR', 'RAW', 'ROUGH', 'TEMP')))

    # ---- 示教文件
    def path_of(key, default):
        p = mc.get(key) or default
        return p if os.path.isabs(p) else os.path.join(root, p)
    rt = _load(path_of('ring_teach_file', 'ring_teach.json')) or {}
    ct = rt.get('RAW_CT') or {}
    if ct.get('near') and (ct.get('slots') or {}).get(str(ct.get('near')), {}).get('a'):
        ok('原料区最近那格示教过(rteach RAW near)')
    else:
        no('原料区最近那格没示教(或者没有抓取角度)：车停在 RAW，arm_calib 调好以后 rteach RAW near 颜色号')
    for z in ('ROUGH', 'TEMP'):
        zz = rt.get(z) or {}
        if zz.get('1') and zz.get('3'):
            ok(f'{z} 的 1、3 号环调过手臂角度' + ('(arm_calib)' if zz.get('src') == 'arm_calib' else ''))
        else:
            no(f'{z} 的 1、3 号环没调手臂角度：会一个环一个环挪车对准(慢)。python3 arm_calib.py --from A1P')
    sc = _load(path_of('servo_cal_file', 'servo_cal.json')) or {}
    for z in ('RAW', 'RING'):
        (ok if (sc.get(z) or {}).get('arm') else no)(
            f'servo_cal.json 里有 {z} 的手臂和画面的对应关系' if (sc.get(z) or {}).get('arm') else
            f'servo_cal.json 里没有 {z} 的手臂和画面的对应关系：vcal {z if z == "RAW" else "RING"} … arm 测一次')
    vc = _load(path_of('vision_cal_file', 'vision_cal.json')) or {}
    cp = vc.get('claw_px') or {}
    left = [k for k in ('RAW', 'RING') if not cp.get(k)]
    log('  · 爪子点' + ('都量过(vclaw)' if not left else f' {"、".join(left)} 用的是配置里的默认值(没做 vclaw)；放得偏再做'))

    # ---- 设备、自启动、空间
    dev = str(((mc.get('camera') or {}).get('device')) or '/dev/video0')
    cams = sorted(glob.glob('/dev/video*'))
    if os.path.exists(dev):
        ok(f'摄像头设备在：{dev}')
    elif cams:
        no(f'配置的摄像头 {dev} 不在，插着的是 {"、".join(cams)}(程序会自动找别的 USB 摄像头)')
    else:
        no('没找到摄像头(/dev/video*)：USB 没插好？')
    (ok if os.path.exists(str(lidar_port)) else no)(
        f'雷达设备在：{lidar_port}' if os.path.exists(str(lidar_port)) else f'雷达设备 {lidar_port} 不在：雷达 USB 没插好？')
    auto = os.path.expanduser('~/.config/autostart/car_boot_race.desktop')
    (ok if os.path.exists(auto) else no)('开机自启动装好了' if os.path.exists(auto) else '开机自启动没装：bash install_boot_race.sh(只装一次)')
    try:
        free = shutil.disk_usage(root).free / 1e6
        (ok if free > 300 else no)(f'SD 卡还剩 {free:.0f}MB' + ('' if free > 300 else '：太少了，删掉 map_captures、logs 里旧的文件'))
    except Exception:
        pass
    log(f'自检完：{len(bad)} 项要处理' + ('。' if bad else '，可以比赛。'))
    return bad
