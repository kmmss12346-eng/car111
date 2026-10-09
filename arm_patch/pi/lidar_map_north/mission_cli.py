"""树莓派终端里的机械臂 / 视觉测试命令（接在 map_merge_live 的命令行上）。

arm <指令>                 直接给 STM32 发一条机械臂指令并打印回复，例如：
                             arm LIFT ZERO        把现在的升降位置记为 0(最低点；数字越大越高)
                             arm OBS RAW O        手臂摆到原料盘观察姿态并张开夹爪
                             arm AD 1 0.5         ID1 再转 0.5 度
                             arm GET              看全部参数(含机械臂的)
qr                         读 STM32 里存的任务码
mcode <任务码>             手动设置任务码(不扫码也能测)，例如：mcode 156+123+516+231
vmask                      标定爪子在画面里占的区域(存 claw_mask.png)：手臂在原料盘上方、爪子张开、爪子附近没有物料时用
vclaw RAW <颜色号>         实测"爪子夹物料时，物料在画面里的位置"(claw_px.RAW，存 vision_cal.json)：
                             先 arm OBS RAW O，把一个物料放在爪子正下方，再输入 vclaw RAW 1(颜色号)；
                             程序会降下去夹一下(物料会被夹正)、松开、升回观察高度，测出物料圆心
vclaw RING                 实测"放下的物料落在画面哪里"(claw_px.RING)：圆环纸摆好、物料拿走以后用(步骤见 README)
vcal RING                  视觉校准(圆环)：手臂/底盘各动几个小动作，测出"动作量 ↔ 画面移动量"，存进 servo_cal.json
vcal RAW <颜色号>          视觉校准(原料盘上的物料；颜色号 1红 2黄 3蓝 4绿 5黑 6浅蓝)
                             只校准手臂不动底盘：在后面加 arm，例如  vcal RING arm
vdbg [RING | RAW <颜色号>]  存一张带标注的画面到 vdebug.png：爪子位置(绿十字)、识别到的圆环/物料
mtest QR                   单独测读码并显示
mtest RAW <批次>           单独测原料盘抓取(批次 1 或 2；需要先 qr 或 mcode)
mtest ROUGH <批次>         单独测粗加工区放置+取回(转盘里要有这批物料，没有就加 force 假定有：mtest ROUGH 1 force)
mtest TEMP <批次>          单独测暂存区放置/码垛
mtest START                单独测回家后的显示
mtest reset                清空任务码和记录，重新开始
gtest <颜色号> [nogo]      夹取测试：摄像头找这个颜色的物料 → 手臂对准 → 下降夹住 → 抬起来(不放转盘；要 ARMOK=1，因为先 OBS RAW)
                             颜色号 1红 2黄 3蓝 4绿 5黑 6浅蓝；nogo = 只识别和对准，不下降不夹
                             要先标定好：A1G A2E(手臂在原料上方的角度)、ZOBRAW(看的高度)、ZGRAB(夹的高度)、ZHI(抬起高度)，并 arm LIFT ZERO
mot                        看 5 个电机驱动器(1~4 号轮子、5 号升降)的电压、是否使能、是否触发堵转保护——车不动时先用它
mot en                     让 5 个驱动器解除堵转保护并使能，然后再看一次状态
"""
import math
import re
import threading

from task_plan import parse_code, TaskError

_S = {'hooks': None}

MOT_NAMES = {1: '1号(右前轮)', 2: '2号(左前轮)', 3: '3号(右后轮)', 4: '4号(左后轮)', 5: '5号(升降)'}
MOT_LOW_V = 10.5                     # 低于这个电压(伏)提示充电
_MOT = re.compile(r'MOT (\d) (.*)')


def explain_mot(lines):
    """把 STM32 回的 MOT 行翻译成中文。返回 (每个驱动器一行的说明, 总结和建议)。"""
    rows, advice = [], []
    st = {}
    for line in lines:
        m = _MOT.match(line.strip())
        if not m:
            continue
        n, rest = int(m.group(1)), m.group(2).strip()
        name = MOT_NAMES.get(n, f'{n}号')
        if rest.startswith('NOREPLY'):
            raw = rest[len('NOREPLY'):].strip()
            rows.append(f'  {name}：没有回复' + (f'(收到了对不上格式的字节：{raw[3:].strip()})' if raw.startswith('RX=') else ''))
            st[n] = None
            continue
        kv = dict(p.split('=', 1) for p in rest.split() if '=' in p)
        try:
            v = float(kv.get('V', ''))
        except ValueError:
            v = None
        d = dict(v=v, en=kv.get('EN'), prot=kv.get('PROT'), stall=kv.get('STALL'))
        st[n] = d
        bits = [f'电压 {v:.2f}V' if v is not None else '电压没读到']
        if d['en'] is not None:
            bits.append('已使能' if d['en'] == '1' else '没使能(电机松着)')
        if d['prot'] == '1':
            bits.append('★触发了堵转保护(电机松轴，不再响应运动指令)')
        if d['stall'] == '1':
            bits.append('★现在处于堵转')
        if v is not None and v < MOT_LOW_V:
            bits.append(f'★电压偏低(低于 {MOT_LOW_V}V)')
        rows.append(f'  {name}：' + '，'.join(bits))
    if not st:
        return rows, ['STM32 没回 MOT 行：STM32 是不是旧程序(没有 MOT? 指令)？先烧录新程序。']
    wheels = [st.get(i, None) for i in (1, 2, 3, 4)]
    silent = [i for i in (1, 2, 3, 4) if st.get(i) is None]
    if len(silent) == 4:
        advice.append('4 个轮子的驱动器都没回复。可能是：① 驱动器没电(电机电池没开/没电/保险或开关断了，看驱动器屏幕亮不亮)；'
                      '② 驱动器的 TX 没接到 STM32 的 PC11(以前的程序只发不收，这根线可能本来就没接——这种情况下"没回复"不代表坏了，'
                      '用下面的"抬起车轮测试"判断)。')
    elif silent:
        advice.append('这几个没回复：' + '、'.join(MOT_NAMES[i] for i in silent) + '。检查它们的电源线、串口线，以及驱动器里设的地址(要分别是 1~4)。')
    if any(d and d['prot'] == '1' for d in st.values()):
        advice.append('有驱动器触发了堵转保护：输入 mot en 解除(或把电机电源关掉再开)。触发保护说明之前电机转不动：'
                      '电池电压低、车太重、轮子被卡住、或者加速度太大。')
    if any(d and d['en'] == '0' for d in st.values()):
        advice.append('有驱动器没使能：输入 mot en。')
    vs = [d['v'] for d in wheels if d and d['v'] is not None]
    if vs and min(vs) < MOT_LOW_V:
        advice.append(f'轮子驱动器电压最低 {min(vs):.2f}V：电池快没电了，先充电/换电池(电压低时电机没劲，转弯时最先转不动)。')
    if len(vs) >= 2 and max(vs) - min(vs) > 0.8:
        advice.append(f'各驱动器电压相差 {max(vs) - min(vs):.2f}V：电源线太细或接头接触不好。')
    if not advice:
        advice.append('驱动器看起来都正常(有电、已使能、没触发保护)。车还是不动的话：把车抬起来(轮子悬空)输入 send R 90，'
                      '看 4 个轮子转不转——悬空能转、放地上转不动是电池没劲/车太重；悬空也不转，查电机线和驱动器设置。')
    return rows, advice


def _mot(link, log, enable):
    if enable:
        ok, reply = link.request('MOT EN', 3.0)
        log(f'MOT EN -> {reply}')
        if not ok:
            raise ValueError(f'STM32 拒绝 MOT EN：{reply}(STM32 是不是旧程序？)')
    ok, reply, info = link.request('MOT?', 5.0, collect=True)
    if not ok:
        raise ValueError(f'STM32 拒绝 MOT?：{reply}(STM32 是不是旧程序？先烧录新程序)')
    rows, advice = explain_mot(info)
    log('电机驱动器状态：')
    for r in rows:
        log(r)
    for a in advice:
        log('→ ' + a)


def _hooks(link, raw_cfg, log):
    if _S['hooks'] is None:
        from mission_hooks import MissionHooks
        _S['hooks'] = MissionHooks(raw_cfg, log=log)
    h = _S['hooks']
    h.log = log
    return h


def release():
    """释放测试命令占着的摄像头和记录(go 开始前由 map_merge_live 调用，免得两个地方同时开 /dev/video0)。"""
    h = _S.get('hooks')
    _S['hooks'] = None
    if h is not None:
        h.close()


def _run_async(state, log, fn):
    if state.get('auto') or state.get('busy'):
        raise ValueError('正在运行或扫描，等结束再用')
    state['busy'] = True

    def worker():
        try:
            fn()
        except Exception as ex:                      # 包括 Abort
            log(f'出错/停止：{ex!r}')
        finally:
            state['busy'] = False
    threading.Thread(target=worker, daemon=True).start()


def handle_cli(k, parts, link=None, raw_cfg=None, state=None, log=print):
    state = state if state is not None else {}
    if link is None:
        raise ValueError('没有连接 STM32（--stm-port）')

    if k == 'mot':
        sub = parts[1].lower() if len(parts) > 1 else ''
        if sub not in ('', 'en'):
            raise ValueError('格式：mot(看 5 个电机驱动器的状态) / mot en(解除堵转保护并使能，再看一次状态)')
        return _run_async(state, log, lambda: _mot(link, log, sub == 'en'))

    h = _hooks(link, raw_cfg or {}, log)

    if k == 'gtest':
        if len(parts) < 2 or not parts[1].isdigit() or not 1 <= int(parts[1]) <= 6:
            raise ValueError('格式：gtest 1(颜色号：1红 2黄 3蓝 4绿 5黑 6浅蓝)；只识别对准不夹：gtest 1 nogo')
        color = int(parts[1])
        nogo = any(p.lower() == 'nogo' for p in parts[2:])

        def go():
            h.ctx = type('C', (), {'aborted': staticmethod(lambda: bool(state.get('abort')))})()
            state['abort'] = False
            _gtest(h, link, color, nogo, log)
        return _run_async(state, log, go)

    if k == 'arm':
        if len(parts) < 2:
            raise ValueError('格式：arm LIFT ZERO / arm OBS RAW O / arm AD 1 0.5 / arm GET   (见 mission_cli.py 开头)')
        text = ' '.join(parts[1:]).upper() if parts[1].upper() not in ('SCR', 'SCMD') else ' '.join(parts[1:])

        def go():
            h._ensure_arm_only(link)
            ok, reply, info = h.arm.request(text)
            for line in info:
                log('  ' + line)
            log(f'{text} -> {reply}')
        return _run_async(state, log, go)

    if k == 'qr':
        def go():
            h._ensure_arm_only(link)
            log(f'STM32 里的任务码：{h.arm.qr() or "没有"}')
        return _run_async(state, log, go)

    if k == 'mcode':
        if len(parts) != 2:
            raise ValueError('格式：mcode 156+123+516+231')
        try:
            h.plan = parse_code(parts[1])
        except TaskError as ex:
            raise ValueError(str(ex))
        log(f'任务码已设置：{h.plan.code}  {h.plan.describe()}')
        return None

    if k == 'vmask':
        def go():
            h._ensure_arm_only(link)
            h._ensure_vision()
            _vmask(h, log)
        return _run_async(state, log, go)

    if k == 'vclaw':
        if len(parts) < 2 or parts[1].upper() not in ('RAW', 'RING'):
            raise ValueError('格式：vclaw RAW 1(颜色号)   或   vclaw RING')
        kind = parts[1].upper()
        color = None
        if kind == 'RAW':
            if len(parts) < 3 or not parts[2].isdigit() or not 1 <= int(parts[2]) <= 6:
                raise ValueError('vclaw RAW 要给颜色号：1红 2黄 3蓝 4绿 5黑 6浅蓝，例如 vclaw RAW 1')
            color = int(parts[2])

        def go():
            _vclaw(h, link, kind, color, log)
        return _run_async(state, log, go)

    if k == 'vdbg':
        def go():
            h._ensure_arm_only(link)
            kind = parts[1].upper() if len(parts) > 1 else 'RING'
            color = int(parts[2]) if len(parts) > 2 else None
            h._ensure_vision()
            ok = h.vision.save_debug('vdebug.png', kind, color)
            log('已存 vdebug.png' if ok else '存图失败(摄像头没有画面？)')
        return _run_async(state, log, go)

    if k == 'vcal':
        if len(parts) < 2 or parts[1].upper() not in ('RING', 'RAW'):
            raise ValueError('格式：vcal RING   或   vcal RAW 1(颜色号)；只校准手臂不动底盘：vcal RING arm')
        kind = parts[1].upper()
        color = None
        rest = [p.lower() for p in parts[2:]]
        if kind == 'RAW':
            if not parts[2:] or not parts[2].isdigit():
                raise ValueError('vcal RAW 要给颜色号：1红 2黄 3蓝 4绿 5黑 6浅蓝，例如 vcal RAW 1')
            color = int(parts[2])
        chassis = 'arm' not in rest

        def go():
            h._ensure(link)
            if h.disabled:
                raise ValueError(h.disabled)
            _vcal(h, kind, color, chassis, log)
        return _run_async(state, log, go)

    if k == 'mtest':
        if len(parts) < 2:
            raise ValueError('格式：mtest QR / mtest RAW 1 / mtest ROUGH 1 [force] / mtest TEMP 1 / mtest START / mtest reset')
        what = parts[1].upper()
        if what == 'RESET':
            release()                                   # 关掉摄像头(后台线程)再丢掉记录，不然摄像头一直被占着
            log('已清空 mtest 的任务码和记录')
            return None
        if what not in ('QR', 'RAW', 'ROUGH', 'TEMP', 'START'):
            raise ValueError('mtest 后面是 QR / RAW / ROUGH / TEMP / START / reset')
        batch = int(parts[2]) if len(parts) > 2 and parts[2].isdigit() else 1
        force = any(p.lower() == 'force' for p in parts[2:])

        def go():
            h._ensure(link)
            h.ctx = type('C', (), {'aborted': staticmethod(lambda: bool(state.get('abort')))})()
            state['abort'] = False
            if force and h.plan is not None and what in ('ROUGH', 'TEMP'):
                for it in h.plan.items(batch):
                    h.in_tray[it.slot] = it
                log('  (force：假定转盘里已经有这批物料)')
                if what == 'TEMP' and batch == 2:
                    for it in h.plan.items(1):
                        h.on_ring.setdefault(('TEMP', it.ring), []).append(it)
                    log('  (force：假定暂存区已经平放了第一批)')
            h.run_role(what, batch)
            log(f'mtest {what} {batch if what in ("RAW", "ROUGH", "TEMP") else ""} 结束：{h.stats.grab_text()}  {h.stats.place_text()}')
        return _run_async(state, log, go)

    raise ValueError('未知命令 ' + k)


COLOR_NAMES = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝'}


def _tools(h, link):
    """只准备指令通道、摄像头、视觉闭环(不初始化升降、不检查 ARMOK)。"""
    h._ensure_arm_only(link)
    h._ensure_vision()
    if getattr(h, 'servo', None) is None:
        from arm_link import ArmActuators
        from visual_servo import VisualServo, JacStore
        if getattr(h, 'store', None) is None:
            h.store = JacStore(h.cfg.get('servo_cal_file'))
        h.act = ArmActuators(h.arm, link, h.cfg['chassis_fine_rpm'], log=h.log)
        h.servo = VisualServo(h.act, h.store, cfg=h.cfg['servo'], log=h.log, sleep=h.sleep)
        if hasattr(h, 'sync_params'):
            h.sync_params()


def _gtest(h, link, color, nogo, log):
    name = COLOR_NAMES[color]
    _tools(h, link)
    P = h.arm.params(refresh=True)
    need = ['A1G', 'A2E', 'ZOBRAW', 'ZGRAB', 'ZHI']
    if any(n not in P for n in need):
        raise ValueError('STM32 里没有机械臂参数，是不是没烧带 arm.c 的正式程序？')
    if P.get('ARMOK', 0) < 0.5:
        raise ValueError('gtest 和比赛一样先 OBS RAW 摆到原料上方，要先把姿态标定好并 set ARMOK 1')
    known, mm = h.arm.lift_state()
    if not known:
        raise ValueError('升降位置不知道了(急停打断过？)：先把升降放到最低点，再输入 arm LIFT ZERO')
    log(f'夹取测试：{name}色物料。用的参数 A1G={P["A1G"]:g} A2E={P["A2E"]:g} ZOBRAW={P["ZOBRAW"]:g} ZGRAB={P["ZGRAB"]:g} ZHI={P["ZHI"]:g}')
    log('① 张开夹爪，手臂先升到最高再摆到原料上方(OBS RAW，和比赛时一样)')
    h.arm.obs('RAW', open_claw=True)
    _warn_blue(h, color, log)
    log(f'② 摄像头找{name}色物料……')
    still, last = h.vision.wait_still(color, timeout_s=h.cfg['raw_wait_s'])
    if last is None:
        ok = h.vision.save_debug('vdebug.png', 'RAW', color)
        raise ValueError(f'看不到{name}色物料。' + ('画面存到了 vdebug.png，看看物料在不在画面里、颜色认得对不对。' if ok else '摄像头没有画面？'))
    e0 = h.vision.material_error(color)
    if e0 is not None:
        md = h.vision.material_detector_obj() if hasattr(h.vision, 'material_detector_obj') else None
        res = getattr(md, 'last', None) if md is not None else None
        extra = f'，物料半径 {res["radius"]:.0f} 像素，被挡住 {res["occluded"] * 100:.0f}%' if res else ''
        log(f'   看到了：离爪子 {e0[0]:+.0f}, {e0[1]:+.0f} 像素(约 {(e0[0] ** 2 + e0[1] ** 2) ** 0.5 / h.vision.scale("RAW"):.1f}mm{extra}，'
            f'按 {h.vision.scale("RAW"):.2f} 像素/毫米)')
    log('③ 手臂对准(第一次会先小幅动几下，测出手臂和画面的对应关系)')
    res = h.servo.run('RAW', lambda: h.vision.material_error(color), h.vision.scale('RAW'), h.cfg['tol_mm']['RAW'],
                      allow_chassis=False, label=f'测试{name}', bounds=h.vision.bounds('RAW'), confirm=False)
    log(f'   对准结果：{res}')
    if h.store is not None and getattr(h.store, 'path', None):
        h.store.save()
    if nogo:
        log('nogo：只对准，不夹。看看爪子是不是在物料正上方。')
        return
    if not res.ok and res.err_mm > h.cfg['accept_mm']['RAW']:
        why = f'偏差 {res.err_mm:.1f}mm 太大' if math.isfinite(res.err_mm) else f'没对准({res.reason})'
        log(f'   ★ {why}，不夹(免得夹偏)。可以先 gtest {color} nogo 看看对准情况')
        return
    log('④ 下降、夹紧、抬起')
    h.arm.do(f'LIFT {P["ZGRAB"]:g}')
    h.arm.do('CLAW C')
    h.arm.do(f'LIFT {P["ZHI"]:g}')
    log('完成。夹起来了吗？没夹到：夹的位置太高就把 ZGRAB 调小(set ZGRAB 数字)，太低撞到就调大；夹偏了先用 nogo 看对准。松开：arm CLAW O')


def _warn_blue(h, color, log):
    """蓝色/浅蓝物料和蓝色爪子颜色一样：没做 vmask 时可能把爪子当物料，提醒一下。"""
    md = h.vision.material_detector_obj() if hasattr(h.vision, 'material_detector_obj') else None
    if md is not None and hasattr(md, 'need_vmask') and md.need_vmask(color):
        log(f'  ★ 找{COLOR_NAMES.get(color, color)}色物料，但还没标定爪子区域(claw_mask.png)：物料挨着蓝色爪子时可能认错。'
            '先做一次 vmask(手臂 OBS RAW O、爪子附近没有物料时输入 vmask)')


def _vmask(h, log):
    md = h.vision.material_detector_obj()
    if md is None:
        raise ValueError('现在用的不是 matdet 识别(detector=wuliao)，不用标定爪子区域')
    frames = []
    for _ in range(10):
        frames.append(h.vision._frame())
        h.sleep(0.05)
    from vision import load_vision_cal
    measured = 'RAW' in (load_vision_cal(h.cfg.get('vision_cal_file')).get('claw_px') or {})
    # 爪子点已经用 vclaw 实测过：它被算进爪子区域(爪子没张开/里面有蓝色物料)就不存。
    # 还没 vclaw 时爪子点只是配置里的估计值，可能本来就落在爪子上，不拿它检查
    res = md.save_claw_mask(frames, keep_clear=h.vision.claw('RAW') if measured else None)
    if res is None:
        raise ValueError('摄像头没有画面')
    path, frac = res
    log(f'爪子区域已存：{path}(占画面 {frac * 100:.0f}%)。用 python3 vlive.py 看：画面里变暗的部分就是爪子区域。')
    if frac < 0.03:
        log('  ★ 几乎没找到爪子：爪子不是蓝色，或者画面里看不到爪子。可以删掉 claw_mask.png，程序会每帧自动找')


def _measure_px(fn, n, log, what):
    pts = []
    for _ in range(n):
        p = fn()
        if p is not None:
            pts.append(p)
    if len(pts) < max(3, n // 2):
        raise ValueError(f'{what}：{n} 次里只认到 {len(pts)} 次，认不稳，没有保存。用 python3 vlive.py 看看画面')
    import numpy as np
    a = np.array(pts, float)
    med = np.median(a, axis=0)
    spread = float(np.max(np.abs(a - med)))
    return (float(med[0]), float(med[1])), spread, len(pts)


def _vclaw(h, link, kind, color, log):
    from vision import save_vision_cal
    h._ensure_arm_only(link)
    h._ensure_vision()
    P = h.arm.params(refresh=True)
    extra = {}
    if kind == 'RAW':
        need = ['ZGRAB', 'ZOBRAW']
        if any(n not in P for n in need):
            raise ValueError('STM32 里没有 ZGRAB/ZOBRAW 参数：是不是没烧新的 arm.c')
        known, _mm = h.arm.lift_state()
        if not known:
            raise ValueError('升降位置不知道了：先把升降放到最低点，再输入 arm LIFT ZERO')
        name = COLOR_NAMES.get(color, str(color))
        md = h.vision.material_detector_obj() if hasattr(h.vision, 'material_detector_obj') else None
        if md is not None and hasattr(md, 'need_vmask') and md.need_vmask(color):
            raise ValueError(f'{name}色物料和蓝色爪子颜色一样，没标定爪子区域会认错：先做 vmask(爪子张开、附近没有物料)，'
                             '或者换红/黄/绿/黑色物料做 vclaw RAW')
        if md is not None:                              # 物料半径重新量，不用以前存的(观察高度可能改过)
            md.r_ref.pop(color, None)
            md._r_hist.pop(color, None)
        h.arm.do(f'LIFT {P["ZOBRAW"]:g}')
        h.sleep(0.4)
        p0 = h.vision.material_px(color, n=5)
        if p0 is None:
            raise ValueError(f'看不到{name}色物料：先 arm OBS RAW O，把物料放在爪子正下方(原料盘要停住)')
        old = h.vision.claw('RAW')
        if math.hypot(p0[0] - old[0], p0[1] - old[1]) > 60:
            log(f'  注意：物料离现在的爪子点 {math.hypot(p0[0] - old[0], p0[1] - old[1]):.0f} 像素，'
                '如果下面夹的时候把物料推歪了，就把物料放得更靠近爪子中间再做一次')
        pts = []
        for k in range(2):                              # 夹正、松开、升回去测；做两遍，两遍要一致
            log(f'{"①②"[k]} 降到 ZGRAB={P["ZGRAB"]:g} 夹一下把{name}色物料夹正，松开，升回 ZOBRAW={P["ZOBRAW"]:g} 测圆心')
            h.arm.do('CLAW O')
            h.arm.do(f'LIFT {P["ZGRAB"]:g}')
            h.arm.do('CLAW C')
            h.sleep(0.5)
            h.arm.do('CLAW O')
            h.sleep(0.3)
            h.arm.do(f'LIFT {P["ZOBRAW"]:g}')
            h.sleep(0.4)
            pts.append(_measure_px(lambda: h.vision.material_px(color, n=3), 10, log, '测物料'))
        (u1, v1), _s1, _n1 = pts[0]
        uv, spread, n = pts[1]
        if math.hypot(uv[0] - u1, uv[1] - v1) > 2.5:
            raise ValueError(f'两遍测的不一致(差 {math.hypot(uv[0] - u1, uv[1] - v1):.1f} 像素)：物料被夹歪/推动了，没有保存。放正以后再做一次')
        r = (getattr(md, 'r_ref', {}) or {}).get(color) if md is not None else None
        last = getattr(md, 'last', None) if md is not None else None
        if r is None and last:
            r = last.get('radius')
        if r:
            extra['r_px'] = round(float(r), 2)
        extra['ZOBRAW'] = float(P['ZOBRAW'])
    else:
        known, _mm = h.arm.lift_state()
        if not known:
            raise ValueError('升降位置不知道了：先把升降放到最低点，再输入 arm LIFT ZERO')
        log('手臂回到圆环上方的观察姿态(OBS RING：和刚才 DROP 放物料时同一个 ID1/ID2 角度)')
        h.arm.obs('RING', open_claw=True)
        h.sleep(0.5)
        if 'ZOBRNG' in P:
            extra['ZOBRNG'] = float(P['ZOBRNG'])
        log('测圆环圆心(圆环纸已经按放下的物料摆正、物料已经拿走)……')
        any_size = h.vision.ring_px(n=3) is None and not h.vision.cfg.get('ring_rmax_px')
        if any_size:
            log('  按配置的大小没认到圆环，不限大小再找一次(第一次标定时圆环大小还不知道)')
        rmax = []

        def one():
            p = h.vision.ring_px(n=3, any_size=any_size)
            if p is not None and getattr(h.vision, 'last_ring_rmax', None):
                rmax.append(h.vision.last_ring_rmax)
            return p
        uv, spread, n = _measure_px(one, 12, log, '测圆环')
        if rmax:
            rmax.sort()
            r = rmax[len(rmax) // 2]
            extra['ring_rmax_px'] = round(float(r), 2)
            h.vision.cfg['ring_rmax_cal'] = float(r)
    old = h.vision.claw(kind)
    path = save_vision_cal(kind, uv, h.cfg.get('vision_cal_file') or None, extra)
    h.vision.cfg['claw_px'][kind] = [uv[0], uv[1]]
    log(f'爪子像素 claw_px.{kind} = ({uv[0]:.1f}, {uv[1]:.1f})  [原来 ({old[0]:.1f}, {old[1]:.1f})，'
        f'{n} 次测量最大跳动 {spread:.1f} 像素]，已存进 {path}')
    if extra.get('r_px'):
        log(f'  物料半径 {extra["r_px"]:.1f} 像素(物料被挡住时用它固定半径拟合，也用来算每毫米多少像素)')
    if extra.get('ring_rmax_px'):
        d = h.vision.cfg.get('ring_outer_diam_mm') or 95.0
        log(f'  圆环最外圈半径 {extra["ring_rmax_px"]:.1f} 像素 -> 每毫米 {h.vision.scale("RING"):.2f} 像素'
            f'(按最外圈直径 {d:g}mm 算；圆环不是这么大的话改 mission_cfg.ring_outer_diam_mm)')
    if spread > 3.0:
        log('  ★ 测量跳动比较大：物料/圆环可能没放稳，或者光线不好，可以再做一次')


def _vcal(h, kind, color, chassis, log):
    import numpy as np
    from visual_servo import ServoError
    h._ensure_vision()
    log(f'视觉校准 {kind}：先把手臂摆到观察姿态(张开夹爪)，请确认摄像头能看到' + ('圆环' if kind == 'RING' else f'颜色 {color} 的物料(要放稳不动)') + '……')
    h.arm.obs(kind, open_claw=True)
    if kind == 'RING':
        measure = h.vision.ring_error
    else:
        def measure():
            return h.vision.material_error(color)
    scale_cfg = h.vision.scale(kind)
    try:
        h.servo.dev = np.zeros(2)
        Ja = h.servo._probe(measure, 'arm')
        h.store.put(kind, 'arm', Ja)
        mm_ps = np.linalg.norm(Ja, axis=0) / scale_cfg
        log(f'  手臂：ID2 每转 1° ≈ {mm_ps[0]:.3f}mm(画面移动 {np.linalg.norm(Ja[:, 0]):.2f} 像素)；'
            f'ID1 每转 1° ≈ {mm_ps[1]:.3f}mm(画面移动 {np.linalg.norm(Ja[:, 1]):.2f} 像素)  [按 {scale_cfg:.3f} 像素/毫米换算]')
        if chassis:
            Jc = h.servo._probe(measure, 'ch')
            h.store.put(kind, 'ch', Jc)
            ps = np.linalg.norm(Jc, axis=0)
            log(f'  底盘：横移 1mm 画面移动 {ps[0]:.3f} 像素；前进 1mm 画面移动 {ps[1]:.3f} 像素')
            meas = float(ps.mean())
            diff = abs(meas - scale_cfg) / scale_cfg
            if kind == 'RING' and h.vision.cfg.get('ring_rmax_cal'):
                src, advice = '按 vclaw RING 量到的圆环大小', '圆环最外圈没认全，或者 ring_outer_diam_mm 不对：重做 vclaw RING'
            elif kind == 'RAW' and h.vision.cfg.get('material_diam_mm'):
                src, advice = '按物料半径', '检查 material_diam_mm(物料直径)，或者重做 vclaw RAW'
            else:
                src, advice = f'配置 px_per_mm.{kind}', f'建议把 mission_cfg.px_per_mm.{kind} 改成 {meas:.2f}'
            log(f'  实测比例 ≈ {meas:.3f} 像素/毫米(现在用的 {scale_cfg:.3f}，{src})' +
                ('' if diff < 0.1 else f'  → 相差 {diff * 100:.0f}%，{advice}'))
            if abs(ps[0] - ps[1]) / max(meas, 1e-9) > 0.2:
                log('  注意：横移和前进的比例相差超过 20%，可能底盘走的距离不准或摄像头有畸变')
        h.store.save()
        log('  校准结果已存进 ' + str(h.store.path) + '，以后对准直接用。' if h.store.path else '  (没有配置存储文件，校准结果只在这次运行里有效)')
    except ServoError as ex:
        raise ValueError(f'校准失败：{ex}')
