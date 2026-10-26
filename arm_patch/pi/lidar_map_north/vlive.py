"""摄像头实时预览：画面上直接画出识别结果，看识别效果用(不动机械臂、不连 STM32)。

在树莓派桌面(VNC)的终端里运行：

    cd ~/lidar_map_north
    python3 vlive.py            # 默认找红色物料(1)
    python3 vlive.py 3          # 一开始就找蓝色

窗口里按键(先用鼠标点一下画面窗口)：
    1~6   换颜色：1 红 2 黄 3 蓝 4 绿 5 黑 6 浅蓝
    r     切换：找物料 / 找地上圆环
    m     标定爪子区域(存 claw_mask.png)：爪子张开、爪子附近没有物料时按
    k     (10-24)现场标定现在找的这个颜色：先按 1~6 选颜色，把这个颜色的物料放到绿十字正下方(爪子别挡住)再按。
          按画面里它实际的颜色定范围，存进 vision_cal.json(color_ranges)，马上生效，比赛时也用它。光线换了(到比赛现场)再标一次
    0     现在这个颜色恢复默认的范围(删掉 k 标定的)
    s     存一张当前画面到 vlive.png
    q     退出

画面上：
    绿十字   (找物料时)原料这边的爪子位置 claw_px.RAW(vision_cal.json 优先，没有就用配置里的 mission_cfg.claw_px)。
             arm_calib 看原料的那几步里输入 g 左右 上下 挪它
    红十字   (10-24 找圆环时)粗加工区、暂存区这边的爪子位置 claw_px.RING：比赛时 2 号环中心对到这里。和原料的绿十字分开存、互不影响，
             arm_calib 看圆环的那几步(2 号环、ZPLC、ZOBRNG)里输入 g 左右 上下 挪它。vision_cal.json 一变这里马上重新读
    橙十字 / 蓝十字  (10-26)--ringno 1 / --ringno 3(arm_calib 的 R1、R3 两步)：1、3 号环自己的十字 claw_px.RING1 / RING3。
             比赛时手臂转到调的 1、3 号环角度以后，摄像头只动手臂把这个环的中心对到它上再放；没调过就和红十字一样。
             在那两步里输入 g 左右 上下 挪它(主程序里 g R1 左右 上下 / g R3 左右 上下)，三个十字互不影响
    红斜十字 + 青色圆  识别到的物料圆心和拟合出来的整个圆(被爪子挡住的部分也画出来)
    黄圈     识别到的圆环(每一圈都画出来，被爪子挡住的部分也画)；灰色细圈 = 认到了但大小不像圆环，被过滤掉了
             (vclaw RING 量过以后按量到的大小过滤；左上角 rmax 是最外圈半径像素)
    变暗的区域  爪子(识别时不算)
    左上角   偏差：目标离爪子多少像素、约多少毫米，物料被挡住多少，每秒处理几帧

摄像头同一时间只能被一个程序打开：map_merge_live 里用过 vdbg / gtest / rtest / mtest 的话，
先在 map_merge_live 里输入 q 退出(或者输入 mtest reset 让它放开摄像头)，再运行这个。
想在 map_merge_live 做 rtest / gtest / mtest 的同时看画面，用 python3 vview.py(不用退出 map_merge_live)。
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

COLOR_NAMES = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝'}
COLOR_EN = {1: 'red', 2: 'yellow', 3: 'blue', 4: 'green', 5: 'black', 6: 'light blue'}


def load_vision_cfg(cfg_path):
    """和 map_merge_live 用同一份配置：mission_cfg 里的 camera / claw_px / px_per_mm(没写的用默认值)。"""
    from mission_hooks import DEFAULTS, deep_merge, vision_cfg
    raw = {}
    if cfg_path is not None and Path(cfg_path).exists():
        raw = json.loads(Path(cfg_path).read_text(encoding='utf-8'))
    cfg = deep_merge(DEFAULTS, raw.get('mission_cfg'))
    vc = vision_cfg(cfg)
    from vision import apply_vision_cal
    if cfg.get('vision_cal_file'):
        apply_vision_cal(vc, cfg['vision_cal_file'])
    return vc


def _reload_ranges(vis):
    """vision_cal.json 里的颜色范围变了：正在用的物料识别马上换成新的。"""
    md = vis.material_detector_obj()
    if md is not None and hasattr(md, 'set_ranges'):
        md.set_ranges((vis.cfg.get('matdet') or {}).get('ranges'))


def calibrate_color_key(vis, color, cal_p, reset=False):
    """(10-24)按 k：现场标定 color 这个颜色(物料放在绿十字正下方；认得到时按认到的圆心)；按 0：恢复默认范围。"""
    from vision import apply_vision_cal, save_color_range
    import matdet
    md = vis.material_detector_obj()
    if md is None:
        print('现在用的不是 matdet 识别，不能标定颜色', flush=True)
        return
    name = COLOR_NAMES.get(color, str(color))
    if reset:
        p = save_color_range(color, None, path=str(cal_p) if cal_p else None)
        apply_vision_cal(vis.cfg, str(p))
        _reload_ranges(vis)
        print(f'{name}色恢复默认范围：{md.ranges.get(color)}', flush=True)
        return
    fr = None
    for _ in range(5):
        f = vis.camera.read()
        fr = f if f is not None else fr
    if fr is None:
        print('没有画面，没标定', flush=True)
        return
    last = md.detect(fr, color)
    if last is not None:
        center, r = last['center'], 0.6 * float(last['radius'])
        where = '认到的物料圆心'
    else:
        r0 = md.r_ref.get(color) or vis.cfg.get('r_px_seed') or 40.0
        center, r = vis.claw('RAW'), 0.55 * float(r0)
        where = '绿十字'
    old = md.ranges.get(color)
    try:
        res = matdet.calibrate_color(fr, color, center, r, claw=vis.claw_mask(fr))
    except ValueError as ex:
        print(f'没标定：{ex}', flush=True)
        return
    p = save_color_range(color, res['ranges'], path=str(cal_p) if cal_p else None)
    apply_vision_cal(vis.cfg, str(p))
    _reload_ranges(vis)
    ok = md.detect(fr, color) is not None
    print(f'{name}色标定好了(按{where}周围 {r:.0f} 像素取的颜色)：{res["text"]}\n'
          f'  原来的范围 {old}\n  新的范围 {res["ranges"]}\n'
          f'  物料上 {res["keep"] * 100:.0f}% 的点算{name}色，画面别处 {res["fp"] * 100:.1f}% 也算{name}色'
          + ('(太多了：光线、转盘颜色和它太像，认的时候可能认到别处)' if res['fp'] > 0.03 else '')
          + f'；这一帧{"认出来了" if ok else "还是没认出来(物料在不在绿十字下面？)"}。已存进 {p}', flush=True)


def _cal_file(cfg_path):
    """(10-23)这份配置用的 vision_cal.json 的完整路径(配置里 vision_cal_file 是空的 = 不用这个文件，返回 None)。"""
    from mission_hooks import DEFAULTS, deep_merge
    from vision import _cal_path
    raw = {}
    if cfg_path is not None and Path(cfg_path).exists():
        raw = json.loads(Path(cfg_path).read_text(encoding='utf-8'))
    name = deep_merge(DEFAULTS, raw.get('mission_cfg')).get('vision_cal_file')
    return _cal_path(name) if name else None


def _mtime(p):
    try:
        return os.path.getmtime(p) if p is not None else None
    except OSError:
        return None


def claw_shift(cfg_path, du, dv, kind='RAW'):
    """(10-23 用户"把这个绿色的十字搞成可移动的，可以根据我输入的数字改变这个绿色十字的位置")绿十字(爪子点 claw_px)挪 (du, dv) 像素，
    往右、往下为正，存进 vision_cal.json(只改这一项，别的不动)。开着的 vlive 发现文件变了会重新读，画面里的绿十字马上跟着动。
    返回 (原来的 (u, v), 现在的 (u, v), 文件路径)。挪出画面就不存，抛 ValueError。"""
    from vision import Vision, save_vision_cal
    path = _cal_file(cfg_path)
    if path is None:
        raise ValueError('配置里 vision_cal_file 是空的(不用 vision_cal.json)，绿十字没地方存')
    vc = load_vision_cfg(cfg_path)
    old = Vision(vc).claw(kind)
    new = (old[0] + float(du), old[1] + float(dv))
    cam = vc.get('camera') or {}
    w, h = float(cam.get('width', 640) or 640), float(cam.get('height', 480) or 480)
    if not (0.0 <= new[0] <= w and 0.0 <= new[1] <= h):
        raise ValueError(f'挪到 ({new[0]:.0f}, {new[1]:.0f}) 就出画面了(画面 {w:.0f}×{h:.0f})')
    save_vision_cal(kind, new, path)
    return old, new, path


def annotate(vis, fr, mode, color, ring_no=2):
    """在一帧上画出爪子、识别结果和偏差，返回 (画好的图, 一行文字说明)。
    (10-24)找物料时爪子点(claw_px.RAW)画绿十字，找圆环时爪子点(claw_px.RING)画红十字；偏差都按离这个十字算。
    (10-26)ring_no = 1 / 3：按 1、3 号环自己的十字(claw_px.RING1 / RING3，橙色 / 蓝色)算偏差。"""
    import cv2
    from vision import cross_color
    out = fr.copy()
    kind = 'RAW' if mode == 'material' else ('RING' if ring_no not in (1, 3) else f'RING{ring_no}')
    cu, cv_ = vis.claw(kind)
    ru, rv = cu, cv_
    target = None
    extra = ''
    if mode == 'material':
        p = vis._material_det()(fr, color)
        md = vis.material_detector_obj()
        claw = getattr(md, 'last_claw', None) if md is not None else None
        if claw is not None and claw.any():
            out[claw > 0] = (out[claw > 0] * 0.45).astype(out.dtype)          # 爪子区域变暗
        if p is not None:
            target = p
            res = getattr(md, 'last', None) if md is not None else None
            if res is not None:
                cv2.circle(out, (int(round(p[0])), int(round(p[1]))), int(round(res['radius'])), (255, 255, 0), 2)
                extra = f'  r={res["radius"]:.0f}px hidden {res["occluded"] * 100:.0f}%'
            cv2.drawMarker(out, (int(p[0]), int(p[1])), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 30, 2)
        what = f'material {color} ({COLOR_EN[color]})'
        if md is not None and hasattr(md, 'need_vmask') and md.need_vmask(color):
            what += ' [no claw mask: press m]'
    else:
        rings = vis._rings_in_frame(fr)
        claw = getattr(vis, 'last_claw', None)
        if claw is not None and claw.any():
            out[claw > 0] = (out[claw > 0] * 0.45).astype(out.dtype)          # 爪子区域变暗(这里的边不用)
        det = vis._ring_det()
        found = {(round(r[0], 1), round(r[1], 1)) for r in rings}
        for d in getattr(det, 'last', None) or []:                          # 认到了但大小不对、被过滤掉的：灰色细圈
            x, y = d['center']
            if (round(x, 1), round(y, 1)) not in found:
                cv2.circle(out, (int(round(x)), int(round(y))), int(round(d['r_max'])), (160, 160, 160), 1)
                cv2.putText(out, f'r={d["r_max"]:.0f} (size?)', (int(x) + 6, int(y) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        best = min(rings, key=lambda r: (r[0] - ru) ** 2 + (r[1] - rv) ** 2) if rings else None
        for ring in rings:
            x, y = ring[0], ring[1]
            col = (0, 255, 255) if ring is best else (0, 160, 160)
            info = next((d for d in (getattr(det, 'last', None) or []) if abs(d['center'][0] - x) < 0.05 and abs(d['center'][1] - y) < 0.05), None)
            for r in (info['radii'] if info else [ring[2]]):                  # 每一圈都画出来(被挡住的部分也画)
                cv2.circle(out, (int(round(x)), int(round(y))), int(round(r)), col, 1)
            cv2.drawMarker(out, (int(round(x)), int(round(y))), col, cv2.MARKER_CROSS, 16, 2)
            if ring is best and info:
                extra = f'  rmax={info["r_max"]:.0f}px {info["n_circles"]} circles seen {info["coverage"] * 100:.0f}%'
        if best is not None:
            target = (best[0], best[1])
        what = f'rings: {len(rings)}' + (f' (ring {ring_no} cross)' if kind != 'RING' else '')
    cv2.drawMarker(out, (int(round(cu)), int(round(cv_))), cross_color(kind), cv2.MARKER_CROSS, 30, 2)
    if kind not in ('RAW', 'RING'):
        cv2.putText(out, f'R{ring_no}', (int(round(cu)) + 8, int(round(cv_)) + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, cross_color(kind), 2)
    if target is None:
        msg = f'{what}: NOT FOUND'
    else:
        du, dv = target[0] - ru, target[1] - rv
        mm = (du * du + dv * dv) ** 0.5 / vis.scale(kind)
        msg = f'{what}: off {du:+.0f},{dv:+.0f} px  ~{mm:.1f} mm{extra}'
        cv2.line(out, (int(round(ru)), int(round(rv))), (int(target[0]), int(target[1])), (255, 0, 255), 1)
    return out, msg


def main(argv=None):
    ap = argparse.ArgumentParser(description='摄像头实时预览识别效果')
    ap.add_argument('color', nargs='?', type=int, default=1, help='颜色号 1红 2黄 3蓝 4绿 5黑 6浅蓝')
    ap.add_argument('--ring', action='store_true', help='一开始就找地上圆环')
    ap.add_argument('--ringno', type=int, choices=(1, 2, 3), default=2,
                    help='(10-26)找圆环时偏差按几号环的十字算：2 = 红十字；1、3 = 这两个环自己的十字(arm_calib 的 R1、R3 用)')
    ap.add_argument('--config', default=None, help='默认：run_live.sh 里的 --config，没有就用 map_config_start2_roi.json')
    args = ap.parse_args(argv)
    if args.color not in COLOR_NAMES:
        ap.error('颜色号只能是 1~6')

    import cv2
    from vision import Vision, VisionError
    try:
        from arm_calib import config_from_run_live
        cfg_path = Path(args.config) if args.config else (config_from_run_live() or ROOT / 'map_config_start2_roi.json')
    except Exception:
        cfg_path = Path(args.config) if args.config else ROOT / 'map_config_start2_roi.json'

    vis = Vision(load_vision_cfg(cfg_path))
    try:
        vis.open()
    except VisionError as ex:
        print(f'{ex}。map_merge_live 里用过 vdbg / gtest / mtest 的话，先把 map_merge_live 关掉再运行这个。')
        return 1
    print(f'配置：{cfg_path}')
    print(f'爪子位置：原料(绿十字) {vis.claw("RAW")}  圆环(红十字) {vis.claw("RING")}  '
          f'1 号环(橙十字) {vis.claw("RING1")}  3 号环(蓝十字) {vis.claw("RING3")}')
    print('按键：1~6 换颜色，r 切换物料/圆环，k 现场标定这个颜色(物料放在绿十字下面)，0 这个颜色恢复默认，s 存图，q 退出(先用鼠标点一下画面窗口)')

    mode = 'ring' if args.ring else 'material'
    color = args.color
    t_last, fps, last_msg = time.monotonic(), 0.0, ''
    try:
        cal_p = _cal_file(cfg_path)
    except Exception:
        cal_p = None
    cal_mt, t_cal = _mtime(cal_p), 0.0
    try:
        while True:
            if cal_p is not None and time.monotonic() - t_cal >= 0.5:
                # (10-23)arm_calib 里 g 挪了绿十字(改了 vision_cal.json)：重新读，画面里马上跟着动
                t_cal = time.monotonic()
                mt = _mtime(cal_p)
                if mt != cal_mt:
                    cal_mt = mt
                    from vision import apply_vision_cal
                    apply_vision_cal(vis.cfg, str(cal_p))
                    _reload_ranges(vis)
                    print(f'爪子位置更新了：原料(绿十字) {vis.claw("RAW")}  圆环(红十字) {vis.claw("RING")}  '
                          f'1 号环(橙十字) {vis.claw("RING1")}  3 号环(蓝十字) {vis.claw("RING3")}', flush=True)
            fr = vis.camera.read()
            if fr is None:
                time.sleep(0.05)
                continue
            try:
                out, msg = annotate(vis, fr, mode, color, ring_no=args.ringno)
            except VisionError as ex:
                out, msg = fr, f'ERROR: {ex}'
            now = time.monotonic()
            dt = now - t_last
            t_last = now
            if dt > 0:
                fps = 0.8 * fps + 0.2 * (1.0 / dt) if fps else 1.0 / dt
            cv2.putText(out, f'{msg}   {fps:.1f} fps', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(out, f'{msg}   {fps:.1f} fps', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            try:
                cv2.imshow('vlive  (1-6 color, k calib color, r ring/material, m claw mask, s save, q quit)', out)
            except cv2.error:
                print('显示不了窗口：要在树莓派桌面(VNC)里打开的终端运行，不能用 ssh；'
                      '或者装的是不带界面的 opencv(opencv-python-headless)。', flush=True)
                return 1
            if msg != last_msg:
                print(msg, flush=True)
                last_msg = msg
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if ord('1') <= key <= ord('6'):
                color, mode = key - ord('0'), 'material'
                print(f'→ 找{COLOR_NAMES[color]}色物料', flush=True)
            elif key == ord('r'):
                mode = 'ring' if mode == 'material' else 'material'
                print('→ 找地上圆环' if mode == 'ring' else f'→ 找{COLOR_NAMES[color]}色物料', flush=True)
            elif key == ord('m'):
                md = vis.material_detector_obj()
                if md is None:
                    print('现在用的不是 matdet 识别，不用标定爪子区域', flush=True)
                else:
                    frs = [vis.camera.read() for _ in range(10)]
                    try:
                        from vision import load_vision_cal
                        measured = 'RAW' in (load_vision_cal().get('claw_px') or {})     # 没 vclaw 过就不按爪子点检查
                        res = md.save_claw_mask(frs, keep_clear=vis.claw('RAW') if measured else None)
                        print('没有画面，没存' if res is None else f'爪子区域已存：{res[0]}(占画面 {res[1] * 100:.0f}%)', flush=True)
                    except ValueError as ex:
                        print(f'没存：{ex}', flush=True)
            elif key == ord('s'):
                cv2.imwrite(str(ROOT / 'vlive.png'), out)
                print(f'已存：{ROOT / "vlive.png"}', flush=True)
            elif key in (ord('k'), ord('0')):
                if mode != 'material':
                    print('先按 1~6 选要标定的颜色(现在是找圆环)', flush=True)
                else:
                    calibrate_color_key(vis, color, cal_p, reset=(key == ord('0')))
                    cal_mt = _mtime(cal_p)                   # 自己改的 vision_cal.json：不当成别处改的再读一遍
    except KeyboardInterrupt:
        pass
    finally:
        vis.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
