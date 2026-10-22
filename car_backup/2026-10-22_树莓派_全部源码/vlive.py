"""摄像头实时预览：画面上直接画出识别结果，看识别效果用(不动机械臂、不连 STM32)。

在树莓派桌面(VNC)的终端里运行：

    cd ~/lidar_map_north
    python3 vlive.py            # 默认找红色物料(1)
    python3 vlive.py 3          # 一开始就找蓝色

窗口里按键(先用鼠标点一下画面窗口)：
    1~6   换颜色：1 红 2 黄 3 蓝 4 绿 5 黑 6 浅蓝
    r     切换：找物料 / 找地上圆环
    m     标定爪子区域(存 claw_mask.png)：爪子张开、爪子附近没有物料时按
    s     存一张当前画面到 vlive.png
    q     退出

画面上：
    绿十字   爪子位置(vclaw 实测的 vision_cal.json 优先，没有就用配置里的 mission_cfg.claw_px)
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


def annotate(vis, fr, mode, color):
    """在一帧上画出爪子、识别结果和偏差，返回 (画好的图, 一行文字说明)。"""
    import cv2
    out = fr.copy()
    kind = 'RAW' if mode == 'material' else 'RING'
    cu, cv_ = vis.claw(kind)
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
        best = min(rings, key=lambda r: (r[0] - cu) ** 2 + (r[1] - cv_) ** 2) if rings else None
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
        what = f'rings: {len(rings)}'
    cv2.drawMarker(out, (int(cu), int(cv_)), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
    if target is None:
        msg = f'{what}: NOT FOUND'
    else:
        du, dv = target[0] - cu, target[1] - cv_
        mm = (du * du + dv * dv) ** 0.5 / vis.scale(kind)
        msg = f'{what}: off {du:+.0f},{dv:+.0f} px  ~{mm:.1f} mm{extra}'
        cv2.line(out, (int(cu), int(cv_)), (int(target[0]), int(target[1])), (255, 0, 255), 1)
    return out, msg


def main(argv=None):
    ap = argparse.ArgumentParser(description='摄像头实时预览识别效果')
    ap.add_argument('color', nargs='?', type=int, default=1, help='颜色号 1红 2黄 3蓝 4绿 5黑 6浅蓝')
    ap.add_argument('--ring', action='store_true', help='一开始就找地上圆环')
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
    print(f'爪子位置(绿十字)：原料 {vis.claw("RAW")}  圆环 {vis.claw("RING")}')
    print('按键：1~6 换颜色，r 切换物料/圆环，s 存图，q 退出(先用鼠标点一下画面窗口)')

    mode = 'ring' if args.ring else 'material'
    color = args.color
    t_last, fps, last_msg = time.monotonic(), 0.0, ''
    try:
        while True:
            fr = vis.camera.read()
            if fr is None:
                time.sleep(0.05)
                continue
            try:
                out, msg = annotate(vis, fr, mode, color)
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
                cv2.imshow('vlive  (1-6 color, r ring/material, m claw mask, s save, q quit)', out)
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
    except KeyboardInterrupt:
        pass
    finally:
        vis.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
