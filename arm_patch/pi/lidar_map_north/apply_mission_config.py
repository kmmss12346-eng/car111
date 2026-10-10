"""给 map_config_start2_roi.json 加上机械臂任务的配置 mission_cfg。

  python3 apply_mission_config.py                  加配置(默认启用)
  python3 apply_mission_config.py --disable        把 mission_cfg.enabled 设成 false(go 时不做夹取，只走路线)
  python3 apply_mission_config.py 别的配置文件.json

只加不改：你已经有的值一个都不动，只补上缺的；原文件自动备份成 .json.bak_mission。
"""
import json
import shutil
import sys
from pathlib import Path

from mission_hooks import DEFAULTS, deep_merge

NOTE = ('机械臂任务配置。claw_px=爪子轴线在画面里的位置(调它就能微调放置位置，1 像素≈0.34mm)；px_per_mm=每毫米多少像素；'
        'tol_mm=对准到多小算好；ring_offset_mm=圆环相对停车点沿车头方向的位置(要去现场核对圆环编号方向)；'
        'time_limit_s=超过这个时间不再做夹放，留时间回家。详见 README_mission.md')


# 改过默认值的项：配置里还是旧的默认值(说明没人改过)就换成新的。(None, 键) = mission_cfg 下面直接的键
OLD_DEFAULTS = {('px_per_mm', 'RAW'): (4.36, 1.97), ('tol_mm', 'RAW'): (3.0, 2.0), ('accept_mm', 'RAW'): (6.0, 4.0),
                (None, 'learn_pick'): (True, False),          # 10-10：放完不再回去拍物料(慢)，取回认圆环外圈
                (None, 'chassis_fine_rpm'): (60, 100),        # 10-10：视觉微调时底盘快一点
                ('tol_mm', 'RING'): (1.0, 2.0), ('tol_mm', 'STACK'): (2.0, 2.5),   # 10-10：差不多就行，不为零点几毫米反复微调
                ('accept_mm', 'RING'): (2.5, 3.0)}


def main(argv):
    disable = '--disable' in argv
    paths = [a for a in argv if not a.startswith('--')]
    path = Path(paths[0]) if paths else Path(__file__).resolve().parent / 'map_config_start2_roi.json'
    if not path.exists():
        print(f'找不到 {path}')
        return 1
    cfg = json.loads(path.read_text(encoding='utf-8'))
    old = cfg.get('mission_cfg') or {}
    new = deep_merge(DEFAULTS, old)          # DEFAULTS 打底，已有的值覆盖它(不会改你已经设的)
    for (sec, k), (was, now) in OLD_DEFAULTS.items():     # 以前的默认值、没改过的：换成新的默认值
        cur = old.get(k, None) if sec is None else (old.get(sec) or {}).get(k)
        if cur is not None and type(cur) is type(was) and cur == was:
            if sec is None:
                new[k] = now
            else:
                new[sec][k] = now
            print(f'  {(sec + ".") if sec else ""}{k}：旧的默认值 {was} -> 新的默认值 {now}')
    if disable:
        new['enabled'] = False
    elif 'enabled' not in old:
        new['enabled'] = True
    bak = path.with_suffix('.json.bak_mission')
    if not bak.exists():
        shutil.copyfile(path, bak)
    cfg['mission_cfg'] = new
    cfg['_mission_cfg_note'] = NOTE
    path.write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding='utf-8')
    added = [k for k in new if k not in old]
    print(f'已写入 {path}（原文件备份在 {bak.name}）')
    print(f'  mission_cfg.enabled = {new["enabled"]}')
    print('  新增的项：' + (', '.join(added) if added else '(没有，都已经有了)'))
    stops = cfg.get('mission')
    print(f'  路线(mission)：{stops}')
    if stops and sum(1 for s in stops if str(s).upper().startswith('RAW')) > 1:
        print('  提示：路线里有两批。时间紧的话，先只做第一批：把 "mission" 改成 ["QR","RAW","ROUGH","TEMP","START"]，稳了再加第二批。')
    print('下一步：看 README_mission.md 的上车测试步骤(物料颜色用 wuliao.py 里的范围，没有就用 matdet.py 里一样的默认值)。')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
