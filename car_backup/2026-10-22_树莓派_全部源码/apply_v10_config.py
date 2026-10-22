#!/usr/bin/env python3
"""把 v10 需要的新配置项写进你现在的 map_config_start2_roi.json（保留你原来的雷达标定、障碍物识别参数等所有设置）。
用法：python3 apply_v10_config.py [配置文件路径]      原文件会先备份成 .json.bak_v10
只改下面 NEW 里列出的键；second_pose_mm(旧的实测第二站位置，是按旧的转弯方式量的)会改名成 _second_pose_mm_old 不再使用。
"""
import json
import shutil
import sys
from pathlib import Path

NEW = {
    # 转弯现在绕车中心(STM32 里转的同时叠加平移)，规划按转轴在中心算
    'turn_pivot_back_mm': 0,
    'turn_pivot_left_mm': 0,
    'turn_margin_mm': 20,
    # 第二站动作：树莓派逐条发 S(左移) F(前进) R(顺时针转)。数值保证车停在之前实测的第二站位置(2208,233)附近
    'second_relative_moves': {'left_mm': 42, 'forward_mm': 83, 'cw_deg': 60},
    'second_use_gyro': True,
    # 规划：按用时算代价；横移不再限制长度
    'strafe_cost_factor': 1.5,
    'strafe_max_mm_per_leg': None,
    'clearance_pref_mm': 120,
    'clearance_penalty': 1.0,
    # STM32 的运动快慢(规划估时间、路线行驶发给 STM32 的速度都用它；STM32 里的默认值也是这些)
    'route_speed_rpm': 220,
    'route_acc_rpm_s': 300,
    'strafe_speed_rpm': 170,
    'strafe_acc_rpm_s': 200,
    'fwd_mm_per_rev': 249.0,
    'strafe_mm_per_rev': 235.0,
    'turn90_s': 1.8,
    'turn180_s': 2.5,
    'cmd_overhead_s': 0.25,
}
DEFAULTS_IF_MISSING = {'stm32_params': {}}


def main():
    p = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parent / 'map_config_start2_roi.json')
    cfg = json.loads(p.read_text(encoding='utf-8'))
    bak = p.with_suffix('.json.bak_v10')
    if not bak.exists():
        shutil.copyfile(p, bak)
    changed = []
    for k, v in NEW.items():
        if cfg.get(k, '<none>') != v:
            changed.append(f'  {k}: {cfg.get(k, "(没有)")!r} -> {v!r}')
        cfg[k] = v
    for k, v in DEFAULTS_IF_MISSING.items():
        cfg.setdefault(k, v)
    if 'second_pose_mm' in cfg:
        cfg['_second_pose_mm_old'] = cfg.pop('second_pose_mm')
        changed.append('  second_pose_mm -> 改名 _second_pose_mm_old（旧转弯方式量的，不再使用；第二站位置按动作+陀螺仪推算）')
    cfg['_v10_note'] = ('v10：转弯绕车中心；第二站=左移42/前进83/顺时针60(车停在(2208,233)，和之前实测的位置一样)；'
                        '路线按用时规划；运动参数用 set/fix/cal 命令改，存在 stm32_params 里，每次启动自动发给 STM32。')
    p.write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding='utf-8')
    print(f'已更新 {p}（原文件备份在 {bak.name}）。改动：')
    print('\n'.join(changed) if changed else '  (没有变化，已经是 v10 配置)')


if __name__ == '__main__':
    main()
