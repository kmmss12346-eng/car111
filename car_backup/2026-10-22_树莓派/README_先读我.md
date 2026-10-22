# 车上树莓派文件备份(2026-10-22)

用户从车上 `/home/zstu/lidar_map_north/` 打包发来的(`car_backup.tar.gz`)，原样放在这里，**只是备份，不是仓库里正在用的代码**。
要恢复到车上：把需要的文件 scp 回 `/home/zstu/lidar_map_north/`(先备份车上现有的)。

| 文件 | 是什么 |
|---|---|
| map_config_start2_roi.json | 车上正在用的配置(run_live 用它)：停车点、路线、STM32 参数(stm32_params)、机械臂任务配置(mission_cfg)。10-22 07:03 存的 |
| map_config_start2_initial.json、map_config_before_*.json | 以前的配置备份 |
| servo_cal.json | vcal 测的手臂/车轮和画面的对应关系(RING、RAW 各 arm/ch) |
| ring_teach.json | 原料区"最近那格"示教(RAW_CT)、工位 1/3 号环角度 |
| place_adj.json | 以前 pcal 量的放置补偿(10-20 起比赛不用) |
| lidar_map_live.py、ld14p_scan.py、pose_fix.py、merge_stations.py、reference_correction.py、stm32_link.py、home_fix.py | 只在车上有的程序(仓库里没有) |
| run_live.sh | 启动脚本 |

**车上没有的(打包时 Cannot stat)：** vision_cal.json(没做过 vclaw RING / vclaw RAW)、claw_mask.png(没做过 vmask)。

备注：配置里 mission_cfg.place_order 是 "sweep"(10-10 apply_mission_config 写进去的)，10-22 起程序已经不用它(一律按任务码顺序放)；
ring2_off_mm 是 F=0、S=0(还没设故意对偏)。
