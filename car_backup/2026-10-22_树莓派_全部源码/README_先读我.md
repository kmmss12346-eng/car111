# 车上树莓派 ~/lidar_map_north 的全部源码(2026-10-22)

用户在车上用 find 只挑源码和配置(.py .sh .cpp .h .c .txt .json .md，单个 2M 以下)打的包，原样放在这里，**只是备份**。

和仓库 `arm_patch/pi/lidar_map_north/` 比(10-22 a06ca37)：
- mission_hooks.py、mission_cli.py、arm_link.py、arm_calib.py、vision.py、matdet.py、map_merge_live.py、route_plan.py 等比赛用的程序**和仓库一样**(用户装好了 order_1022b)。
- 不一样的只有测试/模拟(test_*.py、sim_mission.py)和 apply_mission_config.py、qr_claw_test.py(车上是旧的，不影响比赛)。
- **只在车上有的(仓库 arm_patch 里没有)：**
  - 雷达：`build.sh`、`scan_bridge.cpp`、`vendor/ldlidar_driver/`(乐动 LD14P 驱动源码；build.sh 编译出 run_live.sh 要用的 scan_bridge)、`ld14p_scan.py`
  - 建图/定位：`lidar_map_live.py`(和两个备份)、`pose_fix.py`、`merge_stations.py`、`reference_correction.py`、`home_fix.py`、`obstacles.py`、
    `field_map_stage1.py`、`field_map_stage2.py`、`calibrate_field.py`、`apply_v10_config.py`
  - 通信：`stm32_link.py`(树莓派 <-> STM32 串口)、`check_pc10_pc11.py`
  - 摄像头：`cam_side.py`、`cam_side_cal.py`、`cam_side_test.py`、`cam_dist_cal.py`、`ring_detect.py`(旧的认圆环)、`wuliao.py`(调过的物料颜色范围，matdet.py 优先用它)
  - 模拟：`sim_lidar.py`、`sim_run.py`
  - `run_live.sh`、各版本的"先读我"说明(v9~v16)、`backup_*` 旧版本
