#!/bin/bash
# 一条命令跑完所有测试：STM32 的 arm.c(主机编译) + 树莓派端全部模块 + 整场模拟。需要 gcc、python3、numpy、opencv。
cd "$(dirname "$0")"
echo "===== STM32 arm.c 主机回归测试 ====="
bash tests_stm32/run.sh || exit 1
echo
echo "===== 树莓派端测试(任务规划、视觉闭环、识别、命令行、整场模拟) ====="
cd pi/lidar_map_north && python3 -m unittest test_task_plan test_visual_servo test_vision test_mission_cli sim_mission 2>&1 | tail -30
