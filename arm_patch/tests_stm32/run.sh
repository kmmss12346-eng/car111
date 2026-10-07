#!/bin/bash
# 在电脑上编译并运行 STM32 代码的回归测试(不需要 Keil/开发板)。要有 gcc。
#   test_arm.c     机械臂 arm.c
#   test_chassis.c 底盘 chassis.c：转弯卡住检测(ERR STALL)、驱动器诊断(MOT? / MOT EN)
set -e
cd "$(dirname "$0")"
OUT="${TMPDIR:-/tmp}"
gcc -std=gnu90 -Wall -Wextra -Wdeclaration-after-statement -Istub -I../Core/Inc -o "$OUT/test_arm_host" ../Core/Src/arm.c test_arm.c -lm 2>&1 | grep -v "test_arm.c" || true
"$OUT/test_arm_host"
echo
gcc -std=gnu90 -Wall -Wextra -Wdeclaration-after-statement -Istub_ch -I../Core/Inc -o "$OUT/test_chassis_host" ../Core/Src/chassis.c test_chassis.c -lm 2>&1 | grep -v "test_chassis.c" || true
"$OUT/test_chassis_host"
