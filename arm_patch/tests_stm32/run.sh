#!/bin/bash
# 在电脑上编译并运行 STM32 代码的回归测试(不需要 Keil/开发板)。要有 gcc。
#   test_arm.c     机械臂 arm.c
#   test_chassis.c 底盘 chassis.c：转弯卡住检测(ERR STALL)、驱动器诊断(MOT? / MOT EN)、
#                  v13 的转弯末段/横移前馈/精确模式/急停/R 带小数的解析/新参数(假车带陀螺仪噪声和 x/y 里程)
#   test_servo.c   main.c 里的舵机：卡住时停在原地(不再顶着发热)、太慢时多等、SV? 状态
set -e
cd "$(dirname "$0")"
OUT="${TMPDIR:-/tmp}"
gcc -std=gnu90 -Wall -Wextra -Wdeclaration-after-statement -Istub -I../Core/Inc -o "$OUT/test_arm_host" ../Core/Src/arm.c test_arm.c -lm 2>&1 | grep -v "test_arm.c" || true
"$OUT/test_arm_host"
echo
gcc -std=gnu90 -Wall -Wextra -Wdeclaration-after-statement -Istub_ch -I../Core/Inc -o "$OUT/test_chassis_host" ../Core/Src/chassis.c test_chassis.c -lm 2>&1 | grep -v "test_chassis.c" || true
"$OUT/test_chassis_host"
echo
# main.c 里舵机那一节(从 "ID 1、ID 2 串口舵机" 到 "原来的接口")摘出来单独测
tr -d '\r' < ../Core/Src/main.c | awk '/^\/\* ================= ID 1、ID 2 串口舵机/{f=1} /^\/\* 原来的接口：转到绝对角度/{f=0} f' > "$OUT/servo_section.c"
gcc -std=gnu90 -Wall -Wextra -I"$OUT" -o "$OUT/test_servo_host" test_servo.c -lm 2>&1 | grep -v "test_servo.c" || true
"$OUT/test_servo_host" | grep -v "^  ok\|^SV "
