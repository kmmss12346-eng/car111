#!/bin/bash
# 在电脑上编译并运行 arm.c 的回归测试(不需要 Keil/开发板)。要有 gcc。
set -e
cd "$(dirname "$0")"
gcc -std=gnu90 -Wall -Wextra -Wdeclaration-after-statement -Istub -I../Core/Inc -o /tmp/test_arm_host ../Core/Src/arm.c test_arm.c 2>&1 | grep -v "test_arm.c" || true
/tmp/test_arm_host
