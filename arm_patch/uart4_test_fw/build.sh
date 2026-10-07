#!/bin/bash
# 需要 arm-none-eabi-gcc。生成 uart4_test.hex / uart4_test.bin
set -e
cd "$(dirname "$0")"
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Os -Wall -Wextra -ffreestanding -nostdlib -T link.ld -o uart4_test.elf main.c
arm-none-eabi-objcopy -O ihex uart4_test.elf uart4_test.hex
arm-none-eabi-objcopy -O binary uart4_test.elf uart4_test.bin
arm-none-eabi-size uart4_test.elf
