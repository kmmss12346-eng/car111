/* 主机测试用：陀螺仪接口(实现在 test_chassis.c 里，读假车的车头) */
#pragma once
#include "main.h"
float HWT101_GetYaw(void);
float HWT101_GetYawContinuous(void);
uint8_t HWT101_IsFresh(uint32_t timeout_ms);
uint8_t HWT101_IsReady(void);
