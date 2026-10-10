/* 主机测试用：hwt101.c 的头文件(和工程里的 hwt101.h 声明一样)。test_arm.c 把 hwt101.c 整个包含进来，测串口回调 */
#pragma once
#include "main.h"
typedef struct { uint32_t frames_ok; uint32_t frames_bad; float gyro_dps; uint32_t last_tick; } HWT101_Debug_t;
extern volatile HWT101_Debug_t hwt101_dbg;
void HWT101_Init(void);
void HWT101_Update(void);
void HWT101_OnByte(uint8_t b);
float HWT101_GetYaw(void);
float HWT101_GetYawContinuous(void);
void HWT101_ZeroSoft(void);
float HWT101_AngleDiff(float target, float current);
uint8_t HWT101_IsReady(void);
uint8_t HWT101_IsFresh(uint32_t timeout_ms);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
