/* 主机测试用：和工程里的 Emm_V5.h 一样的函数声明(实现在 test_chassis.c 里，是假的电机) */
#pragma once
#include "main.h"
#include <stdbool.h>
extern UART_HandleTypeDef huart4;
void UART4_SendByte(uint8_t Byte);
void UART4_SendArray(uint8_t *Array, uint16_t Length);
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Synchronous_motion(uint8_t addr);
