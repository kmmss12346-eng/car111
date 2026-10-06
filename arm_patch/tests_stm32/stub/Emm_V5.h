/* 主机测试用：只声明 arm.c 用到的 Emm 驱动函数(真的在您工程的 Emm_V5.h 里) */
#pragma once
#include <stdint.h>
#include <stdbool.h>
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
void Emm_V5_Origin_Interrupt(uint8_t addr);
