#ifndef __HWT101_H
#define __HWT101_H

#include "main.h"

typedef struct
{
    uint32_t frames_ok;
    uint32_t frames_bad;
    float    gyro_dps;
    uint32_t last_tick;
} HWT101_Debug_t;

extern volatile HWT101_Debug_t hwt101_dbg;

void  HWT101_Init(void);
void  HWT101_Update(void);
float HWT101_GetYaw(void);
float HWT101_GetYawContinuous(void);
void  HWT101_ZeroSoft(void);
float HWT101_AngleDiff(float target, float current);
uint8_t HWT101_IsReady(void);
uint8_t HWT101_IsFresh(uint32_t timeout_ms);
void  HWT101_OnByte(uint8_t b);

#endif
