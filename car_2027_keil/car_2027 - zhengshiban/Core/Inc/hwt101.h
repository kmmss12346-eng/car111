#ifndef __HWT101_H
#define __HWT101_H

#include "main.h"

/* HWT101 (WIT 协议) 接 USART1 (PA9/PA10, 115200)，用 RXNE 中断接收，不阻塞主循环 */

/* 调试数据块：用 OpenOCD 可以直接读这块内存(tools/hwt101_openocd_read.py) */
typedef struct
{
    uint32_t magic;        /* 0x48575431 'HWT1'，用来确认读到的是这块数据 */
    uint32_t frames_ok;    /* 校验通过的帧数 */
    uint32_t frames_bad;   /* 校验失败的帧数 */
    float    yaw_deg;      /* 原始偏航角 -180..180 */
    float    yaw_cont_deg; /* 连续偏航角(已去掉 ±180 跳变，已减去零点) */
    float    gyro_dps;     /* Z 轴角速度 */
    uint32_t last_tick;    /* 最近一次收到角度帧的 HAL_GetTick() */
    uint32_t now_tick;     /* 由 HWT101_Poll() 写入的当前 HAL_GetTick() */
} HWT101_Debug_t;

extern volatile HWT101_Debug_t hwt101_dbg;

/* 复位状态并开始中断接收；必须在 MX_USART1_UART_Init() 之后调用 */
void HWT101_Init(void);

/* 兼容旧代码：以前需要不断调用，现在接收在中断里完成，这里只刷新 now_tick */
void HWT101_Update(void);

/* 原始偏航角 -180..180 (度) */
float HWT101_GetYaw(void);

/* 连续偏航角(度)：跨过 ±180 不跳变，逆/顺时针方向沿用传感器符号 */
float HWT101_GetYawContinuous(void);

/* 把当前位置作为连续偏航角的零点(只改软件零点，不向传感器写任何指令) */
void HWT101_ZeroSoft(void);

/* 目标角 - 当前角，折算到 -180..180，用于航向保持，避免 ±180 跳变 */
float HWT101_AngleDiff(float target, float current);

/* 是否已经成功收到过有效角度数据 */
uint8_t HWT101_IsReady(void);

/* 最近 timeout_ms 内是否还有新数据 */
uint8_t HWT101_IsFresh(uint32_t timeout_ms);

/* 在 USART1 接收中断里调用：喂入一个字节 */
void HWT101_OnByte(uint8_t b);

#endif
