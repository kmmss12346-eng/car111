/* 主机测试用的假 HAL：只有 arm.c 用到的那一点点 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct { struct { uint32_t BaudRate; } Init; } UART_HandleTypeDef;
typedef struct { int dummy; } TIM_HandleTypeDef;
#define TIM_CHANNEL_2 2
#define TIM_CHANNEL_3 3
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
int HAL_UART_Transmit(UART_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
int HAL_UART_Receive_IT(UART_HandleTypeDef*, uint8_t*, uint16_t);
int HAL_UART_Init(UART_HandleTypeDef*);
int HAL_TIM_PWM_Start(TIM_HandleTypeDef*, uint32_t);
void tim_set(uint32_t ch, uint32_t v);
#define __HAL_TIM_SET_COMPARE(h, ch, v) tim_set((ch), (v))
#define __disable_irq() ((void)0)
#define __enable_irq()  ((void)0)
