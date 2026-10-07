/* 主机测试 chassis.c 用的假 HAL：只有 chassis.c 用到的那一点点 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct { volatile uint32_t SR; volatile uint32_t DR; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t t);
/* 每次检查 RXNE 时，假串口把下一个字节放进 DR(检查之后代码只读一次 DR，和真的硬件用法一致) */
uint32_t fake_uart4_poll(void);
#define USART_SR_RXNE (fake_uart4_poll())
