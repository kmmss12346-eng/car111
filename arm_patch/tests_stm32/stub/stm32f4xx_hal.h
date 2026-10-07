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
/* Flash(升降编码器标定存在这里)：测试里用一个数组代替 */
#include <stdint.h>
#define HAL_OK 0
typedef struct { uint32_t TypeErase, Banks, Sector, NbSectors, VoltageRange; } FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_SECTORS 0
#define FLASH_BANK_1 1
#define FLASH_SECTOR_7 7
#define FLASH_VOLTAGE_RANGE_3 2
#define FLASH_TYPEPROGRAM_WORD 2
#define FLASH_FLAG_EOP 1
#define FLASH_FLAG_OPERR 2
#define FLASH_FLAG_WRPERR 4
#define FLASH_FLAG_PGAERR 8
#define FLASH_FLAG_PGPERR 16
#define FLASH_FLAG_PGSERR 32
#define __HAL_FLASH_CLEAR_FLAG(x) ((void)(x))
int HAL_FLASH_Unlock(void);
int HAL_FLASH_Lock(void);
int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *err);
int HAL_FLASH_Program(uint32_t type, uintptr_t addr, uint64_t data);
extern uint32_t fake_flash[8];
#define ARM_CAL_ADDR ((uintptr_t)fake_flash)
