#ifndef _BUJIN_H
#define _BUJIN_H

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "Emm_V5.h"
#include <stdio.h>
#include <stdarg.h>


#define RCO_BUJIN1 HAL_GPIO_ReadPin(GPIOE ,GPIO_PIN_15)
#define RCO_BUJIN2 HAL_GPIO_ReadPin(GPIOB ,GPIO_PIN_14)
#define RCO_BUJIN3 HAL_GPIO_ReadPin(GPIOA ,GPIO_PIN_4)
#define RCO_BUJIN4 HAL_GPIO_ReadPin(GPIOB ,GPIO_PIN_15)
#define RCO_BUJIN5 HAL_GPIO_ReadPin(GPIOE ,GPIO_PIN_7)



int myabs(int num);

void LED_show(uint8_t number,uint8_t kaiguan);

void move_front(uint16_t vel, uint8_t acc, uint32_t clk, bool raF);

void move_back(uint16_t vel, uint8_t acc, uint32_t clk, bool raF);

void move_left(uint16_t vel, uint8_t acc, uint32_t clk, bool raF);


void move_right(uint16_t vel, uint8_t acc, uint32_t clk, bool raF);


void move_front_left(uint16_t vel, uint8_t acc, uint32_t clk, bool raF);

void move_front_right(uint16_t vel, uint8_t acc, uint32_t clk, bool raF);

void move_back_left(uint16_t vel, uint8_t acc, uint32_t clk, bool raF);

void move_back_right(uint16_t vel, uint8_t acc, uint32_t clk, bool raF);

void move_shun(uint16_t vel, uint8_t acc, uint32_t clk, bool raF);

void move_ni(uint16_t vel, uint8_t acc, uint32_t clk, bool raF);

void move_shun90(void);
void move_shun180(void);
void move_ni90(void);
void move_ni180(void);
void move_stopall(void);
void move_tongbudai(uint16_t vel, uint8_t acc, uint32_t clk);

void Ve_bujin1( int32_t vel, uint8_t acc);

void Ve_bujin2( int32_t vel, uint8_t acc);

void Ve_bujin3( int32_t vel, uint8_t acc);

void Ve_bujin4( int32_t vel, uint8_t acc);

void V_xy( int32_t V_x,int32_t V_y,uint8_t acc);





void move(uint8_t mode ,uint16_t vel, uint8_t acc,int32_t x,int32_t y);//x y µ¥Î»Îª1mm 

void car_state(uint8_t change);
#endif
