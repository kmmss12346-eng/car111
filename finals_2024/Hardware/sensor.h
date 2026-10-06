#ifndef _SENSOR_H
#define _SENSOR_H

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "Emm_V5.h"
#include <stdio.h>
#include <stdarg.h>
#include "task.h"

extern __O uint16_t bujin5_upflag,bujin5_downflag,task1_flag,task2_flag,task3_flag,task4_flag,task5_flag,task6_flag,task7_flag,task8_flag,task9_flag,taskf_flag,taska_flag,taskb_flag,taskc_flag,taskd_flag;
extern __O uint16_t tiaoshi_flag;

extern uint8_t color_mode1,mode1_position,mode1_xerr,mode1_yerr,mode1_receiveflag,
mode2_position,mode2_xerr,mode2_yerr,mode2_receiveflag;
extern uint8_t paixu[8];
extern uint8_t nano_ok;
extern uint8_t flag_saomaqi;
extern float angle[3];
int fputc(int ch, FILE *f);
int fgetc(FILE *f);

int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...);

void OLEDShow_Angle(void);
void send_yaw0(void);//Z轴归零==yaw角归零
void send_pitch0(void);//加计校准,pitch角偏移时使用
void JY62_Init(void);
void Mode_lingpiao(void);
void Receiver(uint8_t num,uint8_t res);


void open_uart(uint8_t num);

void close_uart(uint8_t num);


char act_one(uint8_t num);

void act_all(void);



#endif
