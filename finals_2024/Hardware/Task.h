#ifndef _TASK_H
#define _TASK_H

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "Emm_V5.h"
#include "sensor.h"
#include "Bujin.h"
#include <stdio.h>
#include <stdarg.h>
#include "RGB.h"


extern uint8_t modeone[3];
extern uint8_t modetwo[3];
extern float yaw_begin,yaw_tiaozheng1st;
extern uint32_t high_yuanpan;
extern uint8_t di_yuanpan[7];




#define Red_Pin GPIO_PIN_2
#define Red_GPIO_Port GPIOE
#define Blue_Pin GPIO_PIN_3
#define Blue_GPIO_Port GPIOE
#define Green_Pin GPIO_PIN_4
#define Green_GPIO_Port GPIOE
#define Beep_Pin GPIO_PIN_5
#define Beep_GPIO_Port GPIOE


#define RED 1
#define GREEN 2
#define BLUE 3
#define BEEP 4


#define tiaozheng_mode 2
#define tiaozheng_vel 5
#define tiaozheng_acc 0
#define tiaozheng_min 3
#define yuanhuan_1distance 150
#define yuanhuan_2distance 300
#define bujin5_vel 5000
#define bujin5_acc 240


#define duoji_jiazhao_songkai_init 1700
#define duoji_jiazhao_songkai 2090
#define duoji_jiazhao_jiajin 2910

#define duoji_zhedie_down 3000
#define duoji_zhedie_up 2400

#define duoji_liaopan_liaopan1 2608
#define duoji_liaopan_liaopan2 2608-900
#define duoji_liaopan_liaopan3 2608-2*900

#define duoji_zhuanbi_wai 2940
#define duoji_zhuanbi_nei 1324

#define pulse_onecircle 3200.0f
#define distance_onecircle 246.0f


//3200脉冲=247mm，
//计算结果为77*3.141=241.857

#define pulse_shun90 4515.0f
#define pulse_shun180 2*4515.0f
#define pulse_ni90 4515.0f
#define pulse_ni180 2*4515.0f


void Fashion_servo_Init(void);
void jiguang(uint8_t kaiguan);
void page(void);
void saoma(void);
void light(uint8_t i);
void duoji_jiazhao(uint32_t angle);
void duoji_liaopan(uint32_t angle);//(PB3)1:物料1,2:物料2,3:物料3
void duoji_zhuanbi(uint32_t angle);//(PA5)
void duoji_zhedie(uint32_t angle);//PC8
uint8_t get_taskflag(uint8_t task);
void LED(uint8_t number);
void duoji_zhedie_slowaction(uint32_t angle,uint8_t wait);
void duoji_zhuanbi_slowaction(uint32_t angle,uint8_t wait);
void duoji_jiazhao_slowaction(uint32_t angle);
void camera_jiaozhun(void);
//void camera_shibie(uint8_t number);
void camera_jiaozhun_yuanliao(uint8_t number);
void camera_jiaozhun_mode3(void);
void maduo_from_car_to_ground(uint8_t level,uint8_t number);//level：第几层，number：第几个
void quwuliao_from_ground_to_car(uint8_t number);//number：第几个
void angle_tiaozheng(int16_t goal);
void PID_ERR_move(void);
void PID_ERR_move_mode1(void);
void task_cheshi(uint8_t number);
void gohome(uint8_t i);
void qidong_to_yuanliao(void);
void task1(void);
void task2(void);
void task3(void);
void task4(void);

void task9(void);
void task_all(void);
void task_Init(void);
void task_yuanliao(uint8_t number);
void task_cujiagong(uint8_t number);
void task_zancunqu(uint8_t level);
void cu_to_zan(uint8_t number);
void yuanliao_to_cu(void);
void zan_to_yuanliao(void);

#endif
