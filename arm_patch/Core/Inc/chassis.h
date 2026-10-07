#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include <stdint.h>

void Car_Stop(void);
void Car_Forward(uint16_t speed, uint32_t pulse);
void Car_Backward(uint16_t speed, uint32_t pulse);
void Car_Left(uint16_t speed, uint32_t pulse);
void Car_Right(uint16_t speed, uint32_t pulse);
void Car_RotateLeft(uint16_t speed, uint32_t pulse);
void Car_RotateRight(uint16_t speed, uint32_t pulse);
void Car_Forward_mm(uint16_t speed, float distance_mm);
void Car_Backward_mm(uint16_t speed, float distance_mm);
void Car_Left_mm(uint16_t speed, float distance_mm);
void Car_Right_mm(uint16_t speed, float distance_mm);
void Car_Forward_Yaw_mm(uint16_t speed, float distance_mm);
void Car_TurnBy(float delta_deg);                 /* 相对现在的车头转，绕车自己的转轴 */
void Car_TurnBy_Center(float delta_deg);          /* 路线里的 R：绕车中心转，按累计目标航向转 */
void Car_Forward_Hold(uint16_t speed, float distance_mm);
void Car_Forward_Align(uint16_t speed, float distance_mm);
void Car_Straight_Closed(uint16_t speed, float distance_mm);   /* 闭环直行：正数前进，负数后退 */
void Car_Strafe_Closed(uint16_t speed, float distance_mm);     /* 闭环横移：正数左移，负数右移 */
void Car_Strafe_Calib(uint16_t speed, float distance_mm);      /* 横移校准：不纠偏左移再右移，串口打印 CAL 并立刻生效 */
void Car_Move_Align(char kind, uint16_t speed, float distance_mm);
void Car_Home(void);                              /* 现在的车头方向 = 要保持的方向 */

int  Car_Param_Set(const char *name, float v);    /* 0=没有这个名字 1=成功 2=超出范围 */
void Car_Param_Dump(void);                        /* 串口打印全部参数 */
void Car_Motor_Report(void);                      /* MOT?：读 1~5 号驱动器的电压、使能、堵转保护，串口打印 */
void Car_Motor_Enable(void);                      /* MOT EN：1~5 号解除堵转保护并使能(开机和 HOME 时自动做一次) */
int  Car_Motor_Query(uint8_t addr, uint8_t func, uint8_t *out, uint8_t len);   /* 读驱动器参数，收到回复返回 1 */

extern volatile uint8_t car_abort;                /* 串口收到 '!' 时置 1，闭环动作马上停车 */
extern volatile float   car_last_err;             /* 上一个闭环动作结束时车头误差(度) */
extern volatile uint8_t car_stalled;              /* 转弯时轮子没转起来(陀螺仪几乎不变)置 1，这条指令回 ERR STALL */

#define ROUTE_SPEED         220    /* 路线直行/后退的最高速(转/分) */
#define ROUTE_STRAFE_SPEED  170    /* 路线横移的最高速(转/分) */
#endif
