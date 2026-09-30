#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include <stdint.h>

/* ==============================
   小车底盘控制函数
   ============================== */

/* 小车停止 */
void Car_Stop(void);

/* 小车前进 */
void Car_Forward(uint16_t speed, uint32_t pulse);

/* 小车后退 */
void Car_Backward(uint16_t speed, uint32_t pulse);

/* 小车左平移 */
void Car_Left(uint16_t speed, uint32_t pulse);

/* 小车右平移 */
void Car_Right(uint16_t speed, uint32_t pulse);

/* 小车原地左转 */
void Car_RotateLeft(uint16_t speed, uint32_t pulse);

/* 小车原地右转 */
void Car_RotateRight(uint16_t speed, uint32_t pulse);

/* 按毫米前进 */
void Car_Forward_mm(uint16_t speed, float distance_mm);

/* 按毫米后退 */
void Car_Backward_mm(uint16_t speed, float distance_mm);

/* 按毫米左平移 */
void Car_Left_mm(uint16_t speed, float distance_mm);

/* 按毫米右平移 */
void Car_Right_mm(uint16_t speed, float distance_mm);


void Car_Forward_Yaw_mm(uint16_t speed,float distance_mm);


void Car_Forward_Yaw_mm(uint16_t speed,float distance_mm);
#endif