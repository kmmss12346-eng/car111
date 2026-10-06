/*
 * 串口总线舵机示例代码
 * ---------------------------------------------------
 * ---------------------------------------------------
 */
#ifndef __FASHION_STAR_USART_SERVO_EXAMPLE_H
#define __FASHION_STAR_USART_SERVO_EXAMPLE_H

// 导入系统依赖库

#include "sys_tick.h"
#include "usart.h"
#include "fashion_star_uart_servo.h"

// 演示例程序号(宏定义)
// #define FSUS_EXAMPLE_LED_BLINK 1			  // LED闪烁
// #define FSUS_EXAMPLE_USART_ECHO 2			  // USART收发测试
#define FSUS_EXAMPLE_PING_SERVO 3			  // 舵机通讯检测
#define FSUS_EXAMPLE_SET_SERVO_ANGLE 4		  // 控制单个舵机的角度
#define FSUS_EXAMPLE_SET_N_SERVO_ANGLE 5	  // 控制多个舵机的角度
#define FSUS_EXAMPLE_SET_SERVO_ANGLE_MTURN 6  // 控制单个舵机的角度(多圈模式)
#define FSUS_EXAMPLE_SET_SERVO_DAMPING 7	  // 舵机阻尼模式与角度回读
#define FSUS_EXAMPLE_SET_WHEEL_KEEP_MOVE 8	  // 轮式模式控制(一直旋转)
#define FSUS_EXAMPLE_SET_WHEEL_MOVE_NCIRCLE 9 // 轮式模式控制(定圈)
#define FSUS_EXAMPLE_SET_WHEEL_MOVE_TIME 10	  // 轮式控制模式(定时)
// #define FSUS_EXAMPLE_RESET_DATA 11			  // 重置用户自定义表
#define FSUS_EXAMPLE_READ_DATA 12			  // 读取数据
// #define FSUS_EXAMPLE_WRITE_DATA 13			  // 写入数据

// 变量定义
extern Usart_DataTypeDef *servo_usart;

// 根据例程编号执行测试例程
void FSUSExample_Run(uint8_t example_id);

/* LED闪烁例程 */
void FSUSExample_LEDBlink(void);

/* USART收发测试 */
void FSUSExample_UsartEcho(void);

/* 舵机通讯检测 */
void FSUSExample_PingServo(void);

/* 控制单个舵机的角度 */
void FSUSExample_SetServoAngle(void);

/* 控制多个舵机的角度 */
void FSUSExample_SetNServoAngle(void);

/* 设置舵机角度(多圈模式) */
void FSUSExample_SetServoAngleMTurn(void);

/* 舵机阻尼模式与角度回传 */
void FSUSExample_SetServoDamping(void);

/* 轮式模式控制(一直旋转) */
void FSUSExample_WheelKeepMove(void);

/* 估计旋转圈数所需要花费的时间 */
uint16_t FSUSExample_EstimateTimeMs(uint16_t n_circle, uint16_t speed);

/* 轮式模式控制(定圈) */
void FSUSExample_WheelMoveNCircle(void);

/* 轮式控制模式(定时) */
void FSUSExample_WheelMoveNTime(void);

/* 用户自定义数据-重置 */
void FSUSExample_ResetData(void);

/* 用户自定义数据-读取 */
void FSUSExample_ReadData(void);

/* 用户自定义数据-写入 */
void FSUSExample_WriteData(void);

#endif
