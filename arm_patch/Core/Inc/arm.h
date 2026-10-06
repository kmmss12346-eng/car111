/* arm.h  机械臂总控：升降(Emm ID5)、夹爪、转盘、夹取/放置流程、二维码读取(UART5)、串口屏(USART2)
 *
 * 树莓派 -> STM32 新增指令(每条一行，和原来的 F/S/R/A/U 一样，做完回 DONE 或 ERR xxx)：
 *   CLAW O | CLAW C | CLAW <微秒>         夹爪张开 / 夹紧 / 直接给脉宽
 *   TT <1~3> | TT <微秒>                   车上转盘转到 1/2/3 号位 / 直接给脉宽
 *   LIFT ZERO                              把"现在的高度"记为 0 (升降最高点)
 *   LIFT HOME                              让 ID5 驱动器自己回零(需先在驱动器里配好回零方式)
 *   LIFT <mm>                              升降到离零点向下 mm 毫米(只能在 ZERO/HOME 之后)
 *   LIFT?                                  回 "LIFT <是否已回零> <当前mm>"
 *   GRAB <1~3>                             从原料盘夹一个物料，放进车上转盘的 n 号位
 *   PLACE <1~3>                            从车上转盘 n 号位取出物料，放到下面的圆环上
 *   QR?                                    回 "QR <任务码>" 或 "QR NONE"
 *   QR CLR                                 清掉已读到的任务码
 *   SCR <控件名> <文字>                    往串口屏某个文本控件写字，例如 SCR t0 452+321+254+312
 * 另外 SET / GET 可以改/看下面这些机械臂参数(名字见 arm.c 的 tun 表)。
 *
 * STM32 -> 树莓派 额外会主动发：QR <任务码>   (扫码模块读到合格的任务码时发一次)
 */
#ifndef __ARM_H
#define __ARM_H

#include "main.h"

void Arm_Init(void);                 /* 开机调用一次：启动夹爪/转盘 PWM，开始接收二维码 */
void Arm_Poll(void);                 /* 主循环里反复调用：处理扫码模块收到的数据 */

/* 返回 0 = 这条不是机械臂的指令，请继续往下判断
 *      1 = 做完了(调用者再回 DONE)
 *     -1 = 出错，err 里已经填好 "ERR xxx" */
int  Arm_Command(const char *cmd, char *err, int errlen);

int  Arm_Param_Set(const char *name, float v);   /* 0=没有这个名字 1=成功 2=超出范围 */
void Arm_Param_Dump(void);

void Arm_QR_RxCplt(void);            /* UART5 收到 1 字节(在串口中断回调里调用) */
void Arm_QR_RxRestart(void);         /* UART5 出错后重新开始接收 */

void Screen_Text(const char *obj, const char *text);

#endif
