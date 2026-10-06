#ifndef __HWT101_H
#define __HWT101_H

#include "main.h"

/* 初始化HWT101软件状态 */
void HWT101_Init(void);

/* 接收并解析HWT101数据，需要不断调用 */
void HWT101_Update(void);

/* 获取当前Yaw角 */
float HWT101_GetYaw(void);

/* 是否已经成功收到过有效角度数据 */
uint8_t HWT101_IsReady(void);

#endif