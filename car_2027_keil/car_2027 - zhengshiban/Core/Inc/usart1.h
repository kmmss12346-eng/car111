#ifndef __USART1_H
#define __USART1_H

#include "main.h"
#include "ring_buffer.h"

/* Fashion Star驱动内部使用的发送/接收缓冲区大小 */
#define USART_RECV_BUF_SIZE 256
#define USART_SEND_BUF_SIZE 256

/*
 * Fashion Star驱动自己的串口对象
 *
 * 注意：
 * usart2只是旧程序给这个对象起的名字，
 * 真正的STM32硬件使用的是USART6。
 */
typedef struct
{
    USART_TypeDef *pUSARTx;

    RingBufferTypeDef *sendBuf;
    RingBufferTypeDef *recvBuf;

} Usart_DataTypeDef;


/* CubeMX在main.c里生成的USART6句柄 */
extern UART_HandleTypeDef huart6;


/* Fashion Star总线使用的串口对象 */
extern Usart_DataTypeDef usart2;


/* 初始化Fashion Star自己的缓冲区 */
void Usart_Init(void);


/* 发送一个字节 */
void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch);


/* 发送字节数组 */
void Usart_SendByteArr(USART_TypeDef *pUSARTx,
                       uint8_t *byteArr,
                       uint16_t size);


/* 发送字符串 */
void Usart_SendString(USART_TypeDef *pUSARTx, char *str);


/* 把发送环形缓冲区里的数据全部发送出去 */
void Usart_SendAll(Usart_DataTypeDef *usart);


#endif