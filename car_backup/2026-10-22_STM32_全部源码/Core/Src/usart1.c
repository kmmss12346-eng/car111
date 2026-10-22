#include "usart1.h"


/* -------------------------------
 * Fashion Star 串口缓冲区
 * ------------------------------- */

static uint8_t usart2SendBuf[USART_SEND_BUF_SIZE + 1];
static uint8_t usart2RecvBuf[USART_RECV_BUF_SIZE + 1];

static RingBufferTypeDef usart2SendRingBuf;
static RingBufferTypeDef usart2RecvRingBuf;


/*
 * 旧驱动虽然把它叫 usart2，
 * 但实际硬件使用 USART6。
 */
Usart_DataTypeDef usart2;


/* 初始化Fashion Star串口对象 */
void Usart_Init(void)
{
    /*
     * 这里只是告诉旧驱动：
     * 这个对象对应STM32的USART6。
     *
     * USART6本身的115200、PC6、PC7等
     * 已经由CubeMX初始化，不需要在这里重新配置。
     */
    usart2.pUSARTx = USART6;


    /* 初始化发送环形缓冲区 */
    RingBuffer_Init(&usart2SendRingBuf,
                    USART_SEND_BUF_SIZE,
                    usart2SendBuf);


    /* 初始化接收环形缓冲区 */
    RingBuffer_Init(&usart2RecvRingBuf,
                    USART_RECV_BUF_SIZE,
                    usart2RecvBuf);


    /* 把缓冲区交给usart2对象 */
    usart2.sendBuf = &usart2SendRingBuf;
    usart2.recvBuf = &usart2RecvRingBuf;
}


/* 发送一个字节 */
void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch)
{
    /*
     * 真正从CubeMX生成的USART6发出去
     */
    HAL_UART_Transmit(&huart6,
                      &ch,
                      1,
                      1000);
}


/* 发送字节数组 */
void Usart_SendByteArr(USART_TypeDef *pUSARTx,
                       uint8_t *byteArr,
                       uint16_t size)
{
    HAL_UART_Transmit(&huart6,
                      byteArr,
                      size,
                      1000);
}


/* 发送字符串 */
void Usart_SendString(USART_TypeDef *pUSARTx, char *str)
{
    HAL_UART_Transmit(&huart6,
                      (uint8_t *)str,
                      strlen(str),
                      1000);
}


/*
 * 把Fashion Star发送缓冲区
 * 中的数据全部发出去
 */
void Usart_SendAll(Usart_DataTypeDef *usart)
{
    uint8_t value;

    while (RingBuffer_GetByteUsed(usart->sendBuf) > 0)
    {
        value = RingBuffer_Pop(usart->sendBuf);

        Usart_SendByte(usart->pUSARTx, value);
    }
}