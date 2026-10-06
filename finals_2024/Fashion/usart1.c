/*
 * USART串口通信
 * ---------------------------------------------------
 * ---------------------------------------------------
 */
#include "usart1.h"
#include "usart.h"
#if USART1_ENABLE
uint8_t usart1SendBuf[USART_SEND_BUF_SIZE + 1];
uint8_t usart1RecvBuf[USART_RECV_BUF_SIZE + 1];
RingBufferTypeDef usart1SendRingBuf;
RingBufferTypeDef usart1RecvRingBuf;
Usart_DataTypeDef usart1;

#endif

#if USART2_ENABLE
uint8_t usart2SendBuf[USART_SEND_BUF_SIZE + 1];
uint8_t usart2RecvBuf[USART_RECV_BUF_SIZE + 1];
RingBufferTypeDef usart2SendRingBuf;
RingBufferTypeDef usart2RecvRingBuf;
Usart_DataTypeDef usart2;
#endif

#if USART3_ENABLE
uint8_t usart3SendBuf[USART_SEND_BUF_SIZE + 1];
uint8_t usart3RecvBuf[USART_RECV_BUF_SIZE + 1];
RingBufferTypeDef usart3SendRingBuf;
RingBufferTypeDef usart3RecvRingBuf;
Usart_DataTypeDef usart3;
#endif
typedef struct
{
  uint32_t USART_BaudRate;            /*!< This member configures the USART communication baud rate.
                                           The baud rate is computed using the following formula:
                                            - IntegerDivider = ((PCLKx) / (8 * (OVR8+1) * (USART_InitStruct->USART_BaudRate)))
                                            - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 8 * (OVR8+1)) + 0.5 
                                           Where OVR8 is the "oversampling by 8 mode" configuration bit in the CR1 register. */

  uint16_t USART_WordLength;          /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref USART_Word_Length */

  uint16_t USART_StopBits;            /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref USART_Stop_Bits */

  uint16_t USART_Parity;              /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref USART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */
 
  uint16_t USART_Mode;                /*!< Specifies wether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref USART_Mode */

  uint16_t USART_HardwareFlowControl; /*!< Specifies wether the hardware flow control mode is enabled
                                           or disabled.
                                           This parameter can be a value of @ref USART_Hardware_Flow_Control */
} USART_InitTypeDef;
typedef struct
{
  uint8_t NVIC_IRQChannel;                    /*!< Specifies the IRQ channel to be enabled or disabled.
                                                   This parameter can be an enumerator of @ref IRQn_Type 
                                                   enumeration (For the complete STM32 Devices IRQ Channels
                                                   list, please refer to stm32f4xx.h file) */

  uint8_t NVIC_IRQChannelPreemptionPriority;  /*!< Specifies the pre-emption priority for the IRQ channel
                                                   specified in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref MISC_NVIC_Priority_Table
                                                   A lower priority value indicates a higher priority */

  uint8_t NVIC_IRQChannelSubPriority;         /*!< Specifies the subpriority level for the IRQ channel specified
                                                   in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref MISC_NVIC_Priority_Table
                                                   A lower priority value indicates a higher priority */

  FunctionalState NVIC_IRQChannelCmd;         /*!< Specifies whether the IRQ channel defined in NVIC_IRQChannel
                                                   will be enabled or disabled. 
                                                   This parameter can be set either to ENABLE or DISABLE */   
} NVIC_InitTypeDef;

void Usart_Init(void)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    // 串口配置结构体
    USART_InitTypeDef USART_InitStructure;
    // 外部中断结构体
    NVIC_InitTypeDef NVIC_InitStructure;

#if USART1_ENABLE
    // 赋值结构体usart指针
    usart1.pUSARTx = USART1;
//    // 初始化缓冲区(环形队列)
//    RingBuffer_Init(&usart1SendRingBuf, USART_SEND_BUF_SIZE, usart1SendBuf);
//    RingBuffer_Init(&usart1RecvRingBuf, USART_RECV_BUF_SIZE, usart1RecvBuf);
//    usart1.recvBuf = &usart1RecvRingBuf;
//    usart1.sendBuf = &usart1SendRingBuf;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //使能GPIOA时钟
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //使能USART1时钟

//    //串口1对应引脚复用映射
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  //GPIOA9复用为USART1
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10复用为USART1

//    //USART1端口配置
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //复用功能
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //速度50MHz
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽复用输出
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //上拉
//    GPIO_Init(GPIOA, &GPIO_InitStructure);                  //初始化PA9，PA10

//    //USART1 初始化设置
//    USART_InitStructure.USART_BaudRate = USART1_BAUDRATE;                           //波特率设置
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
//    USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
//    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //收发模式
//    USART_Init(USART1, &USART_InitStructure);                                       //初始化串口1

//    USART_Cmd(USART1, ENABLE); //使能串口1

//    USART_ClearFlag(USART1, USART_FLAG_TC);

//    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启相关中断

//    //Usart1 NVIC 配置
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;         //串口1中断通道
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级3
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级3
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ通道使能
//    NVIC_Init(&NVIC_InitStructure);                           //根据指定的参数初始化VIC寄存器

#endif
#if USART2_ENABLE

    // 赋值结构体usart指针
    usart2.pUSARTx = USART6;
    // 初始化缓冲区(环形队列)
    RingBuffer_Init(&usart2SendRingBuf, USART_SEND_BUF_SIZE, usart2SendBuf);
    RingBuffer_Init(&usart2RecvRingBuf, USART_RECV_BUF_SIZE, usart2RecvBuf);
    usart2.recvBuf = &usart2RecvRingBuf;
    usart2.sendBuf = &usart2SendRingBuf;

//    // 赋值结构体usart指针
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //使能GPIOA时钟
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //使能USART2时钟

//    //串口2引脚复用映射
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //GPIOA2复用为USART2
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //GPIOA3复用为USART2

//    //USART2
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //复用功能
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //速度50MHz
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //推挽复用输出
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           //上拉
//    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //初始化PA2，PA3

//    //USART2 初始化设置
//    USART_InitStructure.USART_BaudRate = USART2_BAUDRATE;                           //波特率设置
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
//    USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
//    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //收发模式
//    USART_Init(USART2, &USART_InitStructure);                                       //初始化串口2

//    USART_Cmd(USART2, ENABLE); //使能串口 2

//    USART_ClearFlag(USART2, USART_FLAG_TC);

//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //开启接受中断

//    //Usart2 NVIC 配置
//    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级3
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级3
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ通道使能
//    NVIC_Init(&NVIC_InitStructure);                           //根据指定的参数初始化VIC寄存器、

#endif

#if USART3_ENABLE
    // 赋值结构体usart指针
    usart3.pUSARTx = USART3;
    // 初始化缓冲区(环形队列)
    RingBuffer_Init(&usart3SendRingBuf, USART_SEND_BUF_SIZE, usart3SendBuf);
    RingBuffer_Init(&usart3RecvRingBuf, USART_RECV_BUF_SIZE, usart3RecvBuf);
    usart3.recvBuf = &usart3RecvRingBuf;
    usart3.sendBuf = &usart3SendRingBuf;

    // 使能USART时钟
    USART3_APBxClkCmd(USART3_CLK, ENABLE);
    // 使能GPIO时钟
    USART3_GPIO_APBxClkCmd(USART3_GPIO_CLK, ENABLE);
    // 配置串口USART的发送管脚 TX
    GPIO_InitStructure.GPIO_Pin = USART3_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStructure);

    // 配置串口USART的接收管脚 RX
    GPIO_InitStructure.GPIO_Pin = USART3_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStructure);

    // Usart NVIC 配置串口外部中断
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                           //根据指定的参数初始化NVIC寄存器

    //USART 初始化设置
    USART_InitStructure.USART_BaudRate = USART3_BAUDRATE;                           //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //收发模式
    // 初始化串口
    USART_Init(USART3, &USART_InitStructure);
    //开启串口接收中断 IT: Interupt
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    // 使能串口
    USART_Cmd(USART3, ENABLE);
#endif
}

/* 发送单个字节 */
void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch)
{
	HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 1000);
        
}

/* 发送8位的字节流 */
void Usart_SendByteArr(USART_TypeDef *pUSARTx, uint8_t *byteArr, uint16_t size)
{
    uint16_t bidx;
    for (bidx = 0; bidx < size; bidx++)
    {
        Usart_SendByte(pUSARTx, byteArr[bidx]);
    }
}

/* 发送字符串 */
void Usart_SendString(USART_TypeDef *pUSARTx, char *str)
{
    uint16_t sidx = 0;
    do
    {
        Usart_SendByte(pUSARTx, (uint8_t)(*(str + sidx)));
        sidx++;
    } while (*(str + sidx) != '\0');
}

// 将串口发送缓冲区的内容全部发出去
void Usart_SendAll(Usart_DataTypeDef *usart)
{
    uint8_t value;
    while (RingBuffer_GetByteUsed(usart->sendBuf))
    {
        value = RingBuffer_Pop(usart->sendBuf);
        // printf("Usart_SendAll pop: %d", value);
        Usart_SendByte(usart->pUSARTx, value);
    }
}
#if USART1_ENABLE
// 定义串口中断处理函数
//void USART1_IRQHandler(void)
//{
//    uint8_t ucTemp;
//    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//    {
//        // 从串口读取一个字符
//        ucTemp = USART_ReceiveData(USART1);
//        // 新的字符添加到串口的环形缓冲队列中
//        RingBuffer_Push(usart1.recvBuf, ucTemp);
//    }
//}
//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB

#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}

// 注: 不定义这个函数会报错
void _ttywrch(int ch){

}

// 重定向c库函数printf到串口，重定向后可使用printf函数
//int fputc(int ch, FILE *f)
//{
//	/* 发送一个字节数据到串口DEBUG_USART */
//	HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 1000);	
//	
//	return (ch);
//}
#endif
#endif

#if USART2_ENABLE
// 定义串口中断处理函数
//void USART2_IRQHandler(void)
//{
//    uint8_t ucTemp;
//    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
//    {
//        // 从串口读取一个字符
//        ucTemp = USART_ReceiveData(usart2.pUSARTx);
//        // 新的字符添加到串口的环形缓冲队列中
//        RingBuffer_Push(usart2.recvBuf, ucTemp);
//    }
//}
#endif

#if USART3_ENABLE
void USART3_IRQHandler(void)
{
    uint8_t ucTemp;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        // 从串口读取一个字符
        ucTemp = USART_ReceiveData(usart3.pUSARTx);
        // 新的字符添加到串口的环形缓冲队列中
        RingBuffer_Push(usart3.recvBuf, ucTemp);
    }
}
#endif
