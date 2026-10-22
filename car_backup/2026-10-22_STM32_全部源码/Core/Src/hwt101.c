#include "hwt101.h"

extern UART_HandleTypeDef huart1;

volatile HWT101_Debug_t hwt101_dbg;

static uint8_t hwt_rx_byte;
static uint8_t hwt_buf[11];
static uint8_t hwt_idx;

static volatile float hwt_yaw = 0.0f;
static volatile float hwt_cont = 0.0f;
static volatile float hwt_zero = 0.0f;
static volatile uint8_t hwt_ready = 0;
static float hwt_last_raw = 0.0f;


void HWT101_Init(void)
{
    hwt_idx = 0;
    hwt_yaw = 0.0f;
    hwt_cont = 0.0f;
    hwt_zero = 0.0f;
    hwt_last_raw = 0.0f;
    hwt_ready = 0;

    hwt101_dbg.frames_ok = 0;
    hwt101_dbg.frames_bad = 0;
    hwt101_dbg.gyro_dps = 0.0f;
    hwt101_dbg.last_tick = 0;

    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, &hwt_rx_byte, 1);
}


void HWT101_Update(void)
{
}


/* 0x55 <类型> D0..D7 SUM；校验失败时找下一个0x55重新同步 */
void HWT101_OnByte(uint8_t b)
{
    uint8_t i, sum;
    int16_t raw;
    float yaw, d;

    if (hwt_idx == 0 && b != 0x55)
    {
        return;
    }

    hwt_buf[hwt_idx++] = b;

    if (hwt_idx < 11)
    {
        return;
    }

    sum = 0;
    for (i = 0; i < 10; i++)
    {
        sum += hwt_buf[i];
    }

    if (sum != hwt_buf[10])
    {
        uint8_t n, k;

        hwt101_dbg.frames_bad++;

        for (i = 1; i < 11; i++)
        {
            if (hwt_buf[i] == 0x55)
            {
                break;
            }
        }
        n = 11 - i;
        for (k = 0; k < n; k++)
        {
            hwt_buf[k] = hwt_buf[i + k];
        }
        hwt_idx = n;
        return;
    }

    hwt_idx = 0;

    if (hwt_buf[1] == 0x53)            /* 角度帧：偏航角在 buf[6],buf[7] */
    {
        raw = (int16_t)(((uint16_t)hwt_buf[7] << 8) | hwt_buf[6]);
        yaw = (float)raw * 180.0f / 32768.0f;

        if (!hwt_ready)
        {
            hwt_cont = yaw;
        }
        else
        {
            d = yaw - hwt_last_raw;
            if (d > 180.0f)       d -= 360.0f;
            else if (d < -180.0f) d += 360.0f;
            hwt_cont += d;
        }

        hwt_last_raw = yaw;
        hwt_yaw = yaw;
        hwt_ready = 1;

        hwt101_dbg.frames_ok++;
        hwt101_dbg.last_tick = HAL_GetTick();
    }
    else if (hwt_buf[1] == 0x52)       /* 角速度帧 */
    {
        raw = (int16_t)(((uint16_t)hwt_buf[7] << 8) | hwt_buf[6]);
        hwt101_dbg.gyro_dps = (float)raw * 2000.0f / 32768.0f;
    }
}


float HWT101_GetYaw(void)           { return hwt_yaw; }
float HWT101_GetYawContinuous(void) { return hwt_cont - hwt_zero; }
void  HWT101_ZeroSoft(void)         { hwt_zero = hwt_cont; }


float HWT101_AngleDiff(float target, float current)
{
    float d = target - current;

    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;

    return d;
}


uint8_t HWT101_IsReady(void)
{
    return hwt_ready;
}


uint8_t HWT101_IsFresh(uint32_t timeout_ms)
{
    return hwt_ready && ((HAL_GetTick() - hwt101_dbg.last_tick) <= timeout_ms);
}


extern void Link_RxCplt(void);
extern void Link_RxRestart(void);
extern void Arm_QR_RxCplt(void);
extern void Arm_QR_RxRestart(void);
extern void Arm_Scr_RxCplt(void);
extern void Arm_Scr_RxRestart(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HWT101_OnByte(hwt_rx_byte);
        HAL_UART_Receive_IT(&huart1, &hwt_rx_byte, 1);
    }
    else if (huart->Instance == USART3)
    {
        Link_RxCplt();
    }
    else if (huart->Instance == UART5)
    {
        Arm_QR_RxCplt();
    }
    else if (huart->Instance == USART2)
    {
        Arm_Scr_RxCplt();                   /* 串口屏发回来的触摸坐标 */
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(&huart1, &hwt_rx_byte, 1);
    }
    else if (huart->Instance == USART3)
    {
        Link_RxRestart();
    }
    else if (huart->Instance == UART5)
    {
        Arm_QR_RxRestart();
    }
    else if (huart->Instance == USART2)
    {
        Arm_Scr_RxRestart();                /* 一定要重新开始接收：不然出一次溢出错误以后，屏上再也按不动 */
    }
}