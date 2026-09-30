#include "hwt101.h"

extern UART_HandleTypeDef huart1;

#define HWT_MAGIC 0x48575431u

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

    hwt101_dbg.magic = HWT_MAGIC;
    hwt101_dbg.frames_ok = 0;
    hwt101_dbg.frames_bad = 0;
    hwt101_dbg.yaw_deg = 0.0f;
    hwt101_dbg.yaw_cont_deg = 0.0f;
    hwt101_dbg.gyro_dps = 0.0f;
    hwt101_dbg.last_tick = 0;
    hwt101_dbg.now_tick = 0;

    /* 串口中断默认没开：这里手动使能 USART1 中断并启动单字节接收 */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, &hwt_rx_byte, 1);
}


void HWT101_Update(void)
{
    hwt101_dbg.now_tick = HAL_GetTick();
}


/* 0x55 <类型> D0..D7 SUM；校验失败时丢掉一个字节重新找帧头 */
void HWT101_OnByte(uint8_t b)
{
    uint8_t i;
    uint8_t sum;
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
        hwt101_dbg.frames_bad++;

        /* 在剩余字节里找下一个 0x55，重新同步 */
        for (i = 1; i < 11; i++)
        {
            if (hwt_buf[i] == 0x55)
            {
                break;
            }
        }
        {
            uint8_t n = 11 - i;
            uint8_t k;
            for (k = 0; k < n; k++)
            {
                hwt_buf[k] = hwt_buf[i + k];
            }
            hwt_idx = n;
        }
        return;
    }

    hwt_idx = 0;

    if (hwt_buf[1] == 0x53)            /* 角度帧：偏航角 D4,D5 = buf[6],buf[7] */
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
        hwt101_dbg.yaw_deg = yaw;
        hwt101_dbg.yaw_cont_deg = hwt_cont - hwt_zero;
        hwt101_dbg.last_tick = HAL_GetTick();
    }
    else if (hwt_buf[1] == 0x52)       /* 角速度帧：Z 轴 D4,D5 */
    {
        raw = (int16_t)(((uint16_t)hwt_buf[7] << 8) | hwt_buf[6]);
        hwt101_dbg.gyro_dps = (float)raw * 2000.0f / 32768.0f;
    }
}


float HWT101_GetYaw(void)
{
    return hwt_yaw;
}


float HWT101_GetYawContinuous(void)
{
    return hwt_cont - hwt_zero;
}


void HWT101_ZeroSoft(void)
{
    hwt_zero = hwt_cont;
}


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


/* HAL 中断回调：USART1 收到一个字节 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HWT101_OnByte(hwt_rx_byte);
        HAL_UART_Receive_IT(&huart1, &hwt_rx_byte, 1);
    }
}


/* 溢出/帧错误后 HAL 会停止接收，这里清错误并重新启动 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(&huart1, &hwt_rx_byte, 1);
    }
}
