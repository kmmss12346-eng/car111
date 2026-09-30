#include "hwt101.h"

extern UART_HandleTypeDef huart1;

static uint8_t hwt_byte = 0;
static uint8_t hwt_buf[11] = {0};
static uint8_t hwt_count = 0;

static volatile float hwt_yaw = 0.0f;
static volatile uint8_t hwt_ready = 0;


void HWT101_Init(void)
{
    hwt_count = 0;
    hwt_yaw = 0.0f;
    hwt_ready = 0;
}


void HWT101_Update(void)
{
    HAL_StatusTypeDef status;
    uint8_t sum;
    uint8_t i;
    int16_t raw_yaw;

    status = HAL_UART_Receive(&huart1,
                              &hwt_byte,
                              1,
                              10);

    if (status != HAL_OK)
    {
        return;
    }

    if (hwt_count < 11)
    {
        hwt_buf[hwt_count++] = hwt_byte;
    }

    if (hwt_count == 11)
    {
        if ((hwt_buf[0] == 0x55) &&
            (hwt_buf[1] == 0x53))
        {
            sum = 0;

            for (i = 0; i < 10; i++)
            {
                sum += hwt_buf[i];
            }

            if (sum == hwt_buf[10])
            {
                raw_yaw =
                    (int16_t)(((uint16_t)hwt_buf[7] << 8)
                    | hwt_buf[6]);

                hwt_yaw =
                    (float)raw_yaw * 180.0f / 32768.0f;

                hwt_ready = 1;
            }

            hwt_count = 0;
        }
        else
        {
            for (i = 0; i < 10; i++)
            {
                hwt_buf[i] = hwt_buf[i + 1];
            }

            hwt_count = 10;
        }
    }
}


float HWT101_GetYaw(void)
{
    return hwt_yaw;
}


uint8_t HWT101_IsReady(void)
{
    return hwt_ready;
}