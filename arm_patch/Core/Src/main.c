/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Emm_V5.h"
#include "fashion_star_uart_servo.h"
#include "usart1.h"
#include "chassis.h"
#include "arm.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "hwt101.h"
#include "yaw_control.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
float yaw_target;
float debug_yaw;
float yaw_error;
float yaw_correction;


volatile float hwt_test_yaw = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Link_Init(void);
void Link_RxCplt(void);
void Link_RxRestart(void);
void Link_Poll(void);
void Servo2_MoveRelative(float delta_angle);
void Claw_Set(uint32_t pulse_us);
void Claw_Open(void);
void Claw_Close(void);
void Turntable_Set(uint32_t pulse_us);
void Turntable_GoTo(uint8_t slot);
void Servos_PWM_Test(void);
void Delay_Report(uint32_t ms);

/* ID 1、ID 2 两个串口舵机(都在 usart2 上) */
float Servo_GetAngle(uint8_t id);
void  Servo_SetAngle(uint8_t id, float angle_deg, float speed_dps);
void  Servo_MoveRelative(uint8_t id, float delta_deg, float speed_dps);
void  Servo_Release(uint8_t id);
int   Servo_ReadAngle(uint8_t id, float *angle);
uint32_t Servo_Start(uint8_t id, float *angle_deg, float speed_dps);   /* 只发指令不等，返回估计要多少毫秒 */
void  Servo_Move(uint8_t id, float angle_deg, float speed_dps, float tol_deg, uint32_t extra_ms);   /* 可调到位误差的转动 */
void  Servo_StopIfStuck(uint8_t id, float target);   /* 没转到位又停住了：停在原地，不再一直顶着 */
void  Servo_Report(uint8_t id);                      /* SV?：电压/电流/功率/温度/状态 */
void  Servo_SetPower(uint16_t mw);                   /* 舵机转动时允许的最大功率(参数 SPOW) */
void  Servo_SetComp(uint8_t id, float deg);          /* 舵机往前多给的补偿角度(参数 S1COMP / S2COMP) */
void  Servo_Hold(uint8_t id, float angle_deg);       /* 停在 angle_deg，不再往原来的目标使劲 */
int   Servo_Arrived(uint8_t id, float a, float tol_deg);   /* 到了没有(转过目标一点也算到) */
void  Servo_Params(uint8_t id);                      /* SVP?：舵机内部的保护设置 */
int   Servo_WriteParam(uint8_t id, const char *name, long value);   /* SVW：改舵机内部设置 */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 1 = 开机时执行下面 main() 里 #if SERVO_TEST_ON_BOOT 那一段舵机测试；平时、跑地图都保持 0 */
#define SERVO_TEST_ON_BOOT   1

/* 诊断开关(2026-10-07)：0 = 不启动新加的扫码(UART5)、串口屏(USART2)、夹爪/转盘 PWM，程序和加机械臂之前完全一样；
 *                      1 = 全部启动(正常版本)。
 * 用来判断"轮子不动"是不是新加的这几样引起的：先用 0 烧一次试 send S 100，再用 1 烧一次对比。 */
#define ARM_MODULES_ON       1

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
#if ARM_MODULES_ON
  MX_UART5_Init();
  MX_USART2_UART_Init();
#endif
  /* USER CODE BEGIN 2 */
  Usart_Init();

  HWT101_Init();

  /* 等第一帧陀螺仪角度，再开始接收树莓派指令 */
  {
      uint32_t t0 = HAL_GetTick();
      while (!HWT101_IsReady() && (HAL_GetTick() - t0) < 2000)
      {
      }
  }
  yaw_target = HWT101_GetYaw();      /* 开机时的车头方向 */

  /* 开机先关掉两个舵机的"上电锁力"，设置堵转时降功率，关掉舵机自己的角度限位，再让它们松手 */
  {
      uint8_t v;
      uint8_t id;
      for (id = 1; id <= 2; id++)
      {
          v = 0x00;  FSUS_WriteData(&usart2, id, FSUS_PARAM_POWER_ON_LOCK_SWITCH, &v, 1);
          v = 0x00;  FSUS_WriteData(&usart2, id, FSUS_PARAM_STALL_PROTECT, &v, 1);
          v = 0x00;  FSUS_WriteData(&usart2, id, FSUS_PARAM_ANGLE_LIMIT_SWITCH, &v, 1);   /* 舵机自己的角度限位会把 ID1 卡在 325/396，程序里有限位 */
          Servo_Release(id);
          HAL_Delay(20);
      }
  }

  /* 轮子和升降的驱动器：解除堵转保护并使能。驱动器触发堵转保护后会一直松轴不动，
   * 而驱动器是电池单独供电的，只重启/重新烧录 STM32 清不掉，所以开机先清一次(HOME 时也会清) */
  Car_Motor_Enable();

#if ARM_MODULES_ON
  Arm_Init();                        /* 启动夹爪/转盘 PWM，开始接收二维码 */
  /* 扫码模块、串口屏的中断优先级降到 3，比陀螺仪(1)、树莓派通信(2)低：没接模块时乱码也不会打断陀螺仪 */
  HAL_NVIC_SetPriority(UART5_IRQn, 3, 0);
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
#endif
  Link_Init();                       /* 开始接收树莓派指令 */
#if ARM_MODULES_ON
  HAL_UART_Transmit(&huart3, (uint8_t *)"READY ARM=1\r\n", 13, 100);
#else
  HAL_UART_Transmit(&huart3, (uint8_t *)"READY ARM=0\r\n", 13, 100);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
    static uint32_t last_send = 0;

    Link_Poll();                   /* 有指令就执行，执行完回 DONE 或 ERR */
#if ARM_MODULES_ON
    Arm_Poll();                    /* 处理扫码模块收到的任务码 */
#endif

    if (HAL_GetTick() - last_send >= 100)
    {
        char msg[40];
        int n;

        last_send = HAL_GetTick();
        n = snprintf(msg, sizeof(msg), "YAW100 %ld\r\n",
                     (long)(HWT101_GetYawContinuous() * 100.0f));
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, n, 100);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */


  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 等待 ms 毫秒，同时每 100ms 通过 USART3 发一次角度 */
void Delay_Report(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t last = 0;
    char msg[40];
    int n;

    while ((HAL_GetTick() - t0) < ms)
    {
        if ((HAL_GetTick() - last) >= 100)
        {
            last = HAL_GetTick();
            n = snprintf(msg, sizeof(msg), "YAW100 %ld\r\n",
                                                 (long)(HWT101_GetYawContinuous() * 100.0f));
            HAL_UART_Transmit(&huart3, (uint8_t *)msg, n, 100);
        }
    }
}


/* ======== 树莓派指令通道 (USART3) ========
 * 树莓派 -> STM32(每条一行文本)：
 *   PING | HOME | P2 | YAW? | GET | SET 名字 数值 | CAL [距离mm [速度]]
 *   F <mm> [速度] | S <mm> [速度] | R <度>        前进(负数后退) / 左移(负数右移) / 逆时针转(负数顺时针)
 *                                                   R 可以带 1 位小数(例如 R -1.4)，R 0 = 把现在的车头记为要保持的方向
 *   A <id> <度> | U <id>                            舵机
 *   MOT? | MOT EN                                   读 1~5 号电机驱动器的电压/使能/堵转保护 | 解除堵转保护并使能
 *   !                                               (单独一个字符，不用换行)紧急停车
 * STM32 -> 树莓派：DONE [t=用时ms e=车头误差0.01度] | ERR <原因> | PONG | READY | YAW100 <角度x100> | P 名字=数值 | CAL … | MOT …
 *   转弯时轮子没转起来(陀螺仪几乎不变)回 ERR STALL t=… e=…
 * 同一时间只发一条，等到 DONE/ERR 再发下一条(动作没做完时又来一行，会存下来、做完再执行，见 Link_RxCplt)。 */
#define LINK_BUF 48

static uint8_t           link_rx;
static char              link_line[LINK_BUF];
static uint8_t           link_len = 0;
static char              link_cmd[LINK_BUF];
static volatile uint8_t  link_ready = 0;

static void Link_Reply(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

void Link_Init(void)
{
    link_len = 0;
    link_ready = 0;
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    HAL_UART_Receive_IT(&huart3, &link_rx, 1);
}

/* 在串口中断里调用：收到的字节拼成一行，收到换行就交给主循环去执行；收到 '!' 马上急停 */
void Link_RxCplt(void)
{
    uint8_t b = link_rx;

    if (b == '!')
    {
        car_abort = 1;                         /* 闭环动作每个控制周期都会检查它，马上停车 */
    }
    else if (b == '\r')
    {
        /* 忽略 */
    }
    else if (b == '\n')
    {
        /* 动作执行时 link_ready 已经清掉了(Link_Poll 先取走指令再执行)，所以动作没做完时来的第一行会存下来，
         * 等这个动作做完再执行；存下一行以后、Link_Poll 取走它之前再来的行才丢掉 */
        if (link_len > 0 && !link_ready)
        {
            uint8_t i;
            for (i = 0; i < link_len; i++) link_cmd[i] = link_line[i];
            link_cmd[link_len] = 0;
            link_ready = 1;
        }
        link_len = 0;
    }
    else if (link_len < LINK_BUF - 1)
    {
        link_line[link_len++] = (char)b;
    }
    else
    {
        link_len = 0;                          /* 太长，丢掉这一行 */
    }

    HAL_UART_Receive_IT(&huart3, &link_rx, 1);
}

void Link_RxRestart(void)
{
    link_len = 0;
    HAL_UART_Receive_IT(&huart3, &link_rx, 1);
}

/* 第二站移动(树莓派现在会自己按配置里的 second_relative_moves 逐条发 S、F、R；这个 P2 是兼容旧版树莓派程序用的，
 * 数值要和树莓派配置里的 second_relative_moves 一致)：左移 42、前进 83、顺时针转 60°(绕车中心转) */
#define P2_LEFT_MM   42.0f
#define P2_FWD_MM    83.0f
#define P2_CW_DEG    60.0f
static void Second_Station_Move(void)
{
    Car_Home();
    Car_Move_Align('L', ROUTE_STRAFE_SPEED, P2_LEFT_MM);
    Car_Move_Align('F', ROUTE_SPEED, P2_FWD_MM);
    Car_TurnBy_Center(-P2_CW_DEG);
}

/* 动作做完回复：被急停打断回 ERR ABORT；转弯时轮子没转起来回 ERR STALL t=… e=…；否则回 DONE t=用时(毫秒) e=车头误差(0.01度) */
static void Link_Done(uint32_t t0)
{
    char m[40];
    int n;

    if (car_abort)
    {
        car_abort = 0;
        Link_Reply("ERR ABORT\r\n");
        return;
    }
    n = snprintf(m, sizeof(m), "%s t=%lu e=%ld\r\n", car_stalled ? "ERR STALL" : "DONE",
                 (unsigned long)(HAL_GetTick() - t0), (long)(car_last_err * 100.0f));
    car_stalled = 0;
    HAL_UART_Transmit(&huart3, (uint8_t *)m, (uint16_t)n, 100);
}

/* 主循环里调用：有指令就执行，执行完回 DONE 或 ERR
 * F / S 后面可以多带一个速度(转/分)，例如 "F 300 120" = 用 120转/分 走 300mm；不带就用 ROUTE_SPEED / ROUTE_STRAFE_SPEED
 * 舵机："A 1 90" = 1号舵机转到 90°(绝对角度)；"A 2 -30" = 2号舵机转到 -30°；"U 2" = 2号舵机松手 */
void Link_Poll(void)
{
    char cmd[LINK_BUF];
    char c;
    long v;
    long sp;
    int pr;
    uint16_t speed;
    uint32_t t0;

    if (!link_ready)
    {
        return;
    }
    strcpy(cmd, (const char *)link_cmd);
    link_ready = 0;
    car_abort = 0;                                 /* 清掉上一条留下的急停标志 */
    car_stalled = 0;
    t0 = HAL_GetTick();

    if (strcmp(cmd, "PING") == 0)
    {
        Link_Reply("PONG\r\n");
        return;
    }
    if (strcmp(cmd, "HOME") == 0)
    {
        if (!HWT101_IsFresh(500)) { Link_Reply("ERR GYRO\r\n"); return; }
        Car_Motor_Enable();                        /* 一键流程开始：驱动器解除堵转保护并使能 */
        Car_Home();                                /* 现在的车头方向 = 要保持的方向 */
        Link_Reply("DONE\r\n");
        return;
    }
    if (strcmp(cmd, "MOT?") == 0)
    {
        Car_Motor_Report();                        /* 每个驱动器一行 MOT … */
        Link_Reply("DONE\r\n");
        return;
    }
    if (strcmp(cmd, "MOT EN") == 0)
    {
        Car_Motor_Enable();
        Link_Reply("DONE\r\n");
        return;
    }
    if (strcmp(cmd, "P2") == 0)
    {
        if (!HWT101_IsFresh(500)) { Link_Reply("ERR GYRO\r\n"); return; }
        Second_Station_Move();
        Link_Done(t0);
        return;
    }
    if (strcmp(cmd, "YAW?") == 0)
    {
        char m[32];
        int n = snprintf(m, sizeof(m), "YAW100 %ld\r\n", (long)(HWT101_GetYawContinuous() * 100.0f));
        HAL_UART_Transmit(&huart3, (uint8_t *)m, (uint16_t)n, 100);
        return;
    }
    if (strcmp(cmd, "GET") == 0)
    {
        Car_Param_Dump();
        Arm_Param_Dump();
        Link_Reply("DONE\r\n");
        return;
    }

    /* 在线改参数："SET 名字 数值" */
    if (cmd[0] == 'S' && cmd[1] == 'E' && cmd[2] == 'T' && cmd[3] == ' ')
    {
        char name[12];
        char *p = &cmd[4];
        char *e;
        float val;
        int n = 0;
        int r;

        while (*p != 0 && *p != ' ' && n < 11)
        {
            name[n++] = *p++;
        }
        name[n] = 0;
        if (*p != ' ')                               { Link_Reply("ERR ARG\r\n"); return; }
        val = (float)strtod(p + 1, &e);
        if (e == p + 1 || *e != 0)                   { Link_Reply("ERR ARG\r\n"); return; }
        r = Car_Param_Set(name, val);
        if (r == 0)      r = Arm_Param_Set(name, val);   /* 机械臂参数 */
        if (r == 1)      Link_Reply("DONE\r\n");
        else if (r == 2) Link_Reply("ERR RANGE\r\n");
        else             Link_Reply("ERR NAME\r\n");
        return;
    }

    /* 横移校准："CAL" / "CAL 800" / "CAL 800 170"：不纠偏地左移再右移，自动测出车自己的偏转模型，结果立刻生效并打印 */
    if (cmd[0] == 'C' && cmd[1] == 'A' && cmd[2] == 'L' && (cmd[3] == 0 || cmd[3] == ' '))
    {
        long dist = 800;
        long cs = ROUTE_STRAFE_SPEED;

        if (cmd[3] == ' ')
        {
            char *e1;
            char *e2;
            dist = strtol(&cmd[4], &e1, 10);
            if (e1 == &cmd[4] || dist < 200 || dist > 1500) { Link_Reply("ERR ARG\r\n"); return; }
            if (*e1 == ' ')
            {
                cs = strtol(e1 + 1, &e2, 10);
                if (e2 == e1 + 1 || *e2 != 0 || cs < 20 || cs > 300) { Link_Reply("ERR ARG\r\n"); return; }
            }
            else if (*e1 != 0)                          { Link_Reply("ERR ARG\r\n"); return; }
        }
        if (!HWT101_IsFresh(500))                       { Link_Reply("ERR GYRO\r\n"); return; }
        Car_Strafe_Calib((uint16_t)cs, (float)dist);
        Link_Done(t0);
        return;
    }

    /* 舵机：A + 空格 + ID(1或2) + 空格 + 角度 */
    if (cmd[0] == 'A' && cmd[1] == ' ')
    {
        char *e1;
        char *e2;
        long id = strtol(&cmd[2], &e1, 10);
        long ang;
        if (e1 == &cmd[2] || *e1 != ' ' || (id != 1 && id != 2)) { Link_Reply("ERR ARG\r\n"); return; }
        ang = strtol(e1 + 1, &e2, 10);
        if (e2 == e1 + 1 || *e2 != 0)                            { Link_Reply("ERR ARG\r\n"); return; }
        Servo_SetAngle((uint8_t)id, (float)ang, 0.0f);
        Link_Reply("DONE\r\n");
        return;
    }

    /* 舵机松手："U 1" / "U 2" */
    if (cmd[0] == 'U' && cmd[1] == ' ')
    {
        char *e1;
        long id = strtol(&cmd[2], &e1, 10);
        if (e1 == &cmd[2] || *e1 != 0 || (id != 1 && id != 2)) { Link_Reply("ERR ARG\r\n"); return; }
        Servo_Release((uint8_t)id);
        Link_Reply("DONE\r\n");
        return;
    }

    /* F / S + 空格 + 整数 mm [+ 空格 + 速度]；R + 空格 + 角度(可以带 1 位小数) */
    pr = Car_Parse_Move(cmd, &c, &v, &sp);           /* R 的 v 是角度×10 */
    if (pr != 0)
    {
        if (pr < 0)                                  { Link_Reply("ERR ARG\r\n"); return; }
        if (!HWT101_IsFresh(500))                    { Link_Reply("ERR GYRO\r\n"); return; }

        if (c == 'R')
        {
            if (v > 1800 || v < -1800)               { Link_Reply("ERR RANGE\r\n"); return; }
            if (v != 0)
            {
                Car_TurnBy_Center((float)v / 10.0f); /* 绕车中心转，按累计的目标航向转 */
            }
            else
            {
                Car_Home();                          /* R 0 = 把现在的车头记为要保持的方向 */
            }
        }
        else
        {
            if (v > 2500 || v < -2500)               { Link_Reply("ERR RANGE\r\n"); return; }
            speed = (sp > 0) ? (uint16_t)sp : (uint16_t)((c == 'F') ? ROUTE_SPEED : ROUTE_STRAFE_SPEED);
            if (v != 0)
            {
                if (c == 'F')      Car_Move_Align((v > 0) ? 'F' : 'B', speed, (float)(v > 0 ? v : -v));
                else               Car_Move_Align((v > 0) ? 'L' : 'R', speed, (float)(v > 0 ? v : -v));
            }
        }
        Link_Done(t0);
        return;
    }

    /* 机械臂指令：CLAW / TT / LIFT / GRAB / PLACE / QR / SCR (见 arm.h) */
#if ARM_MODULES_ON
    {
        char aerr[24];
        int ar = Arm_Command(cmd, aerr, (int)sizeof(aerr));

        if (ar > 0)
        {
            Link_Done(t0);
            return;
        }
        if (ar < 0)
        {
            Link_Reply(aerr);
            Link_Reply("\r\n");
            return;
        }
    }
#endif

    Link_Reply("ERR CMD\r\n");
}


#define CLAW_OPEN_US        2090u
#define CLAW_CLOSE_US       2910u
#define CLAW_MIN_US         1700u
#define CLAW_MAX_US         2910u
#define TURNTABLE_SLOT1_US  2608u
#define TURNTABLE_STEP_US   900u
#define TURNTABLE_MIN_US    500u
#define TURNTABLE_MAX_US    2608u

void Claw_Set(uint32_t pulse_us)
{
    if (pulse_us < CLAW_MIN_US) pulse_us = CLAW_MIN_US;
    if (pulse_us > CLAW_MAX_US) pulse_us = CLAW_MAX_US;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_us);
}

void Claw_Open(void)  { Claw_Set(CLAW_OPEN_US); }
void Claw_Close(void) { Claw_Set(CLAW_CLOSE_US); }

void Turntable_Set(uint32_t pulse_us)
{
    if (pulse_us < TURNTABLE_MIN_US) pulse_us = TURNTABLE_MIN_US;
    if (pulse_us > TURNTABLE_MAX_US) pulse_us = TURNTABLE_MAX_US;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_us);
}

void Turntable_GoTo(uint8_t slot)   /* slot = 1/2/3 */
{
    if (slot < 1) slot = 1;
    if (slot > 3) slot = 3;
    Turntable_Set(TURNTABLE_SLOT1_US - (uint32_t)(slot - 1) * TURNTABLE_STEP_US);
}

void Servos_PWM_Test(void)
{
    uint8_t i;

    Claw_Open();   HAL_Delay(1000);
    Claw_Close();  HAL_Delay(1000);
    Claw_Open();   HAL_Delay(1000);

    for (i = 1; i <= 3; i++)
    {
        Turntable_GoTo(i);
        HAL_Delay(1500);
    }
    Turntable_GoTo(1);
}


/* 旧函数，保留：2 号舵机在当前角度上再转 delta_angle 度(改成调用新的安全版本，不会卡死) */
void Servo2_MoveRelative(float delta_angle)
{
    Servo_MoveRelative(2, delta_angle, 50.0f);
}


/* ================= ID 1、ID 2 串口舵机(都在 usart2 上) ================= */
#define SERVO_DEFAULT_SPEED   60.0f     /* 默认转速 度/秒 */
#define SERVO_ACC_MS          100       /* 加速时间 ms */
#define SERVO_DEC_MS          100       /* 减速时间 ms */
#define SERVO_POWER           30000     /* 开机时的最大功率 mW(= 当时 set SPOW 30000 的值；舵机内部 PMAX 已改成 40000)。可以用 SET SPOW 在线改 */
#define SERVO_ARRIVE_TOL      2.0f      /* 离目标多少度以内算到位 */
#define SERVO_WAIT_EXTRA_MS   1000      /* 按速度算的时间之外最多再等多久，到时间没到位就放弃，不再死等 */
#define SERVO_RELEASE_POWER   0         /* 松手时的阻尼功率 mW：0 = 完全松开；机构会掉下来就调大一点 */
#define SERVO_STUCK_DEG       2.0f      /* 停住不动、离目标还差这么多度以上，才去看是不是被挡住 */
#define SERVO_STILL_MS        500       /* 角度这么久变化不到 0.5° 算停住了 */
#define SERVO_CHECK_AFTER_MS  800       /* 开始转以后至少过这么久才判断停没停(舵机起步慢、中途停顿都不算) */
#define SERVO_PUSH_MA         200       /* 停住时电流到这么大(或者舵机报堵转) = 在使劲顶着东西，才让它停在原地 */
#define SERVO_SLOW_EXTRA_MS   2000      /* 到了估计的时间还没到位、但还在动，最多再多等这么久 */
#define SERVO_WAIT_MAX_MS     9000      /* 一次转动最多等这么久(树莓派那边 AF 最多等 10 秒) */
#define SERVO_COMP1_DEG       5.0f      /* ID1 补偿：手臂有摩擦，舵机总停在离目标差 4~5° 的地方，就往前多给 5°，到了目标马上停(SET S1COMP 改) */
#define SERVO_COMP2_DEG       0.0f      /* ID2 补偿(SET S2COMP 改，0 = 不补) */
#define SERVO_FINE_SPEED      15.0f     /* 小动作的最高转速 度/秒 */
#define SERVO_POLL_FINE_MS    8u        /* 离目标 SERVO_FINE_DEG 以内时多久查一次角度(平时 30ms) */

static uint16_t servo_power = SERVO_POWER;      /* 现在用的最大功率(SET SPOW 改) */
static float servo_comp[3] = { 0.0f, SERVO_COMP1_DEG, SERVO_COMP2_DEG };   /* 每个舵机往前多给的角度 */
static float servo_goal[3];                     /* Servo_Start 记下的真正目标 */
static float servo_dir[3];                      /* 转的方向：1 往大、-1 往小、0 = 不知道(读不到起点，不补偿) */

void Servo_SetPower(uint16_t mw)
{
    servo_power = mw;
}

void Servo_SetComp(uint8_t id, float deg)
{
    if (id == 1 || id == 2)
    {
        servo_comp[id] = deg;
    }
}

/* 限位(度，多圈角度)：你之前测好的数 */
#define SERVO1_MIN_DEG    232.0f   /* 测试用：放开限位，测完改回 232.0f */
#define SERVO1_MAX_DEG   440.0f    /* 测试用：放开限位，测完改回 417.6f */
#define SERVO2_MIN_DEG   -1220.0f
#define SERVO2_MAX_DEG    -503.5f

/* 读当前角度(多圈)。读成功返回 1，角度放进 *angle；读失败返回 0 */
int Servo_ReadAngle(uint8_t id, float *angle)
{
    float a = 0.0f;
    if (FSUS_QueryServoAngleMTurn(&usart2, id, &a) == FSUS_STATUS_SUCCESS)
    {
        *angle = a;
        return 1;
    }
    return 0;
}

/* 读当前角度(多圈)，读失败返回 0(只用来打印，不要用它来算目标) */
float Servo_GetAngle(uint8_t id)
{
    float a = 0.0f;
    Servo_ReadAngle(id, &a);
    return a;
}

/* 把目标角度限制在限位里 */
static float Servo_Clamp(uint8_t id, float angle_deg)
{
    float lo = (id == 1) ? SERVO1_MIN_DEG : SERVO2_MIN_DEG;
    float hi = (id == 1) ? SERVO1_MAX_DEG : SERVO2_MAX_DEG;

    if (angle_deg < lo) angle_deg = lo;
    if (angle_deg > hi) angle_deg = hi;
    return angle_deg;
}

/* 只发转动指令，不等它转完(机械臂让 ID1、ID2 同时转就用它)。
 * *angle_deg 会被限位修正；speed_dps<=0 用默认速度。返回估计要多少毫秒转完(含加减速)，id 不对返回 0。
 * 补偿：手臂有摩擦，舵机总停在离目标差几度的地方，所以发给舵机的目标往转的方向多给 servo_comp 度；
 * 等的时候(Servo_Wait、arm.c 的 Servos_To)一到真正的目标就 Servo_Hold 让它停住，不会多转过去。 */
uint32_t Servo_Start(uint8_t id, float *angle_deg, float speed_dps)
{
    float start, d, cmd;

    if (id != 1 && id != 2)
    {
        return 0;
    }
    *angle_deg = Servo_Clamp(id, *angle_deg);
    if (speed_dps <= 0.0f)
    {
        speed_dps = SERVO_DEFAULT_SPEED;
    }

    servo_goal[id] = *angle_deg;
    servo_dir[id] = 0.0f;
    start = *angle_deg;
    if (FSUS_QueryServoAngleMTurn(&usart2, id, &start) == FSUS_STATUS_SUCCESS)
    {
        d = *angle_deg - start;
        if (d > 0.05f)  servo_dir[id] = 1.0f;
        if (d < -0.05f) servo_dir[id] = -1.0f;
    }
    else
    {
        start = *angle_deg + 90.0f;              /* 读不到就按最多转 90° 估时间，也不补偿(不知道往哪边转) */
    }
    d = *angle_deg - start;
    if (d < 0.0f) d = -d;
    if (d < SERVO_FINE_DEG && speed_dps > SERVO_FINE_SPEED)
    {
        speed_dps = SERVO_FINE_SPEED;            /* 小动作慢一点：多给了补偿角度，转得快到了来不及停会转过头 */
    }

    cmd = Servo_Clamp(id, *angle_deg + servo_dir[id] * servo_comp[id]);   /* 往前多给几度，但不超出限位 */
    FSUS_SetServoAngleMTurnByVelocity(&usart2, id, cmd, speed_dps,
                                      SERVO_ACC_MS, SERVO_DEC_MS, servo_power, 0);   /* 最后的 0 = 不用库里会卡死的等待 */
    return (uint32_t)(d / speed_dps * 1000.0f) + SERVO_ACC_MS + SERVO_DEC_MS;
}

/* 让舵机就停在 angle_deg(一般是它现在的位置)：目标改成这个角度，它就不会再使劲往原来的目标(多给了补偿的目标)顶 */
void Servo_Hold(uint8_t id, float angle_deg)
{
    FSUS_SetServoAngleMTurnByVelocity(&usart2, id, angle_deg, SERVO_DEFAULT_SPEED,
                                      SERVO_ACC_MS, SERVO_DEC_MS, servo_power, 0);
}

/* 往目标方向还差多少度(转过了是负数)。target 是 Servo_Start 记下的目标才按方向算，否则按离目标的距离算 */
static float Servo_Left(uint8_t id, float a, float target)
{
    float e = target - a;

    if (target == servo_goal[id] && servo_dir[id] > 0.5f)
    {
        return e;
    }
    if (target == servo_goal[id] && servo_dir[id] < -0.5f)
    {
        return -e;
    }
    return (e < 0.0f) ? -e : e;
}

/* 到了没有：离目标 tol_deg 以内，或者已经转过了目标(往前多给了补偿角度，转过一点也算到) */
int Servo_Arrived(uint8_t id, float a, float tol_deg)
{
    if (id != 1 && id != 2)
    {
        return 0;
    }
    return Servo_Left(id, a, servo_goal[id]) <= tol_deg;
}

/* 读舵机现在的电流(mA)和状态字节(见 fashion_star_uart_servo.h 的 FSUS_PARAM_SERVO_STATUS)。读不到是 -1 / 0 */
static void Servo_Load(uint8_t id, long *ma, uint8_t *st)
{
    uint8_t buf[FSUS_PACK_RESPONSE_MAX_SIZE];
    uint8_t sz = 0;

    *ma = -1;
    *st = 0;
    if (FSUS_ReadData(&usart2, id, FSUS_PARAM_CURRENT, buf, &sz) == FSUS_STATUS_SUCCESS && sz >= 2)
    {
        *ma = (long)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    }
    sz = 0;
    if (FSUS_ReadData(&usart2, id, FSUS_PARAM_SERVO_STATUS, buf, &sz) == FSUS_STATUS_SUCCESS && sz >= 1)
    {
        *st = buf[0];
    }
}

static void Servo_Say(const char *what, uint8_t id, float a, float target, long ma, uint8_t st)
{
    char m[112];
    int n = snprintf(m, sizeof(m), "SERVO%d %s at=%ld target=%ld (x0.1deg) I=%ldmA ST=0x%02X\r\n",
                     (int)id, what, (long)(a * 10.0f), (long)(target * 10.0f), ma, (unsigned)st);
    HAL_UART_Transmit(&huart3, (uint8_t *)m, (uint16_t)n, 100);
}

/* 等舵机转到 target(离 tol_deg 以内，或者已经转过一点)。tmax = 估计要多久，check_after = 开始多久以后才判断"停住了"。到位返回 1。
 * 停住了(SERVO_STILL_MS 内变化不到 0.5°)又离目标还远：
 *   - 电流大(>= SERVO_PUSH_MA)或舵机报堵转 = 被挡住了：让它停在原地，不再一直使劲顶(顶着几十秒就烫手)，打印 STUCK；
 *   - 电流小 = 没在使劲，只是停顿了：重新发一次目标(打印 PAUSED)，接着等。重发过还停着就不再等。
 * 到了估计的时间还在动(舵机比设定的速度慢)：最多再多等 SERVO_SLOW_EXTRA_MS。
 * 不管怎么结束，最后都让它停在现在的位置(Servo_Hold)：到了就不再往多给的补偿角度使劲，没到也不再一直顶着。 */
static int Servo_Wait(uint8_t id, float target, float speed_dps, float tol_deg, uint32_t tmax, uint32_t check_after)
{
    float a = 0.0f;
    float ref = 0.0f;
    float d;
    uint32_t t0 = HAL_GetTick();
    uint32_t tref = t0;
    uint32_t el;
    int have = 0;
    int kicked = 0;
    long ma;
    uint8_t st;
    uint32_t poll = 30u;
    float left;

    for (;;)
    {
        HAL_Delay(poll);
        el = HAL_GetTick() - t0;
        if (FSUS_QueryServoAngleMTurn(&usart2, id, &a) == FSUS_STATUS_SUCCESS)
        {
            left = Servo_Left(id, a, target);
            if (left <= tol_deg)
            {
                Servo_Hold(id, a);                 /* 到了(或者刚转过一点)：就停在这里，不再往多给的补偿角度使劲 */
                return 1;
            }
            poll = (left < SERVO_FINE_DEG) ? SERVO_POLL_FINE_MS : 30u;   /* 快到了查勤一点，到了马上停 */
            d = a - ref;
            if (d < 0.0f) d = -d;
            if (!have || d > 0.5f)
            {
                ref = a;                           /* 还在动 */
                tref = HAL_GetTick();
                have = 1;
            }
            else
            {
                uint32_t still = HAL_GetTick() - tref;

                d = a - target;
                if (d < 0.0f) d = -d;
                if (d <= SERVO_STUCK_DEG)
                {
                    if (el >= (uint32_t)SERVO_ACC_MS + 300u && still >= 300u)
                    {
                        break;                     /* 停在目标附近了(只差一点点)：就这样 */
                    }
                }
                else if (el >= check_after && still >= (uint32_t)SERVO_STILL_MS)
                {
                    Servo_Load(id, &ma, &st);      /* 停住了，离目标还远：看它在不在使劲 */
                    if (ma >= SERVO_PUSH_MA || (st & 0x04u))
                    {
                        Servo_Hold(id, a);         /* 在使劲顶着东西：停在原地 */
                        Servo_Say("STUCK, hold here", id, a, target, ma, st);
                        return 0;
                    }
                    if (kicked)
                    {
                        break;                     /* 重发过还是停着：不再等 */
                    }
                    kicked = 1;                    /* 没使劲，只是停顿了：重发一次目标 */
                    Servo_Say("PAUSED, resend", id, a, target, ma, st);
                    tmax = el + Servo_Start(id, &target, speed_dps) + 400u;
                    tref = HAL_GetTick();
                    continue;
                }
            }
        }
        if (el >= (uint32_t)SERVO_WAIT_MAX_MS || el >= tmax + (have ? (uint32_t)SERVO_SLOW_EXTRA_MS : 0u))
        {
            break;
        }
    }
    if (tol_deg >= 1.0f || kicked)
    {
        Servo_Load(id, &ma, &st);
        Servo_Say("NOT ARRIVED", id, a, target, ma, st);
    }
    Servo_Hold(id, have ? a : target);             /* 没转到：停在现在的位置(一次都没读到就停在目标)，不再往前使劲 */
    return 0;
}

/* 转到绝对角度 angle_deg(度)，speed_dps 速度(度/秒，填 0 用默认值)，离目标 tol_deg 度以内算到位。
 * 不用库里的 FSUS_Wait(它没到位会一直等半个多小时、舵机一直使劲顶着发热)，等法见 Servo_Wait。
 * tol_deg 大(>=1°)才在没到位时通过串口打印提示；微调用的小误差不打印(卡住/停顿一定打印)。 */
void Servo_Move(uint8_t id, float angle_deg, float speed_dps, float tol_deg, uint32_t extra_ms)
{
    uint32_t tmax;

    if (id != 1 && id != 2)
    {
        return;
    }
    tmax = Servo_Start(id, &angle_deg, speed_dps) + extra_ms;
    Servo_Wait(id, angle_deg, speed_dps, tol_deg, tmax, SERVO_CHECK_AFTER_MS);
}

/* 给 arm.c 用(ID1、ID2 一起转，等到时间还没到位的那个)：接着等它到位，处理办法和 Servo_Move 一样
 * (在使劲顶 = 停在原地；只是停顿 = 重发一次；还在慢慢转 = 多等一会儿) */
void Servo_StopIfStuck(uint8_t id, float target)
{
    Servo_Wait(id, target, 0.0f, 1.0f, (uint32_t)SERVO_STILL_MS + 1000u, 0u);
}

/* SV? <id>：读舵机的角度、电压、电流、功率、温度、状态，查"发热 / 没劲 / 转不到位"用。例如
 *   SV 1 ANG=367.6 V=7412mV I=850mA P=6300mW T=58C(adc=1500) ST=0x04 STALL
 * 停着不动时电流还很大(几百 mA 以上) = 舵机在使劲顶着什么东西(被挡住、或者有外力在推它) */
void Servo_Report(uint8_t id)
{
    static const uint8_t addr[5] = { FSUS_PARAM_VOLTAGE, FSUS_PARAM_CURRENT, FSUS_PARAM_POWER,
                                     FSUS_PARAM_TEMPRATURE, FSUS_PARAM_SERVO_STATUS };
    static const char *flag[8] = { " BUSY", " CMDERR", " STALL", " VHIGH", " VLOW", " OVERCURRENT", " OVERPOWER", " OVERHEAT" };
    long v[5];
    uint8_t buf[FSUS_PACK_RESPONSE_MAX_SIZE];
    uint8_t sz;
    float a = 0.0f;
    long ta, tc = -999;
    char m[160];
    int i, n;

    for (i = 0; i < 5; i++)
    {
        v[i] = -1;
        sz = 0;
        if (FSUS_ReadData(&usart2, id, addr[i], buf, &sz) == FSUS_STATUS_SUCCESS && sz >= 1)
        {
            v[i] = (sz >= 2) ? (long)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8)) : (long)buf[0];
        }
        HAL_Delay(5);
    }
    if (v[3] > 0 && v[3] < 4096)                   /* 温度：NTC 10k、B=3435 的分压 ADC 值换算成摄氏度(约) */
    {
        float r = (float)v[3] / (4096.0f - (float)v[3]);
        tc = (long)(1.0f / (logf(r) / 3435.0f + 1.0f / 298.15f) - 273.15f + 0.5f);
    }
    if (!Servo_ReadAngle(id, &a))
    {
        a = 0.0f;
    }
    ta = (long)(a * 10.0f + ((a >= 0.0f) ? 0.5f : -0.5f));
    n = snprintf(m, sizeof(m), "SV %d ANG=%s%ld.%ld V=%ldmV I=%ldmA P=%ldmW T=%ldC(adc=%ld) ST=0x%02lX",
                 (int)id, (ta < 0) ? "-" : "", ((ta < 0) ? -ta : ta) / 10, ((ta < 0) ? -ta : ta) % 10,
                 v[0], v[1], v[2], tc, v[3], (v[4] < 0) ? 0xFFL : v[4]);
    for (i = 1; i < 8 && v[4] >= 0; i++)           /* bit0(正在执行)不打印 */
    {
        if ((v[4] >> i) & 1L)
        {
            n += snprintf(m + n, sizeof(m) - (size_t)n, "%s", flag[i]);
        }
    }
    n += snprintf(m + n, sizeof(m) - (size_t)n, "\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t *)m, (uint16_t)n, 100);
}

/* 舵机内部的设置(飞特舵机的"用户数据"，存在舵机里，断电不丢)。SVP? 打印，SVW 改。地址见 fashion_star_uart_servo.h */
typedef struct
{
    uint8_t     addr;
    uint8_t     size;          /* 1 或 2 字节 */
    uint8_t     sgn;           /* 1 = 有符号 */
    const char *name;
} SvParam;

static const SvParam sv_params[] =
{
    { FSUS_PARAM_RESPONSE_SWITCH,      1, 0, "RESP"   },   /* 0 = 新指令直接覆盖旧指令(程序要求是 0) */
    { FSUS_PARAM_STALL_PROTECT,        1, 0, "STALLM" },   /* 堵转保护：0 = 降功率到 STALLP，1 = 松开 */
    { FSUS_PARAM_STALL_POWER_LIMIT,    2, 0, "STALLP" },   /* 堵转时降到多少 mW */
    { FSUS_PARAM_OVER_VOLT_LOW,        2, 0, "VLOW"   },   /* 电压低于这个(mV)保护 */
    { FSUS_PARAM_OVER_VOLT_HIGH,       2, 0, "VHIGH"  },   /* 电压高于这个(mV)保护 */
    { FSUS_PARAM_OVER_TEMPERATURE,     2, 0, "TMAX"   },   /* 温度上限(℃) */
    { FSUS_PARAM_OVER_POWER,           2, 0, "PMAX"   },   /* 功率上限(mW)：超过就报 OVERPOWER(ST 的 0x40)并限功率 */
    { FSUS_PARAM_OVER_CURRENT,         2, 0, "IMAX"   },   /* 电流上限(mA) */
    { FSUS_PARAM_ACCEL_SWITCH,         1, 0, "ACCEL"  },
    { FSUS_PARAM_POWER_ON_LOCK_SWITCH, 1, 0, "LOCK"   },
    { FSUS_PARAM_ANGLE_LIMIT_SWITCH,   1, 0, "ALIM"   },   /* 舵机自己的角度限位开关 */
    { FSUS_PARAM_SOFT_START_SWITCH,    1, 0, "SOFT"   },
    { FSUS_PARAM_SOFT_START_TIME,       2, 0, "SOFTT"  },
    { FSUS_PARAM_ANGLE_LIMIT_HIGH,     2, 1, "AHIGH"  },   /* 0.1 度 */
    { FSUS_PARAM_ANGLE_LIMIT_LOW,      2, 1, "ALOW"   },
};
#define SV_PARAM_N  ((int)(sizeof(sv_params) / sizeof(sv_params[0])))

/* SVP? <id>：例如 "SVP 1 RESP=0 STALLM=0 STALLP=4000 VLOW=4500 … PMAX=8000 IMAX=1500 …"(读不到的是 ?) */
void Servo_Params(uint8_t id)
{
    uint8_t buf[FSUS_PACK_RESPONSE_MAX_SIZE];
    uint8_t sz;
    char m[320];
    int n, i;
    long v;

    n = snprintf(m, sizeof(m), "SVP %d", (int)id);
    for (i = 0; i < SV_PARAM_N && n < (int)sizeof(m) - 24; i++)
    {
        sz = 0;
        if (FSUS_ReadData(&usart2, id, sv_params[i].addr, buf, &sz) == FSUS_STATUS_SUCCESS && sz >= 1)
        {
            if (sz >= 2)
            {
                v = (long)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
                if (sv_params[i].sgn && v >= 32768L)
                {
                    v -= 65536L;
                }
            }
            else
            {
                v = (long)buf[0];
            }
            n += snprintf(m + n, sizeof(m) - (size_t)n, " %s=%ld", sv_params[i].name, v);
        }
        else
        {
            n += snprintf(m + n, sizeof(m) - (size_t)n, " %s=?", sv_params[i].name);
        }
        HAL_Delay(5);
    }
    n += snprintf(m + n, sizeof(m) - (size_t)n, "\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t *)m, (uint16_t)n, 200);
}

/* SVW <id> <名字> <数值>：改一项舵机内部设置。返回 1 成功，0 没有这个名字，2 数值不对，-1 写失败。
 * 舵机 ID、波特率不在表里，改不了(改错了 STM32 就再也找不到舵机)；RESP 只能写 0 */
int Servo_WriteParam(uint8_t id, const char *name, long value)
{
    uint8_t b[2];
    int i;

    for (i = 0; i < SV_PARAM_N; i++)
    {
        if (strcmp(sv_params[i].name, name) == 0)
        {
            const SvParam *p = &sv_params[i];
            long lo = p->sgn ? -32768L : 0L;
            long hi = (p->size == 1) ? 255L : (p->sgn ? 32767L : 65535L);

            if (value < lo || value > hi || (p->addr == FSUS_PARAM_RESPONSE_SWITCH && value != 0))
            {
                return 2;
            }
            b[0] = (uint8_t)((unsigned long)value & 0xFFu);
            b[1] = (uint8_t)(((unsigned long)value >> 8) & 0xFFu);
            return (FSUS_WriteData(&usart2, id, p->addr, b, p->size) == FSUS_STATUS_SUCCESS) ? 1 : -1;
        }
    }
    return 0;
}

/* 原来的接口：转到绝对角度，到位误差 2°，多等 1 秒 */
void Servo_SetAngle(uint8_t id, float angle_deg, float speed_dps)
{
    Servo_Move(id, angle_deg, speed_dps, SERVO_ARRIVE_TOL, SERVO_WAIT_EXTRA_MS);
}

/* 在当前角度上再转 delta_deg 度(正负都可以)，同样受限位保护。
 * 读不到当前角度时不转，只打印提示，防止按 0° 算出错误目标一下转好几圈 */
void Servo_MoveRelative(uint8_t id, float delta_deg, float speed_dps)
{
    float a0;
    if (!Servo_ReadAngle(id, &a0))
    {
        char m[48];
        int n = snprintf(m, sizeof(m), "SERVO%d READ FAIL, NOT MOVED\r\n", (int)id);
        HAL_UART_Transmit(&huart3, (uint8_t *)m, (uint16_t)n, 100);
        return;
    }
    Servo_SetAngle(id, a0 + delta_deg, speed_dps);
}

/* 松手：进入阻尼模式，舵机不再主动出力，就不会一直发热。下次 Servo_SetAngle 会自动重新出力。
 * 注意：松手后机构如果会因为重力掉下来，就别在那个姿势松手，或者把 SERVO_RELEASE_POWER 调大(有阻尼，慢慢落) */
void Servo_Release(uint8_t id)
{
    FSUS_DampingMode(&usart2, id, SERVO_RELEASE_POWER);
}


void UART4_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart4, &Byte, 1, 1000);
}

void UART4_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;

    for (i = 0; i < Length; i++)
    {
        UART4_SendByte(Array[i]);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
