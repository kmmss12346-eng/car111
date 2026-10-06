/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include <string.h>
#include "sensor.h"
#include "motor.h"
#include "nano.h"
#include "Emm_V5.h"
#include "stdio.h" 
#include "stdarg.h"
#include "DataScope_DP.h"
#include "Task.h" 
#include "Bujin.h" 
#include "RGB.h"
#include "sw_i2c.h"
#include "gw_grayscale_sensor.h"
#include "ring_buffer.h"
#include "usart1.h"
#include "fashion_star_uart_servo.h"
#include "fashion_star_uart_servo_examples.h"
#include "sys_tick.h"
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

/* USER CODE BEGIN PV */
extern uint8_t uart2_rxbuff,uart3_rxbuff,uart1_rxbuff,uart4_rxbuff,uart5_rxbuff,uart6_rxbuff;
extern uint8_t chuankouping_progress,shibie_progress;
extern uint8_t task[3];
extern uint8_t shibie_progress;

              
extern uint8_t mode1,mode2;

 static uint8_t  C1='S',C2='T',C3='O',C4='K',C5='E',C6='N';


extern uint8_t liaopan[4];

extern uint8_t duoji1_angle;

extern float angle[3];

uint8_t b[4]={0x01,0xFD,0X9F,0X6B};

uint8_t a;



uint8_t scan_addr[128];//设备地址
volatile uint8_t count;//iic总线上设备数量
uint8_t digital_data;
uint8_t R=2,G=2,B=2;//RGB模式下为RGB参数 HSL模式下为HSL参数



 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  

  HAL_UART_Receive_IT(&huart2,&uart2_rxbuff,1);
  HAL_UART_Receive_IT(&huart3,&uart3_rxbuff,1); 
  HAL_UART_Receive_IT(&huart1,&uart1_rxbuff,1);  
  HAL_UART_Receive_IT(&huart4,&uart4_rxbuff,1);
  HAL_UART_Receive_IT(&huart5,&uart5_rxbuff,1);
  HAL_UART_Receive_IT(&huart6,&uart6_rxbuff,1);
   
   
//   HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);//PC8
   
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);//PA2
//  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//PA5
//  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//PB3
//  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);//PA3
//  HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_1);
 
 
  /*OLED初始化*/
	OLED_Init();
	OLED_Clear();
	/*调用OLED_Update函数，将OLED显存数组的内容更新到OLED硬件进行显示*/
	OLED_Update();
	
//   UART_printf(&huart2,"x4.val=%d\xff\xff\xff",1000);
	task_Init();
//	  UART_printf(&huart2,"x4.val=%d\xff\xff\xff",1000);





//      test=FSUS_Ping(&usart2, 3); //通讯测试
//      FSUS_SetServoAngle(&usart2,3,0.0,20,0,0);//简易控制
//		FSUS_SetServoAngleByInterval(&usart2,3,35.0,1500,20,700,0,0);
//	  FSUS_SetServoAngleByInterval(&usart2,3,-145.0,1500,20,700,0,0);
	   
//    FSUS_SetServoAngleByVelocity(&usart2,3,-180,15,50,50,0,1);//加减速控制

	
/******************************夹爪抖动测试*************************************/
//	duoji_jiazhao(duoji_jiazhao_jiajin);
//	HAL_Delay(2000);
//	duoji_zhedie_slowaction(duoji_zhedie_up);
//	HAL_Delay(2000);
//	duoji_zhedie_slowaction(duoji_zhedie_down);
//	HAL_Delay(2000);

//HAL_Delay(1000);
//	duoji_jiazhao(duoji_jiazhao_songkai);
//	HAL_Delay(200);
//	
//	duoji_jiazhao_slowaction(duoji_jiazhao_jiajin);
//	
//	HAL_Delay(2000);
//	duoji_jiazhao_slowaction(duoji_jiazhao_songkai);
	
//	while(1);
	
	
	

	
	
	
	// DWT_Init();
	
	/* 设置软件IIC驱动 */
	// sw_i2c_interface_t i2c_interface = 
	// {
	// 	.sda_in = sda_in,
	// 	.scl_out = scl_out,
	// 	.sda_out = sda_out,
	// 	.user_data = 0, //用户数据，可在输入输出函数里得到
	// };
	
	
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  



//while(1)
//{
// for(uint8_t i=0;i<3;i++)
//{
//	HAL_UART_Transmit(&huart3, (uint8_t *)&modetwo[i], 1, 1000);
//}
//HAL_Delay(5);
//}
  while (1)
  {
//	  if(flag_saomaqi==1)
//	  {
//		  UART_printf(&huart2,"t2.txt=\"%s\"\xff\xff\xff",paixu);//可以这样发连续的,串口屏显示
//		  flag_saomaqi=0;
//	  }
	  
	 while(tiaoshi_flag==2)//任务模式
	 {
		 if(get_taskflag(1)==1)
		 {
			  task1(); 
//			 task_Init();
//			 jiguang(0);
//			send_yaw0();
//			HAL_Delay(500);
//			yaw_begin=angle[2];
//	
//			duoji_zhedie(duoji_zhedie_down);
//			duoji_jiazhao(duoji_jiazhao_songkai);
			
//			 send_yaw0();
//		HAL_Delay(500);
//		yaw_begin=angle[2];
//		 HAL_Delay(3000);
//	 angle_tiaozheng(-90);
		 }
		 else if(get_taskflag(2)==1)
		 {
			  task2(); 
		 }
		 else if(get_taskflag(3)==1)
		 {
			  task3(); 
		 }
		 else if(get_taskflag(4)==1)
		 {
			  task4(); 
		 }
		 else if(get_taskflag(5)==1)
		 {
			  qidong_to_yuanliao(); 
		 }
		 else if(get_taskflag(6)==1)
		 {
			yuanliao_to_cu();
		 }
		 else if(get_taskflag(7)==1)
		 {
			  cu_to_zan(1); 	
		 }
		 else if(get_taskflag(8)==1)
		 {
			zan_to_yuanliao();
		 }
		 else if(get_taskflag(9)==1)//测试自转角度
		 {
			 task9(); 
		 }
		 else if(get_taskflag('A')==1)
		 {
			task_yuanliao(1);
		 }
		  else if(get_taskflag('B')==1)
		 {
			task_cujiagong(1);
		 }
		  else if(get_taskflag('C')==1)
		 {
			 task_zancunqu(1);	
		 }
		  else if(get_taskflag('D')==1)
		 {
			 task_Init();
			
		 }
		  else if(get_taskflag('F')==1)
		 {
			 task_all(); 
			
		 }
		
        // sw_i2c_hal_stop(&i2c_interface);        
		}
	while(tiaoshi_flag==1)//调试模式
	{
		  OLEDShow_Angle();
//			send_yaw0();
	}
	 


	 // sw_i2c_write_byte(&i2c_interface, 0x4C << 1, GW_GRAY_DIGITAL_MODE);      
        // sw_i2c_read_byte(&i2c_interface, 0x4C << 1, &digital_data);
        // R = sw_i2c_hal_read_byte(&i2c_interface, ACK);
        // G = sw_i2c_hal_read_byte(&i2c_interface, ACK);
        // B = sw_i2c_hal_read_byte(&i2c_interface, ACK);
	
	
	//夹爪舵机A3,升降云台舵机A5,物料云台B3
//	   while(1);
//	   move_front(200,10,4*3200,0);
//	   while(1); 


	  
	  
	  //3200脉冲=247mm，
	  //计算结果为77*3.141=241.857
	  
	  
	  
	  
	  
	  
	  
	  
//	  printf("t2.txt=\"到位OK\"\xff\xff\xff");
//		  UART_printf(&huart2,"t2.txt=\"%s\"\xff\xff\xff",RxBuffer1);//可以这样发连续的	
//	  printf("t2.txt=\"%s\"\xff\xff\xff",RxBuffer1);


















	  
	 

	  
	  /**********************************示波器Minibalance************************************
			
//	        DataScope_Get_Channel_Data(encoder_count4, 1 );
//			DataScope_Get_Channel_Data(Target_Velocity, 2 );// Incremental_PI_1


//			uint8_t Send_Count = DataScope_Data_Generate(2);
//			for(uint8_t i = 0 ; i < Send_Count; i++) 
//			{
//			while((UART4->SR&0X40)==0);  
//			UART4->DR = DataScope_OutPut_Buffer[i]; 
//			}
//			HAL_Delay(50);
	
	
	
//	       a+=0.1;
//			if(a>3.14)  a=-3.14; 
//			DataScope_Get_Channel_Data(500*sin(a), 1 );
//			DataScope_Get_Channel_Data(500* tan(a), 2 );
//			DataScope_Get_Channel_Data( 500*cos(a), 3 ); 
//			DataScope_Get_Channel_Data( 100*a , 4 );   
//			DataScope_Get_Channel_Data(0, 5 );
//			DataScope_Get_Channel_Data(0 , 6 );
//			DataScope_Get_Channel_Data(0, 7 );
//			DataScope_Get_Channel_Data( 0, 8 ); 
//			DataScope_Get_Channel_Data(0, 9 );  
//			DataScope_Get_Channel_Data( 0 , 10);
//			Send_Count = DataScope_Data_Generate(10);
//			for( i = 0 ; i < Send_Count; i++) 
//			{
//			while((USART2->SR&0X40)==0);  
//			USART2->DR = DataScope_OutPut_Buffer[i]; 
//			}
//			HAL_Delay(50);
	**********************************示波器Minibalance************************************/
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	 
//	  uint8_t a[3]={1,0x02,0x03};
//	   UART_printf(&huart4,"%s",a);//可以这样发连续的
//	  while(1);
	//	  OLEDShow_Angle();
//	  UART_printf(&huart4,"DIFF=\r\n"); 
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C1, 1, 1000);
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C2, 1, 1000);
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C3, 1, 1000);
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C4, 1, 1000);
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C5, 1, 1000);
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C6, 1, 1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      


	  
//	  printf("page1_3.x0.val=%d\xff\xff\xff",a);
//	  
//	  a++;
//	  HAL_Delay(1000);
	  
//	 OLEDShow_Angle();
//	 if(memcmp(b,bujin_array1,4)==0)
//	{
//				OLEDShow_Angle();			
//	}
//	  a=1;
//	while(a==0); 	a = 0;
//	  OLEDShow_Angle();	
		
	  
//	 
//////	  
//HAL_Delay(8000);


//		Emm_V5_Pos_Control(2, 1, 4000, 255, 109081, 0, 1);
//////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);
//		Emm_V5_Pos_Control(1, 1, 4000, 255,11454, 0, 1);	
//		HAL_Delay(100);
//////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);		
//		Emm_V5_Synchronous_motion(0x00);
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
////		HAL_Delay(100);
////		while(bujin1_daoweiflag == false||bujin2_daoweiflag == false);bujin1_daoweiflag = false;bujin2_daoweiflag = false;
//		HAL_Delay(500);
//		
//		
//		
//		Emm_V5_Pos_Control(2, 0, 5, 10, 190, 0, true);
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);
//		Emm_V5_Pos_Control(1, 0, 5, 10, 0, 0, true);	
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);		
//		Emm_V5_Synchronous_motion(0x00);
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);
////	while(bujin1_daoweiflag == false||bujin2_daoweiflag == false);bujin1_daoweiflag = false;bujin2_daoweiflag = false;
//		HAL_Delay(500);
//			
//		Emm_V5_Pos_Control(2, 0, 5, 10, 0, 0, true);
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);
//		Emm_V5_Pos_Control(1, 0, 5, 10, 180, 0, true);	
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);		
//		Emm_V5_Synchronous_motion(0x00);
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);
////		while(bujin1_daoweiflag == false||bujin2_daoweiflag == false);bujin1_daoweiflag = false;bujin2_daoweiflag = false;
//		HAL_Delay(500);
//		
//		
//		Emm_V5_Pos_Control(2, 1, 7, 10, 175, 0, true);
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);
//		Emm_V5_Pos_Control(1, 1, 5, 10, 0, 0, true);	
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);		
//		Emm_V5_Synchronous_motion(0x00);
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);
////		while(bujin1_daoweiflag == false||bujin2_daoweiflag == false);bujin1_daoweiflag = false;bujin2_daoweiflag = false;
//		HAL_Delay(500);
////		


//		Emm_V5_Pos_Control(2, 1, 1, 10, 10, 0, true);
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);
//		Emm_V5_Pos_Control(1, 1, 5, 10, 180, 0, true);	
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);		
//		Emm_V5_Synchronous_motion(0x00);
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
//		HAL_Delay(100);
////		while(bujin1_daoweiflag == false||bujin2_daoweiflag == false);bujin1_daoweiflag = false;bujin2_daoweiflag = false;
//		HAL_Delay(1000);





//		 Emm_V5_Origin_Trigger_Return(0x02,0,1);
//		 HAL_Delay(100);
//		 
//		Emm_V5_Origin_Trigger_Return(0x01,0,1);
//		HAL_Delay(100);
//		Emm_V5_Synchronous_motion(0x00);
//		
//		
////		Emm_V5_Pos_Control(2, 0, 100, 10, 100, 0, true);
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
////		Emm_V5_Pos_Control(1, 0, 100, 10, 100, 0, true);	
////		while(rxFrameFlag == false); 	rxFrameFlag = false;	  
////		Emm_V5_Synchronous_motion(0x00);
////		while(rxFrameFlag == false); 	rxFrameFlag = false;
////		while(bujin1_daoweiflag == false||bujin2_daoweiflag == false);bujin1_daoweiflag = false;bujin2_daoweiflag = false;

//		while(1);
		
		

//		HAL_Delay(1000);
//	  OLED_ShowChar(0,0,task[0],OLED_8X16);
//	  OLED_ShowNum(0,16,duoji1_angle,4,OLED_8X16);
//	  OLED_ShowChar(0,32,task[1],OLED_8X16);
//	  OLED_Update();
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

/* USER CODE BEGIN 4 */

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
