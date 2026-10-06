#include "stm32f4xx.h"                  // Device header
#include "sensor.h"
#include "motor.h"

#define basic_speed 400  // 自动寻迹基础速度
#define hand_control 300 // 手动控制速度
extern uint8_t distance_info_on;
static uint8_t  C1='S',C2='T',C3='O',C4='K',C5='E',C6='N';
void nano_service(char buffer[],uint8_t len)
{
	//机械臂控制
	if(buffer[0]=='A' && buffer[1]=='R')
	{
//		if(buffer[2]=='F')
//		{
//			arm_find(buffer[3]);
//		}
//		else if(buffer[2]=='C')
//		{
//			get_baowu();
//		}
//		else if(buffer[2]=='1')
//		{
//			int angle = buffer[3] + buffer[4]*255;
//			set_angle(arm1,(float)(angle));
//		}
//		else if(buffer[2]=='2')
//		{
//			int angle = buffer[3] + buffer[4]*255;
//			set_angle(arm2,(float)(angle));
//		}
//		else if(buffer[2]=='3')
//		{
//			int angle = buffer[3] + buffer[4]*255;
//			set_angle(arm3,(float)(angle));
//		}
//		else if(buffer[2]=='4')
//		{
//			int angle = buffer[3] + buffer[4]*255;
//			set_angle(arm4,(float)(angle));
//		}
	}
	//电机控制
	else if(buffer[0]=='M' && buffer[1]=='O')
	{
		if(buffer[2]=='S')//电机停转  STMOS\xFF\xFF
		{
//			stop();
		}
		else if(buffer[2]=='T')//车体自转 STMOT+\x01\x01\x01\x01\xFF\xFF
		{
			int val_l = buffer[5]*256 + buffer[4];
			int val_r = buffer[7]*256 + buffer[6];
			if(buffer[3]=='+')
			{
				Left_Ctrl(val_l);
				Right_Ctrl(-val_r);
			}
			else
			{
				Left_Ctrl(-val_l);
				Right_Ctrl(val_r);
			}
		}
		else if(buffer[2]=='F')//循迹逻辑  STMOF+\x01\x01\xFF\xFF
		{
			int val = buffer[5]*256 + buffer[4];
			if(buffer[3]=='+')
			{
				Left_Ctrl(basic_speed + val);
				Right_Ctrl(basic_speed - val);
			}
			else
			{
				Left_Ctrl(basic_speed - val);
				Right_Ctrl(basic_speed + val);
			}
		}
		else if(buffer[2]=='C') //电脑直接设置轮子占空比 STMOC+\x01\x01\-\x01\x01\xFF\xFF
		{
			int val_l = buffer[6]*256 + buffer[5];
			int val_r = buffer[8]*256 + buffer[7];
			if(buffer[3]=='+'){Left_Ctrl(val_l);}
			else{Left_Ctrl(-val_l);}
			if(buffer[4]=='+'){Right_Ctrl(val_r);}
			else{Right_Ctrl(-val_r);}
		}
		else if(buffer[2]=='L')//低速处理
		{
			int val = buffer[5]*256 + buffer[4];
			if(buffer[3]=='+')
			{
				Left_Ctrl(basic_speed + val - 80);
				Right_Ctrl(basic_speed - val - 80);
			}
			else
			{
				Left_Ctrl(basic_speed - val - 80);
				Right_Ctrl(basic_speed + val - 80);
			}
		}
		else if(buffer[2]=='H') //手动控制
		{
			if(buffer[3]=='+')
			{Left_Ctrl(hand_control);}
			else if(buffer[3]=='-')
			{Left_Ctrl(-hand_control);}
			if(buffer[4]=='+')
			{Right_Ctrl(hand_control);}
			else if(buffer[4]=='-')
			{Right_Ctrl(-hand_control);}
		}
		else if(buffer[2]=='P') // 下台循迹
		{
			int val = buffer[5]*256 + buffer[4];
			if(buffer[3]=='+')
			{
				Left_Ctrl(basic_speed + val-120);
				Right_Ctrl(basic_speed - val - 120);
			}
			else
			{
				Left_Ctrl(basic_speed - val - 120);
				Right_Ctrl(basic_speed + val - 120);
			}
		}
//		else if(buffer[2]=='A' && buffer[3]=='N' && buffer[4]=='G') //依靠901转动一个角度
//		{
//			while(USART_GetFlagStatus(NANO_UART, USART_FLAG_TXE) == RESET);
//			USART_SendData(NANO_UART,'S');
//			switch(buffer[5])
//			{
//				case '1': self_turn(buffer[6],180);  break;
//				case '2': self_turn(buffer[6],90);  break;
//				case '3': self_turn(buffer[6],120);  break;
//				case '4': self_turn(buffer[6],45);  break;
//			}
//			while(USART_GetFlagStatus(NANO_UART, USART_FLAG_TXE) == RESET);
//			USART_SendData(NANO_UART,'E');
//		}
	}
//	else if(buffer[0]=='D' && buffer[1]=='I' && buffer[2]=='S') // 测距获取
//	{
//		if(buffer[3]=='E')
//		{
//			GPIO_WriteBit(GPIOA,GPIO_Pin_8,(BitAction)(0)); // 开灯
//			distance_info_on = 1;
//		}
//		else if(buffer[3]=='F')
//		{
//			GPIO_WriteBit(GPIOA,GPIO_Pin_8,(BitAction)(1)); // 灭灯
//			distance_info_on = 0;
//		}
//	}
//	else if(buffer[0]=='L' && buffer[1]=='E')
//	{
//		if(buffer[2] == 'N')
//		{
//			LED_ON();
//		}
//		else if(buffer[2] == 'F')
//		{
//			LED_OFF();
//		}
//	}
//	//步进电机
//	else if(buffer[0]=='S' && buffer[1]=='T')
//	{
//		set_stepping_motor_dir(buffer[2]);
//		stepping_motor_turn(100*buffer[3],buffer[4]);
//	}
//	//HMI转发
//	else if(buffer[0]=='H' && buffer[1]=='M')
//	{
//		for(u8 i=2;i<len;i++)
//		{
//			while(USART_GetFlagStatus(HMI_UART, USART_FLAG_TXE) == RESET);
//			USART_SendData(HMI_UART,buffer[i]);
//		}
//	}
	//连接检测
	else if(buffer[0]=='M' && buffer[1]=='C' && buffer[2]=='U' && buffer[3]=='C' && buffer[4]=='O' && buffer[5]=='T')
	{
		
		
		
		
		
		
		
		
		
					HAL_UART_Transmit(&huart3, (uint8_t *)&C1, 1, 1000);
					
					HAL_UART_Transmit(&huart3, (uint8_t *)&C2, 1, 1000);
					
					HAL_UART_Transmit(&huart3, (uint8_t *)&C3, 1, 1000);
					
					HAL_UART_Transmit(&huart3, (uint8_t *)&C4, 1, 1000);
					
					HAL_UART_Transmit(&huart3, (uint8_t *)&C5, 1, 1000);
					
					HAL_UART_Transmit(&huart3, (uint8_t *)&C6, 1, 1000);
		
		
		
		
		
		
//		
//		while(USART_GetFlagStatus(NANO_UART, USART_FLAG_TXE) == RESET);
//		USART_SendData(NANO_UART,'S');
//		while(USART_GetFlagStatus(NANO_UART, USART_FLAG_TXE) == RESET);
//		USART_SendData(NANO_UART,'T');
//		while(USART_GetFlagStatus(NANO_UART, USART_FLAG_TXE) == RESET);
//		USART_SendData(NANO_UART,'O');
//		while(USART_GetFlagStatus(NANO_UART, USART_FLAG_TXE) == RESET);
//		USART_SendData(NANO_UART,'K');
//		while(USART_GetFlagStatus(NANO_UART, USART_FLAG_TXE) == RESET);
//		USART_SendData(NANO_UART,'E');
//		while(USART_GetFlagStatus(NANO_UART, USART_FLAG_TXE) == RESET);
//		USART_SendData(NANO_UART,'N');
	}
	else
	{
		for(uint8_t i=0;i<len;i++)
		{
//			while(USART_GetFlagStatus(NANO_UART, USART_FLAG_TXE) == RESET);
//			USART_SendData(NANO_UART,buffer[i]);
			HAL_UART_Transmit(&huart3, (uint8_t *)&buffer[i], 1, 1000);
		}
	}
}
