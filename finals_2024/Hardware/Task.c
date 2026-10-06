#include "Task.h"
#include "sensor.h"
#include "fashion_star_uart_servo.h"
uint8_t saoma_chufa[9]={0x7E,0X00,0X08,0X01,0X00,0X02,0X01,0XAB,0XCD};

#define di_yuanpan_zuo 3
#define di_yuanpan_zhong 2
#define di_yuanpan_you 1



uint32_t duoji_zhuanbi_DMA[8];
uint32_t duoji_liaopan_DMA[5];
uint32_t duoji_jiazhao_DMA[5];



uint8_t liaopan[4]={0,0,0,0};
uint8_t mode1='1',mode2='2';
uint8_t modeone[3]={'0','1','9'};
uint8_t modetwo[3]={'0','4','9'};
uint8_t modethree[3]={'0','5','9'};


uint32_t high_yuanpan=6920+79.53*0;//127mm对应10100脉冲，1mm=79.53个脉冲
//学校场地圆盘高度为80mm


uint8_t move_slow_flag=0;

uint8_t angle_tiaozheng_ok=0;

float yaw_begin,yaw_tiaozheng1st;

uint8_t X_last,Y_last;

uint16_t move_tiaozheng=0;

uint8_t a1,a2,a3,a4;


uint8_t get_taskflag(uint8_t task)
{
	uint8_t task_flag=0;
	switch(task)
	{
		case 1:
			if(task1_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			task1_flag=0;
		    break;
		
		case 2:
			if(task2_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			task2_flag=0;
		    break;
		case 3:
			if(task3_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			task3_flag=0;
		    break;
		case 4:
			if(task4_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			task4_flag=0;
		    break;
			
			
		case 5:
			if(task5_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			task5_flag=0;
		    break;	
			
		case 6:
			if(task6_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			task6_flag=0;
		    break;		
		case 7:
			if(task7_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			task7_flag=0;
		    break;			
			
		case 8:
			if(task8_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			task8_flag=0;
		    break;			
			
		case 9:
			if(task9_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			task9_flag=0;
		    break;	
		case 'A':
			if(taska_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			taska_flag=0;
		    break;	
		case 'B':
			if(taskb_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			taskb_flag=0;
		    break;	
		case 'C':
			if(taskc_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			taskc_flag=0;
		    break;	
		case 'D':
			if(taskd_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			taskd_flag=0;
		    break;	
		case 'F':
			if(taskf_flag==1)
			{
				task_flag=1;
			}
			else
			{
				task_flag=0;
			}
			taskf_flag=0;
		    break;	
			
	}
	if(task_flag==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
	
	
}

void Fashion_servo_Init(void)
{
	   SysTick_Init();
		Usart_Init();
}
void jiguang(uint8_t kaiguan)
{
	if(kaiguan==1)//开定位激光
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);//激光
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);//激光
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,0);//激光
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);//激光
	}
}



void page(void)
{
	UART_printf(&huart2,"page 14\xff\xff\xff");
	UART_printf(&huart2,"page 14\xff\xff\xff");	
}
void light(uint8_t i)
{
	L=i;
	RGB_Show_64();
}
void LED(uint8_t number)
{
	switch(number)
	{
		case 0:
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,0);
		HAL_GPIO_WritePin(Green_GPIO_Port,Green_Pin,0);
		HAL_GPIO_WritePin(Blue_GPIO_Port,Blue_Pin,0);
		break;
		case 1:
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,1);
		HAL_GPIO_WritePin(Green_GPIO_Port,Green_Pin,0);
		HAL_GPIO_WritePin(Blue_GPIO_Port,Blue_Pin,0);
		break;
		case 2:
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,0);
		HAL_GPIO_WritePin(Green_GPIO_Port,Green_Pin,1);
		HAL_GPIO_WritePin(Blue_GPIO_Port,Blue_Pin,0);
		break;
		case 3:
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,0);
		HAL_GPIO_WritePin(Green_GPIO_Port,Green_Pin,0);
		HAL_GPIO_WritePin(Blue_GPIO_Port,Blue_Pin,1);
		break;
		case 4:
		HAL_GPIO_WritePin(Beep_GPIO_Port,Beep_Pin,1);
		HAL_Delay(50);//200
		HAL_GPIO_WritePin(Beep_GPIO_Port,Beep_Pin,0);
		
		break;
		
	}
}
void saoma(void)
{
	for(uint8_t i=0;i<9;i++)
	{
		HAL_UART_Transmit(&huart5, (uint8_t *)&saoma_chufa[i], 1, 1000);
//		HAL_UART_Transmit_IT(&huart5, (uint8_t *)&saoma_chufa[i], 1);
	}
}
void duoji_jiazhao(uint32_t angle)//PA2
{
	if(angle>=duoji_jiazhao_jiajin)
	{
		angle=duoji_jiazhao_jiajin;
	}
	else if(angle<=duoji_jiazhao_songkai_init)
	{
		angle=duoji_jiazhao_songkai_init;
	}
	
//	for(uint8_t i=0;i<5;i++)
//	{
//		duoji_jiazhao_DMA[i]=angle;
//	}

//	HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_3,&duoji_jiazhao_DMA[0],5);//PA2
__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,angle);	
	
}
void duoji_liaopan(uint32_t angle)//(PB3)1:物料1,2:物料2,3:物料3
{	
	
	for(uint8_t i=0;i<5;i++)
	{
		duoji_liaopan_DMA[i]=angle;
	}

	HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_2,&duoji_liaopan_DMA[0],5);
	
//	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,angle);	
}
void duoji_zhuanbi(uint32_t angle)//(PA5)
{

}
void duoji_zhedie(uint32_t angle)//PC8
{	
	
	
	
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,angle);
	
}
void duoji_zhedie_slowaction(uint32_t angle,uint8_t wait)
{
	if(angle==duoji_zhedie_down)
	{	
		
//		duoji_zhedie(duoji_zhedie_down-400);
//		for(uint16_t i=duoji_zhedie_down-400;i<=duoji_zhedie_down;i++)
//		{
//			duoji_zhedie(i);
//			HAL_Delay(1);
//		}
//		HAL_Delay(10);
//		duoji_zhedie(duoji_zhedie_down);
		
		FSUS_SetServoAngleByVelocity(&usart2,2,-1.0,130,900,900,0,0);
		if(wait)
		{
			HAL_Delay(200);
		}
	}
	else if(angle==duoji_zhedie_up)
	{
//		for(uint16_t i=duoji_zhedie_down;i>=duoji_zhedie_down-200;i--)
//		{
//			duoji_zhedie(i);
//			HAL_Delay(1);
//		}
//		HAL_Delay(10);
//		duoji_zhedie(duoji_zhedie_up);
		
		FSUS_SetServoAngleByVelocity(&usart2,2,45.0,220,500,500,0,0);
		if(wait)
		{
			HAL_Delay(200);
		}
	}
}

void duoji_zhuanbi_slowaction(uint32_t angle,uint8_t wait)
{
	
	if(angle==duoji_zhuanbi_wai)
	{
		FSUS_SetServoAngleByVelocity(&usart2,3,-146.0,240,20,1800,0,0);
		if(wait)
		{
			HAL_Delay(1000);
		}
		
		
//		duoji_zhuanbi(duoji_zhuanbi_wai-300);
//		HAL_Delay(600);		 
//		 for(uint32_t i=duoji_zhuanbi_wai-300;i<=duoji_zhuanbi_wai;i=i+5)
//		{
//			duoji_zhuanbi(i); 
//			for(uint32_t i=0;i<=200000;i++);
//		}
	}
	else if(angle==duoji_zhuanbi_nei)
	{
		FSUS_SetServoAngleByVelocity(&usart2,3,34,240,20,1800,0,0);
//FSUS_SetServoAngleByVelocity(&usart2,3,34.0,240,20,20,0,0);
		if(wait)
		{
			HAL_Delay(200);
		}
		
//		duoji_zhuanbi(duoji_zhuanbi_nei+450);
//		HAL_Delay(200); 
//		for(uint32_t i=duoji_zhuanbi_nei+450;i>=duoji_zhuanbi_nei;i--)
//		{	
//			duoji_zhuanbi(i);
//			HAL_Delay(1);		
//		}
//		HAL_Delay(20);
//		duoji_zhuanbi(duoji_zhuanbi_nei);
	}	
}
void duoji_jiazhao_slowaction(uint32_t angle)
{
	if(angle==duoji_jiazhao_jiajin)
	{
//		duoji_jiazhao(duoji_jiazhao_jiajin-300);
//		 for(uint32_t i=duoji_jiazhao_jiajin-300;i<=duoji_jiazhao_jiajin;i++)
//		{	
//			duoji_jiazhao(i);
//			HAL_Delay(1);		
//		}
//		duoji_jiazhao(duoji_jiazhao_jiajin);
		
//		duoji_jiazhao(duoji_jiazhao_jiajin-450);
		 for(uint32_t i=duoji_jiazhao_jiajin-220;i<=duoji_jiazhao_jiajin;i++)
		{	
			duoji_jiazhao(i);
			HAL_Delay(1);		
		}
		duoji_jiazhao(duoji_jiazhao_jiajin);
	}
	else if(angle==duoji_jiazhao_songkai)
	{
		 for(uint32_t i=duoji_jiazhao_jiajin;i>=duoji_jiazhao_jiajin-200;i--)
		{	
			duoji_jiazhao(i);
//			for(uint32_t i=0;i<=200000;i++);
			HAL_Delay(1);		
		}
//		HAL_Delay(20);
//		for(uint32_t i=0;i<=2000000;i++);
		duoji_jiazhao(duoji_jiazhao_songkai);
	}
}

void camera_jiaozhun(void)
{
	 /******************************校准处理,速度环*************************************/
	//1mm=7.2个像素点
		
	modetwo[0]='0';
	modetwo[1]='4';
	modetwo[2]='9';
	
	move_slow_flag=0;
	
	
	while(1)
	{
		mode2_receiveflag=0;
		
		while(mode2_receiveflag==0)//没收到一直发
		{
				 for(uint8_t i=0;i<3;i++)
				{
					HAL_UART_Transmit(&huart3, (uint8_t *)&modetwo[i], 1, 1000);
				}
				if(mode2_receiveflag==1)
				{
					break;
				}
				HAL_Delay(5);
		}
		if(mode2_receiveflag==1 && move_slow_flag==1&&(mode2_xerr<=2 && mode2_yerr<=2))
		{
			move_stopall();
			X_last=mode2_xerr;//无实际作用
			Y_last=mode2_yerr;//无实际作用
			mode2_receiveflag=0;
			move_slow_flag=0;
			break;
		}
		mode2_receiveflag=0;
		PID_ERR_move();
		
	}
	LED(4);
	mode2_xerr=0;//清零误差
	mode2_yerr=0;			
}
void camera_jiaozhun_mode3(void)
{
	 /******************************校准处理,速度环*************************************/
			//1mm=7.2个像素点
	
	
		modethree[0]='0';
		modethree[1]='5';
		modethree[2]='9';
	
		move_slow_flag=0;
	
			while(1)
			{
				mode2_receiveflag=0;
				while(mode2_receiveflag==0)//没收到一直发
				{
						 for(uint8_t i=0;i<3;i++)
						{
							HAL_UART_Transmit(&huart3, (uint8_t *)&modethree[i], 1, 1000);
						}
						if(mode2_receiveflag==1)
						{
							break;
						}
						HAL_Delay(5);
				}
				if(mode2_receiveflag==1 && move_slow_flag==1&&(mode2_xerr<=15 && mode2_yerr<=15))//5
				{
					move_stopall();
					X_last=mode2_xerr;
					Y_last=mode2_yerr;
					mode2_receiveflag=0;
					move_slow_flag=0;
					break;
				}
				mode2_receiveflag=0;
				PID_ERR_move();
				
			}
			LED(4);
			mode2_xerr=0;//清零误差
			mode2_yerr=0;
}
/*******************************************************************
void camera_shibie(uint8_t number)
{
	  while(1)
	  {
		  mode1='1';

		 for(uint8_t i=0;i<3;i++)
		{
			HAL_UART_Transmit(&huart3, (uint8_t *)&modeone[i], 1, 1000);
		}
		HAL_Delay(5);
		  if(color_mode1==1||color_mode1==2||color_mode1==3)
		  {
			  if(paixu[number-1]==color_mode1+'0')
			  {
				liaopan[number]=color_mode1;//记录料盘1物料颜色
				LED(4);
				duoji_jiazhao_slowaction(duoji_jiazhao_jiajin);
				move_tongbudai(bujin5_vel,bujin5_acc,11100);	
				HAL_Delay(200);
				duoji_zhuanbi_slowaction(duoji_zhuanbi_nei,1);
				HAL_Delay(1200);
				move_tongbudai(bujin5_vel,bujin5_acc,8300);
				HAL_Delay(400);
				duoji_jiazhao_slowaction(duoji_jiazhao_songkai);
				  
				 
				 if(number==1||number==5)
				 {
					 duoji_liaopan(duoji_liaopan_liaopan2);
				 }
				 else if(number==2||number==6)
				 {
					 duoji_liaopan(duoji_liaopan_liaopan3);
				 }
				 else if(number==3||number==7)
				 {
					 duoji_liaopan(duoji_liaopan_liaopan1);
				 }	 
				  if((number==5||number==6||number==7)&&angle_tiaozheng_ok==0)//防止第一个就是自己想要的颜色，然后不纠正车上姿态
				  {
					angle_tiaozheng(0);
					angle_tiaozheng_ok=1;
					LED(4);
				  }
				  break;	  
			  }
			  else if((number==5||number==6||number==7)&&angle_tiaozheng_ok==0)//不是想要的颜色，开始纠正角度
			  {
				  angle_tiaozheng(0);
				  angle_tiaozheng_ok=1;
				  LED(4);
			  }
			 
		  }    
	  }  
}
*******************************************************************/
void camera_jiaozhun_yuanliao(uint8_t number)
{
	modeone[0]='0';
	modeone[1]=paixu[number-1];
	modeone[2]='9';
	
	mode1_xerr=0;//清零误差
	mode1_yerr=0;//清零误差
	mode1_position=0;
	mode1_receiveflag=0;
	move_slow_flag=0;
	
	while(number==1||number==5)//位置矫正,只有每一轮第一个要位置矫正
	{
		mode1_receiveflag=0;
		while(mode1_receiveflag==0)//等待接收成功，没收到一直发
		{
			 for(uint8_t i=0;i<3;i++)
			{
				HAL_UART_Transmit(&huart3, (uint8_t *)&modeone[i], 1, 1000);
			}
			if(mode1_receiveflag==1)//等待接收成功，没收到一直发
			{
				break;
			}
			HAL_Delay(5);//防止发太快
		}
		//number==1||number==2||number==3||number==5||number==6||number==7
		if(number==1||number==5)//只有每一轮第一个要位置矫正
		{
			if(mode1_receiveflag==1 && mode1_position!=0 &&(mode1_xerr<=12 && mode1_yerr<=12))//退出阈值，如果速度慢可以去掉低速阶段检测，或者把P增大
			{
				HAL_Delay(100);//
				while(mode1_receiveflag==0)//等待接收成功，没收到一直发
				{
					for(uint8_t i=0;i<3;i++)
					{
						HAL_UART_Transmit(&huart3, (uint8_t *)&modeone[i], 1, 1000);
					}
					if(mode1_receiveflag==1)//等待接收成功，没收到一直发
					{
						break;
					}
					HAL_Delay(5);//防止发太快
				}
				if(mode1_receiveflag==1 && mode1_position!=0 &&(mode1_xerr<=12 && mode1_yerr<=12))
				{
					move_stopall();
					X_last=mode1_xerr;//无实际作用
					Y_last=mode1_yerr;//无实际作用
					
					mode1_xerr=0;//清零误差
					mode1_yerr=0;//清零误差
					mode1_position=0;
					mode1_receiveflag=0;
					move_slow_flag=0;
					
					LED(4);
					break;
				}
				
			}
			mode1_receiveflag=0;
			PID_ERR_move_mode1();
		}
		
		
	}
	move_stopall();
	
	while(1)//根据err大小来判断抓取，小于阈值时抓取，阈值需要调节
	{
		mode1_receiveflag=0;
		mode1_xerr=0;//清零误差
		mode1_yerr=0;//清零误差
		while(mode1_receiveflag==0)//等待接收成功，没收到一直发
		{
			 for(uint8_t i=0;i<3;i++)
			{
				HAL_UART_Transmit(&huart3, (uint8_t *)&modeone[i], 1, 1000);
			}
			if(mode1_receiveflag==1)//等待接收成功，没收到一直发
			{
				break;
			}
			HAL_Delay(5);//防止发太快
		}

		if((paixu[number-1]==color_mode1+'0')&& mode1_receiveflag==1 && mode1_position!=0 && (mode1_xerr<=50 && mode1_yerr<=50))//根据err大小来判断抓取，小于阈值时抓取，阈值需要调节(通过象限位！=0来避免镜头没有看到物料的情况)
		{
			liaopan[number]=color_mode1;//记录料盘1物料颜色
			LED(4);
			duoji_jiazhao_slowaction(duoji_jiazhao_jiajin);
			move_tongbudai(bujin5_vel,bujin5_acc,11300);	
			HAL_Delay(200);
			duoji_zhuanbi_slowaction(duoji_zhuanbi_nei,1);
			HAL_Delay(1200);
			move_tongbudai(bujin5_vel,bujin5_acc,8800);
			HAL_Delay(400);
			duoji_jiazhao_slowaction(duoji_jiazhao_songkai);
			  
			 if(number==1||number==5)
			 {
				 duoji_liaopan(duoji_liaopan_liaopan2);
			 }
			 else if(number==2||number==6)
			 {
				 duoji_liaopan(duoji_liaopan_liaopan3);
			 }
			 else if(number==3||number==7)
			 {
				 duoji_liaopan(duoji_liaopan_liaopan1);
			 }	 
			mode1_xerr=0;//清零误差
			mode1_yerr=0;//清零误差 
			mode1_position=0;
			mode1_receiveflag=0;//接收标志位清零
			break;
		}
		
	}
	if(number==7)//抓完第二轮的最后一个后角度纠偏，考虑到前面镜头校准时车身角度是歪的，如果在中途进行校准，则会出现判断是否抓取的阈值重新变化，因此在抓完最后一个时角度纠偏
	{
		angle_tiaozheng(0);
	}	
	
}
void maduo_from_car_to_ground(uint8_t level,uint8_t number)//level：第几层，number：第几个
{
	/******************************从车上取物料，放置第n个物料*************************************/
	
	if(level==1)
	{
		move_tongbudai(bujin5_vel,bujin5_acc,5300);
	}
	else if(level==2)
	{
		move_tongbudai(bujin5_vel,bujin5_acc,5300+5000);
	}
	duoji_zhedie_slowaction(duoji_zhedie_down,1);
	HAL_Delay(500);
	
	if(level==1)//第一层物料
	{
		move_tongbudai(bujin5_vel,bujin5_acc,455);
		HAL_Delay(700);//1200
	}
	else if(level==2)//第二层物料
	{
		move_tongbudai(bujin5_vel,bujin5_acc,6100);
		HAL_Delay(500);
	}
	duoji_jiazhao_slowaction(duoji_jiazhao_songkai);
	HAL_Delay(100);
			
	if(number==1)//从车上取物料，放置到地上第一个物料
	{
		duoji_liaopan(duoji_liaopan_liaopan2);
	}
	else if(number==2)//从车上取物料，放置到地上第二个物料
	{
		duoji_liaopan(duoji_liaopan_liaopan3);
	}
	else if(number==3)//从车上取物料，放置到地上第三个物料
	{
		duoji_liaopan(duoji_liaopan_liaopan1);
	}	

}
void quwuliao_from_ground_to_car(uint8_t number)//number：第几个
{
	 /******************************从地上取物料，放置到车上，第n个物料*************************************/
			HAL_Delay(200);
			move_tongbudai(bujin5_vel,bujin5_acc,410);
			HAL_Delay(400);
	
			duoji_jiazhao_slowaction(duoji_jiazhao_jiajin);
			move_tongbudai(bujin5_vel,bujin5_acc,11100);
			HAL_Delay(300);
	
			duoji_zhuanbi_slowaction(duoji_zhuanbi_nei,1);
			HAL_Delay(200);
	
	 /******************************从地上取物料，放置到车上，第n个物料*************************************/
	
			if(number==3)
			{
				move_left(4000,40,pulse_onecircle/distance_onecircle*60,0);
			}
			
			HAL_Delay(1700);
			
			
			move_tongbudai(bujin5_vel,bujin5_acc,8800);
			HAL_Delay(200);
			

			duoji_jiazhao_slowaction(duoji_jiazhao_songkai);
	
			if(number==1)//从地上取物料，放置到车上，第一个物料
			{
				duoji_liaopan(duoji_liaopan_liaopan2);
			}
			else if(number==2)//从地上取物料，放置到车上，第二个物料
			{
				duoji_liaopan(duoji_liaopan_liaopan3);
			}
			else if(number==3)//从地上取物料，放置到车上，第三个物料
			{
				duoji_liaopan(duoji_liaopan_liaopan1);
			}
}



void angle_tiaozheng(int16_t goal)
{
	

	if(goal==0)
	{
		yaw_tiaozheng1st=angle[2];
		if(yaw_tiaozheng1st-yaw_begin<=0)
		{
			move_shun(2000,20,(yaw_begin-yaw_tiaozheng1st)*pulse_shun90/90.0,1);
			
		}else
		{
			move_ni (2000,20,(yaw_tiaozheng1st-yaw_begin)*pulse_shun90/90.0,1);
		}
		HAL_Delay(1000);
	}
	else if(goal==-90)
	{
		yaw_tiaozheng1st=angle[2];
		if((yaw_tiaozheng1st+90)-yaw_begin<=0)
		{
			move_shun(2000,20,(yaw_begin-(yaw_tiaozheng1st+90))*pulse_shun90/90.0,1);
			
		}else
		{
			move_ni (2000,20,(yaw_tiaozheng1st+90-yaw_begin)*pulse_shun90/90.0,1);
		}
		HAL_Delay(200);
	}
	
	
}

void PID_ERR_move(void)//用于粗加工区和暂存区校准
{
	int32_t Ve_x,Ve_y;
	
	if((mode2_xerr<=30 && mode2_xerr>=13)||(mode2_yerr<=30 && mode2_yerr>=13))
	{
		Ve_x=(float)mode2_xerr*0.08;//将误差转换到速度
		Ve_y=(float)mode2_yerr*0.08;//将误差转换到速度
		move_slow_flag=1;
	}
	else
	{
		if(mode2_xerr<=13 && mode2_yerr<=13)
		{
			move_slow_flag=1;
		}
		
		Ve_x=(float)mode2_xerr*0.20;//将误差转换到速度
		Ve_y=(float)mode2_yerr*0.20;//将误差转换到速度
	}
	
	
	if(Ve_x>=Ve_y)
	{
		if(Ve_x>=8)//速度限制
		{
			Ve_x=8;
		}
		Ve_y=Ve_x*(float)mode2_yerr/(float)mode2_xerr;//误差限速比等于速度比
	}
	else
	{
		if(Ve_y>=8)//速度限制
		{
			Ve_y=8;	
		}
		Ve_x=Ve_y*(float)mode2_xerr/(float)mode2_yerr;//误差限速比等于速度比
	}
	
	switch(mode2_position)//根据象限处理正负值
	{
		case 1://右上角,第一象限 
			Ve_x=-Ve_x;
			Ve_y=Ve_y;
			break;
		case 2://左上角，第二象限
			Ve_x=Ve_x;
			Ve_y=Ve_y;
			break;
		case 3://左下角,第三象限
			Ve_x=Ve_x;
			Ve_y=-Ve_y;
			break;
		case 4://右下角，第四象限
			Ve_x=-Ve_x;
			Ve_y=-Ve_y;
			break;	
	}
	V_xy(Ve_x,Ve_y,0);
	
}

void PID_ERR_move_mode1(void)//用于原料区位置矫正
{
	int32_t Ve_x,Ve_y;
	
	if((mode1_xerr<=30 && mode1_xerr>=13)||(mode1_yerr<=30 && mode1_yerr>=13))
	{
		Ve_x=(float)mode1_xerr*0.08;//将误差转换到速度
		Ve_y=(float)mode1_yerr*0.08;//将误差转换到速度
		move_slow_flag=1;
	}
	else
	{
		if(mode1_xerr<=13 && mode1_yerr<=13)
		{
			move_slow_flag=1;
		}
//		Ve_x=(float)mode1_xerr*0.20;//将误差转换到速度
//		Ve_y=(float)mode1_yerr*0.20;//将误差转换到速度
		Ve_x=(float)mode1_xerr*0.10;//将误差转换到速度
		Ve_y=(float)mode1_yerr*0.10;//将误差转换到速度
	}
	
	
	if(Ve_x>=Ve_y)
	{
		if(Ve_x>=5)//速度限制
		{
			Ve_x=5;
		}
		Ve_y=Ve_x*(float)mode1_yerr/(float)mode1_xerr;//误差限速比等于速度比
	}
	else
	{
		if(Ve_y>=5)//速度限制
		{
			Ve_y=5;	
		}
		Ve_x=Ve_y*(float)mode1_xerr/(float)mode1_yerr;//误差限速比等于速度比
	}
	
	switch(mode1_position)//根据象限处理正负值
	{
		case 1://右上角,第一象限 
			Ve_x=-Ve_x;
			Ve_y=Ve_y;
			break;
		case 2://左上角，第二象限
			Ve_x=Ve_x;
			Ve_y=Ve_y;
			break;
		case 3://左下角,第三象限
			Ve_x=Ve_x;
			Ve_y=-Ve_y;
			break;
		case 4://右下角，第四象限
			Ve_x=-Ve_x;
			Ve_y=-Ve_y;
			break;	
	}
	V_xy(Ve_x,Ve_y,0);
	
}














void task_cheshi(uint8_t number)
{
	if(number==1)
	{
		L=128;
		RGB_Show_64();
	
	/******************************转臂向外，准备夹取*************************************/
			move_tongbudai(bujin5_vel,bujin5_acc,10100);	
			HAL_Delay(100);
	
			move_right(4000,40,pulse_onecircle/distance_onecircle*30,0);  
	
			duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,1);
			
		   /******************************颜色识别并夹取，第一个*************************************/
			duoji_jiazhao(duoji_jiazhao_jiajin-450);
	}
	else if(number==2)
	{
//		move()
	}
}


void task1(void)
{	
	jiguang(0);//关闭激光
	send_yaw0();//Yaw角清零
	HAL_Delay(500);
	yaw_begin=angle[2];//读取初始Yaw角
	
	duoji_zhedie_slowaction(duoji_zhedie_down,1);
	duoji_jiazhao(duoji_jiazhao_songkai);
	
	 move_tongbudai(bujin5_vel,bujin5_acc,high_yuanpan);
}
void task2(void)
{

			
}
void task3(void)
{
	
}
void task4(void)
{
							
				
}
void task9(void)//
{
	move_shun90();
	HAL_Delay(2000);
	move_shun90();	
	HAL_Delay(2000);	
	move_shun90();	
	HAL_Delay(2000);	
	move_shun90();	
	HAL_Delay(2000);
}
		
void gohome(uint8_t i)
{
	
}
void task_Init(void)
{

	Fashion_servo_Init();
	jiguang(1);
	light(0);
	move_stopall();
	duoji_zhedie_slowaction(duoji_zhedie_up,1);
	duoji_jiazhao(duoji_jiazhao_songkai_init);
	HAL_Delay(200);
	duoji_zhuanbi_slowaction(duoji_zhuanbi_nei,0);
	duoji_liaopan(duoji_liaopan_liaopan1);
	HAL_Delay(600);
	Emm_V5_Origin_Trigger_Return(0x05,2,0);//回零
	HAL_Delay(2800);
	LED(4);
	page();
	
	paixu[0]='1';
	paixu[1]='3';
	paixu[2]='2';
	paixu[4]='2';
	paixu[5]='3';
	paixu[6]='1';

}
void qidong_to_yuanliao(void)
{
	paixu[0]='1';
	paixu[1]='3';
	paixu[2]='2';
	paixu[4]='2';
	paixu[5]='3';
	paixu[6]='1';
	
	
	jiguang(0);//关闭激光
	send_yaw0();//Yaw角清零
	HAL_Delay(500);
	yaw_begin=angle[2];//读取初始Yaw角

	

	move_left(4000,100,pulse_onecircle/distance_onecircle*200,0);
	
	duoji_zhedie_slowaction(duoji_zhedie_down,1);
	duoji_jiazhao(duoji_jiazhao_songkai);
	
	HAL_Delay(1100);
	LED(4);
	
	saoma();
	
	move_front(4000,100,pulse_onecircle/distance_onecircle*765,0);
	HAL_Delay(2300);
	LED(4);
	HAL_Delay(500);
	while(paixu[3]!='+')
	{
		move_front(1000,100,pulse_onecircle/distance_onecircle*30,0);
		HAL_Delay(600);
		move_back(1000,100,pulse_onecircle/distance_onecircle*30,0);
		HAL_Delay(600);
	}
	
	
	
	move_front(4000,100,pulse_onecircle/distance_onecircle*700,0); 
	HAL_Delay(500);
	
	
	move_tongbudai(bujin5_vel,bujin5_acc,high_yuanpan);	//10100	
	HAL_Delay(100);
	duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,1);
	
	HAL_Delay(800);
	LED(4);
}

void task_yuanliao(uint8_t number)
{
	light(128);
	
	uint8_t i=0;
	if(number==1)
	{
		i=0;
	}else if(number==2)
	{
		i=4;
	}
			if(number==1)
			{
				move_right(4000,40,pulse_onecircle/distance_onecircle*30,0);  //靠近转盘
			}
	 /******************************夹爪微紧，位置校准，颜色识别并夹取，第一个*************************************/
			duoji_jiazhao(duoji_jiazhao_jiajin-450);
			camera_jiaozhun_yuanliao(i+1);
			
			/******************************上升，转臂向外，准备夹取*************************************/
			move_tongbudai(bujin5_vel,bujin5_acc,high_yuanpan);	//10100
			HAL_Delay(50);
			duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,1);
	
			/******************************夹爪微紧，颜色识别并夹取，第二个*************************************/
			duoji_jiazhao(duoji_jiazhao_jiajin-450);
			camera_jiaozhun_yuanliao(i+2);
			/******************************上升，转臂向外，准备夹取*************************************/
			move_tongbudai(bujin5_vel,bujin5_acc,high_yuanpan);	//10100	
			HAL_Delay(50);
			duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,1); 
			
			/******************************夹爪微紧，颜色识别并夹取，第三个*************************************/
			duoji_jiazhao(duoji_jiazhao_jiajin-450);
			camera_jiaozhun_yuanliao(i+3);
			
			light(0);
			
			
	
}
void yuanliao_to_cu(void)
{
	/******************************移动到粗加工区*************************************/
		move_back(4000,100,pulse_onecircle/distance_onecircle*410,0);//4000,40
		HAL_Delay(1700);
		LED(4);
		move_shun90();
		HAL_Delay(1700);
		LED(4);
		move_back(2000,80,pulse_onecircle/distance_onecircle*1760,0);//3000,40
		HAL_Delay(3900); 
		LED(4);	 
}
void task_cujiagong(uint8_t number)
{
	
//	paixu[0]='1';
//	paixu[1]='3';
//	paixu[2]='2';
	light(128);
	
	uint8_t i=0;
	if(number==1)
	{
		i=0;
	}else if(number==2)
	{
		i=4;
	}
	 /******************************位移，对准第一个圆心*************************************/
			 
move_shun90();
HAL_Delay(60);
	
			 /******************************转臂向外，校准环节*************************************/ 
			move_tongbudai(bujin5_vel,bujin5_acc,8300);	
			HAL_Delay(300);
			 
			duoji_jiazhao_slowaction(duoji_jiazhao_jiajin);
			HAL_Delay(100);
			 
			move_tongbudai(bujin5_vel,bujin5_acc,11100);
			HAL_Delay(200);
			duoji_zhedie_slowaction(duoji_zhedie_up,1); 
			 
			 duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,0);
			 HAL_Delay(400);
			
			 switch(paixu[i]-'0') 
			 {
				 case di_yuanpan_you : move_back(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(50);break;
				 case di_yuanpan_zhong:break;
				 case di_yuanpan_zuo:  move_front(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(50);break;
			 }
			move_tongbudai(bujin5_vel,bujin5_acc,100);	
			HAL_Delay(1200); 
			LED(4);
			 
		  /******************************校准处理*************************************/

			camera_jiaozhun();
			
		/******************************从车上取物料，放置第一个物料*************************************/
			
			maduo_from_car_to_ground(1,1);
			
		/******************************转臂向内*************************************/	
			
			duoji_zhuanbi_slowaction(duoji_zhuanbi_nei,1);
			move_tongbudai(bujin5_vel,bujin5_acc,8300);	
			HAL_Delay(100);
			
			 /******************************位移，对准第二个圆心*************************************/
			 
			
			 switch(paixu[i+1]-'0')
			 {
				 case  di_yuanpan_you:
						if(paixu[i]-'0'==di_yuanpan_zhong)//绿红
						{
							move_back(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
						else//蓝红 
						{
							move_back(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(20);break;
						}
				 case  di_yuanpan_zhong:
						if(paixu[i]-'0'==di_yuanpan_you)//红绿
						{
							move_front(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
						else //蓝绿
						{
							move_back(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
						
				 case di_yuanpan_zuo:
						if(paixu[i]-'0'==di_yuanpan_you)//红蓝
						{
							move_front(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(20);break;
						}
						else //绿蓝
						{
							move_front(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
					
			 }
			 
			 /******************************夹取物料*************************************/
				HAL_Delay(600);
				duoji_jiazhao_slowaction(duoji_jiazhao_jiajin);
				HAL_Delay(100);
				 
				move_tongbudai(bujin5_vel,bujin5_acc,11100);
				HAL_Delay(200);
				duoji_zhedie_slowaction(duoji_zhedie_up,1); 
				 
				duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,0);
				HAL_Delay(500);
			 
			
		 /******************************校准处理*************************************/
			move_tongbudai(bujin5_vel,bujin5_acc,100);		
			HAL_Delay(1000);
		 
			camera_jiaozhun();

			 /******************************从车上取物料，放置第二个物料*************************************/
			
			maduo_from_car_to_ground(1,2);
			
			
			/******************************转臂向内*************************************/	
			
			duoji_zhuanbi_slowaction(duoji_zhuanbi_nei,1);
			move_tongbudai(bujin5_vel,bujin5_acc,8300);	
			HAL_Delay(100);
			

			/******************************位移，对准第三个圆心*************************************/
	
			 switch(paixu[i+1]-'0')
			 {
				 case  di_yuanpan_you:
						if(paixu[i]-'0'==di_yuanpan_zhong)//绿红蓝
						{
							move_front(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(20);break;
						}
						else//蓝红绿 
						{
							move_front(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
				 case di_yuanpan_zhong:
						if(paixu[i]-'0'==di_yuanpan_you)//红绿蓝
						{
							move_front(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
						else //蓝绿红
						{
							move_back(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
						
				 case di_yuanpan_zuo:
						if(paixu[i]-'0'==di_yuanpan_you)//红蓝绿
						{
							move_back(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
						else //绿蓝红
						{
							move_back(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(20);break;
						}
					
			 }
			   /******************************夹取物料*************************************/
				HAL_Delay(600);
				duoji_jiazhao_slowaction(duoji_jiazhao_jiajin);
				HAL_Delay(100);
				move_tongbudai(bujin5_vel,bujin5_acc,11100);
				HAL_Delay(200);
				duoji_zhedie_slowaction(duoji_zhedie_up,1); 
				 
				duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,0);
				HAL_Delay(500);

		 /******************************校准处理*************************************/
			move_tongbudai(bujin5_vel,bujin5_acc,100);		
			HAL_Delay(1000);
		 
			camera_jiaozhun();

			 /******************************从车上取物料，放置第三个物料*************************************/
			
			maduo_from_car_to_ground(1,3);

			 /******************************对准第一个物料*************************************/
			
			 switch(paixu[i+1]-'0')
			 {
				 case di_yuanpan_you:
						if(paixu[i]-'0'==di_yuanpan_zhong)//绿红蓝
						{
							move_back(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}
						else//蓝红绿 
						{
							move_front(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}
				 case di_yuanpan_zhong:
						if(paixu[i]-'0'==di_yuanpan_you)//红绿蓝
						{
							move_back(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(650);break;
						}
						else //蓝绿红
						{
							move_front(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(650);break;
						}
						
				 case di_yuanpan_zuo:
						if(paixu[i]-'0'==di_yuanpan_you)//红蓝绿
						{
							move_back(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}
						else //绿蓝红
						{
							move_front(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}
					
			 }
			 /******************************从地上取物料，放置到车上，第一个物料*************************************/

			quwuliao_from_ground_to_car(1);

			/******************************转臂向外*************************************/
				
			duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,0);
			
			/******************************对准第二个物料*************************************/
			
			 switch(paixu[i+1]-'0')
			 {
				 case di_yuanpan_you:
						if(paixu[i]-'0'==di_yuanpan_zhong)//绿红蓝，绿->红
						{
							move_back(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}
						else//蓝红绿 
						{
							move_back(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(650);break;
						}
				 case di_yuanpan_zhong:
						if(paixu[i]-'0'==di_yuanpan_you)//红绿蓝
						{
							move_front(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}
						else //蓝绿红
						{
							move_back(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}
						
				 case di_yuanpan_zuo:
						if(paixu[i]-'0'==di_yuanpan_you)//红蓝绿
						{
							move_front(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(650);break;
						}
						else //绿蓝红
						{
							move_front(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}
					
			 }
			
		/******************************从地上取物料，放置到车上，第二个物料*************************************/	
			
			quwuliao_from_ground_to_car(2);
		
			/******************************转臂向外*************************************/
			
			duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,0);

		/******************************对准第三个物料*************************************/
			
			 switch(paixu[i+1]-'0')
			 {
				 case di_yuanpan_you:
						if(paixu[i]-'0'==di_yuanpan_zhong)//绿红蓝，红->蓝
						{
							move_front(4000,40,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(650);break;
						}
						else//蓝红绿 ，红->绿
						{
							move_front(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}
				 case di_yuanpan_zhong:
						if(paixu[i]-'0'==di_yuanpan_you)//红绿蓝，绿->蓝
						{
							move_front(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}
						else //蓝绿红，绿->红
						{
							move_back(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}	
				 case di_yuanpan_zuo:
						if(paixu[i]-'0'==di_yuanpan_you)//红蓝绿，蓝->绿
						{
							move_back(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(300);break;
						}
						else //绿蓝红，蓝->红
						{
							move_back(4000,80,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(650);break;
						}
			 }
			
		/******************************从地上取物料，放置到车上，第三个物料*************************************/	
			quwuliao_from_ground_to_car(3);
			 
			
//			 move_left(4000,40,pulse_onecircle/distance_onecircle*20,0);
//			 HAL_Delay(700);
			 LED(4);
			 
}
void cu_to_zan(uint8_t number)
{
//	
//	paixu[0]='1';
//	paixu[1]='3';
//	paixu[2]='2';
//	

	uint8_t i=0;
	if(number==1)
	{
		i=0;
	}else if(number==2)
	{
		i=4;
	}
	switch(paixu[i+2]-'0')
	{
		case di_yuanpan_you:move_back(4000,100,pulse_onecircle/distance_onecircle*(840-yuanhuan_1distance),0);break;
		case di_yuanpan_zhong:move_back(4000,100,pulse_onecircle/distance_onecircle*840,0);break;
		case di_yuanpan_zuo:move_back(4000,100,pulse_onecircle/distance_onecircle*(840+yuanhuan_1distance),0);break;
	}
	HAL_Delay(300);
	move_tongbudai(bujin5_vel,bujin5_acc,8300);		
	HAL_Delay(2500);
	LED(4);
	
	move_shun90();
	HAL_Delay(1700);
	LED(4);
	
	switch(paixu[i]-'0')
	{
		case di_yuanpan_you:move_back(4000,100,pulse_onecircle/distance_onecircle*(820+yuanhuan_1distance),0);HAL_Delay(500);break;
		case di_yuanpan_zhong:move_back(4000,100,pulse_onecircle/distance_onecircle*820,0);HAL_Delay(500);break;
		case di_yuanpan_zuo:move_back(4000,100,pulse_onecircle/distance_onecircle*(820-yuanhuan_1distance),0);HAL_Delay(500);break;
	}
	LED(4);

}
void task_zancunqu(uint8_t level)
{
	
//	paixu[0]='1';
//	paixu[1]='3';
//	paixu[2]='2';
	
	uint8_t i=0;
	if(level==1)
	{
		i=0;
	}else if(level==2)
	{
		i=4;
	} 
	
	 /******************************转臂向外，校准环节*************************************/ 
			move_tongbudai(bujin5_vel,bujin5_acc,8300);	
			HAL_Delay(300);
			 
			duoji_jiazhao_slowaction(duoji_jiazhao_jiajin);
			HAL_Delay(100);
			 
			move_tongbudai(bujin5_vel,bujin5_acc,11100);
			HAL_Delay(200);
			duoji_zhedie_slowaction(duoji_zhedie_up,1); 
			 
			duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,0);
			HAL_Delay(400);
			if(level==1)
			{
				move_tongbudai(bujin5_vel,bujin5_acc,100);	
			}
			else if(level==2)
			{
				move_tongbudai(bujin5_vel,bujin5_acc,100+5000);
			}
			HAL_Delay(900);
			LED(4);
			
			
			 /******************************靠近圆环*************************************/
			move_right(4000,100,pulse_onecircle/distance_onecircle*30,0);
			HAL_Delay(700); 
			LED(4);
			
			
		  /******************************校准处理*************************************/
			if(level==1)
			{
				camera_jiaozhun();
			}else if(level==2)
			{
				camera_jiaozhun_mode3();
			} 
		/******************************从车上取物料，放置第一个物料*************************************/
			
			maduo_from_car_to_ground(level,1);//第几层
			
			 /******************************转臂向内*************************************/
				duoji_zhuanbi_slowaction(duoji_zhuanbi_nei,1);
				move_tongbudai(bujin5_vel,bujin5_acc,8300);	
				HAL_Delay(100);
			 /******************************位移，对准第二个圆心*************************************/

			 switch(paixu[i+1]-'0')
			 {
				 case di_yuanpan_you:
						if(paixu[i]-'0'==di_yuanpan_zhong)//绿红
						{
							move_back(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
						else//蓝红 
						{
							move_back(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(20);break;
						}
				 case di_yuanpan_zhong:
						if(paixu[i]-'0'==di_yuanpan_you)//红绿
						{
							move_front(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
						else //蓝绿
						{
							move_back(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
						
				 case di_yuanpan_zuo:
						if(paixu[i]-'0'==di_yuanpan_you)//红蓝
						{
							move_front(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(20);break;
						}
						else //绿蓝
						{
							move_front(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
					
			 }
			 
			 /******************************转臂向内，夹取物料*************************************/
			
				HAL_Delay(600);
				
				duoji_jiazhao_slowaction(duoji_jiazhao_jiajin);
				HAL_Delay(100);
				 
				move_tongbudai(bujin5_vel,bujin5_acc,11100);
				HAL_Delay(200);
				duoji_zhedie_slowaction(duoji_zhedie_up,1); 
				 
				duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,0);
				HAL_Delay(500);
			 
			
		 /******************************校准处理*************************************/
			if(level==1)
			{
				move_tongbudai(bujin5_vel,bujin5_acc,100);	
			}
			else if(level==2)
			{
				move_tongbudai(bujin5_vel,bujin5_acc,100+5000);
			}		
			HAL_Delay(1000);
		 
			if(level==1)
			{
				camera_jiaozhun();
			}else if(level==2)
			{
				camera_jiaozhun_mode3();
			} 

			 /******************************从车上取物料，放置第二个物料*************************************/
			
			maduo_from_car_to_ground(level,2);//第几层
			 /******************************转臂向内*************************************/
				duoji_zhuanbi_slowaction(duoji_zhuanbi_nei,1);
				move_tongbudai(bujin5_vel,bujin5_acc,8300);	
				HAL_Delay(100);

			/******************************位移，对准第三个圆心*************************************/
			 switch(paixu[i+1]-'0')
			 {
				 case di_yuanpan_you:
						if(paixu[i]-'0'==di_yuanpan_zhong)//绿红蓝
						{
							move_front(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(20);break;
						}
						else//蓝红绿 
						{
							move_front(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
				 		}
				 
				 case di_yuanpan_zhong:
						if(paixu[i]-'0'==di_yuanpan_you)//红绿蓝
						{
							move_front(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
						else //蓝绿红
						{
							move_back(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
				 	
				 case di_yuanpan_zuo:
						if(paixu[i]-'0'==di_yuanpan_you)//红蓝绿
						{
							move_back(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_1distance,0);HAL_Delay(20);break;
						}
						else //绿蓝红
						{
							move_back(4000,100,pulse_onecircle/distance_onecircle*yuanhuan_2distance,0);HAL_Delay(20);break;
						}
			
			 }
			 
			  /******************************转臂向内，夹取物料*************************************/
			
				HAL_Delay(600);
				
				duoji_jiazhao_slowaction(duoji_jiazhao_jiajin);
				HAL_Delay(100);
				 
				move_tongbudai(bujin5_vel,bujin5_acc,11100);
				HAL_Delay(200);
				duoji_zhedie_slowaction(duoji_zhedie_up,1); 
				 
				duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,0);
				HAL_Delay(500);
			 
			
		 /******************************校准处理*************************************/
			if(level==1)
			{
				move_tongbudai(bujin5_vel,bujin5_acc,100);	
			}
			else if(level==2)
			{
				move_tongbudai(bujin5_vel,bujin5_acc,100+5000);
			}			
			HAL_Delay(1000);
		 
			if(level==1)
			{
				camera_jiaozhun();
			}else if(level==2)
			{
				camera_jiaozhun_mode3();
			} 

			 /******************************从车上取物料，放置第三个物料*************************************/
			
			maduo_from_car_to_ground(level,3);//第几层
			
			angle_tiaozheng(-90);
//			while(1);
//			HAL_Delay(1000);
}
void zan_to_yuanliao(void)
{
	switch(paixu[2]-'0')
	{
		case di_yuanpan_you:move_back(4000,100,pulse_onecircle/distance_onecircle*(880-yuanhuan_1distance),0);HAL_Delay(200);break;
		case di_yuanpan_zhong:move_back(4000,100,pulse_onecircle/distance_onecircle*880,0);HAL_Delay(400);break;
		case di_yuanpan_zuo:move_back(4000,100,pulse_onecircle/distance_onecircle*(880+yuanhuan_1distance),0);HAL_Delay(800);break;
	}
	move_tongbudai(bujin5_vel,bujin5_acc,high_yuanpan);
	HAL_Delay(2400);
	LED(4);
	move_shun90();
	HAL_Delay(1700);
	LED(4);
	move_back(4000,100,pulse_onecircle/distance_onecircle*450,0);
	HAL_Delay(1800);
	LED(4);
	move_right(4000,40,pulse_onecircle/distance_onecircle*50,0);  
	HAL_Delay(800);
	LED(4);
}
void task_all(void)
{
	qidong_to_yuanliao();
	task_yuanliao(1);
	
	yuanliao_to_cu();
	task_cujiagong(1);
	
	cu_to_zan(1);
	task_zancunqu(1);
	
	zan_to_yuanliao();
	task_yuanliao(2);
	yuanliao_to_cu();
	
	task_cujiagong(2);
	cu_to_zan(2);
	task_zancunqu(2);
}