#include "sensor.h"
#include "oled.h"
#include "tim.h"
#include <math.h>
#include <stdlib.h>
#include "nano.h"
#include <string.h>
#include <stdio.h>
#include "RGB.h"
#include "usart1.h"

#define JY62 1
#define HWT101 0
#define select_JY62orHWT101 0 





__O uint16_t bujin5_upflag=0,bujin5_downflag=0,task1_flag=0,task2_flag=0,task3_flag=0,task4_flag=0,task5_flag=0,task6_flag=0,task7_flag=0,task8_flag=0,task9_flag=0,taskf_flag=0,taska_flag=0,taskb_flag=0,taskc_flag=0,taskd_flag=0;

__O uint16_t tiaoshi_flag=0;
uint8_t paixu[8];
float angle[3];
uint8_t	chuankouping_progress,shibie_progress;
uint8_t task[3];

uint8_t uart2_rxbuff,uart3_rxbuff,uart1_rxbuff,uart4_rxbuff,uart5_rxbuff,uart6_rxbuff;

uint32_t Time_flag,Time_count;

uint8_t flag_saomaqi=0;

uint16_t angle_duoji1=0,angle_duoji4=0;


void chuankouping_Receive_Data(uint8_t com_data)//包长为9
{
		uint8_t i;
		static uint8_t RxCounter1=0;//计数
		static uint16_t RxBuffer1[10]={0};
		static uint8_t RxState = 0;	
    	static uint8_t RxFlag1;
 
		if(RxState==0&&com_data=='S')  //'S'帧头
		{
			RxState=1;
			RxBuffer1[RxCounter1++]=com_data;   
		}
		else if(RxState==1&&com_data=='T')  //'T'帧头
		{
			RxState=2;
			RxBuffer1[RxCounter1++]=com_data;
		}
		else if(RxState==2)
		{
			RxBuffer1[RxCounter1++]=com_data;
			if((RxCounter1>=9)&&(com_data =='E'))       //定包长，包长为9，RxBuffer1接受满了,接收数据结束
			{
				RxState=0;
				RxFlag1=1;
				RxCounter1 = 0;
				chuankouping_progress=1;
				switch(RxBuffer1[2])
				{
					case 'A'://舵机调试
						
						switch(RxBuffer1[5])
						{
							case '1'://转臂
								if(RxBuffer1[4]=='N')//内
								{
									duoji_zhuanbi_slowaction(duoji_zhuanbi_nei,0);
									
								}
								else if(RxBuffer1[4]=='W')//外
								{
									duoji_zhuanbi_slowaction(duoji_zhuanbi_wai,0);
								}
//								duoji_zhuanbi(RxBuffer1[6]+256*RxBuffer1[7]);
								break;
							
							case '2'://料盘
								duoji_liaopan(RxBuffer1[6]+256*RxBuffer1[7]);
								break;	
							
							case '3'://夹爪
								if(RxBuffer1[4]=='J')//夹紧
								{
									duoji_jiazhao(duoji_jiazhao_jiajin);
								}
								else if(RxBuffer1[4]=='S')//松开
								{
									duoji_jiazhao(duoji_jiazhao_songkai);
								}
								else if(RxBuffer1[4]=='I')//初始化
								{
									duoji_jiazhao(duoji_jiazhao_songkai_init);
								}
								else
								{
									duoji_jiazhao(RxBuffer1[6]+256*RxBuffer1[7]);
								}
								
								break;
							
							case '4'://折叠
								
								if(RxBuffer1[4]=='D')//DOWN
								{
									duoji_zhedie_slowaction(duoji_zhedie_down,0);
									
								}
								else if(RxBuffer1[4]=='U')//UP
								{
									duoji_zhedie_slowaction(duoji_zhedie_up,0);
								}
//								duoji_zhedie(RxBuffer1[6]+256*RxBuffer1[7]);
								break;
						}
					break;
					case 'B'://陀螺仪
						
						switch(RxBuffer1[3])
						{
							case 'Y':
								send_yaw0();
							break;
							
							case 'P':
								send_pitch0();
							break;
							case 'R':
								OLEDShow_Angle();
							break;
						}
					
						break;
						
					case 'C'://任务模式
						
						switch(RxBuffer1[5])
						{
							case '1':task1_flag=1;break;
							case '2':task2_flag=1;break;
							case '3':task3_flag=1;break;
							case '4':task4_flag=1;break;
							case '5':task5_flag=1;break;
							case '6':task6_flag=1;break;
							case '7':task7_flag=1;break;
							case '8':task8_flag=1;break;
							case '9':task9_flag=1;break;
							case 'A':taska_flag=1;break;
							case 'B':taskb_flag=1;break;
							case 'C':taskc_flag=1;break;
							case 'D':taskd_flag=1;break;
							case 'F':taskf_flag=1;break;
						}
						break;
					case 'D':
						
						if(RxBuffer1[4]=='0' && RxBuffer1[5]=='5')//升降电机
						{
							if(RxBuffer1[6]=='U')//上升
							{
								if(RxBuffer1[7]=='1')//开始
								{
									bujin5_upflag=1;
									 Emm_V5_Pos_Control(0x05, 1, 100,20,500, 0, 0);		
								}
								else if(RxBuffer1[7]=='0')//结束
								{
									bujin5_upflag=0;
								}
							}
							else if(RxBuffer1[6]=='D')//下降
							{
								if(RxBuffer1[7]=='1')//开始
								{
									bujin5_downflag=1;
									Emm_V5_Pos_Control(0x05, 0, 100,20,500, 0, 0);	
								}
								else if(RxBuffer1[7]=='0')//结束
								{
									bujin5_downflag=0;
								}
							}
							else if(RxBuffer1[6]=='0'&& RxBuffer1[7]=='0')//失能
							{
								Emm_V5_En_Control(0x05,0,0);
							}
							else if(RxBuffer1[6]=='1'&& RxBuffer1[7]=='1')//使能
							{
								Emm_V5_En_Control(0x05,1,0);
							}
							else if(RxBuffer1[6]=='A'&& RxBuffer1[7]=='A')//回零 
							{
								Emm_V5_Origin_Trigger_Return(0x05,2,0);
							}	
							
						}
						else if(RxBuffer1[4]=='1' && RxBuffer1[5]=='4')
						{
							
							
							
							
						}
		
						break;
						
						case 'F': //扫码器
							if(RxBuffer1[5]=='1')//触发扫码
							{
								saoma();
							}
							break;
							
						case 'I':
							
						
						break;
						case 'H':
							if(RxBuffer1[5]=='1')//激光开
							{
								jiguang(1);
							}
							else if(RxBuffer1[5]=='0')//激光关
							{
								jiguang(0);
							}
						
						break;
						case 'G':
							if(RxBuffer1[3]=='T' && RxBuffer1[4]=='I'&& RxBuffer1[5]=='A'&& RxBuffer1[6]=='O')
							{
								tiaoshi_flag=RxBuffer1[7]-'0';
							}
						
						break;
						
						case 'R'://RGB调试
							if(RxBuffer1[6]=='1')
							{
								L=RxBuffer1[7];
								RGB_Show_64();
							}
						
							break;
						
						
				}				
			}
		}
		else   //接收异常
		{
				RxState = 0;
				RxCounter1=0;
				for(i=0;i<10;i++)
				{
					RxBuffer1[i]=0x00;      //将存放数据数组清零
				}
		}
}


int cheat_flag = 0;
uint8_t color_mode1=0,mode1_position=0,mode1_xerr=0,mode1_yerr=0,mode1_receiveflag=0,
mode2_position=0,mode2_xerr=0,mode2_yerr=0,mode2_receiveflag=0;


void shijue_Receive_Data(uint8_t com_data)
{
		uint8_t i;
		static uint8_t RxCounter1=0;//计数
		static uint16_t RxBuffer1[10]={0};
		static uint8_t RxState = 0;	
    	static uint8_t RxFlag1;
		static uint16_t shijue_array1[10]={'S','T','M','C','U','C','O','T',0xff,0xff};
		static uint8_t  C1='S',C2='T',C3='O',C4='K',C5='E',C6='N';
		if(RxState==0&&com_data==0x2C)  
		{
			RxState=1;
			RxBuffer1[RxCounter1++]=com_data; 		
		}
		else if(RxState==1&&com_data==0x12)  
		{
			RxState=2;
			RxBuffer1[RxCounter1++]=com_data;
		}
		else if(RxState==2)
		{
			RxBuffer1[RxCounter1++]=com_data;
			if((RxCounter1>=8)&&(com_data ==0xFF))       //RxBuffer1接受满了,接收数据结束
			{
				
				RxState=0;
				RxFlag1=1;
				RxCounter1=0;
				
				//'S''T' 0x01 0x01 0x01 0x00 0x00 0Xff
				
				if(RxBuffer1[2]==0x01)//模式1,识别旋转的物料颜色
				{
					color_mode1=RxBuffer1[3];
					mode1_position=RxBuffer1[4];
					mode1_xerr=RxBuffer1[5];
					mode1_yerr=RxBuffer1[6];
					mode1_receiveflag=1;	
					
					
					switch(RxBuffer1[3])
					{
						case 0x01:LED(1);break;//红色
						case 0x02:LED(2);break;//绿色
						case 0x03:LED(3);break;//蓝色
						
					}
					
					
				}
				else if(RxBuffer1[2]==0x02)
				{
					mode2_position=RxBuffer1[4];
					mode2_xerr=RxBuffer1[5];
					mode2_yerr=RxBuffer1[6];
					mode2_receiveflag=1;	
				}
				else if(RxBuffer1[2]==0x03)
				{
					mode2_position=RxBuffer1[4];
					mode2_xerr=RxBuffer1[5];
					mode2_yerr=RxBuffer1[6];
					mode2_receiveflag=1;	
					
				}
				
				
				
				
				
//				if(RxBuffer1[2]=='G'&& RxBuffer1[3]=='O'&& RxBuffer1[4]=='T'&& RxBuffer1[5]=='O')
//				{
//					postion[0]=(RxBuffer1[6]-'0'-1)*3+(RxBuffer1[7]-'0');
//				}
//				else if(RxBuffer1[2]=='C'&& RxBuffer1[3]=='H')
//				{
//					postion_from=(RxBuffer1[4]-'0'-1)*3+(RxBuffer1[5]-'0');
//					
//					postion_to=(RxBuffer1[6]-'0'-1)*3+(RxBuffer1[7]-'0');
//					
//					cheat_flag=RxBuffer1[6];

//						
//					if(RxBuffer1[6]=='0'&& RxBuffer1[7]=='0')
//					{
//						postion[0]=postion_from;
//						win_flag=2;//和棋
//						
//					}
//					else if(RxBuffer1[6]=='0'&& RxBuffer1[7]=='1')
//					{
//						postion[0]=postion_from;
//						win_flag=1;//机器胜利
//						
//					}
//				}
//				else if(RxBuffer1[2]=='I'&& RxBuffer1[3]=='W'&& RxBuffer1[4]=='I'&& RxBuffer1[5]=='N'&& RxBuffer1[6]=='E'&& RxBuffer1[7]=='R')
//				{
//					win_flag=1;//机器胜利
//					
//				}
//				else if(RxBuffer1[2]=='N'&& RxBuffer1[3]=='O'&& RxBuffer1[4]=='W'&& RxBuffer1[5]=='I'&& RxBuffer1[6]=='N'&& RxBuffer1[7]==' ')
//				{
//					win_flag=2;//和棋
//					
//				}
//				else if(RxBuffer1[2]=='S')
//				{
//					switch(RxBuffer1[3])
//					{
//						LED_show(2,1);
//						case '1':  XY[0][0]=RxBuffer1[5]*256 + RxBuffer1[4]-320;   XY[0][1]= RxBuffer1[7]*256 + RxBuffer1[6]-240;     break;
//						case '2':  XY[1][0]=RxBuffer1[5]*256 + RxBuffer1[4]-320;   XY[1][1]= RxBuffer1[7]*256 + RxBuffer1[6]-240;      break;
//						case '3':  XY[2][0]=RxBuffer1[5]*256 + RxBuffer1[4]-320;   XY[2][1]= RxBuffer1[7]*256 + RxBuffer1[6]-240;       break;
//						case '4':  XY[3][0]=RxBuffer1[5]*256 + RxBuffer1[4]-320;    XY[3][1]= RxBuffer1[7]*256 + RxBuffer1[6]-240;       break;
//						case '5':  XY[4][0]=RxBuffer1[5]*256 + RxBuffer1[4]-320;    XY[4][1]= RxBuffer1[7]*256 + RxBuffer1[6]-240;       break;
//						case '6':  XY[5][0]=RxBuffer1[5]*256 + RxBuffer1[4]-320;    XY[5][1]= RxBuffer1[7]*256 + RxBuffer1[6]-240;       break;
//						case '7':  XY[6][0]=RxBuffer1[5]*256 + RxBuffer1[4]-320;    XY[6][1]= RxBuffer1[7]*256 + RxBuffer1[6]-240;       break;
//						case '8':  XY[7][0]=RxBuffer1[5]*256 + RxBuffer1[4]-320;    XY[7][1]= RxBuffer1[7]*256 + RxBuffer1[6]-240;       break;
//						case '9':  XY[8][0]=RxBuffer1[5]*256 + RxBuffer1[4]-320;    XY[8][1]= RxBuffer1[7]*256 + RxBuffer1[6]-240;    XY_flag=1;  break;
//							
//					}
//				}
//				
//				
//				
//				
//				
//				if(memcmp(shijue_array1,RxBuffer1,10)==0)
//				{
//					
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C1, 1, 1000);
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C2, 1, 1000);
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C3, 1, 1000);
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C4, 1, 1000);
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C5, 1, 1000);
//					HAL_UART_Transmit(&huart3, (uint8_t *)&C6, 1, 1000);
//					nano_ok=1;
//					
//				}
				
				shibie_progress=1;
				
			}
		}
		else   //接收异常
		{
				RxState = 0;
				RxCounter1=0;
				for(i=0;i<8;i++)
				{
					RxBuffer1[i]=0x00;      //将存放数据数组清零
				}
		}
}

void JY62_Receive_Data(uint8_t com_data)
{
		uint8_t i;
		static uint8_t RxCounter1=0;//计数
		static uint16_t RxBuffer1[11]={0};
		static uint8_t RxState = 0;	
    	static uint8_t RxFlag1;
 
		if(RxState==0&&com_data==0x55)  //0x55帧头
		{
			RxState=1;
			RxCounter1=0;
			RxBuffer1[RxCounter1++]=com_data;   
		}
		else if(RxState==1)
		{
			RxBuffer1[RxCounter1++]=com_data;
			if(RxCounter1>=11)       //RxBuffer1接受满了,接收数据结束
			{
				RxState=0;
				RxFlag1=1;
				RxCounter1 = 0;
				switch(RxBuffer1[1])
				{
					case 0x53:
						angle[0]=(short)(RxBuffer1[3]<<8|RxBuffer1[2])/32768.0*180;//Pitch：俯仰角
						angle[1]=(short)(RxBuffer1[5]<<8|RxBuffer1[4])/32768.0*180;//Roll：滚转角
						angle[2]=(short)(RxBuffer1[7]<<8|RxBuffer1[6])/32768.0*180;//yaw：偏航角
						break;
				}
			

				
			}
		}
		else   //接收异常
		{
				RxState = 0;
				RxCounter1=0;
				for(i=0;i<11;i++)
				{
					RxBuffer1[i]=0x00;      //将存放数据数组清零
				}
		}
}

void uart4_Receive_Data(uint8_t com_data)//包长为4，步进电机
{
		uint8_t i;
		static uint8_t RxCounter1=0;//计数
		static uint8_t RxBuffer1[4]={0};
		static uint8_t RxState = 0;	
    	static uint8_t RxFlag1;
 
		if(RxState==0&&(com_data==0x01||com_data==0x02||com_data==0x00))  //0x01帧头
		{
			RxState=1;
			RxBuffer1[RxCounter1++]=com_data;   
		}
		else if(RxState==1)
		{
			RxBuffer1[RxCounter1++]=com_data;
			if(RxCounter1>=4)       //定包长，包长为4，RxBuffer1接受满了,接收数据结束
			{
				RxState=0;
				RxFlag1=1;
				RxCounter1 = 0;	
				if(memcmp(RxBuffer1,bujin2_array1,4)==0)
				{
					bujin2_daoweiflag= true ;
//				printf("page1_5.t1.txt=\"到位OK\"\xff\xff\xff");
				}
				else if(memcmp(RxBuffer1,bujin1_array1,4)==0)
				{
					bujin1_daoweiflag=true;
//				printf("page1_4.t1.txt=\"到位OK\"\xff\xff\xff");
				}
				else 
				{
					rxFrameFlag = true;
				}
				
				 // 一帧数据接收完成，置位帧标志位
				for(i=0;i<4;i++)
				{
					RxBuffer1[i]=0x00;      //将存放数据数组清零
				}
			}
		}
		else   //接收异常
		{
				RxState = 0;
				RxCounter1=0;
				for(i=0;i<4;i++)
				{
					RxBuffer1[i]=0x00;      //将存放数据数组清零
				}
		}
}
void uart5_Receive_Data(uint8_t com_data)//扫码器
{
		#define uart5_Bufferlong 8
		uint8_t i;
		static uint8_t RxCounter1=0;//计数
		static uint8_t RxBuffer1[15]={0};
		static uint8_t RxState = 0;	
    	static uint8_t RxFlag1;
		static uint8_t shoudao[7]={0x02,0X00,0X00,0X01,0X00,0X33,0X31};
		
		
//		if(RxState==0&&(com_data=='1'||com_data=='2'||com_data=='3')) 
//		{
//			RxState=1;
//			RxBuffer1[RxCounter1++]=com_data;   
//		}
//		else if(RxState==1)
//		{
//			RxBuffer1[RxCounter1++]=com_data;
//			if(RxCounter1>=uart5_Bufferlong)   
//			{
//				if(RxBuffer1[3]=='+')
//				{
//					for(i=0;i<uart5_Bufferlong-1;i++)
//					{
//						paixu[i]=RxBuffer1[i];
//					}
//					UART_printf(&huart2,"t2.txt=\"%s\"\xff\xff\xff",paixu);//可以这样发连续的,串口屏显示
//					RxState=0;
//					RxFlag1=1;
//					RxCounter1 = 0;	
//				}
//				else
//				{
//					for(i=0;i<uart5_Bufferlong;i++)
//					{
//						RxBuffer1[i]=0x00;      //将存放数据数组清零
//					}
//					RxState = 0;
//					RxCounter1=0;
//				}
//			}
//		}
//		else   //接收异常
//		{
//				RxState = 0;
//				RxCounter1=0;
//				for(i=0;i<uart5_Bufferlong;i++)
//				{
//					RxBuffer1[i]=0x00;      //将存放数据数组清零
//				}
//		}
		
		
		if(RxState==0&&com_data==0x02)  //0x02帧头
		{
			RxState=1;
			RxBuffer1[RxCounter1++]=com_data;   
		}
		else if(RxState==1&&com_data==0x00)  //0x00帧头
		{
			RxState=2;
			RxBuffer1[RxCounter1++]=com_data;   
		}
		else if(RxState==2)
		{
			RxBuffer1[RxCounter1++]=com_data;
			if((RxCounter1>=7 )&& (memcmp(RxBuffer1,shoudao,7)==0))   
			{
				RxState=3;
			}
		}
		else if(RxState==3)
		{
			RxBuffer1[RxCounter1++]=com_data;
			
			paixu[RxCounter1-8]=RxBuffer1[RxCounter1-1];
			if(RxCounter1>=15)
			{
				UART_printf(&huart2,"t2.txt=\"%s\"\xff\xff\xff",paixu);//可以这样发连续的,串口屏显示
				
//				flag_saomaqi=1;
				
				
				for(i=0;i<8;i++)
				{
					paixu[i]=RxBuffer1[7+i];
				}
			
				 // 一帧数据接收完成，置位帧标志位
				for(i=0;i<15;i++)
				{
				RxBuffer1[i]=0x00;      //将存放数据数组清零
				}
				
				RxState=0;
				RxFlag1=1;
				RxCounter1 = 0;	
			}
			
		}
		else   //接收异常
		{
				RxState = 0;
				RxCounter1=0;
				for(i=0;i<15;i++)
				{
					RxBuffer1[i]=0x00;      //将存放数据数组清零
				}
		}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)		//串口接收数据，使用状态机的方式。
{
    uint8_t tem1,tem3,tem2,tem4,tem5,tem6;// 这里的是无符号的
	
	
	if(huart->Instance== USART1)     //陀螺仪
	 {
		tem1=uart1_rxbuff;
		JY62_Receive_Data(tem1);
		HAL_UART_Receive_IT(&huart1,&uart1_rxbuff,1); 
	 }
	 
	 if(huart->Instance== USART3)     //视觉
	 {
		tem3=uart3_rxbuff;
		shijue_Receive_Data(tem3);
		HAL_UART_Receive_IT(&huart3,&uart3_rxbuff,1); 
	 }	
	if(huart->Instance== USART2)    //串口屏
	 {	   
		tem2=uart2_rxbuff;
		chuankouping_Receive_Data(tem2);
		HAL_UART_Receive_IT(&huart2,&uart2_rxbuff,1); 
	 }	
	 if(huart->Instance== UART4)    //蓝牙
	 {	   
		tem4=uart4_rxbuff;
		uart4_Receive_Data(tem4);
		HAL_UART_Receive_IT(&huart4,&uart4_rxbuff,1); 
	 }
	 if(huart->Instance== UART5)    //扫码器
	 {	  
			 
		tem5=uart5_rxbuff;
		uart5_Receive_Data(tem5);
		HAL_UART_Receive_IT(&huart5,&uart5_rxbuff,1); 
	 }
     if(huart->Instance== USART6)    //总线舵机
	 {	  
			 
		tem6=uart6_rxbuff;
		RingBuffer_Push(usart2.recvBuf, tem6);
		HAL_UART_Receive_IT(&huart6,&uart6_rxbuff,1); 
	 }
	 
	
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if(htim->Instance == TIM9 )
    {
	
		
    }
	
	if(htim->Instance == TIM6 )
    {
		/**********************************示波器Minibalance************************************/
	
//	        DataScope_Get_Channel_Data(Target_Velocity, 1 );
//			DataScope_Get_Channel_Data(encoder_count1, 2 );// Incremental_PI_1
//			DataScope_Get_Channel_Data( encoder_count4, 3 ); //Incremental_PI_2
//			DataScope_Get_Channel_Data( encoder_count5 , 4 );   //Incremental_PI_3
//			DataScope_Get_Channel_Data(encoder_count8, 5 );//Incremental_PI_4
//			Send_Count = DataScope_Data_Generate(5);
//			for( i = 0 ; i < Send_Count; i++) 
//			{
//			while((USART2->SR&0X40)==0);  
//			USART2->DR = DataScope_OutPut_Buffer[i]; 
//			}
	
//	
//	
//	        a+=0.1;
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
//			
//	/**********************************示波器Minibalance************************************/
		
	}

	if(htim->Instance == TIM7 )//5ms
	{
		Time_count++;
		if(Time_count>=1)
		{
			Time_count=0;

		}
	}
	
	
	
	
	
}
//uint16_t h=0,f=0,g=0;
void  HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2 )
	{
//	
//		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
//		{
////			h++;
//			HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_1);
//		}
//		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
//		{
////			f++;
//			HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_2);
//		}
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)
		{
//			g++;
			HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_3);
		}
	
	}
	

}
	



void OLEDShow_Angle(void)
{
//	  OLED_Clear();
//		
//	  OLED_ShowString(0,16,"pitch:",OLED_8X16);
//	  OLED_ShowFloatNum(48, 16, angle[0],3,4,OLED_8X16);//pitch
//	  OLED_ShowString(0,0,"  yaw:",OLED_8X16);
//	  OLED_ShowFloatNum(48, 0, angle[2],3,4,OLED_8X16);//yaw
//	  OLED_ShowString(0,32," roll:",OLED_8X16);
//	  OLED_ShowFloatNum(48, 32, angle[1],3,4,OLED_8X16);//roll
//	  
//	  OLED_Update();
	
		UART_printf(&huart2,"x0.val=%d\xff\xff\xff",(int)(angle[2]*1000));
		UART_printf(&huart2,"x1.val=%d\xff\xff\xff",(int)(angle[0]*1000));
		UART_printf(&huart2,"x2.val=%d\xff\xff\xff",(int)(angle[1]*1000));
	
	
	
//	   printf("page1_3.x0.val=%d\xff\xff\xff",(int)(angle[2]*1000));//串口屏
//	   printf("page1_3.x1.val=%d\xff\xff\xff",(int)(angle[0]*1000));
//	   printf("page1_3.x2.val=%d\xff\xff\xff",(int)(angle[1]*1000));
	
	
	
}

void send_HWT101_unlock(void)//解锁寄存器
{
	static uint8_t  UNLOCK[5]={0xFF,0xAA,0x69,0x88,0xB5};	
	for(uint8_t i=0;i<5;i++)	
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)&UNLOCK[i], 1, 1000);
	}	
}
void send_HWT101_lock(void)//保存设置
{
	static uint8_t  LOCK[5]={0xFF,0xAA,0x00,0x00,0x00};	
	for(uint8_t i=0;i<5;i++)	
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)&LOCK[i], 1, 1000);
	}	
}
void HWT101_yaw0(void)
{
	static uint8_t  Z0[5]={0xFF,0xAA,0x76,0x00,0x00};	
	for(uint8_t i=0;i<5;i++)	
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)&Z0[i], 1, 1000);
	}	
}
void send_yaw0(void)//Z轴归零==yaw角归零
{
	
	if(select_JY62orHWT101==HWT101)
	{
		send_HWT101_unlock();
		for(uint32_t i=0;i<=15000;i++);
		HWT101_yaw0();
		for(uint32_t i=0;i<=15000;i++);
		send_HWT101_lock();
	}
	else//JY62
	{
		static uint8_t  Y1=0xFF,Y2=0xAA,Y3=0x52;
		HAL_UART_Transmit(&huart1, (uint8_t *)&Y1, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t *)&Y2, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t *)&Y3, 1, 1000);
	}
	
	

}
void Mode_lingpiao(void)
{
	static uint8_t  piao[5]={0xFF,0xAA,0x48,0x01,0x00};
	if(select_JY62orHWT101==HWT101)
	{
		send_HWT101_unlock();
		for(uint32_t i=0;i<=15000;i++);
			
		for(uint8_t i=0;i<5;i++)	
		{
			HAL_UART_Transmit(&huart1, (uint8_t *)&piao[i], 1, 1000);
		}	
		
		HAL_Delay(30*1000);
		
		
		send_HWT101_unlock();
		for(uint32_t i=0;i<=15000;i++);
		send_HWT101_lock();
	}
	LED(4);
}




void send_pitch0(void)//加计校准,pitch角偏移时使用
{
	if(select_JY62orHWT101==HWT101)
	{
		
	}
	else//JY62
	{
		static uint8_t  Cx=0xFF,Cy=0xAA,Cz=0x67;
		HAL_UART_Transmit(&huart1, (uint8_t *)&Cx, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t *)&Cy, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t *)&Cz, 1, 1000);
	}
	 
}
void JY62_Init(void)
{
	send_yaw0();
	HAL_Delay(500);
	send_pitch0();
	HAL_Delay(800);
}



///重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
//使用时把“Use MicroLIB”勾选上
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}

///重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
//使用时把“Use MicroLIB”勾选上
int fgetc(FILE *f)
{
		
	int ch;
	HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}








int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    int length;
    char buffer[128];

    length = vsnprintf(buffer, 128, fmt, ap);

//    HAL_UART_Transmit_IT(huart, (uint8_t *)buffer, length);
HAL_UART_Transmit(huart, (uint8_t *)buffer, length, HAL_MAX_DELAY);
    va_end(ap);
    return length;
}




































/********************************nano***********************************/

//# define usart_buffer_length  20


//char usart_buffer[5][usart_buffer_length]={0};
//char usart_pointer[5] = {0};
//char connect_usart[5] = {0};											//连接标志位

///*
//		connect_usart[*] 标志位说明
//		0xFF   1       2       3       4       5       6       7       8

//    第一位	接收允许标志位 置1允许，否则不允许 	connect_usart[*]&0x80

//		第二位	连接建立标志位 置位表示连接已经建立 connect_usart[*]&0x40

//		第三位	第一次握手	首次接收到'S'时置位     
//						if( (res=='S') && !(connect_usart[*]&0x40)){connect_usart[*]|=0x20;}

//		第四位	第二次握手	首次接收到'T'时置位			
//						if( (res=='T') && !(connect_usart[*]&0x40) && (connect_usart[*]&0x20))
//						{connect_usart[*]|=0x10;connect_usart[*]|=0x40;}

//		第五位	连接注销标志位1 接收到0xFF时置位，若后面接收到的字符不是0xFF则将会被取消置位
//						if(res==0xFF){connect_usart[*]|=0x08;}

//		第六位	连接注销标志位2 接收到'N'时且上一个字符0xFF时置位 ,同时清空连接建立标志位
//						if(res==0xFF && (connect_usart[*]&=0x08) )
//						{connect_usart[*]|=0x06;connect_usart[*]&=0xBF;}
//			
//		第七位	数据可读位 置位可以读取，否则不可读取
//						
//		第八位	reserved
//*/
//void Receiver(uint8_t num,uint8_t res)
//{
//	if(connect_usart[num] & 0x80)
//	{
//		if(connect_usart[num]&0x40)
//		{
//			if((res==0xFF) && !(connect_usart[num] & 0x08) )
//			{connect_usart[num]|=0x08;}
//			else if((res==0xFF) && (connect_usart[num] & 0x08) )
//			{
//				//清空连接标志位和连接建立标志位
//				connect_usart[num]&= 0x8F;
//				//连接注销标志位2置位，并设置可读标志位 
//				connect_usart[num]|= 0x06;
//			}
//			else
//			{
////				while(USART_GetFlagStatus(HMI_UART, USART_FLAG_TXE) == RESET);
////				USART_SendData(HMI_UART,res);
//				connect_usart[num]&=0xF7;
//				usart_buffer[num][usart_pointer[num]]=res;
//				usart_pointer[num]++;
//				if(usart_pointer[num]>=usart_buffer_length)
//				{
//					usart_pointer[num] = 0;
//				}
//			}
//		}
//		else
//		{
//			if( (res=='S') && !(connect_usart[num]&0x40))
//			{connect_usart[num]|=0x20;}
//			else if( (res=='T') && !(connect_usart[num]&0x40) && (connect_usart[num]&0x20))
//			{
//				//第二次握手与连接建立标志位置位
//				connect_usart[num]|=0x50;
//				//清空注销标志位并设置数据不可读取状态
//				connect_usart[num]&=0xF1;	
//				//指针归零
//				usart_pointer[num]=0;
//			}
//		}
//	}
//}



//void open_uart(uint8_t num)
//{
//	connect_usart[num-1]|=0x80;
//}

//void close_uart(uint8_t num)
//{
//	connect_usart[num-1]&=0x7F;
//}

//char act_one(uint8_t num)
//{
//	uint8_t temp = num-1;
//	if(connect_usart[temp]&0x02)
//	{
//		//暂停接收数据
//		connect_usart[temp]&=0x7F;
//		switch(temp)
//		{
//			case 0:	break;
//			case 1:	break;
//			case 2:	nano_service(usart_buffer[2],usart_pointer[2]);break;
//			case 3:	break;
//			case 4:	break;
//		}
//		for(uint8_t i=0;i<usart_pointer[temp];i++)
//		{
//			usart_buffer[temp][i] = 0;
//		}
//		//继续接收数据，设置数据不可读取
//		connect_usart[temp]&= 0xFD;
//		connect_usart[temp]|=0x80;
//		return 1;
//	}
//	else{return 0;}
//}


//uint8_t distance_info_on = 0;

//void act_all(void)
//{
//	for(uint8_t i=1;i<4;i++)
//	{
//		act_one(i);
//	}
//}

/********************************nano***********************************/
