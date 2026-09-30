#include "Bujin.h"
#include "Task.h"













void move_front(uint16_t vel, uint8_t acc, uint32_t clk, bool raF)
{
	
	  Emm_V5_Pos_Control(1, 0, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(2, 1, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Pos_Control(3, 0, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(4, 1, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Synchronous_motion(0x00);
	
}
void move_back(uint16_t vel, uint8_t acc, uint32_t clk, bool raF)
{
	
	  Emm_V5_Pos_Control(1, 1, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(2, 0, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Pos_Control(3, 1, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(4, 0, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Synchronous_motion(0x00);
	
}
void move_left(uint16_t vel, uint8_t acc, uint32_t clk, bool raF)
{
	
	  Emm_V5_Pos_Control(1, 1, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(2, 1, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Pos_Control(3, 0, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(4, 0, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Synchronous_motion(0x00);
	
}

void move_right(uint16_t vel, uint8_t acc, uint32_t clk, bool raF)
{
	
	  Emm_V5_Pos_Control(1, 0, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(2, 0, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Pos_Control(3, 1, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(4, 1, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Synchronous_motion(0x00);
	
}

void move_front_left(uint16_t vel, uint8_t acc, uint32_t clk, bool raF)
{
	
//	  Emm_V5_Pos_Control(1, 0, vel, acc,clk, 0, 1);	
//	  HAL_Delay(100);	
	  Emm_V5_Pos_Control(2, 1, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Pos_Control(3, 0, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
//	  Emm_V5_Pos_Control(4, 1, vel, acc,clk, 0, 1);
//	  HAL_Delay(100);
	  Emm_V5_Synchronous_motion(0x00);
	
}
void move_front_right(uint16_t vel, uint8_t acc, uint32_t clk, bool raF)
{
	
	  Emm_V5_Pos_Control(1, 0, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
//	  Emm_V5_Pos_Control(2, 1, vel, acc,clk, 0, 1);
//	  HAL_Delay(100);
//	  Emm_V5_Pos_Control(3, 0, vel, acc,clk, 0, 1);	
//	  HAL_Delay(100);	
	  Emm_V5_Pos_Control(4, 1, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Synchronous_motion(0x00);
}
void move_back_left(uint16_t vel, uint8_t acc, uint32_t clk, bool raF)
{
	
//	  Emm_V5_Pos_Control(1, 0, vel, acc,clk, 0, 1);	
//	  HAL_Delay(100);	
	  Emm_V5_Pos_Control(2, 0, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Pos_Control(3, 1, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
//	  Emm_V5_Pos_Control(4, 1, vel, acc,clk, 0, 1);
//	  HAL_Delay(20);
	  Emm_V5_Synchronous_motion(0x00);
	
}
void move_back_right(uint16_t vel, uint8_t acc, uint32_t clk, bool raF)
{
	
	  Emm_V5_Pos_Control(1, 1, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
//	  Emm_V5_Pos_Control(2, 1, vel, acc,clk, 0, 1);
//	  HAL_Delay(100);
//	  Emm_V5_Pos_Control(3, 0, vel, acc,clk, 0, 1);	
//	  HAL_Delay(100);	
	  Emm_V5_Pos_Control(4, 0, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Synchronous_motion(0x00);
}
void move_shun(uint16_t vel, uint8_t acc, uint32_t clk, bool raF)
{
	
	  Emm_V5_Pos_Control(1, 0, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(2, 0, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Pos_Control(3, 0, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(4, 0, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Synchronous_motion(0x00);
	
}
void move_ni(uint16_t vel, uint8_t acc, uint32_t clk, bool raF)
{
	
	  Emm_V5_Pos_Control(1, 1, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(2, 1, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Pos_Control(3, 1, vel, acc,clk, 0, 1);	
	  HAL_Delay(20);	
	  Emm_V5_Pos_Control(4, 1, vel, acc,clk, 0, 1);
	  HAL_Delay(20);
	  Emm_V5_Synchronous_motion(0x00);	
}
void move_stopall(void)
{
	Emm_V5_Stop_Now(1,1);
	HAL_Delay(12);
	Emm_V5_Stop_Now(2,1);
	HAL_Delay(12);
	Emm_V5_Stop_Now(3,1);
	HAL_Delay(12);
	Emm_V5_Stop_Now(4,1);
	HAL_Delay(12);
	Emm_V5_Synchronous_motion(0x00);
}





void move_shun90(void)
{
	move_shun(3000,100,pulse_shun90,0);//3000,20
	car_state(1);
}
void move_shun180(void)
{
	move_shun(100,10,pulse_shun180,0);
	car_state(1);
	car_state(1);
}
void move_ni90(void)
{
	move_ni(100,10,pulse_ni90,0);
	car_state(0);
}
void move_ni180(void)
{
	move_ni(100,10,pulse_ni180,0);
	car_state(0);
	car_state(0);
}


void move_tongbudai(uint16_t vel, uint8_t acc, uint32_t clk)
{
	if(clk>=11300)
	{
		clk=11300;
	}
	else if(clk<=0)
	{
		clk=0;
	}
	Emm_V5_Pos_Control(0x05, 1, vel,acc,clk, 1, 0);		
}

void Ve_bujin1( int32_t vel, uint8_t acc)
{
	if(vel>=0){Emm_V5_Vel_Control(1,0,vel,acc,1);}
	else{Emm_V5_Vel_Control(1,1,-vel,acc,1);}
}
void Ve_bujin2( int32_t vel, uint8_t acc)
{
	if(vel>=0){Emm_V5_Vel_Control(2,1,vel,acc,1);}
	else{Emm_V5_Vel_Control(2,0,-vel,acc,1);}
}
void Ve_bujin3( int32_t vel, uint8_t acc)
{
	if(vel>=0){Emm_V5_Vel_Control(3,0,vel,acc,1);}
	else{Emm_V5_Vel_Control(3,1,-vel,acc,1);}
}
void Ve_bujin4( int32_t vel, uint8_t acc)
{
	if(vel>=0){Emm_V5_Vel_Control(4,1,vel,acc,1);}
	else{Emm_V5_Vel_Control(4,0,-vel,acc,1);}
}
void V_xy(int32_t V_x,int32_t V_y,uint8_t acc)
{
	
	int32_t V_bujin1,V_bujin2,V_bujin3,V_bujin4;
	V_bujin1=V_x+V_y;
	V_bujin2=V_x-V_y;
	V_bujin3=V_x-V_y;
	V_bujin4=V_x+V_y;
	
	Ve_bujin1(V_bujin1,acc);
	HAL_Delay(15);
	Ve_bujin2(V_bujin2,acc);
	HAL_Delay(15);
	Ve_bujin3(V_bujin3,acc);
	HAL_Delay(15);
	Ve_bujin4(V_bujin4,acc);
	HAL_Delay(15);
	Emm_V5_Synchronous_motion(0x00);
}
//void P_xy()
//{
//	
//}



/*
*	函数功能：取绝对值
*	入口参数：int 
*	返回值：无 unsingned int 
*/

int myabs(int num)
{
	int temp;
	if(num<0)	temp=-num;
	else temp =num;
	return temp;
}
uint8_t car_state_number=1;
int32_t last_x=0,last_y=0,x_sum=0,y_sum=0;
void move(uint8_t mode ,uint16_t vel, uint8_t acc,int32_t x,int32_t y)//x y 单位为1mm 
{
    int32_t pulse_x,pulse_y,distance_x,distance_y;
    distance_x=x-x_sum;
    distance_y=y-y_sum;
    pulse_x=pulse_onecircle/distance_onecircle*distance_x;//脉冲转换距离
    pulse_y=pulse_onecircle/distance_onecircle*distance_y;//脉冲转换距离
	switch(mode)
	{
		case 0://先走X，再走Y
            
			if(car_state_number==1)
            {
                //判断距离值正负 判断前进后退方向
                if(pulse_x>0)
                {
                    move_front(vel, acc,pulse_x,0);
                }
                else if(pulse_x<=0)
                {
                    pulse_x=myabs(pulse_x);
                    move_back(vel, acc,pulse_x,0);
                }
                
                if(pulse_y>0)
                {
                    move_left(vel, acc,pulse_y,0);
                }
                else if(pulse_y<=0)
                {
                    pulse_y=myabs(pulse_y);
                    move_right(vel, acc,pulse_y,0);
                }

            }
            
            else if(car_state_number==2)
            {
                if(pulse_x>0)
                {
                    move_left(vel, acc,pulse_x,0);
                }
                else if(pulse_x<=0)
                {
                    pulse_x=myabs(pulse_x);
                    move_right(vel, acc,pulse_x,0);
                }
                
                if(pulse_y>0)
                {
                    move_back(vel, acc,pulse_y,0);
                }
                else if(pulse_y<=0)
                {
                    pulse_y=myabs(pulse_y);
                    move_front(vel, acc,pulse_y,0);
                }                
            }
            
            else if(car_state_number==3)
            {
                if(pulse_x>0)
                {
                    move_back(vel, acc,pulse_x,0);
                }
                else if(pulse_x<=0)
                {
                    pulse_x=myabs(pulse_x);
                    move_front(vel, acc,pulse_x,0);
                }
                
                if(pulse_y>0)
                {
                    move_right(vel, acc,pulse_y,0);
                }
                else if(pulse_y<=0)
                {
                    pulse_y=myabs(pulse_y);
                    move_left(vel, acc,pulse_y,0);
                }                
            }           
            
            else if(car_state_number==4)
            {
                if(pulse_x>0)
                {
                    move_right(vel, acc,pulse_x,0);
                }
                else if(pulse_x<=0)
                {
                    pulse_x=myabs(pulse_x);
                    move_left(vel, acc,pulse_x,0);
                }
                
                if(pulse_y>0)
                {
                    move_front(vel, acc,pulse_y,0);
                }
                else if(pulse_y<=0)
                {
                    pulse_y=myabs(pulse_y);
                    move_back(vel, acc,pulse_y,0);
                }                
            }                
		
		
			break;
		
		case 1://先y后x
            
			if(car_state_number==1)
            {
                //判断距离值正负 判断前进后退方向
                if(pulse_y>0)
                {
                    move_left(vel, acc,pulse_y,0);
                }
                else if(pulse_y<=0)
                {
                    pulse_y=myabs(pulse_y);
                    move_right(vel, acc,pulse_y,0);
                }
                
                if(pulse_x>0)
                {
                    move_front(vel, acc,pulse_x,0);
                }
                else if(pulse_x<=0)
                {
                    pulse_x=myabs(pulse_x);
                    move_back(vel, acc,pulse_x,0);
                }

            }
            
			else if(car_state_number==2)
            {
                //判断距离值正负 判断前进后退方向
                if(pulse_y>0)
                {
                    move_back(vel, acc,pulse_y,0);
                }
                else if(pulse_y<=0)
                {
                    pulse_y=myabs(pulse_y);
                    move_front(vel, acc,pulse_y,0);
                }
                
                if(pulse_x>0)
                {
                    move_left(vel, acc,pulse_x,0);
                }
                else if(pulse_x<=0)
                {
                    pulse_x=myabs(pulse_x);
                    move_right(vel, acc,pulse_x,0);
                }

            }
            
			else if(car_state_number==3)
            {
                //判断距离值正负 判断前进后退方向
                if(pulse_y>0)
                {
                    move_right(vel, acc,pulse_y,0);
                }
                else if(pulse_y<=0)
                {
                    pulse_y=myabs(pulse_y);
                    move_left(vel, acc,pulse_y,0);
                }
                
                if(pulse_x>0)
                {
                    move_back(vel, acc,pulse_x,0);
                }
                else if(pulse_x<=0)
                {
                    pulse_x=myabs(pulse_x);
                    move_front(vel, acc,pulse_x,0);
                }

            }         
            
			else if(car_state_number==4)
            {
                //判断距离值正负 判断前进后退方向
                if(pulse_y>0)
                {
                    move_front(vel, acc,pulse_y,0);
                }
                else if(pulse_y<=0)
                {
                    pulse_y=myabs(pulse_y);
                    move_back(vel, acc,pulse_y,0);
                }
                
                if(pulse_x>0)
                {
                    move_right(vel, acc,pulse_x,0);
                }
                else if(pulse_x<=0)
                {
                    pulse_x=myabs(pulse_x);
                    move_left(vel, acc,pulse_x,0);
                }

            }                 
			break;
            
		case 2:
			break;
	}
    x_sum+=x;
    y_sum+=y;
}


void car_state(uint8_t change)
{
	if(change==1)//顺时针
	{
		car_state_number++;
	}
	else//逆时针
	{
		car_state_number--;
	}
	if(car_state_number>4){car_state_number=1;}
	else if(car_state_number<1){car_state_number=4;}
}	
//	  









 void LED_show(uint8_t number,uint8_t kaiguan)
{
	if(number==1)
	{
		if(kaiguan==1)
		{
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);//绿灯
		}else
		{
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);//绿灯
		}
	}
	else if(number==2)
	{
		if(kaiguan==1)
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);//黄灯
		}
		else
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);//黄灯
		}
	}
	else if(number==3)
	{
		if(kaiguan==1)
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);//红灯
		}
		else
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);//红灯
		}
	}
	else if(number==4)
	{
		if(kaiguan==1)
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);//蜂鸣器
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);//蜂鸣器
		}
	}
    else if(number==5)//灯全关
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);//绿灯
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);//黄灯
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);//红灯
    }
	
}