#include "motor.h"



void Right_Ctrl(int pwm_val)
{
	int temp=pwm_val;
	if(temp>1344){temp=1344;}
	if(temp<-1344){temp=-1344;}
	if(temp>=0)											//正转
	{
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,temp);
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
	}
	else if(temp<0)							  //反转
	{
		
		temp=-temp;
		 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,temp);
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
	}
}

void Left_Ctrl(int pwm_val)
{
	int temp=pwm_val;
	if(temp>1344){temp=1344;}
	if(temp<-1344){temp=-1344;}
	if(temp>=0)											//正转
	{
		 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,temp);
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	}
	else if(temp<0)							  	//反转
	{
		temp=-temp;
		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,temp);
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	}
}

