#ifndef __RGB_H__
#define __RGB_H__
 
#include "main.h"
 
//0码和1码的定义，设置的时CCR寄存器的值
//由于使用的思PWM输出模式1，计数值<CCR时，输出有效电平-高电平（CubeMX配置默认有效电平为高电平）
#define CODE_1       (20)       //1码定时器计数次数，控制占空比为20/30 = 66%
#define CODE_0       (10)       //0码定时器计数次数，控制占空比为10/30 = 33%
 
extern uint8_t L;
 
 
 
//单个LED的颜色控制结构体
typedef struct
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
}RGB_Color_TypeDef;
 
#define Pixel_NUM 8  //LED数量宏定义，我们灯板上有8个，

static void Reset_Load(void); //该函数用于将数组最后24个数据变为0，代表RESET_code

    //发送最终数组
static void RGB_SendArray(void); 

static void RGB_Flush(void);  //刷新RGB显示

 
void RGB_SetOne_Color(uint8_t LedId,RGB_Color_TypeDef Color);//给一个LED装载24个颜色数据码（0码和1码）
     

//void RGB_RED(uint16_t Pixel_Len);  //显示红灯

//控制多个LED显示相同的颜色
void RGB_SetMore_Color(uint8_t head, uint8_t heal,RGB_Color_TypeDef color);


void RGB_Show_64(void); //RGB写入函数
 
#endif


