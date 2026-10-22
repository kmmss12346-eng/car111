#include "yaw_control.h"


float Yaw_Control(float target_yaw,float current_yaw)
{


    float error;

    float output;


    //计算角度误差
    error = target_yaw - current_yaw;


    //比例控制
    output = error * 20;


    return output;

}