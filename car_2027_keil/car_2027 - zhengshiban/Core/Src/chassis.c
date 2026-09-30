#include "chassis.h"
#include "Emm_V5.h"
#include "hwt101.h"



#define PULSE_PER_MM_FORWARD   13.10f
#define PULSE_PER_MM_BACKWARD  13.10f
#define PULSE_PER_MM_LEFT      13.10f
#define PULSE_PER_MM_RIGHT     13.10f



extern float yaw_target;



/***********************
 * 距离转换
 ***********************/

void Car_Forward_mm(uint16_t speed,float distance_mm)
{
    uint32_t pulse;

    pulse =
    (uint32_t)(distance_mm * PULSE_PER_MM_FORWARD);


    Car_Forward(speed,pulse);
}



void Car_Backward_mm(uint16_t speed,float distance_mm)
{
    uint32_t pulse;


    pulse =
    (uint32_t)(distance_mm * PULSE_PER_MM_BACKWARD);


    Car_Backward(speed,pulse);
}



void Car_Left_mm(uint16_t speed,float distance_mm)
{
    uint32_t pulse;


    pulse =
    (uint32_t)(distance_mm * PULSE_PER_MM_LEFT);


    Car_Left(speed,pulse);
}



void Car_Right_mm(uint16_t speed,float distance_mm)
{
    uint32_t pulse;


    pulse =
    (uint32_t)(distance_mm * PULSE_PER_MM_RIGHT);


    Car_Right(speed,pulse);
}




/***********************
 * 停止
 ***********************/

void Car_Stop(void)
{

    Emm_V5_Stop_Now(1,true);
    HAL_Delay(10);


    Emm_V5_Stop_Now(2,true);
    HAL_Delay(10);


    Emm_V5_Stop_Now(3,true);
    HAL_Delay(10);


    Emm_V5_Stop_Now(4,true);
    HAL_Delay(10);



    Emm_V5_Synchronous_motion(0x00);

}





/***********************
 * 前进
 ***********************/

void Car_Forward(uint16_t speed,uint32_t pulse)
{


    //ID1 左前
    Emm_V5_Pos_Control(
        1,
        0,
        speed,
        10,
        pulse,
        0,
        true
    );

    HAL_Delay(10);



    //ID2 右前
    Emm_V5_Pos_Control(
        2,
        1,
        speed,
        10,
        pulse,
        0,
        true
    );

    HAL_Delay(10);



    //ID3 左后
    Emm_V5_Pos_Control(
        3,
        1,
        speed,
        10,
        pulse,
        0,
        true
    );

    HAL_Delay(10);



    //ID4 右后
    Emm_V5_Pos_Control(
        4,
        0,
        speed,
        10,
        pulse,
        0,
        true
    );

    HAL_Delay(10);



    Emm_V5_Synchronous_motion(0x00);

}





/***********************
 * 后退
 ***********************/

void Car_Backward(uint16_t speed,uint32_t pulse)
{


    Emm_V5_Pos_Control(
        1,
        1,
        speed,
        10,
        pulse,
        0,
        true
    );

    HAL_Delay(10);



    Emm_V5_Pos_Control(
        2,
        0,
        speed,
        10,
        pulse,
        0,
        true
    );

    HAL_Delay(10);



    Emm_V5_Pos_Control(
        3,
        0,
        speed,
        10,
        pulse,
        0,
        true
    );

    HAL_Delay(10);



    Emm_V5_Pos_Control(
        4,
        1,
        speed,
        10,
        pulse,
        0,
        true
    );


    HAL_Delay(10);



    Emm_V5_Synchronous_motion(0x00);

}
/***********************
 * 左平移
 ***********************/

void Car_Left(uint16_t speed,uint32_t pulse)
{

    Emm_V5_Pos_Control(
        1,
        0,
        speed,
        10,
        pulse,
        0,
        true
    );

    HAL_Delay(10);



    Emm_V5_Pos_Control(
        2,
        0,
        speed,
        10,
        pulse,
        0,
        true
    );

    HAL_Delay(10);



    Emm_V5_Pos_Control(
        3,
        0,
        speed,
        10,
        pulse,
        0,
        true
    );

    HAL_Delay(10);



    Emm_V5_Pos_Control(
        4,
        0,
        speed,
        10,
        pulse,
        0,
        true
    );


    HAL_Delay(10);


    Emm_V5_Synchronous_motion(0x00);

}



/***********************
 * 右平移
 ***********************/

void Car_Right(uint16_t speed,uint32_t pulse)
{


    Emm_V5_Pos_Control(
        1,
        1,
        speed,
        10,
        pulse,
        0,
        true
    );


    HAL_Delay(10);



    Emm_V5_Pos_Control(
        2,
        1,
        speed,
        10,
        pulse,
        0,
        true
    );


    HAL_Delay(10);



    Emm_V5_Pos_Control(
        3,
        1,
        speed,
        10,
        pulse,
        0,
        true
    );


    HAL_Delay(10);



    Emm_V5_Pos_Control(
        4,
        1,
        speed,
        10,
        pulse,
        0,
        true
    );


    HAL_Delay(10);


    Emm_V5_Synchronous_motion(0x00);

}





/***********************
 * Yaw方向修正
 ***********************/

void Car_Forward_Yaw_Control(uint16_t speed,float correction)
{

    int16_t left_speed;
    int16_t right_speed;


    left_speed = speed - correction;
    right_speed = speed + correction;


    if(left_speed < 0)
        left_speed = 0;

    if(right_speed < 0)
        right_speed = 0;



    // 用位置模式小步前进
    Emm_V5_Pos_Control(
        1,
        0,
        left_speed,
        10,
        100,
        0,
        true
    );


    Emm_V5_Pos_Control(
        2,
        1,
        right_speed,
        10,
        100,
        0,
        true
    );


    Emm_V5_Pos_Control(
        3,
        1,
        left_speed,
        10,
        100,
        0,
        true
    );


    Emm_V5_Pos_Control(
        4,
        0,
        right_speed,
        10,
        100,
        0,
        true
    );


    Emm_V5_Synchronous_motion(0x00);

}


void Car_Forward_Yaw_mm(uint16_t speed,float distance_mm)
{
    uint32_t pulse_total;

    pulse_total = (uint32_t)(distance_mm * 13.10f);


    while(pulse_total > 0)
    {

        HWT101_Update();


        float yaw_now;
        float yaw_error;
        float correction;


        yaw_now = HWT101_GetYaw();


        yaw_error = yaw_target - yaw_now;


        correction = yaw_error * 3;


        Car_Forward_Yaw_Control(
            speed,
            correction
        );


        pulse_total -= 100;


        HAL_Delay(20);

    }


    Car_Stop();

}