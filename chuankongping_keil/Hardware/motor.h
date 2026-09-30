#ifndef _MOTOR_H
#define _MOTOR_H

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "sensor.h"
#include "tim.h"
#include <stdio.h>
#include <stdarg.h>

void Right_Ctrl(int pwm_val);


void Left_Ctrl(int pwm_val);


#endif
