#ifndef __control_H
#define __control_H

//全局变量的声明或定义
typedef struct PID_Init
{
	float Kp;
	float Ki;
	float kd;
	int Last_bias;
	int bias;
}PID_Init;

//函数声明
void control_Init(void);


#include "stm32f10x.h"                  // Device header

//头文件声明
#include "Key.h"
#include "OLED.h"
#include "PWM.h"
#include "Servo.h"
#include "Timer.h"
#include "pid.h"
#include "uart.h"
#include "Serial.h"
#include "APP.h"
#include "sys.h"

//标准库头文件声明

#include <math.h>
#include <stdio.h>
#include <string.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#endif
