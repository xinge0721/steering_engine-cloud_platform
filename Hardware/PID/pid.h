#ifndef __PID_H
#define __PID_H
#include "sys.h"                

int turn_pid(int now_position,float tar_position);
float pwm_control(float pwm);

int FeedbackControl(int TargetVelocity, int CurrentVelocity,PID_Init* pid);

float p_pid(int16_t now_position,float tar_position);
float p_pidR(int16_t now_position1,float tar_position1);

int Velocity_Restrict(int PWM_P, int TargetVelocity);
int HC_SR04_Control(int16_t Target_Length,int16_t Actual_Length);
extern int   TargetVelocity,RControlVelocity,ControlVelocity,Last_bias,Last_bias1;

extern float p_p,p_d;//转向参数

extern float pid_p,pid_i,pid_d;//左轮位置环
extern float pid_p1,pid_i1,pid_d1;//右轮位置环
#endif
