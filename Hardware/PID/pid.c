#include "control.h"

int16_t myabs(int a)
{ 		   
		a=a>=0?a:(-a);
	  return a;									//原temp
}

float pwm_control(float pwm)	//限幅
{
		pwm=pwm>3599?3599:(pwm<-3599?-3599:pwm);
    return pwm;
}

float p_Err , p_last_err , p_pwm , p_p = 0.175, p_d=0 , ERR_turn;
//转向环，
//40速度,0.198
int turn_pid(int now_position,float tar_position)//当前脉冲，目标脉冲
{
    p_Err = tar_position - now_position;//目标脉冲-现在脉冲=误差脉冲
	
    p_pwm = p_p * p_Err +p_d * (p_Err - p_last_err);
	
    p_last_err=p_Err;
    return p_pwm;
}


//增量式速度环
//先加i消除误差，再加p消除静态误差
int ControlVelocity=0, Last_bias=0, Bias=0;
int FeedbackControl(int TargetVelocity, int CurrentVelocity,PID_Init* pid)
{	
	pid->bias = TargetVelocity - CurrentVelocity; //求速度偏差
		
	ControlVelocity += pid-> Kp *  (pid->bias - pid -> Last_bias)+pid->Ki * pid->bias; 
		//增量式PI控制器                                                          
		//Velcity_Kp*(Bias-Last_bias) 作用为限制加速度	                                                                 
		//Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
		
	pid->Last_bias = pid->bias;	
	
	ControlVelocity = pwm_control(ControlVelocity);//限幅
	
	return ControlVelocity; //返回速度控制值
}


//位置式pid位置控制
//左
float p_Err=0,p_last_err=0,Integral=0,p_pwm=0,pid_p=0,pid_i=0,pid_d=0;

float p_pid(int16_t now_position,float tar_position)//积累脉冲(现在脉冲个数)，目标脉冲
{
    now_position=myabs(now_position);//转成正数
    p_Err=tar_position-now_position;//目标脉冲-现在脉冲=误差脉冲
		Integral+=p_Err;
		if(Integral> 970) Integral= 970;	//积分限幅
	  if(Integral<-970) Integral=-970;	//积分限幅
	//kp参数如果过大，设置电机转一圈就会过冲在回位，调到回位误差很小，这时候就可以调kd压制过冲
    p_pwm=pid_p*p_Err+pid_i*Integral+pid_d*(p_Err-p_last_err);	
//    p_pwm=pwm_control(p_pwm);
    p_last_err=p_Err;
    return p_pwm;
}

//位置式pid位置控制 
//右
float p_Err1=0,p_last_err1=0,Integral1=0,p_pwm1=0,pid_p1=0,pid_i1=0,pid_d1=0;

float p_pidR(int16_t now_position1,float tar_position1)//积累脉冲(现在脉冲个数)，目标脉冲
{
    now_position1=myabs(now_position1);//转成正数
    p_Err1=tar_position1-now_position1;//目标脉冲-现在脉冲=误差脉冲
		Integral1+=p_Err1;
		if(Integral1> 970) Integral1= 970;	//积分限幅
	  if(Integral1<-970) Integral1=-970;	//积分限幅
	//kp参数如果过大，设置电机转一圈就会过冲在回位，调到回位误差很小，这时候就可以调kd压制过冲
    p_pwm1=pid_p1*p_Err1+pid_i1*Integral1+pid_d1*(p_Err1-p_last_err1);
//  p_pwm1=pwm_control(p_pwm1);
    p_last_err1=p_Err1;
    return p_pwm1;
}

/**************************************************************************
函数功能：速度(PWM)限幅
入口参数：PWM_P:位置环输出的PWM值 TargetVelocity:目标速度(速度限制值)
返回  值：无
**************************************************************************/
int   TargetVelocity=0;//此处可调速度
int Velocity_Restrict(int PWM_P, int TargetVelocity)
{
    if     (PWM_P>+TargetVelocity*38) PWM_P=+TargetVelocity*38;
	  else if(PWM_P<-TargetVelocity*38) PWM_P=-TargetVelocity*38;
	  else PWM_P=PWM_P;
	  return PWM_P;
}
/****
2cm-400cm
距离控制


****/
int Length_Kp=0;
int  HC_SR04_Control(int16_t Target_Length,int16_t Actual_Length)
{
	int Err,PWM_out;
	Err=Target_Length-Actual_Length;
	PWM_out=Length_Kp*Err;
	return PWM_out;
}

