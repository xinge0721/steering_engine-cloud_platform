#include "control.h"

int16_t myabs(int a)
{ 		   
		a=a>=0?a:(-a);
	  return a;									//ԭtemp
}

float pwm_control(float pwm)	//�޷�
{
		pwm=pwm>3599?3599:(pwm<-3599?-3599:pwm);
    return pwm;
}

float p_Err , p_last_err , p_pwm , p_p = 0.175, p_d=0 , ERR_turn;
//ת�򻷣�
//40�ٶ�,0.198
int turn_pid(int now_position,float tar_position)//��ǰ���壬Ŀ������
{
    p_Err = tar_position - now_position;//Ŀ������-��������=�������
	
    p_pwm = p_p * p_Err +p_d * (p_Err - p_last_err);
	
    p_last_err=p_Err;
    return p_pwm;
}


//����ʽ�ٶȻ�
//�ȼ�i�������ټ�p������̬���
int ControlVelocity=0, Last_bias=0, Bias=0;
int FeedbackControl(int TargetVelocity, int CurrentVelocity,PID_Init* pid)
{	
	pid->bias = TargetVelocity - CurrentVelocity; //���ٶ�ƫ��
		
	ControlVelocity += pid-> Kp *  (pid->bias - pid -> Last_bias)+pid->Ki * pid->bias; 
		//����ʽPI������                                                          
		//Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�	                                                                 
		//Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
		
	pid->Last_bias = pid->bias;	
	
	ControlVelocity = pwm_control(ControlVelocity);//�޷�
	
	return ControlVelocity; //�����ٶȿ���ֵ
}


//λ��ʽpidλ�ÿ���
//��
float p_Err=0,p_last_err=0,Integral=0,p_pwm=0,pid_p=0,pid_i=0,pid_d=0;

float p_pid(int16_t now_position,float tar_position)//��������(�����������)��Ŀ������
{
    now_position=myabs(now_position);//ת������
    p_Err=tar_position-now_position;//Ŀ������-��������=�������
		Integral+=p_Err;
		if(Integral> 970) Integral= 970;	//�����޷�
	  if(Integral<-970) Integral=-970;	//�����޷�
	//kp��������������õ��תһȦ�ͻ�����ڻ�λ��������λ����С����ʱ��Ϳ��Ե�kdѹ�ƹ���
    p_pwm=pid_p*p_Err+pid_i*Integral+pid_d*(p_Err-p_last_err);	
//    p_pwm=pwm_control(p_pwm);
    p_last_err=p_Err;
    return p_pwm;
}

//λ��ʽpidλ�ÿ��� 
//��
float p_Err1=0,p_last_err1=0,Integral1=0,p_pwm1=0,pid_p1=0,pid_i1=0,pid_d1=0;

float p_pidR(int16_t now_position1,float tar_position1)//��������(�����������)��Ŀ������
{
    now_position1=myabs(now_position1);//ת������
    p_Err1=tar_position1-now_position1;//Ŀ������-��������=�������
		Integral1+=p_Err1;
		if(Integral1> 970) Integral1= 970;	//�����޷�
	  if(Integral1<-970) Integral1=-970;	//�����޷�
	//kp��������������õ��תһȦ�ͻ�����ڻ�λ��������λ����С����ʱ��Ϳ��Ե�kdѹ�ƹ���
    p_pwm1=pid_p1*p_Err1+pid_i1*Integral1+pid_d1*(p_Err1-p_last_err1);
//  p_pwm1=pwm_control(p_pwm1);
    p_last_err1=p_Err1;
    return p_pwm1;
}

/**************************************************************************
�������ܣ��ٶ�(PWM)�޷�
��ڲ�����PWM_P:λ�û������PWMֵ TargetVelocity:Ŀ���ٶ�(�ٶ�����ֵ)
����  ֵ����
**************************************************************************/
int   TargetVelocity=0;//�˴��ɵ��ٶ�
int Velocity_Restrict(int PWM_P, int TargetVelocity)
{
    if     (PWM_P>+TargetVelocity*38) PWM_P=+TargetVelocity*38;
	  else if(PWM_P<-TargetVelocity*38) PWM_P=-TargetVelocity*38;
	  else PWM_P=PWM_P;
	  return PWM_P;
}
/****
2cm-400cm
�������


****/
int Length_Kp=0;
int  HC_SR04_Control(int16_t Target_Length,int16_t Actual_Length)
{
	int Err,PWM_out;
	Err=Target_Length-Actual_Length;
	PWM_out=Length_Kp*Err;
	return PWM_out;
}

