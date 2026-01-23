#include "mm32_device.h"                // Device header
#include "zf_driver_pwm.h"
#include "zf_device_mpu6050.h"
#include "zf_driver_encoder.h"
#include <math.h>


float AngleAcc,AngleGyro;
float Angle;
float Alpha=0.001;
float data_acc_x;
float data_acc_y;
float data_acc_z;
float data_gyro_x;
float data_gyro_y;
float data_gyro_z;
float actualAngle,actualSpeed,actualTurn;
float kp1=0,ki1=0,kd1=0;
float kp2=0,ki2=0,kd2=0;
float kp3=0,ki3=0,kd3=0;
float Error0,Error1,ErrorInt,out;
int16_t leftPWM,rightPWM;
int16_t AvePWM,DifPWM;
float leftSpeed,rightSpeed;
float AveSpeed,DifSpeed;
float Error01,Error11,ErrorInt1,out1;
float Error02,Error12,ErrorInt2,out2;
float Target;





void ztjs()
{
	mpu6050_get_gyro();
	mpu6050_get_acc();
	data_acc_x=mpu6050_acc_transition(mpu6050_acc_x);
	data_acc_y=mpu6050_acc_transition(mpu6050_acc_y);
	data_acc_z=mpu6050_acc_transition(mpu6050_acc_z);
	data_gyro_x = mpu6050_gyro_transition(mpu6050_gyro_x);
	data_gyro_y = mpu6050_gyro_transition(mpu6050_gyro_y);
	data_gyro_z = mpu6050_gyro_transition(mpu6050_gyro_z);
	AngleAcc = -atan2(data_acc_x,data_acc_z)/3.14159*180;
	AngleGyro = Angle + data_gyro_y/32768 * 2000*0.001;
	Angle = Alpha *AngleAcc +(1-Alpha)* AngleGyro;
}
void PID()
{
	int16_t count0=0,count1=0;
	count0++;
	count1++;
	if (count0>10)
	{
		count0=0;
		leftSpeed=encoder_get_count(TIM3_ENCODER)/11.0/0.01/4.4;
		rightSpeed=encoder_get_count(TIM4_ENCODER)/11.0/0.01/4.4;
		AveSpeed=(leftSpeed+rightSpeed)/2.0;
		DifSpeed=leftSpeed-rightSpeed;
		
		actualSpeed = AveSpeed;
		Error11=Error01;
		Error01=0- actualSpeed;
		ErrorInt1+=Error01;
		Target=kp2*Error01+ki2*ErrorInt1+kd2*(Error01-Error11);
		
	}
	if (count1>10)
	{
		count1=0;
		
		actualTurn = DifSpeed;
		Error12=Error02;
		Error02=0- actualSpeed;
		ErrorInt2+=Error02;
		DifPWM=kp3*Error02+ki3*ErrorInt2+kd3*(Error02-Error12);
		
	}
	//要在按键按下调至模式1时清零全部
	actualAngle = Angle;
	Error1=Error0;
	Error0=Target - actualAngle;
	ErrorInt+=Error0;
	out=kp1*Error0+ki1*ErrorInt+kd1*(Error0-Error1);
	AvePWM=out;
	leftPWM=AvePWM+DifPWM/2;
	rightPWM=AvePWM-DifPWM/2;
	if (leftPWM>100) leftPWM=100;else if (leftPWM<-100) leftPWM=-100;
	if (rightPWM>100) rightPWM=100;else if (rightPWM<-100) rightPWM=-100;
	pwm_set_duty(TIM5_PWM_CH2_A1,leftPWM);
	pwm_set_duty(TIM5_PWM_CH4_A3,rightPWM);
	
}
//使用时在主函数加上
//pit_us_init(TIM1_PIT, 1000);



