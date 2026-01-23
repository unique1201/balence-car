#include "mm32_device.h"                // Device header
#include "zf_device_mpu6050.h"
#include <math.h>
#include "pid.h"
#define pi 3.1415926f


float AngleAcc,AngleGyro;
float Angle;
float Alpha=0.01;
float data_acc_x;
float data_acc_y;
float data_acc_z;
float data_gyro_x;
float data_gyro_y;
float data_gyro_z;

uint8_t runflag=1;

int16_t LeftPWM,RightPWM;
int16_t AvePWM,DifPWM;

PID_t AnglePID={
	.kp=0;
	.ki=0;
	.kd=0;
	
	.outMax=100;
	.outmin=-100;

};

void ztjs()
{
	static uint16_t Count0;
	
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
	{
		TIM1->SR &= ~TIM_SR_UIF;
		Count0++;
		if(Count0>=10)
		{
			Count0=0;
			mpu6050_get_gyro();
			mpu6050_get_acc();
	
			data_acc_x=mpu6050_acc_transition(mpu6050_acc_x);
			data_acc_y=mpu6050_acc_transition(mpu6050_acc_y);
			data_acc_z=mpu6050_acc_transition(mpu6050_acc_z);
	
			data_gyro_x = mpu6050_gyro_transition(mpu6050_gyro_x);
			data_gyro_y = mpu6050_gyro_transition(mpu6050_gyro_y);
		`	data_gyro_z = mpu6050_gyro_transition(mpu6050_gyro_z);
	
			AngleAcc = -atan2(data_acc_x,data_acc_z)/pi*180;
		`	AngleGyro = Angle + data_gyro_y/32768 * 2000*0.001;
			Angle = Alpha *AngleAcc +(1-Alpha)* AngleGyro;
			
			if(Angle>50||Angle<-50) runflag=0;
			if(runflag)
			{
				AnglePID.Actual=Angle;
				PID_Update(&AnglePID);
				AvePWM=AnglePID.Out;
				
				LeftPWM=AvePWM+DifPWM/2;
				RightPWM=AvePWM-DifPWM/2；
				
				if(LeftPWM>100) {LeftPWM=100;} else if(LeftPWM<-100) {LeftPWM=-100;}
				if(RightPWM>100) {RightPWM=100;} else if(RightPWM<-100) {RightPWM=-100;}
			}
			else
			{
				Motor_SetSpeed1(LeftPWM);
				Motor_SetSpeed2(RightPWM);
			}
			
			
		}
	

	}
	
}
//使用时在主函数加上
//pit_us_init(TIM1_PIT, 100);



