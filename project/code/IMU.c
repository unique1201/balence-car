#include "mm32_device.h"                // Device header
#include "zf_driver_pwm.h"
#include "zf_device_mpu6050.h"
#include "zf_driver_encoder.h"
#include "path_record.h"
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


uint8_t turn_count=0;          // 衔接处遇到次数（起点算第1次）
uint8_t is_turning=0;          // 转向执行中标记（0：未转向，1：转向中）
float target_turn_angle=0.0f;  // 转向目标角度
#define TURN_ANGLE 40.0f         // 单次转向角度（40度）
#define TURN_THRESHOLD 0.5f      // 转向角度误差阈值（±0.5度视为到位）



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


void turn_in_place(float target_angle)	//新的
{
	is_turning=1;  //标记转向中
	target_turn_angle=target_angle;
	
	//角度闭环PID控制，直到角度误差小于阈值
	while(fabs(Angle-target_turn_angle)>TURN_THRESHOLD)
	{
		ztjs();//实时更新角度
		
		//角度环PID计算（仅控制转向，速度为0）
		actualAngle=Angle;
		Error1=Error0;
		Error0=target_turn_angle-actualAngle;
		ErrorInt+=Error0;
		
		// 限幅积分项，防止积分饱和
		if(ErrorInt>500) ErrorInt=500;
		if(ErrorInt<-500) ErrorInt=-500;
		
		out =kp1*Error0+ki1*ErrorInt+kd1*(Error0-Error1);
		
		// 原地转向：平均速度为0，转向差由PID输出决定
		AvePWM=0;
		DifPWM=(int16_t)out;
		
		// PWM限幅
		leftPWM=AvePWM+DifPWM/2;
		rightPWM=AvePWM-DifPWM/2;
		if(leftPWM>50) leftPWM=50;  // 转向时PWM适当降低，避免过冲
		if(leftPWM<-50) leftPWM=-50;
		if(rightPWM>50) rightPWM=50;
		if(rightPWM<-50) rightPWM=-50;
		
		// 输出PWM控制电机
		pwm_set_duty(TIM5_PWM_CH2_A1,leftPWM);
		pwm_set_duty(TIM5_PWM_CH4_A3,rightPWM);
	}
	
	// 转向到位，停止电机
	leftPWM=0;
	rightPWM=0;
	pwm_set_duty(TIM5_PWM_CH2_A1,leftPWM);
	pwm_set_duty(TIM5_PWM_CH4_A3,rightPWM);
	
	is_turning=0;  // 清除转向标记
	ErrorInt=0;    // 清零积分项
}


void PID(float Target2,float Target3)//2是速度环，设置目标速度。3是转向环，设置的是左右速度差。
{
	static float path_target_left = 0, path_target_right = 0;
    
    // 如果在路径复现模式，覆盖目标速度
    if (path_get_state() == PATH_REPLAYING) {
        // 使用路径目标速度
        Target2 = (path_target_left + path_target_right) / 2.0f;  // 平均速度
        Target3 = path_target_left - path_target_right;          // 速度差
    }
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
		Error01=Target2- actualSpeed;
		ErrorInt1+=Error01;
		Target=kp2*Error01+ki2*ErrorInt1+kd2*(Error01-Error11);
		
	}
	if (count1>10)
	{
		count1=0;
		
		actualTurn = DifSpeed;
		Error12=Error02;
		Error02=Target3- actualTurn;
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
	// 在路径复现时，更新路径目标
    if (path_get_state() == PATH_REPLAYING) {
        path_replay_step(&path_target_left, &path_target_right);
    }
	
}

uint8_t get_turn_count(void)
{
	return turn_count;
}

void trigger_turn(void)
{
	turn_count++;  //次数+1（起点算第1次）
	float current_angle=Angle;
	float target_angle;
	
	if(turn_count%2==1)
	{
		target_angle=current_angle+TURN_ANGLE; //右转40度
	}
	else
	{
		target_angle=current_angle-TURN_ANGLE; //左转40度
	}
	
	// 执行原地转向
	turn_in_place(target_angle);
}


//使用时在主函数加上
//pit_us_init(TIM1_PIT, 1000);



