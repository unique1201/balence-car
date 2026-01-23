#include "mm32_device.h"                // Device header
#include "zf_device_mpu6050.h"
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
//使用时在主函数加上
//pit_us_init(TIM1_PIT, 100);



