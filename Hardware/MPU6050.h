#ifndef __MPU6050_H
#define __MPU6050_H

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_Set_Filter_Coefficients(float alpha, float beta);

void MPU6050_Get_Attitude(float *roll, float *pitch, float *yaw);

void MPU6050_Update_Attitude(void);

void Complementary_Filter(void);
void Calculate_Gyro_Angles(float gyroX, float gyroY, float gyroZ);
void Calculate_Acc_Angles(float accX, float accY, float accZ);
void MPU6050_Calibrate(uint16_t calibration_samples);
void MPU6050_Attitude_Init(void);



#endif
