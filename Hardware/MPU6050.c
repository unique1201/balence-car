#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "MPU6050_Reg.h"
#include <math.h>

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址
#define PI 3.14159265358979323846f

// 姿态解算结构体
typedef struct {
    float roll;     // 横滚角（绕X轴）
    float pitch;    // 俯仰角（绕Y轴）
    float yaw;      // 偏航角（绕Z轴）
    
    float accRoll;  // 加速度计计算的横滚角
    float accPitch; // 加速度计计算的俯仰角
    
    float gyroRoll;  // 陀螺仪积分的横滚角
    float gyroPitch; // 陀螺仪积分的俯仰角
    float gyroYaw;   // 陀螺仪积分的偏航角
    
    float gyroOffsetX; // 陀螺仪X轴零点漂移
    float gyroOffsetY; // 陀螺仪Y轴零点漂移
    float gyroOffsetZ; // 陀螺仪Z轴零点漂移
    
    float accOffsetX;  // 加速度计X轴零点偏移
    float accOffsetY;  // 加速度计Y轴零点偏移
    float accOffsetZ;  // 加速度计Z轴零点偏移
    
    uint32_t lastUpdate; // 上次更新时间戳
    float dt;           // 时间间隔（秒）
    
    // 互补滤波系数
    float alpha;        // 陀螺仪权重
    float beta;         // 加速度计权重
} MPU6050_Attitude;

// 全局姿态数据
static MPU6050_Attitude attitude;

// 系统时间戳（需要外部实现）
extern volatile uint32_t system_tick_ms;

/**
  * 函    数：获取系统时间戳（毫秒）
  * 参    数：无
  * 返 回 值：当前时间戳（毫秒）
  */
static uint32_t Get_Tick(void)
{
    // 返回系统时间戳，假设已经有一个全局变量system_tick_ms
    // 你需要在其他地方维护这个变量（例如SysTick中断）
    return system_tick_ms;
}

// 如果没有math.h的支持，实现一些基本的数学函数
#ifdef NO_MATH_LIB
static float my_sqrtf(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f375a86 - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
    x = x * (1.5f - xhalf * x * x);
    x = x * (1.5f - xhalf * x * x);
    return 1.0f / x;
}

static float my_atan2f(float y, float x)
{
    float abs_y = y < 0 ? -y : y;
    float abs_x = x < 0 ? -x : x;
    float a, r, angle;
    
    if (abs_x > abs_y) {
        a = abs_y / abs_x;
        r = (0.1963f * a * a * a) - (0.9817f * a) + (1.5708f);
    } else {
        if (abs_y == 0.0f) return 0.0f;
        a = abs_x / abs_y;
        r = (0.1963f * a * a * a) - (0.9817f * a) + (1.5708f);
        r = 3.14159f - r;
    }
    
    if (x < 0) r = 3.14159f - r;
    if (y < 0) r = -r;
    
    return r;
}

static float my_asinf(float x)
{
    // 近似计算asin，仅适用于|x| < 1的情况
    if (x > 0.99f) x = 0.99f;
    if (x < -0.99f) x = -0.99f;
    
    float y = x;
    float y2 = y * y;
    float y3 = y * y2;
    float y5 = y3 * y2;
    float y7 = y5 * y2;
    
    return y + y3/6.0f + 3.0f*y5/40.0f + 5.0f*y7/112.0f;
}

#define sqrtf my_sqrtf
#define atan2f my_atan2f
#define asinf my_asinf
#endif

// 原函数保持不变
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(Data);				//发送要写入寄存器的数据
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_Stop();						//I2C终止
}
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答
	Data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
	MyI2C_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
	MyI2C_Stop();						//I2C终止
	
	return Data;
}
void MPU6050_Init(void)
{
	MyI2C_Init();									//先初始化底层的I2C
	
	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
}
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
}

/**
  * 函    数：姿态解算初始化
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_Attitude_Init(void)
{
    // 初始化姿态结构体
    attitude.roll = 0.0f;
    attitude.pitch = 0.0f;
    attitude.yaw = 0.0f;
    
    attitude.accRoll = 0.0f;
    attitude.accPitch = 0.0f;
    
    attitude.gyroRoll = 0.0f;
    attitude.gyroPitch = 0.0f;
    attitude.gyroYaw = 0.0f;
    
    // 初始化零点偏移（需要后续校准）
    attitude.gyroOffsetX = 0.0f;
    attitude.gyroOffsetY = 0.0f;
    attitude.gyroOffsetZ = 0.0f;
    
    attitude.accOffsetX = 0.0f;
    attitude.accOffsetY = 0.0f;
    attitude.accOffsetZ = 0.0f; // 稍后校准
    
    attitude.lastUpdate = Get_Tick();
    attitude.dt = 0.01f; // 默认10ms采样周期
    
    // 互补滤波系数（可调整）
    attitude.alpha = 0.98f; // 陀螺仪权重
    attitude.beta = 0.02f;  // 加速度计权重
}

/**
  * 函    数：校准传感器（需要在静止状态下调用）
  * 参    数：calibration_samples 校准采样次数
  * 返 回 值：无
  */
void MPU6050_Calibrate(uint16_t calibration_samples)
{
    int32_t gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
    int32_t accSumX = 0, accSumY = 0, accSumZ = 0;
    int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
    
    // 采集多组数据求平均值
    for(uint16_t i = 0; i < calibration_samples; i++)
    {
        MPU6050_GetData(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
        
        gyroSumX += gyroX;
        gyroSumY += gyroY;
        gyroSumZ += gyroZ;
        
        accSumX += accX;
        accSumY += accY;
        accSumZ += accZ;
        
        // 简单的延时等待下一次采样
        for(volatile uint32_t j = 0; j < 10000; j++);
    }
    
    // 计算平均值
    attitude.gyroOffsetX = (float)gyroSumX / calibration_samples;
    attitude.gyroOffsetY = (float)gyroSumY / calibration_samples;
    attitude.gyroOffsetZ = (float)gyroSumZ / calibration_samples;
    
    // 加速度计校准（假设传感器水平放置）
    attitude.accOffsetX = (float)accSumX / calibration_samples;
    attitude.accOffsetY = (float)accSumY / calibration_samples;
    attitude.accOffsetZ = (float)accSumZ / calibration_samples;
    
    // 如果需要，可以减去重力加速度（Z轴）
    // attitude.accOffsetZ -= 16384.0f; // 假设1g对应16384
}

/**
  * 函    数：从加速度计计算姿态角
  * 参    数：accX, accY, accZ 加速度计数据
  * 返 回 值：无
  */
static void Calculate_Acc_Angles(float accX, float accY, float accZ)
{
    // 去除偏移
    accX -= attitude.accOffsetX;
    accY -= attitude.accOffsetY;
    accZ -= attitude.accOffsetZ;
    
    // 计算加速度向量的模（用于归一化）
    float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
    
    if(norm > 0.01f) // 避免除以零
    {
        // 归一化
        accX /= norm;
        accY /= norm;
        accZ /= norm;
        
        // 计算俯仰角和横滚角
        attitude.accPitch = asinf(accX) * 180.0f / PI;    // 俯仰角（绕Y轴）
        attitude.accRoll = -atan2f(accY, accZ) * 180.0f / PI; // 横滚角（绕X轴）
    }
}

/**
  * 函    数：从陀螺仪计算姿态角（积分）
  * 参    数：gyroX, gyroY, gyroZ 陀螺仪数据（度/秒）
  * 返 回 值：无
  */
static void Calculate_Gyro_Angles(float gyroX, float gyroY, float gyroZ)
{
    // 去除零点漂移
    gyroX -= attitude.gyroOffsetX;
    gyroY -= attitude.gyroOffsetY;
    gyroZ -= attitude.gyroOffsetZ;
    
    // 转换到度/秒（根据MPU6050量程±2000°/s）
    // 32768对应2000°/s，所以每度/秒对应32768/2000=16.384
    gyroX = gyroX / 16.384f;
    gyroY = gyroY / 16.384f;
    gyroZ = gyroZ / 16.384f;
    
    // 积分得到角度（注意符号处理）
    attitude.gyroRoll -= gyroY * attitude.dt;   // 绕X轴旋转
    attitude.gyroPitch += gyroX * attitude.dt;  // 绕Y轴旋转
    attitude.gyroYaw += gyroZ * attitude.dt;    // 绕Z轴旋转
    
    // 限制角度范围
    if(attitude.gyroRoll > 180.0f) attitude.gyroRoll -= 360.0f;
    else if(attitude.gyroRoll < -180.0f) attitude.gyroRoll += 360.0f;
    
    if(attitude.gyroPitch > 180.0f) attitude.gyroPitch -= 360.0f;
    else if(attitude.gyroPitch < -180.0f) attitude.gyroPitch += 360.0f;
    
    if(attitude.gyroYaw > 180.0f) attitude.gyroYaw -= 360.0f;
    else if(attitude.gyroYaw < -180.0f) attitude.gyroYaw += 360.0f;
}

/**
  * 函    数：互补滤波融合姿态
  * 参    数：无
  * 返 回 值：无
  */
static void Complementary_Filter(void)
{
    // 互补滤波融合
    attitude.roll = attitude.alpha * (attitude.roll + attitude.gyroRoll) + 
                    attitude.beta * attitude.accRoll;
    
    attitude.pitch = attitude.alpha * (attitude.pitch + attitude.gyroPitch) + 
                     attitude.beta * attitude.accPitch;
    
    // 偏航角只能用陀螺仪（需要磁力计校正）
    attitude.yaw = attitude.gyroYaw;
    
    // 重置陀螺仪积分角度（因为已经融合到总角度中）
    attitude.gyroRoll = 0.0f;
    attitude.gyroPitch = 0.0f;
    
    // 限制最终角度范围
    if(attitude.roll > 180.0f) attitude.roll -= 360.0f;
    else if(attitude.roll < -180.0f) attitude.roll += 360.0f;
    
    if(attitude.pitch > 180.0f) attitude.pitch -= 360.0f;
    else if(attitude.pitch < -180.0f) attitude.pitch += 360.0f;
    
    if(attitude.yaw > 180.0f) attitude.yaw -= 360.0f;
    else if(attitude.yaw < -180.0f) attitude.yaw += 360.0f;
}

/**
  * 函    数：更新姿态解算（需要周期性调用）
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_Update_Attitude(void)
{
    int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
    
    // 读取原始数据
    MPU6050_GetData(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
    
    // 计算时间间隔
    uint32_t now = Get_Tick();
    attitude.dt = (float)(now - attitude.lastUpdate) / 1000.0f; // 转换为秒
    attitude.lastUpdate = now;
    
    // 限制dt范围，避免异常值
    if(attitude.dt > 0.1f) attitude.dt = 0.01f; // 最大100ms
    if(attitude.dt < 0.001f) attitude.dt = 0.001f; // 最小1ms
    
    // 计算加速度计角度
    Calculate_Acc_Angles((float)accX, (float)accY, (float)accZ);
    
    // 计算陀螺仪角度
    Calculate_Gyro_Angles((float)gyroX, (float)gyroY, (float)gyroZ);
    
    // 互补滤波融合
    Complementary_Filter();
}

/**
  * 函    数：获取姿态角
  * 参    数：roll, pitch, yaw 输出的姿态角（度）
  * 返 回 值：无
  * 注    意：如果不需要某个角度，可以传递0（空指针）
  */
void MPU6050_Get_Attitude(float *roll, float *pitch, float *yaw)
{
    // 使用条件判断来避免NULL指针访问
    if(roll != 0) *roll = attitude.roll;
    if(pitch != 0) *pitch = attitude.pitch;
    if(yaw != 0) *yaw = attitude.yaw;
}

/**
  * 函    数：设置互补滤波系数
  * 参    数：alpha 陀螺仪权重（通常0.95-0.99）
  *         beta  加速度计权重（通常0.01-0.05）
  * 返 回 值：无
  */
void MPU6050_Set_Filter_Coefficients(float alpha, float beta)
{
    // 确保权重和为1
    if((alpha + beta - 1.0f) < 0.001f && (alpha + beta - 1.0f) > -0.001f)
    {
        attitude.alpha = alpha;
        attitude.beta = beta;
    }
}

/**
  * 函    数：重置姿态角
  * 参    数：reset_yaw 是否重置偏航角
  * 返 回 值：无
  */
void MPU6050_Reset_Attitude(uint8_t reset_yaw)
{
    attitude.roll = 0.0f;
    attitude.pitch = 0.0f;
    if(reset_yaw) attitude.yaw = 0.0f;
    
    attitude.gyroRoll = 0.0f;
    attitude.gyroPitch = 0.0f;
    if(reset_yaw) attitude.gyroYaw = 0.0f;
}

/**
  * 函    数：获取校准状态
  * 参    数：offsets 存储偏移量的数组[6]
  * 返 回 值：无
  */
void MPU6050_Get_Calibration(float offsets[6])
{
    // 检查指针是否有效
    if(offsets != 0)
    {
        offsets[0] = attitude.gyroOffsetX;
        offsets[1] = attitude.gyroOffsetY;
        offsets[2] = attitude.gyroOffsetZ;
        offsets[3] = attitude.accOffsetX;
        offsets[4] = attitude.accOffsetY;
        offsets[5] = attitude.accOffsetZ;
    }
}

/**
  * 函    数：手动设置偏移量
  * 参    数：gyroX, gyroY, gyroZ 陀螺仪偏移量
  *         accX, accY, accZ 加速度计偏移量
  * 返 回 值：无
  */
void MPU6050_Set_Calibration(float gyroX, float gyroY, float gyroZ,
                             float accX, float accY, float accZ)
{
    attitude.gyroOffsetX = gyroX;
    attitude.gyroOffsetY = gyroY;
    attitude.gyroOffsetZ = gyroZ;
    
    attitude.accOffsetX = accX;
    attitude.accOffsetY = accY;
    attitude.accOffsetZ = accZ;
}
