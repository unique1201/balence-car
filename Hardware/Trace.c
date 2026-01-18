#include "Trace.h"
#include "stm32f10x.h"                  // Device header


// 初始化循迹传感器
void TrackingSensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // 配置传感器引脚为输入模式
    GPIO_InitStructure.GPIO_Pin = TRACK_LEFT_PIN | TRACK_LEFT_MID_PIN | 
                                 TRACK_RIGHT_MID_PIN | TRACK_RIGHT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRACK_GPIO_PORT, &GPIO_InitStructure);
}

// 简单延时函数
void TrackingSensor_Delay(uint32_t count)
{
    while(count--);
}

// 读取所有传感器数据
SensorData TrackingSensor_ReadAll(void)
{
    SensorData data;
    
    // 读取传感器状态
    data.left = GPIO_ReadInputDataBit(TRACK_GPIO_PORT, TRACK_LEFT_PIN);
    data.left_mid = GPIO_ReadInputDataBit(TRACK_GPIO_PORT, TRACK_LEFT_MID_PIN);
    data.right_mid = GPIO_ReadInputDataBit(TRACK_GPIO_PORT, TRACK_RIGHT_MID_PIN);
    data.right = GPIO_ReadInputDataBit(TRACK_GPIO_PORT, TRACK_RIGHT_PIN);
    
    return data;
}

// 获取传感器状态的二进制编码（用于调试）
uint8_t TrackingSensor_GetBinaryCode(void)
{
    SensorData sensor = TrackingSensor_ReadAll();
    uint8_t code = 0;
    
    code |= (sensor.left << 3);
    code |= (sensor.left_mid << 2);
    code |= (sensor.right_mid << 1);
    code |= sensor.right;
    
    return code;
}

// 根据传感器数据判断循迹状态
TrackState TrackingSensor_GetState(void)
{
    SensorData sensor = TrackingSensor_ReadAll();
    uint8_t sensor_code = TrackingSensor_GetBinaryCode();
    
    // 十字路口检测（所有传感器都检测到黑线）
    if (sensor.left == SENSOR_BLACK && sensor.left_mid == SENSOR_BLACK && 
        sensor.right_mid == SENSOR_BLACK && sensor.right == SENSOR_BLACK) {
        return TRACK_CROSS_ROAD;
    }
    
    // 直行状态 - 中间两个传感器检测到黑线
    if ((sensor.left_mid == SENSOR_BLACK || sensor.right_mid == SENSOR_BLACK) && 
        sensor.left == SENSOR_WHITE && sensor.right == SENSOR_WHITE) {
        return TRACK_FORWARD;
    }
    
    // 左转慢 - 左中传感器检测到黑线
    if (sensor.left_mid == SENSOR_BLACK && sensor.right_mid == SENSOR_WHITE) {
        return TRACK_TURN_LEFT_SLOW;
    }
    
    // 左转快 - 左边传感器检测到黑线
    if (sensor.left == SENSOR_BLACK) {
        return TRACK_TURN_LEFT_FAST;
    }
    
    // 右转慢 - 右中传感器检测到黑线
    if (sensor.right_mid == SENSOR_BLACK && sensor.left_mid == SENSOR_WHITE) {
        return TRACK_TURN_RIGHT_SLOW;
    }
    
    // 右转快 - 右边传感器检测到黑线
    if (sensor.right == SENSOR_BLACK) {
        return TRACK_TURN_RIGHT_FAST;
    }
    
    // 轻微偏左 - 只有左中检测到
    if (sensor.left_mid == SENSOR_BLACK && sensor.right_mid == SENSOR_WHITE && 
        sensor.left == SENSOR_WHITE) {
        return TRACK_TURN_LEFT_SLOW;
    }
    
    // 轻微偏右 - 只有右中检测到
    if (sensor.right_mid == SENSOR_BLACK && sensor.left_mid == SENSOR_WHITE && 
        sensor.right == SENSOR_WHITE) {
        return TRACK_TURN_RIGHT_SLOW;
    }
    
    // 丢失路线 - 所有传感器都检测到白线
    if (sensor.left == SENSOR_WHITE && sensor.left_mid == SENSOR_WHITE && 
        sensor.right_mid == SENSOR_WHITE && sensor.right == SENSOR_WHITE) {
        return TRACK_LOST;
    }
    
    return TRACK_FORWARD;  // 默认直行
}
