#ifndef __TRACKING_SENSOR_H
#define __TRACKING_SENSOR_H

#include "stm32f10x.h"

// 传感器引脚定义（根据实际连接修改）
#define TRACK_LEFT_PIN      GPIO_Pin_9
#define TRACK_LEFT_MID_PIN  GPIO_Pin_10
#define TRACK_RIGHT_MID_PIN GPIO_Pin_11
#define TRACK_RIGHT_PIN     GPIO_Pin_12
#define TRACK_GPIO_PORT     GPIOA

// 传感器状态
#define SENSOR_WHITE    0   // 检测到白色
#define SENSOR_BLACK    1   // 检测到黑色

// 循迹状态枚举
typedef enum {
    TRACK_STOP = 0,         // 停止
    TRACK_FORWARD,          // 直行
    TRACK_TURN_LEFT_SLOW,   // 左转慢
    TRACK_TURN_LEFT_FAST,   // 左转快
    TRACK_TURN_RIGHT_SLOW,  // 右转慢
    TRACK_TURN_RIGHT_FAST,  // 右转快
    TRACK_BACKWARD,         // 后退
    TRACK_LOST,             // 丢失路线
    TRACK_CROSS_ROAD        // 十字路口
} TrackState;

// 传感器数据结构
typedef struct {
    uint8_t left;
    uint8_t left_mid;
    uint8_t right_mid;
    uint8_t right;
} SensorData;

// 函数声明
void TrackingSensor_Init(void);
SensorData TrackingSensor_ReadAll(void);
TrackState TrackingSensor_GetState(void);
uint8_t TrackingSensor_GetBinaryCode(void);
void TrackingSensor_Delay(uint32_t count);

#endif
