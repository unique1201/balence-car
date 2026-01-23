#ifndef __TRACE_H
#define __TRACE_H

#include "zf_driver_gpio.h"
#include "zf_driver_encoder.h"
#include "IMU.h"
#include "bsp_PID_motor.h"
#include <stdint.h>

// 声光提示引脚（根据实际硬件修改）
#define BEEP_PIN    D6      // 蜂鸣器引脚
#define LED_PIN     D7      // LED指示灯引脚

// 声光提示时长(ms)
#define PROMPT_TIME 500

// 直线距离(mm) - 需根据实际场地校准
#define DISTANCE_A_TO_B 1000    // A→B直线距离
#define DISTANCE_C_TO_D 1000    // C→D直线距离
// 半弧线角度(度)
#define ANGLE_B_TO_C 180        // B→C半弧线角度
#define ANGLE_D_TO_A 180        // D→A半弧线角度
// 行驶速度(mm/s) - 确保总时间≤30秒
#define SPEED_LINE 300          // 直线行驶速度
#define SPEED_CURVE 150         // 弧线行驶速度
// 误差允许范围
#define DISTANCE_ERROR 20       // 距离误差±20mm
#define ANGLE_ERROR 5           // 角度误差±5度

/********************* 函数声明 *********************/
// 模式2初始化（初始化声光引脚、PID、编码器等）
void trace_init(void);

// 模式2主运行函数（调用后执行完整路径）
void Car_Run(void);

// 声光提示函数
void Car_Prompt(void);

#endif
