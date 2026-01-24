#include "trace.h"
#include "zf_driver_pwm.h"
#include <stdio.h>
#include "bsp_PID_motor.h"

// 直线行驶到指定距离
static void Car_Line_Run(uint32_t target_distance);

// 半弧线行驶指定角度
static void Car_Curve_Run(int16_t target_angle);

// 停止电机
static void Car_Stop(void);

// 编码器距离清零
static void Encoder_Clear(void);

// IMU角度清零
static void IMU_Angle_Clear(void);

// 当前行驶状态
static uint8_t g_car_state = 0;

void trace_init(void)
{
    gpio_init(BEEP_PIN, GPO, 0, GPO_PUSH_PULL);		 // 初始化声光引脚（输出模式）
    gpio_init(LED_PIN, GPO, 0, GPO_PUSH_PULL);
    
    // 初始化PID参数（复用已有电机PID）
    PID_Param_Init();
    // 设置直线/弧线行驶的PID参数（可根据实际调试）
    PID_Set_Motor_Parm(0, 1.5, 0, 0.01);  // 左电机
    PID_Set_Motor_Parm(1, 1.5, 0, 0.01);  // 右电机
    
    // 初始化编码器（复用已有配置）
    encoder_init(TIM3_ENCODER);
    encoder_init(TIM4_ENCODER);
    
    // 初始化IMU（复用已有ztjs函数）
    ztjs();
    
    // 初始状态清零
    g_car_state = 0;
    Car_Stop();
}

//声光提示
void Car_Prompt(void)
{
    gpio_set_level(BEEP_PIN, 1);
    gpio_set_level(LED_PIN, 1);
    // 简易延时
    for(uint32_t i=0; i<PROMPT_TIME*1000; i++);
    gpio_set_level(BEEP_PIN, 0);
    gpio_set_level(LED_PIN, 0);
}

//target_distance: 目标距离(mm)

static void Car_Line_Run(uint32_t target_distance)
{
    // 清零编码器和PID
    Encoder_Clear();
    PID_Clear_Motor(MAX_MOTOR);
    
    // 设置目标速度
    PID_Set_Motor_Target(0, SPEED_LINE);
    PID_Set_Motor_Target(1, SPEED_LINE);
    
    // 循环行驶直到到达目标距离
    while(1)
    {
        ztjs();  // 更新IMU数据（顺带更新速度）
        PID(NULL);  // 计算PID输出PID_Calc_Motor
        
        // 计算已行驶距离（取左右轮平均值）
        float left_dist = encoder_get_count(TIM3_ENCODER)/11.0/4.4;
        float right_dist = encoder_get_count(TIM4_ENCODER)/11.0/4.4;
        float current_dist = (left_dist + right_dist) / 2.0;
        
        // 到达目标距离（含误差）
        if(current_dist >= (target_distance - DISTANCE_ERROR))
        {
            Car_Stop();
            break;
        }
    }
    Car_Prompt();
}

//半弧线行驶

static void Car_Curve_Run(int16_t target_angle)
{
    // 清零IMU角度和PID
    IMU_Angle_Clear();
    PID_Clear_Motor(MAX_MOTOR);
    
    // 设置弧线行驶速度（左右轮差速）
    if(target_angle > 0)
    {
        PID_Set_Motor_Target(0, SPEED_CURVE - 50);  // 左轮慢
        PID_Set_Motor_Target(1, SPEED_CURVE + 50);  // 右轮快
    }
    else
    {
        PID_Set_Motor_Target(0, SPEED_CURVE + 50);  // 左轮快
        PID_Set_Motor_Target(1, SPEED_CURVE - 50);  // 右轮慢
    }
    
    // 循环行驶直到到达目标角度
    while(1)
    {
        ztjs();
        PID_Calc_Motor(NULL);
        
        // 角度误差判断
        if((target_angle > 0 && Angle >= (target_angle - ANGLE_ERROR)) ||
           (target_angle < 0 && Angle <= (target_angle + ANGLE_ERROR)))
        {
            Car_Stop();
            break;
        }
    }
    Car_Prompt();
}

static void Car_Stop(void)
{
    pwm_set_duty(TIM5_PWM_CH2_A1, 0);
    pwm_set_duty(TIM5_PWM_CH4_A3, 0);
    PID_Set_Motor_Target(MAX_MOTOR, 0);
}

static void Encoder_Clear(void)
{
    encoder_clear_count(TIM3_ENCODER);
    encoder_clear_count(TIM4_ENCODER);
}

static void IMU_Angle_Clear(void)
{
    Angle = 0;
    AngleAcc = 0;
    AngleGyro = 0;
}

void Car_Run(void)
{
    // 步骤1：A→B 直线行驶
    g_car_state = 1;
    Car_Line_Run(DISTANCE_A_TO_B);
    
    // 步骤2：B→C 半弧线行驶（180度）
    g_car_state = 2;
    Car_Curve_Run(ANGLE_B_TO_C);
    
    // 步骤3：C→D 直线行驶
    g_car_state = 3;
    Car_Line_Run(DISTANCE_C_TO_D);
    
    // 步骤4：D→A 半弧线行驶（180度）
    g_car_state = 4;
    Car_Curve_Run(-ANGLE_D_TO_A);  // 反向180度
    
    // 回到A点，最终停车
    g_car_state = 0;
    Car_Stop();
    Car_Prompt();  // 最后一次提示（回到起点）
}
