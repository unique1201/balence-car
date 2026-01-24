#include "motor.h"
#include "zf_driver_timer.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"


void motor_init(void)
{
	timer_init(TIM_5, TIMER_US);  //初始化TIM_5，us为单位
	timer_clock_enable (TIM_5);   //使能时钟
	timer_start(TIM_5);           //开启时钟

    // 方向控制引脚配置为推挽输出
    gpio_init(A0, GPO, 0, GPO_AF_PUSH_PULL);  // 默认输出低电平
    gpio_init(A2, GPO, 0, GPO_AF_PUSH_PULL);  // 默认输出低电平
	
	// PWM 引脚配置
	afio_init(A1, GPO, (gpio_af_enum)2, GPO_AF_PUSH_PULL);
    afio_init(A3, GPO, (gpio_af_enum)2, GPO_AF_PUSH_PULL);
	
	pwm_init(TIM5_PWM_CH2_A1, 24000, 0);    //系统时钟120MHz PWM频率 = 120MHz / 24000 = 5kHz
	pwm_init(TIM5_PWM_CH4_A3, 24000, 0);   
	
	gpio_set_level(A0, 0);  // 设置电机A初始方向
	gpio_set_level(A2, 0);  // 设置电机B初始方向
	motor_stop();
}

//电机A的方向与速度
void motor_set_A(unsigned char dir,unsigned short pwm_duty)
{
	gpio_set_level(A0, dir);
	if (pwm_duty>24000) pwm_duty=24000;   //限制PWM
	pwm_set_duty(TIM5_PWM_CH2_A1, pwm_duty);
}

//电机B的方向与速度
void motor_set_B(unsigned char dir,unsigned short pwm_duty)
{
	gpio_set_level(A2 ,dir);
	if (pwm_duty>24000) pwm_duty=24000;   //限制PWM
	pwm_set_duty(TIM5_PWM_CH4_A3, pwm_duty);
}

void motor_stop(void)
{
	// 停止电机 A
    gpio_set_level(A0, 0);
    pwm_set_duty(TIM5_PWM_CH2_A1, 0);
    
    // 停止电机 B
    gpio_set_level(A2, 0);
    pwm_set_duty(TIM5_PWM_CH4_A3, 0);
}
