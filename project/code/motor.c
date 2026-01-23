#include "mm32_device.h"                // Device header

#include "zf_driver_pwm.h"

/**
  * 函    数：直流电机初始化
  * 参    数：无
  * 返 回 值：无
  * 注    释：MM32F103兼容STM32F103的GPIO库函数，逻辑完全不变
  */
void Motor_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_13);
	
	pwm_init();
}

/**
  * 函    数：直流电机设置速度（电机1）
  * 参    数：Speed 速度范围：-100~100
  * 返 回 值：无
  */
void Motor_SetSpeed1(int8_t Speed)
{
	if (Speed >= 0)						
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);	
		pwm_set_duty (pwm_channel_enum pin, const uint32 duty)
	}
	else									
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);	
		GPIO_SetBits(GPIOB, GPIO_Pin_13);	
		pwm_set_duty (pwm_channel_enum pin,speed);			//引脚和具体速度还没写
	}
}

/**
  * 函    数：直流电机设置速度（电机2）
  * 参    数：Speed 速度范围：-100~100
  * 返 回 值：无
  */
void Motor_SetSpeed2(int8_t Speed)
{
	if (Speed >= 0)						
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_14);	
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);	
		PWM_SetCompare4(Speed);
	}
	else									
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);	
		GPIO_SetBits(GPIOB, GPIO_Pin_15);	
		pwm_set_duty (pwm_channel_enum pin,speed)			
	}
}