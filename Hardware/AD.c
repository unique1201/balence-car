#include "stm32f10x.h"

/**
  * @brief  ADC初始化
  * @param  无
  * @retval 无
  */
void AD_Init(void)
{
    // 1. 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    // 2. 配置ADC时钟（最大14MHz）
    // 如果系统时钟是72MHz，6分频得到12MHz
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    // 3. 配置GPIO为模拟输入
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;      // 模拟输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 模拟输入下速度设置不影响，但最好保留
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 4. 初始化ADC
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;          // 独立模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;               // 单通道模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;         // 单次转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;      // 右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;                     // 转换1个通道
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // 5. 使能ADC
    ADC_Cmd(ADC1, ENABLE);
    
    // 6. ADC校准
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1) == SET);  // 等待复位校准完成
    
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) == SET);       // 等待校准完成
    
    // 7. 使能ADC的软件触发
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
  * @brief  获取ADC转换值
  * @param  AD_channel: ADC通道号
  *         PA5 -> ADC1通道5
  *         PA6 -> ADC1通道6
  *         PA7 -> ADC1通道7
  * @retval ADC转换值（0-4095）
  */
uint16_t AD_GetValue(uint8_t AD_channel)
{
    // 1. 配置通道和采样时间
    ADC_RegularChannelConfig(ADC1, AD_channel, 1, ADC_SampleTime_55Cycles5);
    
    // 2. 启动转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    // 3. 等待转换完成
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    
    // 4. 清除标志位
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    
    // 5. 返回转换结果
    return ADC_GetConversionValue(ADC1);
}











