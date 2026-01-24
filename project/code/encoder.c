#include "encoder.h"
#include "zf_driver_encoder.h"
#include "zf_driver_timer.h"

// 速度计算变量
static uint32_t last_time = 0;
static int16 last_encoder1 = 0;
static int16 last_encoder2 = 0;
static float encoder1_speed = 0.0f;
static float encoder2_speed = 0.0f;

void encoder_init()
{
	encoder_dir_init(TIM3_ENCODER, TIM3_ENCODER_CH1_B4, TIM3_ENCODER_CH2_B5); //编码器A
	encoder_dir_init(TIM4_ENCODER, TIM4_ENCODER_CH1_B6, TIM4_ENCODER_CH2_B7); //编码器B
	
	encoder_clear_count(TIM3_ENCODER);
    encoder_clear_count(TIM4_ENCODER);
}

signed short A_Speed(void)
{
	uint16_t current_time=timer_get(TIM_3);
	uint16 time_diff = current_time - last_time;
	
	if(time_diff > 0) {
        int16 encoder1_diff = encoder_get_count(TIM3_ENCODER) - last_encoder1;
        encoder1_speed = (float)encoder1_diff * 60000.0f / (time_diff * 370.0f); // 1360RPM电机，减速比假设为370:1
        
        // 更新上一次值
        last_time = current_time;
        last_encoder1 = encoder_get_count(TIM3_ENCODER);
    } 
	return encoder1_speed;
}

signed short B_Speed(void)
{
	uint16_t current_time=timer_get(TIM_4);
	uint16 time_diff = current_time - last_time;
	
	if(time_diff > 0) {
        int16 encoder2_diff = encoder_get_count(TIM4_ENCODER) - last_encoder2;
        encoder2_speed = (float)encoder2_diff * 60000.0f / (time_diff * 370.0f); // 1360RPM电机，减速比假设为370:1
        
        // 更新上一次值
        last_time = current_time;
        last_encoder2 = encoder_get_count(TIM4_ENCODER);
    } 
	return encoder2_speed;
}
