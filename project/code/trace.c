#include "mm32_device.h"                // Device header

#include "trace.h"
#include "IMU.h"

int lap = 0;

uint8_t last_gray_status = 0;  // 上一次灰度传感器状态
#define JUNCTION_THRESHOLD 0x7F // 黑白衔接判定阈值（可根据实际调试）


void gray_sensor_init(void)
{
    for (gray_channel_enum ch = GRAY_CHANNEL_0; ch < 8; ch++)
    {
        gpio_init(ch, GPI, 0, GPI_PULL_UP);		//上拉，0黑1白
    }
}

uint8 gray_sensor_read_channel(gray_channel_enum channel)
{
    // 调用GPIO驱动的gpio_get_level读取引脚电平
    return gpio_get_level(channel);
}

uint16 gray_sensor_read_all(void)
{
    uint16 gray_status = 0;
    
    for (gray_channel_enum ch = GRAY_CHANNEL_0; ch < 8; ch++)
    {
        if (gray_sensor_read_channel(ch))
        {
            gray_status |= (0x01 << (ch - GRAY_CHANNEL_0));
        }
    }
    
    return gray_status;
}

void trace(void)
{
	if(!gpio_get_level(GRAY_CHANNEL_0)||!gpio_get_level(GRAY_CHANNEL_1)||!gpio_get_level(GRAY_CHANNEL_2)) PID(50,-100);
	else if(!gpio_get_level(GRAY_CHANNEL_5)||!gpio_get_level(GRAY_CHANNEL_6||!gpio_get_level(GRAY_CHANNEL_7)) PID(50,100);
	else if(!gpio_get_level(GRAY_CHANNEL_3)&&gpio_get_level(GRAY_CHANNEL_4)) PID(50,50);
	else if(!gpio_get_level(GRAY_CHANNEL_4)&&gpio_get_level(GRAY_CHANNEL_3)) PID(50,-50);
	else if((!gpio_get_level(GRAY_CHANNEL_3)&&!gpio_get_level(GRAY_CHANNEL_4))||(gpio_get_level(GRAY_CHANNEL_3)&&gpio_get_level(GRAY_CHANNEL_4))) PID(100,0); 
}

void countlaps(void)
{
	if(gpio_get_level(GRAY_CHANNEL_3)&&gpio_get_level(GRAY_CHANNEL_4))
	{
		system_delay_ms (780);
		if(gpio_get_level(GRAY_CHANNEL_3)&&gpio_get_level(GRAY_CHANNEL_4)) lap++;
		if(lap==8) 
		{
			if(gpio_get_level(GRAY_CHANNEL_3)&&gpio_get_level(GRAY_CHANNEL_4)) PID(0,0);
		}
	}
	if(gpio_get_level(GRAY_CHANNEL_3)&&gpio_get_level(GRAY_CHANNEL_4))
	{
	led_on();
	buzzer_on();
	system_delay_ms (100);
	led_off();
	buzzer_off();
	}
	if(!gpio_get_level(GRAY_CHANNEL_3)||!gpio_get_level(GRAY_CHANNEL_4))
	{
	led_on();
	buzzer_on();
	system_delay_ms (200);
	led_off();
	buzzer_off();
	}
}

// 新增：黑白跑道衔接处检测函数
uint8_t detect_junction(void)
{
	uint16_t current_gray = gray_sensor_read_all();
	uint8_t is_junction = 0;
	
	// 判定逻辑：灰度状态跳变（全白→有黑 或 全黑→有白），且不是噪声
	if(fabs(current_gray - last_gray_status) > JUNCTION_THRESHOLD)
	{
		is_junction = 1;
	}
	
	// 更新上一次状态
	last_gray_status = current_gray;
	return is_junction;
}
