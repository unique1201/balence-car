//#include "mm32_device.h"                // Device header

#include "trace.h"
#include "IMU.h"

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
	else if(!gpio_get_level(GRAY_CHANNEL_5)||!gpio_get_level(GRAY_CHANNEL_6||!gpio_get_level(GRAY_CHANNEL_7))) PID(50,100);
	else if(!gpio_get_level(GRAY_CHANNEL_3)&&gpio_get_level(GRAY_CHANNEL_4)) PID(50,50);
	else if(!gpio_get_level(GRAY_CHANNEL_4)&&gpio_get_level(GRAY_CHANNEL_3)) PID(50,-50);
	else if((!gpio_get_level(GRAY_CHANNEL_3)&&!gpio_get_level(GRAY_CHANNEL_4))||(gpio_get_level(GRAY_CHANNEL_3)&&gpio_get_level(GRAY_CHANNEL_4))) PID(100,0); 
}
