#ifndef __TRACE_H
#define __TRACE_H

#include "zf_driver_gpio.h"

typedef enum
{
    GRAY_CHANNEL_0 = D0,
    GRAY_CHANNEL_1 = D1,
    GRAY_CHANNEL_2 = D2,
    GRAY_CHANNEL_3 = D3,
    GRAY_CHANNEL_4 = D4,
	GRAY_CHANNEL_5 = D5,
	GRAY_CHANNEL_6 = D6,
	GRAY_CHANNEL_7 = D7,		//看具体引脚
} gray_channel_enum;

void gray_sensor_init(void);

uint8 gray_sensor_read_channel(gray_channel_enum channel);

uint16 gray_sensor_read_all(void);

void trace(int16_t speed);

void trace_default(void);

void countlaps(void);
void prompts();

uint8_t detect_junction(void);

#endif // __TRACE_H
