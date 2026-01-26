#ifndef __TRACE_H
#define __TRACE_H

#include "mm32_device.h"                // Device header
#include "zf_driver_gpio.h"

int lap=0;

void gray_sensor_init(void);

void select_channel(uint8_t ch);

void read_group(uint8_t group, uint8_t *data);

uint8_t gray_sensor_read_all(void);

void trace(int16_t trace_speed, uint8_t data);

void trace_default(void);

void countlaps(uint8_t data);

void prompts(uint8_t data);

uint8_t detect_junction(void);

#endif // __TRACE_H
