#ifndef __LED_H__
#define __LED_H__

#include "zf_driver_gpio.h"

#define LED_PIN     B0

// 函数声明
void led_init(void);
void led_on(gpio_pin_enum led_pin);
void led_off(gpio_pin_enum led_pin);

#endif
