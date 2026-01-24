#include "zf_driver_gpio.h"

// 定义LED引脚
#define LED_PIN     B0

void led_init(void)
{
    // 配置LED引脚为推挽输出，初始输出低电平（关闭LED）
    gpio_init(LED_PIN, GPO, 0, GPO_AF_PUSH_PULL);
    led_off();
}

void led_on(gpio_pin_enum led_pin)
{
    gpio_set_level(led_pin, 1);  // 输出高电平点亮LED（根据硬件可改为0）
}

void led_off(gpio_pin_enum led_pin)
{
    gpio_set_level(led_pin, 0);  // 输出低电平关闭LED（根据硬件可改为1）
}
