#include "zf_driver_gpio.h"
#include "zf_driver_timer.h"

#define BUZZER_PIN   C0

void buzzer_init(void)
{
    timer_init(TIM_5, TIMER_US);
    timer_clock_enable(TIM_5);
    timer_start(TIM_5);
    
    gpio_init(BUZZER_PIN, GPO, 0, GPO_AF_PUSH_PULL);
    buzzer_off();
}

void buzzer_on(void)
{
    gpio_set_level(BUZZER_PIN, 1);
}

void buzzer_off(void)
{
    gpio_set_level(BUZZER_PIN, 0);  // 低电平关闭（根据硬件可改为1）
}

