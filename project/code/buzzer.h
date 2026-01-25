#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "zf_driver_gpio.h"

#define BUZZER_PIN   C0

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);

#endif
