#include "zf_common_headfile.h"
#include "IMU.h"
#include "encoder.h"
#include "motor.h"
#include "path_record.h"
#include "menu.h"
#include "trace.h"

bluetooth_hc04_joystick_string_data_t string_data;


int main(void)
{
    clock_init(SYSTEM_CLOCK_120M);// 初始化芯片时钟 工作频率为 120MHz
    debug_init();                 // 初始化默认 Debug UART
//	extern float Angle;

    key_init(10);
	oled_init();
//	mpu6050_init();
//	bluetooth_hc04_init();
//	
//	encoder_init();
//	pit_us_init(TIM1_PIT, 1000);
//	path_init();
//	pwm_init(TIM5_PWM_CH2_A1, 1,0);
//    pwm_init(TIM5_PWM_CH4_A3, 1,0);
//    gpio_init(A0, GPO, GPIO_HIGH, GPO_PUSH_PULL);
//    gpio_init(A2, GPO, GPIO_HIGH, GPO_PUSH_PULL);

//	uint8_t a=0;
	
    while(1)
    {
//		bluetooth_hc04_printf("[plot,%f]",Angle);
		key_scanner();
		system_delay_ms(10);
		Main_Menu();

//		if (key_get_state(KEY_1))
//			a+=1;
//		oled_show_int(80, 0, a, 3);
//		oled_show_int(80, a, a, 3);
    }
}
