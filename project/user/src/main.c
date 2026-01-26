#include "zf_common_headfile.h"
#include "IMU.h"
#include "encoder.h"
#include "motor.h"
#include "path_record.h"
#include "menu.h"

bluetooth_hc04_joystick_string_data_t string_data;


int main(void)
{
    clock_init(SYSTEM_CLOCK_120M);// 初始化芯片时钟 工作频率为 120MHz
    debug_init();                 // 初始化默认 Debug UART
	extern float Angle;

    key_init(1);
	oled_init();
	mpu6050_init();
	bluetooth_hc04_init();
	motor_init();
	encoder_init();
	pit_us_init(TIM1_PIT, 1000);
	path_init();

    while(1)
    {
		bluetooth_hc04_printf("[plot,%f]",Angle);
		
    }
}
