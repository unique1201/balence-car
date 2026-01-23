#include "zf_common_headfile.h"
#include "mm32_device.h"                // Device header
#include "zf_driver_delay.h"
#include "zf_device_oled.h"
#include "zf_driver_timer.h"
#include "zf_device_key.h"
#include "zf_device_mpu6050.h"
#include "zf_device_encoder.h"
#include "zf_device_bluetooth_ch9141.h"
#include <math.h>
#include "IMU.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
	oled_init();
	mpu6050_init();
	bluetooth_ch9141_init();

    while(1)
    {
    }
}
// **************************** 代码区域 ****************************
