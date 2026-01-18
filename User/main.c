#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Delay.h"
#include "AD.h"
#include "menu.h"
#include "Key.h"
#include "Master.h"
#include "MPU6050.h"
#include "MyI2C.h"


uint16_t Light,Omega,Temp;
uint8_t AD_channel;
int16_t AX, AY, AZ, GX, GY, GZ;
uint8_t rx_data[3];


// 在main.c或其他地方添加以下代码

volatile uint32_t system_tick_ms = 0;



// 初始化SysTick定时器
void SysTick_Init(void)
{
    // 配置SysTick每1ms中断一次（假设系统时钟72MHz）
    // SysTick_Config函数是CMSIS标准函数
    if(SysTick_Config(SystemCoreClock / 1000))
    {
        // 初始化失败
        while(1);
    }
}

int main()
{
	OLED_Init();
	AD_Init();
	Key_Init();
	SPI1_Master_Init();
	MPU6050_Init();
	// 初始化SysTick定时器
    SysTick_Init();
    
    // 初始化I2C和MPU6050
    MyI2C_Init();
    MPU6050_Init();
    MPU6050_Attitude_Init();
    
    // 校准传感器（放置水平静止）
    MPU6050_Calibrate(500); // 采样500次
	
	
	while (1)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		// 周期性更新姿态解算
        MPU6050_Update_Attitude();
        
        // 获取姿态角
        float roll, pitch, yaw;
        MPU6050_Get_Attitude(&roll, &pitch, &yaw);
		uint8_t tx_data[]={roll,pitch,yaw};
		SPI1_Master_Transmit(tx_data, rx_data, 3);
		
	}

}



