//#include "mm32_device.h"                // Device header

#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "zf_device_oled.h"
#include <math.h>
#include "IMU.h"

#include "trace.h"
uint8_t lap=0;

// 定义控制引脚（请根据你的硬件连接修改）
#define GROUP1_EN_PIN    D0    // 第一组传感器使能引脚
#define GROUP2_EN_PIN    D1    // 第二组传感器使能引脚
#define EN_PIN           D2    // 模块总使能引脚
#define AD0_PIN          D3    // 通道选择引脚
#define AD1_PIN          D4    // 通道选择引脚
#define AD2_PIN          D5    // 通道选择引脚
#define OUT_PIN          D6    // 传感器输出引脚

#define X8 (1<<7) 
#define X7 (1<<6) 
#define X6 (1<<5) 
#define X5 (1<<4)
#define X4 (1<<3) 
#define X3 (1<<2) 
#define X2 (1<<1) 
#define X1 (1<<0)

#define DEFAULT_TRACE_SPEED 50	//默认速度
uint8_t last_gray_status = 0;  // 上一次灰度传感器状态
#define JUNCTION_THRESHOLD 0x7F // 黑白衔接判定阈值（可根据实际调试）

// 初始化控制引脚
void gray_sensor_init(void)
{
    // 初始化组使能引脚为推挽输出，默认低电平关闭
    gpio_init(GROUP1_EN_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(GROUP2_EN_PIN, GPO, 0, GPO_PUSH_PULL);
    
    // 初始化通道选择引脚为推挽输出，默认低电平
    gpio_init(EN_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(AD0_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(AD1_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(AD2_PIN, GPO, 0, GPO_PUSH_PULL);
    
    // 初始化输出引脚为上拉输入
    gpio_init(OUT_PIN, GPI, 0, GPI_PULL_UP);
}

void select_channel(uint8_t ch)
{
    gpio_set_level(AD0_PIN, (ch & 0x01) ? 1 : 0);
    gpio_set_level(AD1_PIN, (ch & 0x02) ? 1 : 0);
    gpio_set_level(AD2_PIN, (ch & 0x04) ? 1 : 0);
}

// 读取一组（4路）传感器
void read_group(uint8_t group, uint8_t *data)
{
    if (group == 1)
    {
        gpio_set_level(GROUP1_EN_PIN, 1);
        gpio_set_level(GROUP2_EN_PIN, 0);
    }
    else
    {
        gpio_set_level(GROUP1_EN_PIN, 0);
        gpio_set_level(GROUP2_EN_PIN, 1);
    }
    for (volatile uint32_t i = 0; i < 100; i++);

    // 读取4个通道
    for (uint8_t ch = 0; ch < 4; ch++)
    {
        select_channel(ch);
        for (volatile uint32_t i = 0; i < 50; i++); // 等待切换稳定
        data[ch] = gpio_get_level(OUT_PIN);
    }
}

uint8_t gray_sensor_read_all(void)
{
    uint8_t ch1_4[4], ch5_8[4]; // 原始通道：ch1_4=CH1-CH4，ch5_8=CH5-CH8
    uint8_t all_data = 0;

    read_group(1, ch1_4); // 读原始CH1-CH4 → 物理X4、X3、X2、X1
    read_group(2, ch5_8); // 读原始CH5-CH8 → 物理X5、X6、X7、X8

    all_data |= (ch5_8[3] << 7); // CH8 → 物理X8 → bit7（最左）
    all_data |= (ch5_8[2] << 6); // CH7 → 物理X7 → bit6
    all_data |= (ch5_8[1] << 5); // CH6 → 物理X6 → bit5
    all_data |= (ch5_8[0] << 4); // CH5 → 物理X5 → bit4
    all_data |= (ch1_4[3] << 3); // CH4 → 物理X4 → bit3
    all_data |= (ch1_4[2] << 2); // CH3 → 物理X3 → bit2
    all_data |= (ch1_4[1] << 1); // CH2 → 物理X2 → bit1
    all_data |= (ch1_4[0] << 0); // CH1 → 物理X1 → bit0（最右）

    return all_data;
}

void trace(int16_t trace_speed, uint8_t data)
{
    if((data&(X5|X4))== 0) PID(trace_speed,0);
    else if((data & X6) == 0) PID(trace_speed-30,-50);
    else if((data & X7) == 0) PID(trace_speed-30,-100);
    else if((data & X2) == 0) PID(trace_speed-30,100);
    else if((data & X3) == 0) PID(trace_speed-30,50);
	else if((data&(X7|X6|X5|X4|X3|X2))== 1) PID(trace_speed,0);
}

void trace_default(void)
{
    uint8_t data = gray_sensor_read_all();
    trace(DEFAULT_TRACE_SPEED, data);
}

void countlaps(uint8_t data)
{
	if((data&(X5|X4))== 1)
	{
		system_delay_ms (780);
		if((data&(X5|X4))== 1) lap++;
		if(lap==8) 
		{
			if((data&(X5|X4))== 1) PID(0,0);
		}
	}
}

void prompts(uint8_t data)
{
	if((data&(X5|X4))== 1)
	{
	oled_show_string(0, 1, "LEDon");
	system_delay_ms (100);
	oled_show_string(0, 1, "LEDoff");
	}
	if((data&(X5|X4))== 0)
	{
	oled_show_string(0, 1, "LEDon");
	system_delay_ms (200);
	oled_show_string(0, 1, "LEDoff");
	}
}

extern uint8_t is_turning;

uint8_t detect_junction(void)
{
	uint16_t current_gray = gray_sensor_read_all();
	uint8_t is_junction = 0;
	
	// 判定逻辑：灰度状态跳变（全白→有黑 或 全黑→有白），且不是噪声
	if(abs(current_gray - last_gray_status) > JUNCTION_THRESHOLD)
	{
		is_junction = 1;
		if(!is_turning) trigger_turn();
	}
	
	// 更新上一次状态
	last_gray_status = current_gray;
	return is_junction;
}
