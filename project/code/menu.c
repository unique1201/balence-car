#include "menu.h"
#include <stdint.h>
#include "zf_device_oled.h"
#include "zf_device_key.h"
#include "IMU.h"
#include "zf_device_bluetooth_hc04.h"
#include "zf_driver_delay.h"
#include "path_record.h"
#include "trace.h"

//变量
extern float kp1,ki1,kd1;
extern float kp2,ki2,kd2;
extern float kp3,ki3,kd3;
extern bluetooth_hc04_joystick_string_data_t string_data;
extern float actualspeed;

//发车模式2/3变量
static int16_t trace_speed=50;  // 初始速度
#define SPEED_STEP 5              // 调速步长
#define SPEED_MAX 100             // 最大速度
#define SPEED_MIN 0               // 最小速度

//发车模式3变量
static uint8_t junction_hint_flag = 0;  // 交界提示标记（0-未提示，1-已提示）
#define HINT_DELAY 300                // 声光提示时长(ms)




void bluetooth_hc04_string_callback(bluetooth_hc04_joystick_string_data_t *data)
{
    if (data->valid)
	{
        printf("Received string: [\"%s\",\"%s\",\"%s\",\"%s\",\"%s\"]\r\n",
               data->joystick, data->data1, data->data2, 
               data->data3, data->data4);
		int32 speed = bluetooth_hc04_string_to_int(data->data2);
        int32 Difspeed = bluetooth_hc04_string_to_int(data->data3);
        PID(speed,-Difspeed);   
    }
}

void display_menu(struct option_class* option, int displayItems, int currentLine)
{
	oled_show_string(1, 1, ">");                 //初始指向第一行
	for (int8_t i = 0; i < displayItems; i++) {
        oled_show_string(i, 2, option[i].Name);
    }
}

void display_value(struct option_class* option, int displayItems)
{
	for (int8_t i = 0; i < displayItems; i++)
	{
		//PID参数显示
		if (strcmp(option[i].Name, "kp1") == 0) {
			oled_show_float(i, 8, kp1, 3, 2);
        } 
		else if (strcmp(option[i].Name, "ki1") == 0) {
            oled_show_float(i, 8, ki1, 3, 2);
        }
		else if (strcmp(option[i].Name, "kd1") == 0) {
            oled_show_float(i, 8, kd1, 3, 2);
        }
		else if (strcmp(option[i].Name, "kp2") == 0) {
			oled_show_float(i, 8, kp2, 3, 2);
        } 
		else if (strcmp(option[i].Name, "ki2") == 0) {
            oled_show_float(i, 8, ki2, 3, 2);
        }
		else if (strcmp(option[i].Name, "kd2") == 0) {
            oled_show_float(i, 8, kd2, 3, 2);
        }
		else if (strcmp(option[i].Name, "kp3") == 0) {
			oled_show_float(i, 8, kp3, 3, 2);
        } 
		else if (strcmp(option[i].Name, "ki3") == 0) {
            oled_show_float(i, 8, ki3, 3, 2);
        }
		else if (strcmp(option[i].Name, "kd3") == 0) {
            oled_show_float(i, 8, kd3, 3, 2);
        }
	}
}

void Menu_loop(struct option_class* option)
{
	int8_t totalItems = 0;        //总项数
    int8_t displayItems = 0;      //屏幕上显示的项目数量
    int8_t currentLine = 0;       //光标所在行  
	
	while (option[totalItems].Name[0] != ' ' && option[totalItems].Name[0] != '\0') {totalItems++;}             
	//计算总项数，在这里是到空格停止（不包括空格项）
	
	
    display_menu(option, displayItems, currentLine);
	// 初始显示
	
	while (1)
	{
		if (key_get_state(KEY_1))//上移
		{
			oled_show_string(currentLine, 1, " ");
			currentLine--;
			if (currentLine < 0) {
				currentLine = displayItems - 1;
			}
			oled_show_string(currentLine, 1, ">");
		}   
		if (key_get_state(KEY_2))   //下移
		{
			oled_show_string(currentLine, 1, " ");
			currentLine++;
			if (currentLine > displayItems - 1) {
				currentLine = 0;
			}
			oled_show_string(currentLine, 1, ">");
		} 
		if (key_get_state(KEY_3))   //确认
		{	
			oled_clear();
			if (currentLine < displayItems && option[currentLine].func != NULL) {
				option[currentLine].func();// 从子菜单返回后刷新显示
				display_menu(option, displayItems, currentLine);
			}			
		}
		if (key_get_state(KEY_4)){oled_clear();return;}   //返回
	}
	//这里写调参啥的，也可以用函数让这里简洁一点
}

void Main_Menu(void)
{
    struct option_class option[]=
    {
        {"tiaocan", tiaocan},
        {"Mode", Mode},
		{"Blue", Blue},
        {" ", NULL}
    };
    Menu_loop(option);
}

void tiaocan(void)
{
    struct option_class option[]=
    {
        {"k1", K1},
        {"k2", K2},
        {"k3", K3},
		{" ", NULL}
    };
    Menu_loop(option);
}

void Mode(void)
{
    struct option_class option[]=
    {
        {"M1", M1},
        {"M2", M2},
        {"M3", M3},
		{"M4", M4},
		{" ", NULL}
    };
    Menu_loop(option);
}

void Blue(void)
{
    struct option_class option[]=
    {
        {"Blue", M5},
		{" ", NULL}
    };
    Menu_loop(option);
}

void K1(void)
{
    struct option_class option[]=
    {
        {"kp1", Kp1},
        {"ki1", Ki1},
        {"kd1", Kd1},
		{" ", NULL}
    };
    Menu_loop(option);
}

void K2(void)
{
    struct option_class option[]=
    {
        {"kp2", Kp2},
        {"ki2", Ki2},
        {"kd2", Kd2},
		{" ", NULL}
    };
    Menu_loop(option);
}

void K3(void)
{
    struct option_class option[]=
    {
        {"kp3", Kp3},
        {"ki3", Ki3},
        {"kd3", Kd3},
		{" ", NULL}
    };
    Menu_loop(option);
}

void Kp1(void)
{
	while (1)	
	{
		if (key_get_state(KEY_1))
		{
			kp1+=0.1;
			oled_show_float(0, 8, kp1, 3, 2);
		}
		else if (key_get_state(KEY_2))
		{
			kp1-=0.1;
			if (kp1<0) kp1 = 0.0f;
			oled_show_float(0, 8, kp1, 3, 2);
		}
		else if (key_get_state(KEY_4))
		{
			return;
		}
	}
}
	

void Ki1(void)
{
    while (1)	
	{
		if (key_get_state(KEY_1))
		{
			ki1+=0.1;
			oled_show_float(0, 8, ki1, 3, 2);
		}
		else if (key_get_state(KEY_2))
		{
			ki1-=0.1;
			if (ki1<0) ki1 = 0.0f;
			oled_show_float(0, 8, ki1, 3, 2);
		}
		else if (key_get_state(KEY_4))
		{
			return;
		}
	}
}

void Kd1(void)
{
    while (1)	
	{
		if (key_get_state(KEY_1))
		{
			kd1+=0.1;
			oled_show_float(0, 8, kd1, 3, 2);
		}
		else if (key_get_state(KEY_2))
		{
			kd1-=0.1;
			if (kd1<0) kd1 = 0.0f;
			oled_show_float(0, 8, kd1, 3, 2);
		}
		else if (key_get_state(KEY_4))
		{
			return;
		}
	}
}

void Kp2(void)
{
    while (1)	
	{
		if (key_get_state(KEY_1))
		{
			kp2+=0.1;
			oled_show_float(0, 8, kp2, 3, 2);
		}
		else if (key_get_state(KEY_2))
		{
			kp2-=0.1;
			if (kp2<0) kp2 = 0.0f;
			oled_show_float(0, 8, kp2, 3, 2);
		}
		else if (key_get_state(KEY_4))
		{
			return;
		}
	}
}

void Ki2(void)
{
    while (1)	
	{
		if (key_get_state(KEY_1))
		{
			ki2+=0.1;
			oled_show_float(0, 8, ki2, 3, 2);
		}
		else if (key_get_state(KEY_2))
		{
			ki2-=0.1;
			if (ki2<0) ki2 = 0.0f;
			oled_show_float(0, 8, ki2, 3, 2);
		}
		else if (key_get_state(KEY_4))
		{
			return;
		}
	}
}

void Kd2(void)
{
    while (1)	
	{
		if (key_get_state(KEY_1))
		{
			kd2+=0.1;
			oled_show_float(0, 8, kd2, 3, 2);
		}
		else if (key_get_state(KEY_2))
		{
			kd2-=0.1;
			if (kd2<0) kd2 = 0.0f;
			oled_show_float(0, 8, kd2, 3, 2);
		}
		else if (key_get_state(KEY_4))
		{
			return;
		}
	}
}

void Kp3(void)
{
    while (1)	
	{
		if (key_get_state(KEY_1))
		{
			kp3+=0.1;
			oled_show_float(0, 8, kp3, 3, 2);
		}
		else if (key_get_state(KEY_2))
		{
			kp3-=0.1;
			if (kp3<0) kp3 = 0.0f;
			oled_show_float(0, 8, kp3, 3, 2);
		}
		else if (key_get_state(KEY_4))
		{
			return;
		}
	}
}

void Ki3(void)
{
    while (1)	
	{
		if (key_get_state(KEY_1))
		{
			ki3+=0.1;
			oled_show_float(0, 8, kp1, 3, 2);
		}
		else if (key_get_state(KEY_2))
		{
			ki3-=0.1;
			if (ki3<0) ki3 = 0.0f;
			oled_show_float(0, 8, ki3, 3, 2);
		}
		else if (key_get_state(KEY_4))
		{
			return;
		}
	}
}

void Kd3(void)
{
    while (1)	
	{
		if (key_get_state(KEY_1))
		{
			kd3+=0.1;
			oled_show_float(0, 8, kd3, 3, 2);
		}
		else if (key_get_state(KEY_2))
		{
			kd3-=0.1;
			if (kd3<0) kd3 = 0.0f;
			oled_show_float(0, 8, kd3, 3, 2);
		}
		else if (key_get_state(KEY_4))
		{
			return;
		}
	}
}

void M1(void)
{
    PID(0,0);
}

void M2(void)	//key1：加速，key2：减速，key3：start+暂停循迹，key4；退出当前菜单
{
	uint8_t is_running = 0;
    oled_clear();
    PID(0, 0);  // 强制小车停止
    oled_show_string(0, 1, "M2: Trace");
    oled_show_string(1, 1, "Speed: ");
    oled_show_int(1, 8, trace_speed, 3);  // 显示初始速度

    while (1)
    {
        if(key_get_state(KEY_3))
        {
            is_running = !is_running;
            if(is_running)
            {
                oled_show_string(2,1,"run");
            }
            else
            {
                PID(0,0);
                oled_show_string(2,1,"stop");
            }
            system_delay_ms(200);  //消抖
        }

        if(key_get_state(KEY_1))
        {
            trace_speed+=SPEED_STEP;
            if (trace_speed>SPEED_MAX) trace_speed=SPEED_MAX;
            oled_show_int(1,8,trace_speed,3);
            system_delay_ms(200);  //消抖
        }

        if(key_get_state(KEY_2))
        {
            trace_speed-=SPEED_STEP;
            if(trace_speed<SPEED_MIN) trace_speed=SPEED_MIN;
            oled_show_int(1,8,trace_speed,3);
            system_delay_ms(200);  // 消抖
        }

        if (key_get_state(KEY_4))
        {
            PID(0, 0);
            oled_clear();
            return;
        }

        if (is_running)
        {
            trace(trace_speed);
            ztjs();
        }
    }
}

void M3(void)
{
    uint8_t is_running = 0;
    oled_clear();
    PID(0, 0);
    oled_show_string(0, 1, "M3: Trace+Hint");
    oled_show_string(1, 1, "Speed: ");
    oled_show_int(1, 8, trace_speed, 3);
    oled_show_string(2, 1, "Lap: 0/8");

    uint8_t lap = 0;
	uint8_t turn_count = 0;
    uint8_t is_turning = 0;

    while (1)
    {
        if (lap >= 8)
        {
            PID(0, 0);
            oled_show_string(2, 1, "Lap: 8/8 STOP");
//            led_on();
//			buzzer_on();
			system_delay_ms(1000); // 停止提示，测试用，正式版可删
//            led_off();
//			buzzer_off();
            oled_clear();
            return;
        }
        if(key_get_state(KEY_3)) // 启停
        {
            is_running = !is_running;
            is_running ? oled_show_string(3,1,"run") : (PID(0,0), oled_show_string(3,1,"stop"));
            system_delay_ms(200);
        }
        if(key_get_state(KEY_1))
        {
            trace_speed=(trace_speed+SPEED_STEP)>SPEED_MAX?SPEED_MAX:(trace_speed+SPEED_STEP);
            oled_show_int(1,8,trace_speed,3);
            system_delay_ms(200);
        }
        if(key_get_state(KEY_2))
        {
            trace_speed=(trace_speed-SPEED_STEP)<SPEED_MIN?SPEED_MIN:(trace_speed-SPEED_STEP);
            oled_show_int(1,8,trace_speed,3);
            system_delay_ms(200);
        }
        if (key_get_state(KEY_4))
        {
            PID(0, 0);oled_clear();return;
        }

        if (is_running)
        {
            trace(trace_speed);
            ztjs();
            detect_junction();
            prompts();
            countlaps();
            oled_show_int(2, 5, lap, 1);
        }
    }
}

void M4(void)
{
    //		这是模式4的按键控制，需要再有一个按钮切换path的模式，一共4种。对应模式按下确认键开始对应功能。
	if (key_get_state(KEY_4))
	{
		extern PathState path_state;
		path_state = PATH_RECORDING;
	}
	if (key_get_state(KEY_3))
	{
        switch (path_get_state())
		{
            case PATH_IDLE:
				path_record_start();
				break;
            case PATH_RECORDING:
                path_record_stop();
                path_save_to_flash();
                break;
			case PATH_LOADED:
			case PATH_SAVED:
				path_replay_start();
				break;
			case PATH_REPLAYING:
				path_replay_stop();
				break;
	  }
  }
}

void M5(void)
{
    bluetooth_hc04_get_all_strings(string_data.joystick,string_data.data1,string_data.data2,string_data.data3,string_data.data4);
	bluetooth_hc04_string_callback(&string_data);
		
}
