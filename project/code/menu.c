#include "menu.h"
#include <stdint.h>
#include "zf_device_oled.h"
#include "zf_device_key.h"
#include "IMU.h"
#include "zf_device_bluetooth_hc04.h"
#include "path_record.h"

//变量
extern float kp1,ki1,kd1;
extern float kp2,ki2,kd2;
extern float kp3,ki3,kd3;
extern bluetooth_hc04_joystick_string_data_t string_data;
extern float actualspeed;





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
    kp1+=0.1;
	oled_show_float(0, 8, kp1, 3, 2);
}

void Ki1(void)
{
    ki1+=0.1;
	oled_show_float(0, 8, ki1, 3, 2);
}

void Kd1(void)
{
    kd1+=0.1;
	oled_show_float(0, 8, kd1, 3, 2);
}

void Kp2(void)
{
    kp2+=0.1;
	oled_show_float(0, 8, kp2, 3, 2);
}

void Ki2(void)
{
    ki2+=0.1;
	oled_show_float(0, 8, ki2, 3, 2);
}

void Kd2(void)
{
    kd2+=0.1;
	oled_show_float(0, 8, kd2, 3, 2);
}

void Kp3(void)
{
    kp3+=0.1;
	oled_show_float(0, 8, kp3, 3, 2);
}

void Ki3(void)
{
    ki3+=0.1;
	oled_show_float(0, 8, ki3, 3, 2);
}

void Kd3(void)
{
    kd3+=0.1;
	oled_show_float(0, 8, kd3, 3, 2);
}

void M1(void)
{
    PID(0,0);
}

void M2(void)
{
		
}

void M3(void)
{
    
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
