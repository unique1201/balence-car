#include "menu.h"
#include <stdint.h>
#include "zf_device_oled.h"
#include "zf_device_key.h"

//变量
float kp=0.0f,ki=0.0f,kd=0.0f;

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
		if (strcmp(option[i].Name, "kp") == 0) {
			int32_t temp=kp*100;                              
			oled_show_int(i, 8, temp/100, 3);
			oled_show_string(i, 11, ".");
            oled_show_int(i, 12, temp%100, 2);
        } 
		else if (strcmp(option[i].Name, "ki") == 0) {
            int32_t temp=ki*100;                             
			oled_show_int(i, 8, temp/100, 3);
			oled_show_string(i, 11, ".");
            oled_show_int(i, 12, temp%100, 2);
        }
		else if (strcmp(option[i].Name, "kd") == 0) {
            int32_t temp=kd*100;                              
			oled_show_int(i, 8, temp/100, 3);
			oled_show_string(i, 11, ".");
            oled_show_int(i, 12, temp%100, 2);
        }
	}
}

void Menu_loop(struct option_class* option)
{
	int8_t totalItems = 0;        //总项数
    int8_t displayItems = 0;      //屏幕上显示的项目数量
    int8_t currentLine = 0;       //光标所在行  
	
	while (option[totalItems].Name[0] != ' ' && option[totalItems].Name[0] != '\0') {totalItems++;}             
	//计算总项数，在这里是到空格停止（不不包括空格项）
	
	
    display_menu(option, displayItems, currentLine);
	// 初始显示
	
	while (1)
	{
		
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
        {"kp", NULL},
        {"ki", NULL},
        {"kd", NULL},
		{" ", NULL}
    };
    Menu_loop(option);
}

void Mode(void)
{
    struct option_class option[]=
    {
        {"1", NULL},
        {"2", NULL},
        {"3", NULL},
		{"4", NULL},
		{" ", NULL}
    };
    Menu_loop(option);
}

void Blue(void)
{
    struct option_class option[]=
    {
        {"Blue", NULL},
		{" ", NULL}
    };
    Menu_loop(option);
}
