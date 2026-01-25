#include "menu.h"
#include <stdint.h>
#include "zf_device_oled.h"
#include "zf_device_key.h"
#include "IMU.h"

//变量
extern float kp1,ki1,kd1;
extern float kp2,ki2,kd2;
extern float kp3,ki3,kd3;

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

void M2(void)
{
    
}

void M3(void)
{
    
}

void M4(void)
{
    
}

void M5(void)
{
    
}
