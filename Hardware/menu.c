#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Key.h"
#include "AD.h"


uint8_t keynum,page=1,mode=1;
extern uint16_t Light,Omega,Temp;



void OLED_menu()
{
	OLED_ShowString(1,8,"+/-");
	if (page==1)
	{
		if (mode==0)
		{
			OLED_Clear();
			OLED_ShowString(1,8,"+/-");
			OLED_ShowString(1,1,">ADC");
			OLED_ShowString(2,1,"Store");
			OLED_ShowString(3,1,"IMU");
		}
		else if (mode==1)
		{
			OLED_Clear();
			OLED_ShowString(1,8,"+/-");
			OLED_ShowString(1,1,"ADC");
			OLED_ShowString(2,1,">Store");
			OLED_ShowString(3,1,"IMU");
		}
		else if (mode==2)
		{
			OLED_Clear();
			OLED_ShowString(1,8,"+/-");
			OLED_ShowString(1,1,"ADC");
			OLED_ShowString(2,1,"Store");
			OLED_ShowString(3,1,">IMU");
		}
	}
	else if (page==2)
	{
		OLED_Clear();
		OLED_ShowString(1,8,"+/-");
		OLED_ShowString(2,1,"POT  val");
		OLED_ShowString(3,1,"NTC  val");
		OLED_ShowString(4,1,"LDR  val");
		Omega=AD_GetValue(ADC_Channel_0);
		Temp=AD_GetValue(ADC_Channel_1);
		Light=AD_GetValue(ADC_Channel_2);
		OLED_ShowNum(2,9,Omega,4);
		OLED_ShowNum(3,9,Temp,4);
		OLED_ShowNum(4,9,Light,4);
	}
	else if (page==3)
	{
		OLED_Clear();
		OLED_ShowString(1,8,"+/-");
		OLED_ShowString(2,1,"Pitch");
		OLED_ShowString(3,1,"Roll");
		OLED_ShowString(4,1,"Yaw");

	}
}



void OLED_display()
{
	OLED_menu();
	keynum=Key_GetNum();
	if(keynum==1)//按'下'
	{
		if (page==1)
		{
			mode=(mode+1)%3;
			OLED_menu();
		}
	}
	else if (keynum==2)//按确认
	{
		if(page==1 && mode==0)
		{
			page=2;
			OLED_menu();
		}
		else if (page==1 && mode==1)
		{
			page=3;
			OLED_menu();
		}
		else if (page==1 && mode==2)
		{
			page=3;
			OLED_menu();
		}
	}
	else if (keynum==3)
	{
		if (page!=1)
		{
			page=1;
			OLED_menu();
		}		
	}
}







