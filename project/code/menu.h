#ifndef __MENU_H_
#define __MENU_H_

struct option_class{
	char Name[16];
	void (*func)(void);
};

void display_menu(struct option_class* option, int displayItems, int currentLine);
void display_value(struct option_class* option, int displayItems);
void Menu_loop(struct option_class* option);
void Main_Menu(void);
void tiaocan(void);
void Mode(void);
void Blue(void);

void K1(void);
void K2(void);
void K3(void);

void Kp1(void);
void Ki1(void);
void Kd1(void);
void Kp2(void);
void Ki2(void);
void Kd2(void);
void Kp3(void);
void Ki3(void);
void Kd3(void);

void M1(void);
void M2(void);
void M3(void);
void M4(void);
void M5(void);

#endif
