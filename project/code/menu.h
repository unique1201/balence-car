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

#endif
