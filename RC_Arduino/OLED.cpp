// 
// 
// 

#include "OLED.h"




float bat_volt[6] = { 3.23,3.34,3.12,3.25,3.36,3.40 };
float pack_info[6] = { 21.58,99.50,9.58,0.96,96.56,8.58 };

#define BAR_HIGHT 9
#define BAR_WIDTH 40
#define BAR_X 10
#define BAR_Y_OFFSET 11

Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

void OLED_init() {
	display.begin(SSD1306_SWITCHCAPVCC);
}


void OLED_draw(u8 page) {
	
	display.clearDisplay();

	switch (page) {
		case 1: OLED_draw_page_1(); break;
		case 2: OLED_draw_page_2(); break;
		case 3: OLED_draw_page_3(); break;
		case 4: OLED_draw_page_4(); break;
	}
	
	display.display();
}

void OLED_draw_page_1() {
	float v;

	//v = (((float)servo_value) / 1000.0)*2.5;
	//for (int i = 0; i < 6; i++)
	//	bat_volt[i] = (float)(nunchuck_buf[0]) / 50.0;

	display.clearDisplay();

	display.setTextSize(1);
	display.setTextColor(WHITE);

	// Draw rect bar
	for (int x = 1, y = 0; y < BAR_Y_OFFSET * 6; x++, y = y + BAR_Y_OFFSET) {
		display.setCursor(0, y + 1);
		display.print(x);
		display.drawRect(BAR_X, y, BAR_WIDTH, BAR_HIGHT, WHITE);
	}

	// Fill rect with cell voltage and display cell voltage
	for (int x = 0, y = 0; y < BAR_Y_OFFSET * 6; x++, y = y + BAR_Y_OFFSET) {
		display.fillRect(BAR_X, y, bar_size(bat_volt[x]), BAR_HIGHT, WHITE);
		display.setCursor(BAR_X + BAR_WIDTH + 4, y + 1);
		display.print(bat_volt[x]);
		display.setCursor(BAR_X + BAR_WIDTH + 29, y + 1);
		display.print('V');
	}

	// Draw actual current level
	//display.drawRect(90,  0, 15, 55, WHITE);
	//printVertical("1345mA", 95, 5);

	// Draw charged left in battery
	//display.drawRect(110,  0, 15, 55, WHITE);
	//printVertical("9854mAh", 115, 5);


	// Draw battery pack voltage
	display.setCursor(90, 0);
	display.print(v);//display.print(pack_info[0]);
	display.setCursor(122, 0);
	display.print("V");

	// Draw current draw
	display.setCursor(90, 11);
	display.print(pack_info[1]);
	display.setCursor(122, 11);
	display.print("A");

	// Draw battery pack charge left
	display.setCursor(90, 22);
	display.print(pack_info[2]);
	display.setCursor(116, 22);
	display.print("Ah");

	// Draw battery pack charge used
	display.setCursor(90, 33);
	display.print(pack_info[3]);
	display.setCursor(116, 33);
	display.print("Ah");

	// Draw battery pack % left
	display.setCursor(90, 44);
	display.print(pack_info[4]);
	display.setCursor(122, 44);
	display.print("%");

	// Draw remote control battery pack voltage
	display.setCursor(95, 55);
	display.print(pack_info[5]);
	display.setCursor(122, 55);
	display.print("V");

	display.display();

}

void OLED_draw_page_2() {
	display.setTextSize(4);
	display.setTextColor(WHITE);
	display.setCursor(128 / 2, 64 / 2);
	display.print("2");
}

void OLED_draw_page_3() {
	display.setTextSize(4);
	display.setTextColor(WHITE);
	display.setCursor(128/2, 64/2);
	display.print("3");
}

void OLED_draw_page_4() {
	display.setTextSize(4);
	display.setTextColor(WHITE);
	display.setCursor(128 / 2, 64 / 2);
	display.print("4");
}

unsigned int bar_size(float bat_volt) {
	unsigned int value = 0;

	if (bat_volt > 2.5) {
		value = (bat_volt - 2.5) * 40 / 1.7;
	}

	return value;
}

void printVertical(char * str, unsigned int x, unsigned int y) {
	char * ptr;
	int i;
	ptr = str;
	for (i = 0; *ptr != NULL; ptr++, i += 8) {
		display.setCursor(x, y + i);
		display.print(*ptr);
	}
}



