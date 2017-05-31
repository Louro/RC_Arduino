// OLED.h

#ifndef _OLED_h
#define _OLED_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// If using software SPI (the default case):
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void OLED_init();
void OLED_draw(u8 page);
void OLED_draw_page_1();
void OLED_draw_page_2();
void OLED_draw_page_3();
void OLED_draw_page_4();
void printVertical(char * str, unsigned int x, unsigned int y);
unsigned int bar_size(float bat_volt);

#endif

