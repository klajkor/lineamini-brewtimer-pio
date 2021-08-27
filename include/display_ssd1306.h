#pragma once

#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>

#define OLED_I2C_ADDR 0x3C /* OLED module I2C address */

// Set up OLED display
extern SSD1306AsciiWire oled_ssd1306_display;

void Ssd1306_Oled_Init(void);
void display_Timer_On_Ssd1306(char *pCounterStr, boolean need_Display_Clear, boolean need_Display_Stopped);
void display_Temperature_On_Ssd1306(char *pTemperatureStr);
