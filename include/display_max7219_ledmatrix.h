#pragma once
#include "LedControl.h"
#include "TinyDigit.h"

#define MAX_LED_DEVICES (1)

extern LedControl lc_max7219;
extern byte       Max7219_intensity;
extern byte       Max7219_bright_intensity;
extern byte       Max7219_dimm_intensity;

void max7219_led_matrix_init(void);
void max7219_display_clear(void);
void max7219_display_bright(int dnum2disp);
void max7219_display_dimm(int dnum2disp);
void max7219_display_2_digits_on_5x7(int d_address, int num2disp, int d_intensity);
void max7219_display_tinydigit3x5l(int address, byte x, byte y, char c);
void display_Timer_On_Max7219(int SecCounter, boolean need_Display_Clear, boolean need_Display_Stopped);
