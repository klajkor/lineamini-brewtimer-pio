#pragma once

#include "display.h"

//#include "Adafruit_GFX.h"
#include "Adafruit_LEDBackpack.h"
#include "Mirrored_LEDBackpack.h"
#include "PicopixelMod.h"
#include "ht16k33.h"

#define HT16K33_NORMAL_BRIGHTNESS (8)
#define HT16K33_TEMPERATURE_BRIGHTNESS (3)
#define HT16K33_DIMMED_BRIGHTNESS (1)

#define HT16K33_TIMER_CURSOR_X (5)
#define HT16K33_TIMER_CURSOR_Y (5)
#define HT16K33_TEMPERATURE_CURSOR_X (1)
#define HT16K33_TEMPERATURE_CURSOR_Y (7)

// HT16K33 LED Matrix display
// Define lpaseen library class
extern HT16K33 HT;

// Define Adafruit library class
// Original was: Adafruit_8x16minimatrix matrix = Adafruit_8x16minimatrix();
extern Adafruit_Y_Mirrored_8x16minimatrix matrix_ht16k33;

void Ht16k33_Led_Matrix_Init(void);
void display_Timer_On_Ht16k33(char *pCounterStr, boolean need_Display_Clear, boolean need_Display_Stopped);
void display_Temperature_On_Ht16k33(char *pTemperatureStr);
