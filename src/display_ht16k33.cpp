#include "display_ht16k33.h"

HT16K33 HT;

// Define Adafruit library class
Adafruit_Y_Mirrored_8x16minimatrix matrix_ht16k33 = Adafruit_Y_Mirrored_8x16minimatrix();

void Ht16k33_Led_Matrix_Init(void)
{
    HT.begin(0x70);
    HT.setBrightness(1);
    HT.clearAll();
    matrix_ht16k33.begin(0x70);
    matrix_ht16k33.setFont(&PicopixelMod);
    matrix_ht16k33.setRotation(1);
    matrix_ht16k33.setTextColor(LED_ON);
    matrix_ht16k33.clear();
    matrix_ht16k33.setCursor(HT16K33_TIMER_CURSOR_X, HT16K33_TIMER_CURSOR_Y);
    matrix_ht16k33.print("00");
    matrix_ht16k33.writeDisplay();
}

void display_Timer_On_Ht16k33(char *pCounterStr, boolean need_Display_Clear, boolean need_Display_Stopped)
{
    matrix_ht16k33.setRotation(1);
    matrix_ht16k33.setTextColor(LED_ON);
    matrix_ht16k33.clear();
    if (need_Display_Stopped)
    {
        HT.setBrightness(HT16K33_DIMMED_BRIGHTNESS);
    }
    else
    {
        HT.setBrightness(HT16K33_NORMAL_BRIGHTNESS);
    }
    matrix_ht16k33.setCursor(HT16K33_TIMER_CURSOR_X, HT16K33_TIMER_CURSOR_Y);
    matrix_ht16k33.print(pCounterStr);
    // matrix_ht16k33.drawLine(0, 0, 0, 7, LED_ON);
    // matrix_ht16k33.drawLine(15, 0, 15, 7, LED_ON);
    matrix_ht16k33.drawLine(1, 7, 14, 7, LED_ON);
    matrix_ht16k33.writeDisplay();
}

void display_Temperature_On_Ht16k33(char *pTemperatureStr)
{
    matrix_ht16k33.setRotation(1);
    matrix_ht16k33.setTextColor(LED_ON);
    matrix_ht16k33.clear();
    matrix_ht16k33.setCursor(HT16K33_TEMPERATURE_CURSOR_X, HT16K33_TEMPERATURE_CURSOR_Y);
    HT.setBrightness(HT16K33_TEMPERATURE_BRIGHTNESS);
    matrix_ht16k33.print(pTemperatureStr);
    matrix_ht16k33.drawLine(15, 2, 15, 2, LED_ON);
    matrix_ht16k33.writeDisplay();
}
