#include "display_ssd1306.h"

SSD1306AsciiWire oled_ssd1306_display;

void Ssd1306_Oled_Init(void)
{
    Wire.begin();
    oled_ssd1306_display.begin(&Adafruit128x32, OLED_I2C_ADDR);
    oled_ssd1306_display.clear();
    oled_ssd1306_display.setFont(fixed_bold10x15);
    oled_ssd1306_display.setRow(0);
    oled_ssd1306_display.println("Linea Mini ");
    oled_ssd1306_display.println("Brew Timer ");
    delay(1000);
    oled_ssd1306_display.clear();
    oled_ssd1306_display.setRow(0);
    oled_ssd1306_display.println("Version 1.1");
    delay(500);
    oled_ssd1306_display.clear();
}

/**
 * @brief Displays the pCounterStr string on the OLED screen
 * @param pCounterStr : counter string to be displayed
 * @param need_Display_Clear : is display clear needed?
 * @param need_Display_Stopped : is visualisation of stopped timer needed?
 */
void display_Timer_On_Ssd1306(char *pCounterStr, boolean need_Display_Clear, boolean need_Display_Stopped)
{
    if (need_Display_Clear)
    {
        oled_ssd1306_display.clear();
    }
    if (need_Display_Stopped)
    {
        oled_ssd1306_display.setContrast(5);
    }
    else
    {
        oled_ssd1306_display.setContrast(255);
    }
    oled_ssd1306_display.setFont(fixednums15x31);
    oled_ssd1306_display.setCol(0);
    oled_ssd1306_display.setRow(0);
    oled_ssd1306_display.print(pCounterStr);
}

/**
 * @brief Displays the pTemperatureStr string on the OLED screen
 * @param pTemperatureStr : temperature string to be displayed
 */
void display_Temperature_On_Ssd1306(char *pTemperatureStr)
{
    oled_ssd1306_display.setFont(fixed_bold10x15);
    oled_ssd1306_display.setCol(47);
    oled_ssd1306_display.setRow(1);
    oled_ssd1306_display.print(pTemperatureStr);
    oled_ssd1306_display.print(F(" C"));
}
