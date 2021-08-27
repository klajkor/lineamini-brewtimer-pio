#include "display_max7219_ledmatrix.h"

//#define SERIAL_DEBUG_ENABLED 1

// Setup MAX7219 LED Matrix
// pin 12 is connected to the DataIn on the display
// pin 11 is connected to the CLK on the display
// pin 10 is connected to CS on the display
LedControl lc_max7219 =
    LedControl(12, 11, 10, MAX_LED_DEVICES); // sets the 3 control pins as 12, 11 & 10 and then sets num of display
byte Max7219_intensity = 4;
byte Max7219_bright_intensity = 8;
byte Max7219_dimm_intensity = 1;

/**
 * @brief Initializes the dot matrix display(s)
 */
void max7219_led_matrix_init(void)
{
    uint8_t i;
    // we have already set the number of devices when we created the LedControl
    int devices = lc_max7219.getDeviceCount();
    // we have to init all devices in a loop
    for (int address = 0; address < devices; address++)
    {
        /*The MAX72XX is in power-saving mode on startup*/
        lc_max7219.shutdown(address, false);
        /* Set the brightness to a medium values */
        lc_max7219.setIntensity(address, Max7219_dimm_intensity);
        /* and clear the display */
        lc_max7219.clearDisplay(address);
    }
    lc_max7219.setIntensity(0, Max7219_bright_intensity);
    for (i = 0; i < 8; i++)
    {
        lc_max7219.setLed(0, i, i, true);
        delay(200);
    }
    delay(500);
    lc_max7219.clearDisplay(0);
    lc_max7219.setIntensity(0, Max7219_dimm_intensity);
    max7219_display_2_digits_on_5x7(0, 0, Max7219_dimm_intensity);
#ifdef SERIAL_DEBUG_ENABLED
    Serial.println(F("MAX7219 initialized"));
#endif
}

/**
 * @brief Clears the dot matrix display(s)
 */
void max7219_display_clear(void)
{
    int devices = lc_max7219.getDeviceCount();
    for (byte address = 0; address < devices; address++)
    {
        lc_max7219.clearDisplay(address);
    }
}

/**
 * @brief Displays the number on the LED display with "MAX7219_bright_intensity" brightness
 * @param dnum2disp : number to display
 */
void max7219_display_bright(int dnum2disp)
{
    max7219_display_2_digits_on_5x7(0, dnum2disp, Max7219_bright_intensity);
// debug display
#ifdef SERIAL_DEBUG_ENABLED
    Serial.print(F("Bright dnum2disp: "));
    Serial.println(dnum2disp);
#endif
}

/**
 * @brief Displays the number on the LED display with "Max7219_dimm_intensity" brightness
 * @param dnum2disp : number to display
 */
void max7219_display_dimm(int dnum2disp)
{
    max7219_display_2_digits_on_5x7(0, dnum2disp, Max7219_dimm_intensity);
// debug display
#ifdef SERIAL_DEBUG_ENABLED
    Serial.print(F("Dimm dnum2disp: "));
    Serial.println(dnum2disp);
#endif
}

/**
 * @brief Displays a 2 digits number in landscape mode on a MAX7219 5x7 matrix
 *
 * @param d_address : address of the LED matrix display
 * @param num2disp : number to display
 * @param d_intensity : LED intensity (=brightness) level
 */
void max7219_display_2_digits_on_5x7(int d_address, int num2disp, int d_intensity)
{
    char disp_ch_tens, disp_ch_ones;
    num2disp = num2disp % 100;
    disp_ch_tens = 0x30 + num2disp / 10;
    disp_ch_ones = 0x30 + num2disp % 10;
    lc_max7219.setIntensity(d_address, d_intensity);
    max7219_display_tinydigit3x5l(d_address, 0, 0, disp_ch_tens);
    max7219_display_tinydigit3x5l(d_address, 4, 0, disp_ch_ones);
}

/**
 * @brief Display tiny 3x5 digit in landscape mode on a MAX7219 5x7 matrix
 *
 * @param address : address of the LED matrix display
 * @param x : X coordinate
 * @param y : no Y coordinate can be set! (permanently zeroed currently)
 * @param c : character to display (numbers only)
 */
void max7219_display_tinydigit3x5l(int address, byte x, byte y, char c)
{
    byte dots, dot_col, char_selector;
    y = 0;
    if (c >= '0' && c <= '9')
    {
        char_selector = (c - '0');
        for (dot_col = 0; dot_col < 3; dot_col++)
        {
            dots = pgm_read_byte_near(&digit3x5landscape[char_selector][dot_col]);
            lc_max7219.setRow(address, dot_col + x, dots << 3);
        }
    }
}

void display_Timer_On_Max7219(int SecCounter, boolean need_Display_Clear, boolean need_Display_Stopped)
{
    if (need_Display_Clear)
    {
        max7219_display_clear();
    }
    if (need_Display_Stopped)
    {
        max7219_display_dimm(SecCounter);
    }
    else
    {
        max7219_display_bright(SecCounter);
    }
}
