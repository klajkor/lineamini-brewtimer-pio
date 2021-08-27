/**
 * Arduino Brew Timer and Temperature Display for LaMarzocco Linea Mini espresso machine
 *
 *
 * @author Robert Klajko
 * @url https://github.com/klajkor/lineamini-brewtimer-pio.git
 *
 * Board: Arduino Pro Mini/Nano
 *
 * Extension modules and hw used:
 *  - SSD1306 OLED display, I2C - optional, please see source code for feature switch (SSD1306_ENABLED)
 *  - MAX7219 LED matrix display, I2C - optional, please see source code for feature switch (MAX7219_ENABLED)
 *  - 5x7 dot matrix displaying 3x5 digits (controlled by MAX7219)
 *  - Holtek HT16K33 LED matrix display, I2C - optional, please see source code for feature switch (HT16K33_ENABLED)
 *  - reed switch, sensing magnet valve sc
 *  - INA219, I2C
 * Libraries used:
 *  - SSD1306Ascii by Bill Greiman - Copyright (c) 2019, Bill Greiman
 *  - LedControl by wayoda - Copyright (c) 2012, Eberhard Fahle
 *  - Adafruit INA219 by Adafruit - Copyright (c) 2012, Adafruit Industries
 *  - HT16K33 - Copyright (c) 2017, lpaseen, Peter Sjoberg <peters-alib AT techwiz.ca>
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

// uncomment the line below if you would like to have debug messages
//#define SERIAL_DEBUG_ENABLED 1

// uncomment the line below if you would like to use MAX7219 LED matrix
//#define MAX7219_ENABLED 1

// uncomment the line below if you would like to use HT16K33 LED matrix
//#define HT16K33_ENABLED 1

// uncomment the line below if you would like to use SSD1306 OLED display
#define SSD1306_ENABLED 1

// uncomment the line below if you would like to use ILI9340 TFT display
//#define ILI9340_ENABLED 1

#include <Adafruit_GFX.h>
#include <Adafruit_INA219.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#ifdef MAX7219_ENABLED
#include <LedControl.h>
#endif // #ifdef MAX7219_ENABLED

#ifdef SSD1306_ENABLED
#include "display_ssd1306.h"
#endif

#ifdef HT16K33_ENABLED
#include <Adafruit_LEDBackpack.h>
#include <Fonts/Picopixel.h>
#include <ht16k33.h>
//#include <Fonts/TomThumb.h>
#include "Mirrored_LEDBackpack.h"
#include "TinyDigit.h"
#endif

#define LED_VERT_OFFSET 2
#define LED_HOR_OFFSET 1
#define LED_DIGIT_OFFSET 4

#define SW_ON_LEVEL LOW   /* Switch on level */
#define SW_OFF_LEVEL HIGH /* Switch off level */

#define COUNTER_STATE_RESET 0
#define COUNTER_STATE_DISABLED 1
#define COUNTER_STATE_START 2
#define COUNTER_STATE_COUNTING 3
#define COUNTER_STATE_STOP 4

#define DISPLAY_CLEAR_TRUE true
#define DISPLAY_CLEAR_FALSE false
#define DISPLAY_STOPPED_TRUE true
#define DISPLAY_STOPPED_FALSE false

#define VIRT_REED_SWITCH_OFF 0
#define VIRT_REED_SWITCH_ON 1

#define REED_SWITCH_STATE_RESET 0
#define REED_SWITCH_STATE_START_TIMER 1
#define REED_SWITCH_STATE_STOP_TIMER 2
#define REED_SWITCH_STATE_READ_PIN 3
#define REED_SWITCH_STATE_ROTATE_BIN_COUNTER 4
#define REED_SWITCH_STATE_SET_LOGIC_SWITCH 5

#define VOLT_METER_STATE_RESET 0
#define VOLT_METER_STATE_START_TIMER 1
#define VOLT_METER_STATE_STOP_TIMER 2
#define VOLT_METER_STATE_READ_VOLTAGE 3
#define VOLT_METER_STATE_CONVERT_TO_TEMPERATURE 4

#define DISPLAY_STATE_RESET 0
#define DISPLAY_STATE_TIMER_RUNNING 1
#define DISPLAY_STATE_TIMER_STOPPED 2
#define DISPLAY_STATE_HOLD_TIMER_ON 3
#define DISPLAY_STATE_TEMPERATURE 4
#define DISPLAY_STATE_DO_NOTHING 5

#define HT16K33_NORMAL_BRIGHTNESS 8
#define HT16K33_TEMPERATURE_BRIGHTNESS 4
#define HT16K33_DIMMED_BRIGHTNESS 1

// Thermistor calculation values
// Original idea and code from Jimmy Roasts, https://github.com/JimmyRoasts/LaMarzoccoTempSensor

// resistance at 25 degrees C
#define THERMISTORNOMINAL_V1 50000 // version 1
#define THERMISTORNOMINAL_V2 49120 // version 2 updated calculation
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 100
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT_V1 4400 // version 1
#define BCOEFFICIENT_V2 3977 // version 2, updated calculation
// the value of the 'other' resistor
#define SERIESRESISTOR_V1 6960
#define SERIESRESISTOR_V2 6190 // version 2, measured on board
// scaling value to convert voltage
#define VOLTAGESCALE 12.1
// reference voltage
//#define VOLTAGEREF 4.585
#define VOLTAGEREF 4.16

#define LMREF 5.07 // measured from LMBoard --- GND Board

// Top Level Variables:
int DEBUG = 1; // Set to 1 to enable serial monitor debugging info

// Switch Variables - "sw1"
int           state_Manual_Switch = 0;      // The actual ~state~ of the state machine
int           state_prev_Manual_Switch = 0; // Remembers the previous state (useful to know when the state has changed)
int           pin_Manual_Switch = 10;       // Input/Output (IO) pin for the switch, 10 = pin 10 a.k.a. D10
int           value_Manual_Switch = 0;      // Value of the switch ("HIGH" or "LOW")
unsigned long t_Manual_Switch = 0;          // Typically represents the current time of the switch
unsigned long t_0_Manual_Switch = 0;        // The time that we last passed through an interesting state
unsigned long bounce_delay_Manual_Switch = 20; // The delay to list for bouncing

// Switch Variables - "Reed_Switch"
int state_Reed_Switch = 0; // The actual ~state~ of the state machine
// int pin_Reed_Switch = 3;                   //Input/Output (IO) pin for the switch <- old config!
int           pin_Reed_Switch = 10;         // Input/Output (IO) pin for the switch, 10 = pin 10 a.k.a. D10
int           value_Reed_Switch = 0;        // Value of the switch ("HIGH" or "LOW")
unsigned long t_Reed_Switch = 0;            // Typically represents the current time of the switch
unsigned long t_0_Reed_Switch = 0;          // The time that we last passed through an interesting state
unsigned long bounce_delay_Reed_Switch = 5; // The delay to filter bouncing
unsigned int  bin_counter = 0;              // binary counter for reed switch
int           virtual_Reed_Switch = VIRT_REED_SWITCH_OFF; // virtual switch

// SM "Volt Meter" variables
int           state_Volt_Meter = 0;
unsigned long t_Volt_Meter = 0;
unsigned long t_0_Volt_Meter = 0;
unsigned long delay_Between_2_Measures = 300;

// SM Display variables
int           state_Display = 0;
unsigned long t_Display = 0;
unsigned long t_0_Display = 0;
unsigned long delay_For_Stopped_Timer = 5000; // millisec

// SM Counter variables
int           state_counter1 = 0;      // The actual ~state~ of the state machine
int           prev_state_counter1 = 0; // Remembers the previous state (useful to know when the state has changed)
int           iSecCounter1 = -1;
int           prev_iSecCounter1 = 0;
int           iMinCounter1 = -1;
int           prev_iMinCounter1 = 0;
unsigned long start_counter1 = 0;
unsigned long elapsed_counter1 = 0;

// SM Status LED Variables
int           state_Status_Led = 0;
int           state_prev_Status_Led = 0;
int           pin_Status_Led = LED_BUILTIN;
int           val_Status_Led = 0;
unsigned long t_Status_Led = 0;
unsigned long t_0_Status_Led = 0;
unsigned long on_delay_Status_Led = 100;
unsigned long off_delay_Status_Led = 100;
int           beep_count_Status_Led = 0;
int           beep_number_Status_Led = 2;

// Setup MAX7219 LED Matrix
// pin 12 is connected to the DataIn on the display
// pin 11 is connected to the CLK on the display
// pin 10 is connected to LOAD on the display
#ifdef MAX7219_ENABLED
LedControl lc_max7219 = LedControl(12, 11, 10, 1); // sets the 3 control pins as 12, 11 & 10 and then sets 1 display
byte       Max7219_intensity = 1;
byte       Max7219_bright_intensity = 8;
byte       Max7219_dimm_intensity = 1;
#endif // #ifdef MAX7219_ENABLED

char TimeCounterStr[] = "00:00"; /** String to store time counter value, format: MM:SS */
char SecondsCounterStr[] = "00"; /** String to store time counter value, format: SS */

// Current and voltage sensor class
Adafruit_INA219 ina219_monitor;
#define INA219_VCC_PIN 3
#define INA219_GND_PIN 2

// INA219 sensor variables
float bus_Voltage_V;             /** Measured bus voltage in V*/
float bus_Voltage_mV;            /** Measured bus voltage in mV*/
char  volt_String[] = "99999.9"; /** String to store measured voltage value in mV */

// Calculated temperature
float calc_Temperature_V1 = 0.0;
char  temperature_String_V1[] = "  999.9";
float steinhart = 0.0;
float calc_Temperature_V2 = 0.0;
char  temperature_String_V2[] = "999.9";
char  temperature_String_V2_Led_Matrix[] = "999.9";
float thermistor_Res = 0.00; // Thermistor calculated resistance

#ifdef HT16K33_ENABLED
// HT16K33 LED Matrix display
// Define lpaseen library class
HT16K33 HT;

// Define Adafruit library class
// Adafruit_8x16minimatrix matrix = Adafruit_8x16minimatrix();
Adafruit_Y_Mirrored_8x16minimatrix matrix_ht16k33 = Adafruit_Y_Mirrored_8x16minimatrix();
#endif // #ifdef HT16K33_ENABLED

/* Function declarations */

void StateMachine_counter1(void);
void StateMachine_Manual_Switch(void);
void StateMachine_Reed_Switch(void);
void StateMachine_Status_Led(void);
void StateMachine_Volt_Meter(void);
void StateMachine_Display(void);

void clear_Display_Max7219(void);
void bright_Display_Max7219(int dnum2disp);
void dimm_Display_Max7219(int dnum2disp);
void disp2digit_on_5x7(int d_address, int num2disp, int d_intensity);
void puttinydigit3x5l(int address, byte x, byte y, char c);
void disp_MinsAsColumn_On_Max7219(int dispMinutes, byte dispCol);
void update_TimeCounterStr(int tMinutes, int tSeconds);

void Gpio_Init(void);
void Max7219_Led_Matrix_Init(void);
void display_Timer_On_All(boolean need_Display_Clear, boolean need_Display_Stopped);
void display_Timer_On_Max7219(boolean need_Display_Clear, boolean need_Display_Stopped);
void display_Timer_On_Ht16k33(boolean need_Display_Clear, boolean need_Display_Stopped);
void display_Temperature_On_Ht16k33(void);

void ina219_Init(void);
void get_Voltage(void);
void calculate_Temperature_V2(void);

void Ht16k33_Led_Matrix_Init(void);
void Adafruit_Text_Display_Test(void);

void Display_Running_Timer(void);
void Display_Stopped_Timer(void);
void Display_Temperature(void);
void Display_Clear(void);

/* Functions */

void setup()
{
    // if DEBUG is turned on, intialize serial connection
    Serial.begin(115200);
#ifdef SERIAL_DEBUG_ENABLED
    Serial.println(F("Debugging is ON"));
#endif
    Gpio_Init();
    ina219_Init();
#ifdef MAX7219_ENABLED
    Max7219_Led_Matrix_Init();
#endif
#ifdef SSD1306_ENABLED
    Ssd1306_Oled_Init();
#endif
#ifdef HT16K33_ENABLED
    Ht16k33_Led_Matrix_Init();
#endif
    // SM inits
    StateMachine_counter1();
    StateMachine_Reed_Switch();
    StateMachine_Volt_Meter();
    state_Display = DISPLAY_STATE_RESET;
    StateMachine_Display();
    Display_Clear();
    // Adafruit_Text_Display_Test();
}

void loop()
{
    // Instruct all the state machines to proceed one step
    StateMachine_Reed_Switch();

    // Provide events that can force the state machines to change state
    switch (virtual_Reed_Switch)
    {
    case VIRT_REED_SWITCH_OFF:
        digitalWrite(pin_Status_Led, LOW);
        if (state_counter1 == COUNTER_STATE_COUNTING)
        {
            state_counter1 = COUNTER_STATE_STOP;
        }
        break;
    case VIRT_REED_SWITCH_ON:
        digitalWrite(pin_Status_Led, HIGH);
        if (state_counter1 == COUNTER_STATE_DISABLED)
        {
            state_counter1 = COUNTER_STATE_START;
        }
        break;
    }
    StateMachine_counter1();
    StateMachine_Volt_Meter();
    StateMachine_Display();
}

/**
 * @brief Counter 1 state machine - counts the seconds
 */
void StateMachine_counter1(void)
{

    prev_state_counter1 = state_counter1;

    // State Machine Section
    switch (state_counter1)
    {
    case COUNTER_STATE_RESET:
        iSecCounter1 = 0;
        iMinCounter1 = 0;
        elapsed_counter1 = 0;
        state_counter1 = COUNTER_STATE_DISABLED;
        break;
    case COUNTER_STATE_DISABLED:
        // waiting for START event
        break;
    case COUNTER_STATE_START:
        iSecCounter1 = 0;
        iMinCounter1 = 0;
        elapsed_counter1 = 0;
        start_counter1 = millis();
        state_counter1 = COUNTER_STATE_COUNTING;
        state_Display = DISPLAY_STATE_TIMER_RUNNING;
        break;
    case COUNTER_STATE_COUNTING:
        prev_iSecCounter1 = iSecCounter1;
        elapsed_counter1 = millis() - start_counter1;
        iSecCounter1 = int((elapsed_counter1 / 1000) % 60);
        iMinCounter1 = int((elapsed_counter1 / 1000) / 60);
        if (iSecCounter1 != prev_iSecCounter1)
        {
// debug display
#ifdef SERIAL_DEBUG_ENABLED
            Serial.print(F("iMinCounter1: "));
            Serial.print(iMinCounter1, DEC);
            Serial.print(F(" iSecCounter1: "));
            Serial.println(iSecCounter1, DEC);
#endif
            state_Display = DISPLAY_STATE_TIMER_RUNNING;
        }
        break;
    case COUNTER_STATE_STOP:
        state_counter1 = COUNTER_STATE_DISABLED;
        state_Display = DISPLAY_STATE_TIMER_STOPPED;
        break;
    }
    if (prev_state_counter1 == state_counter1)
    {
        // do nothing
    }
    else
    {
// debug display
#ifdef SERIAL_DEBUG_ENABLED
        Serial.print(F("state_counter1: "));
        Serial.println(state_counter1, DEC);
#endif
    }
}

/**
 * @brief Manual switch state machine
 */
void StateMachine_Manual_Switch(void)
{

    // Common code for every state
    value_Manual_Switch = digitalRead(pin_Manual_Switch);
    state_prev_Manual_Switch = state_Manual_Switch;

    // State Machine Section
    switch (state_Manual_Switch)
    {
    case 0: // RESET!
        // state_Manual_Switch variable initialization
        state_Manual_Switch = 1;
        break;

    case 1: // OFF, wait for on
        // Wait for the pin to go low (switch=ON)
        if (value_Manual_Switch == SW_ON_LEVEL)
        {
            state_Manual_Switch = 2;
        }
        break;

    case 2: // OFF, arming to ON
        // Start debounce timer and proceed to state3, "OFF, armed to ON"
        t_0_Manual_Switch = millis();
        state_Manual_Switch = 3;
        break;

    case 3: // OFF, armed to ON
        // Check to see if debounce delay has passed and switch still ON
        t_Manual_Switch = millis();
        if (t_Manual_Switch - t_0_Manual_Switch > bounce_delay_Manual_Switch && value_Manual_Switch == SW_ON_LEVEL)
        {
            state_Manual_Switch = 4;
        }
        break;

    case 4: // Switched ON
        state_Manual_Switch = 5;
        break;

    case 5: // ON, wait for off
        // Wait for the pin to go high (switch=OFF)
        if (value_Manual_Switch == SW_OFF_LEVEL)
        {
            state_Manual_Switch = 6;
        }
        break;

    case 6: // ON, arming to OFF
        // Start debounce timer and proceed to state7, "OFF, armed to ON"
        t_0_Manual_Switch = millis();
        state_Manual_Switch = 7;
        break;

    case 7: // ON, armed to OFF
        // Check to see if debounce delay has passed and switch still OFF
        t_Manual_Switch = millis();
        if (t_Manual_Switch - t_0_Manual_Switch > bounce_delay_Manual_Switch && value_Manual_Switch == SW_OFF_LEVEL)
        {
            state_Manual_Switch = 8;
        }
        break;
    case 8: // Switched OFF
        // go back to State1
        state_Manual_Switch = 1;
        break;
    }
}

/**
 * @brief Reed switch state machine, extended with a logic switch which handles the 50Hz switching of reed switch on
 * the solenoid
 */
void StateMachine_Reed_Switch(void)
{
    int prev_virtual_Reed_Switch;
    // State Machine Section
    switch (state_Reed_Switch)
    {
    case REED_SWITCH_STATE_RESET: // RESET!
        // variables initialization
        bin_counter = 0;
        virtual_Reed_Switch = VIRT_REED_SWITCH_OFF;
        value_Reed_Switch = SW_OFF_LEVEL;
        state_Reed_Switch = REED_SWITCH_STATE_START_TIMER;
        break;

    case REED_SWITCH_STATE_START_TIMER: // Start SW timer
        // Start debounce timer and proceed to state3, "OFF, armed to ON"
        t_0_Reed_Switch = millis();
        state_Reed_Switch = REED_SWITCH_STATE_STOP_TIMER;
        break;

    case REED_SWITCH_STATE_STOP_TIMER: // Timer stop
        // Check to see if debounce delay has passed
        t_Reed_Switch = millis();
        if (t_Reed_Switch - t_0_Reed_Switch > bounce_delay_Reed_Switch)
        {
            state_Reed_Switch = REED_SWITCH_STATE_READ_PIN;
        }
        break;

    case REED_SWITCH_STATE_READ_PIN: // Read Switch pin
        value_Reed_Switch = digitalRead(pin_Reed_Switch);
        state_Reed_Switch = REED_SWITCH_STATE_ROTATE_BIN_COUNTER;
        break;
    case REED_SWITCH_STATE_ROTATE_BIN_COUNTER: // Rotate binary counter
        bin_counter = bin_counter << 1;
        if (value_Reed_Switch == SW_ON_LEVEL)
        {
            bin_counter++;
        }
        state_Reed_Switch = REED_SWITCH_STATE_SET_LOGIC_SWITCH;
        break;

    case REED_SWITCH_STATE_SET_LOGIC_SWITCH:
        prev_virtual_Reed_Switch = virtual_Reed_Switch;
        if (bin_counter > 0)
        {
            if (prev_virtual_Reed_Switch == VIRT_REED_SWITCH_OFF)
            {
                virtual_Reed_Switch = VIRT_REED_SWITCH_ON;
// debug display
#ifdef SERIAL_DEBUG_ENABLED
                Serial.println(F("Virtual Reed switch ON"));
#endif
            }
        }
        else
        {
            if (prev_virtual_Reed_Switch == VIRT_REED_SWITCH_ON)
            {
                virtual_Reed_Switch = VIRT_REED_SWITCH_OFF;
// debug display
#ifdef SERIAL_DEBUG_ENABLED
                Serial.println(F("Virtual Reed switch OFF"));
#endif
            }
        }
        state_Reed_Switch = REED_SWITCH_STATE_START_TIMER;

        break;
    }
}

/**
 * @brief LED-1 state machine
 */
void StateMachine_Status_Led(void)
{
    // Common code for every state
    state_prev_Status_Led = state_Status_Led;
    // State Machine Section
    switch (state_Status_Led)
    {
    case 0: // RESET
        // Set Beep Count to 0 then proceed to WAIT
        beep_count_Status_Led = 0;
        state_Status_Led = 1;
        break;
    case 1: // WAIT
        // Do nothing.  Only the top level loop can force us out of this state into state 2 "TURN ON"
        break;

    case 2: // TURNING ON
        digitalWrite(pin_Status_Led, HIGH);
// debug display
#ifdef SERIAL_DEBUG_ENABLED
        Serial.println(F(":: LED ON"));
#endif
        t_0_Status_Led = millis();
        state_Status_Led = 3;
        break;
    case 3: // ON
        // Wait for time to elapse, then proceed to TURN OFF
        t_Status_Led = millis();
        if (t_Status_Led - t_0_Status_Led > on_delay_Status_Led)
        {
            state_Status_Led = 4;
        }
        break;
    case 4: // TURNING OFF
        // Increment the beep counter, proceed to OFF
        beep_count_Status_Led++;
        t_0_Status_Led = millis();
        digitalWrite(pin_Status_Led, LOW);
// debug display
#ifdef SERIAL_DEBUG_ENABLED
        Serial.println(F(":: LED off"));
#endif
        state_Status_Led = 5;
        break;
    case 5: // OFF
        t_Status_Led = millis();
        if (t_Status_Led - t_0_Status_Led > off_delay_Status_Led)
        {
            state_Status_Led = 2;
        }
        if (beep_count_Status_Led >= beep_number_Status_Led)
        {
            state_Status_Led = 0;
        }
        break;
    }
}

#ifdef MAX7219_ENABLED
/**
 * @brief Clears the dot matrix display(s)
 */
void clear_Display_Max7219(void)
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
void bright_Display_Max7219(int dnum2disp)
{
    disp2digit_on_5x7(0, dnum2disp, Max7219_bright_intensity);
// debug display
#ifdef SERIAL_DEBUG_ENABLED
    Serial.print(F("Bright: "));
    Serial.println(dnum2disp);
#endif
}

/**
 * @brief Displays the number on the LED display with "Max7219_dimm_intensity" brightness
 * @param dnum2disp : number to display
 */
void dimm_Display_Max7219(int dnum2disp)
{
    disp2digit_on_5x7(0, dnum2disp, Max7219_dimm_intensity);
// debug display
#ifdef SERIAL_DEBUG_ENABLED
    Serial.print(F("Dimm: "));
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
void disp2digit_on_5x7(int d_address, int num2disp, int d_intensity)
{
    char disp_ch_tens, disp_ch_ones;
    num2disp = num2disp % 100;
    disp_ch_tens = 0x30 + num2disp / 10;
    disp_ch_ones = 0x30 + num2disp % 10;
    lc_max7219.setIntensity(d_address, d_intensity);
    puttinydigit3x5l(d_address, 0, 0, disp_ch_tens);
    puttinydigit3x5l(d_address, 4, 0, disp_ch_ones);
}

/**
 * @brief Display tiny 3x5 digit in landscape mode on a MAX7219 5x7 matrix
 *
 * @param address : address of the LED matrix display
 * @param x : X coordinate
 * @param y : no Y coordinate can be set! (permanently zeroed currently)
 * @param c : character to display (numbers only)
 */
void puttinydigit3x5l(int address, byte x, byte y, char c)
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

void disp_MinsAsColumn_On_Max7219(int dispMinutes, byte dispCol)
{
    int  address = 0;
    byte columnBits;
    dispMinutes = dispMinutes % 10;
    switch (dispMinutes)
    {
    case 0:
        columnBits = 0;
        break;
    case 1:
        columnBits = B00000001;
        columnBits = B10000000;
        break;
    case 2:
        columnBits = B00000011;
        columnBits = B11000000;
        break;
    case 3:
        columnBits = B00000111;
        columnBits = B11100000;
        break;
    case 4:
        columnBits = B00001111;
        columnBits = B11110000;
        break;
    case 5:
        columnBits = B00011111;
        columnBits = B11111000;
        break;
    case 6:
        columnBits = B00010000;
        columnBits = B00001000;
        break;
    case 7:
        columnBits = B00011000;
        columnBits = B00011000;
        break;
    case 8:
        columnBits = B00011100;
        columnBits = B00111000;
        break;
    case 9:
        columnBits = B00011110;
        columnBits = B01111000;
        break;
    }
    lc_max7219.setRow(address, dispCol, columnBits);
}

void Max7219_Led_Matrix_Init(void)
{
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
    lc_max7219.setIntensity(0, Max7219_dimm_intensity);
    disp2digit_on_5x7(0, 0, Max7219_dimm_intensity);
}

void display_Timer_On_Max7219(boolean need_Display_Clear, boolean need_Display_Stopped)
{
    if (need_Display_Clear)
    {
        clear_Display_Max7219();
    }
    if (need_Display_Stopped)
    {
        dimm_Display_Max7219(iSecCounter1);
    }
    else
    {
        bright_Display_Max7219(iSecCounter1);
    }
    // disp_MinsAsColumn_On_Max7219(iMinCounter1,3);
}
#endif // #ifdef MAX7219_ENABLED

/**
 * @brief Converts the minutes and seconds to char and updates the TimeCounterStr string
 * @param tMinutes : minutes value
 * @param tSeconds : seconds value
 */
void update_TimeCounterStr(int tMinutes, int tSeconds)
{
    TimeCounterStr[0] = (char)((tMinutes / 10) + 0x30);
    TimeCounterStr[1] = (char)((tMinutes % 10) + 0x30);
    TimeCounterStr[3] = (char)((tSeconds / 10) + 0x30);
    TimeCounterStr[4] = (char)((tSeconds % 10) + 0x30);
    SecondsCounterStr[0] = (char)((tSeconds / 10) + 0x30);
    SecondsCounterStr[1] = (char)((tSeconds % 10) + 0x30);
}

void Gpio_Init(void)
{
    pinMode(pin_Manual_Switch, INPUT_PULLUP); // INPUT => reverse logic!
    pinMode(pin_Reed_Switch, INPUT_PULLUP);   // INPUT => reverse logic!
    pinMode(pin_Status_Led, OUTPUT);
}

void display_Timer_On_All(boolean need_Display_Clear, boolean need_Display_Stopped)
{
    update_TimeCounterStr(iMinCounter1, iSecCounter1);
#ifdef MAX7219_ENABLED
    display_Timer_On_Max7219(need_Display_Clear, need_Display_Stopped);
#endif
#ifdef SSD1306_ENABLED
    display_Timer_On_Ssd1306(SecondsCounterStr, need_Display_Clear, need_Display_Stopped);
#endif
#ifdef HT16K33_ENABLED
    display_Timer_On_Ht16k33(need_Display_Clear, need_Display_Stopped);
#endif
}

void ina219_Init(void)
{
    // pinMode(INA219_GND_PIN, OUTPUT);
    // pinMode(INA219_VCC_PIN, OUTPUT);
    // digitalWrite(INA219_GND_PIN, LOW);
    // digitalWrite(INA219_VCC_PIN, HIGH);
    delay(100);
    ina219_monitor.begin();
}

void get_Voltage(void)
{
    // measure voltage and current
    bus_Voltage_V = (ina219_monitor.getBusVoltage_V());
    bus_Voltage_mV = bus_Voltage_V * 1000;
    // convert to text
    dtostrf(bus_Voltage_mV, 7, 1, volt_String);
// debug display
#ifdef SERIAL_DEBUG_ENABLED
    Serial.print(volt_String);
    Serial.println(F(" mV"));
#endif
}

void calculate_Temperature_V2(void)
{
    thermistor_Res = SERIESRESISTOR_V2 * (1 / ((LMREF / bus_Voltage_V) - 1));
    steinhart = thermistor_Res / THERMISTORNOMINAL_V2;
    steinhart = log(steinhart);                         // ln(R/Ro)
    steinhart /= BCOEFFICIENT_V2;                       // 1/B * ln(R/Ro)
    steinhart += (1.0 / (TEMPERATURENOMINAL + 273.15)); // + (1/To)
    steinhart = 1.0 / steinhart;                        // Invert
    calc_Temperature_V2 = (float)steinhart - 273.15;    // convert to C
    if (calc_Temperature_V2 < -100)
    {
        calc_Temperature_V2 = -99.9;
    }
    dtostrf(calc_Temperature_V2, 5, 1, temperature_String_V2);
    dtostrf(calc_Temperature_V2, 5, 1, temperature_String_V2_Led_Matrix);
// debug display
#ifdef SERIAL_DEBUG_ENABLED
    Serial.print(temperature_String_V1);
    Serial.println(F(" *C"));
#endif
}

void StateMachine_Volt_Meter(void)
{

    switch (state_Volt_Meter)
    {
    case VOLT_METER_STATE_RESET:
        state_Volt_Meter = VOLT_METER_STATE_START_TIMER;
        break;

    case VOLT_METER_STATE_START_TIMER:
        t_0_Volt_Meter = millis();
        state_Volt_Meter = VOLT_METER_STATE_STOP_TIMER;
        break;

    case VOLT_METER_STATE_STOP_TIMER:
        t_Volt_Meter = millis();
        if (t_Volt_Meter - t_0_Volt_Meter > delay_Between_2_Measures)
        {
            state_Volt_Meter = VOLT_METER_STATE_READ_VOLTAGE;
        }
        break;

    case VOLT_METER_STATE_READ_VOLTAGE:
        get_Voltage();
        state_Volt_Meter = VOLT_METER_STATE_CONVERT_TO_TEMPERATURE;
        break;

    case VOLT_METER_STATE_CONVERT_TO_TEMPERATURE:
        calculate_Temperature_V2();
        state_Volt_Meter = VOLT_METER_STATE_START_TIMER;
        if (state_counter1 == COUNTER_STATE_DISABLED && state_Display != DISPLAY_STATE_HOLD_TIMER_ON)
        {
            state_Display = DISPLAY_STATE_TEMPERATURE;
        }
        break;
    }
}

#ifdef HT16K33_ENABLED
void Ht16k33_Led_Matrix_Init(void)
{
    HT.begin(0x70);
    HT.setBrightness(1);
    HT.clearAll();
    matrix_ht16k33.begin(0x70);
    matrix_ht16k33.setFont(&Picopixel);
}

void display_Timer_On_Ht16k33(boolean need_Display_Clear, boolean need_Display_Stopped)
{
    char *tempPointer;
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
    matrix_ht16k33.setCursor(1, 5);
    tempPointer = &TimeCounterStr[1];
    matrix_ht16k33.print(tempPointer);
    // HT.setLedNow((iSecCounter1 % 15)*8+7);
    matrix_ht16k33.setCursor((iSecCounter1 % 15), 7);
    matrix_ht16k33.print(".");
    matrix_ht16k33.writeDisplay();
}

void display_Temperature_On_Ht16k33()
{
    matrix_ht16k33.setRotation(1);
    matrix_ht16k33.setTextColor(LED_ON);
    matrix_ht16k33.clear();
    matrix_ht16k33.setCursor(0, 5);
    HT.setBrightness(HT16K33_TEMPERATURE_BRIGHTNESS);
    matrix_ht16k33.print(temperature_String_V2_Led_Matrix);
    matrix_ht16k33.writeDisplay();
}

void Adafruit_Text_Display_Test_On_Ht16k33(void)
{
    // int8_t x,y;
    matrix_ht16k33.setTextSize(1);
    matrix_ht16k33.setTextWrap(false);
    /**
    Serial.println(F("rotation=0"));
    matrix_ht16k33.setRotation(0);
    matrix_ht16k33.setTextColor(LED_ON);
    matrix_ht16k33.clear();
    matrix_ht16k33.setCursor(0,0);
    matrix_ht16k33.print("123");
    matrix_ht16k33.writeDisplay();
    delay(5000);
    */
    Serial.println("rotation=1");
    matrix_ht16k33.setRotation(1);
    matrix_ht16k33.setTextColor(LED_ON);
    matrix_ht16k33.clear();
    matrix_ht16k33.setCursor(0, 7);
    matrix_ht16k33.print("0:12");
    matrix_ht16k33.writeDisplay();
    delay(5000);
    Serial.println("rotation=3");
    matrix_ht16k33.setRotation(3);
    matrix_ht16k33.clear();
    matrix_ht16k33.setCursor(0, 7);
    matrix_ht16k33.print("1:00");
    matrix_ht16k33.writeDisplay();
    delay(5000);
    Serial.println("rotation=1");
    matrix_ht16k33.setRotation(1);
    matrix_ht16k33.clear();
    matrix_ht16k33.setCursor(0, 7);
    matrix_ht16k33.print("100.0*");
    matrix_ht16k33.writeDisplay();
    delay(5000);
    for (int i = 50; i < 120; i++)
    {
        matrix_ht16k33.clear();
        matrix_ht16k33.setCursor(0, 7);
        matrix_ht16k33.print(i);
        matrix_ht16k33.writeDisplay();
        delay(200);
    }
    matrix_ht16k33.setRotation(0);
}
#endif // #ifdef HT16K33_ENABLED

void StateMachine_Display(void)
{

    switch (state_Display)
    {
    case DISPLAY_STATE_RESET:
        Display_Stopped_Timer();
        state_Display = DISPLAY_STATE_DO_NOTHING;
        break;

    case DISPLAY_STATE_TIMER_RUNNING:
        Display_Running_Timer();
        Display_Temperature();
        state_Display = DISPLAY_STATE_DO_NOTHING;
        break;

    case DISPLAY_STATE_TIMER_STOPPED:
        Display_Stopped_Timer();
        t_0_Display = millis();
        state_Display = DISPLAY_STATE_HOLD_TIMER_ON;
        break;

    case DISPLAY_STATE_HOLD_TIMER_ON:
        t_Display = millis();
        if (t_Display - t_0_Display > delay_For_Stopped_Timer)
        {
            state_Display = DISPLAY_STATE_TEMPERATURE;
        }
        break;

    case DISPLAY_STATE_TEMPERATURE:
        Display_Temperature();
        state_Display = DISPLAY_STATE_DO_NOTHING;
        break;

    case DISPLAY_STATE_DO_NOTHING:
        break;
    }
}

void Display_Running_Timer(void)
{
    display_Timer_On_All(DISPLAY_CLEAR_FALSE, DISPLAY_STOPPED_FALSE);
}

void Display_Stopped_Timer(void)
{
    display_Timer_On_All(DISPLAY_CLEAR_FALSE, DISPLAY_STOPPED_TRUE);
}

void Display_Temperature(void)
{
#ifdef SSD1306_ENABLED
    display_Temperature_On_Ssd1306(temperature_String_V2);
#endif
#ifdef HT16K33_ENABLED
    display_Temperature_On_Ht16k33();
#endif
}

void Display_Clear(void)
{
#ifdef SSD1303_ENABLED
    oled_ssd1306_display.clear();
#endif
}
