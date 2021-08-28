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
 * Wiring of reed switch:
 *  - D3
 *  - GND
 *
 * Wiring of I2C devices (INA219, SSD1306):
 *  - SDA -> A4
 *  - SCL -> A5
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include <Arduino.h>

#include "display.h"
#include "main.h"
#include "temperature_meter.h"

#define SW_ON_LEVEL LOW   /* Switch on level */
#define SW_OFF_LEVEL HIGH /* Switch off level */

#define COUNTER_STATE_RESET 0
#define COUNTER_STATE_DISABLED 1
#define COUNTER_STATE_START 2
#define COUNTER_STATE_COUNTING 3
#define COUNTER_STATE_STOP 4

#define VIRT_REED_SWITCH_OFF 0
#define VIRT_REED_SWITCH_ON 1

#define REED_SWITCH_STATE_RESET 0
#define REED_SWITCH_STATE_START_TIMER 1
#define REED_SWITCH_STATE_STOP_TIMER 2
#define REED_SWITCH_STATE_READ_PIN 3
#define REED_SWITCH_STATE_ROTATE_BIN_COUNTER 4
#define REED_SWITCH_STATE_SET_LOGIC_SWITCH 5

// Switch Variables - "Reed_Switch"
int           state_Reed_Switch = REED_SWITCH_STATE_RESET; // The actual ~state~ of the state machine
int           pin_Reed_Switch = 3;                         // Input/Output (IO) pin for the switch, 3 = pin 3 a.k.a. D3
int           value_Reed_Switch = 0;                       // Value of the switch ("HIGH" or "LOW")
unsigned long t_Reed_Switch = 0;                           // Typically represents the current time of the switch
unsigned long t_0_Reed_Switch = 0;                         // The time that we last passed through an interesting state
unsigned long bounce_delay_Reed_Switch = 5;                // The delay to filter bouncing
unsigned int  bin_counter = 0;                             // binary counter for reed switch
int           virtual_Reed_Switch = VIRT_REED_SWITCH_OFF;  // virtual switch

// State Machine Counter variables
int           state_counter1 = COUNTER_STATE_RESET; // The actual ~state~ of the state machine
int           iSecCounter1 = -1;                    // extern in main.h
int           iMinCounter1 = -1;                    // extern in main.h
int           prev_state_counter1 = 0; // Remembers the previous state (useful to know when the state has changed)
int           prev_iSecCounter1 = 0;
int           prev_iMinCounter1 = 0;
unsigned long start_counter1 = 0;
unsigned long elapsed_counter1 = 0;

int pin_Status_Led = LED_BUILTIN;

char TimeCounterStr[] = "00:00"; /** String to store time counter value, format: MM:SS */
char SecondsCounterStr[] = "00"; /** String to store time counter value, format: SS */

/* Functions */

void setup(void)
{
    // if DEBUG is turned on, intialize serial connection
    Serial.begin(115200);
#ifdef SERIAL_DEBUG_ENABLED
    Serial.println(F("Debugging is ON"));
#endif
    Gpio_Init();
    ina219_Init();
    Display_Init();
    Display_Clear();
    // SM inits
    StateMachine_counter1();
    StateMachine_Reed_Switch();
    StateMachine_Volt_Meter();
    // state_Display = DISPLAY_STATE_RESET;
    StateMachine_Display();
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
    pinMode(pin_Reed_Switch, INPUT_PULLUP); // INPUT => reverse logic!
    pinMode(pin_Status_Led, OUTPUT);
}
