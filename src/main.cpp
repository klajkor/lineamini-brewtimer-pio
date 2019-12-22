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
*  - SSD1306 OLED dsiplay, I2C
*  - MAX7219 LED display driver, I2C
*  - 3x5 digits on 5x7 dot matrix
*  - reed switch, sensing magnet valve sc
*  - INA219, I2C
* Libraries used:
*  - SSD1306Ascii by Bill Greiman - Copyright (c) 2019, Bill Greiman
*  - LedControl by wayoda - Copyright (c) 2012, Eberhard Fahle
*  - Adafruit INA219 by Adafruit - Copyright (c) 2012, Adafruit Industries
*
* BSD license, all text here must be included in any redistribution.
*
*/

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <LedControl.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
//#include "SSD1306AsciiAvrI2c.h" /* Use only when no other I2C devices are used! */
#include <ht16k33.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
#include <Fonts/Picopixel.h>
#include <Fonts/TomThumb.h>
#include "TinyDigit.h"
#include "Mirrored_LEDBackpack.h"

//uncomment the line below if you would like to have debug messages
//#define SERIAL_DEBUG_ENABLED 1

//uncomment the line below if you would like to use MAX7219 LED matrix
//#define SERIAL_MAX7219_ENABLED 1

#define LED_VERT_OFFSET 2
#define LED_HOR_OFFSET 1
#define LED_DIGIT_OFFSET 4

#define SW_ON_LEVEL LOW  /* Switch on level */
#define SW_OFF_LEVEL HIGH  /* Switch off level */

#define OLED_I2C_ADDR 0x3C /* OLED module I2C address */

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

// Thermistor calculation values
// Original idea and code from Jimmy Roasts, https://github.com/JimmyRoasts/LaMarzoccoTempSensor

// resistance at 25 degrees C
#define THERMISTORNOMINAL_V1 50000  // version 1
#define THERMISTORNOMINAL_V2 49120  // version 2 updated calculation
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 100
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT_V1 4400  // version 1
#define BCOEFFICIENT_V2 3977  // version 2, updated calculation
// the value of the 'other' resistor
#define SERIESRESISTOR_V1 6960
#define SERIESRESISTOR_V2 6190  // version 2, measured on board
//scaling value to convert voltage
#define VOLTAGESCALE 12.1
//reference voltage
//#define VOLTAGEREF 4.585
#define VOLTAGEREF 4.16

#define LMREF  5.07 //measured from LMBoard --- GND Board

//Top Level Variables:
int DEBUG = 1;  //Set to 1 to enable serial monitor debugging info

//Switch Variables - "sw1"
int state_Manual_Switch = 0;                   //The actual ~state~ of the state machine
int state_prev_Manual_Switch = 0;              //Remembers the previous state (useful to know when the state has changed)
int pin_Manual_Switch = 3;                    //Input/Output (IO) pin for the switch
int value_Manual_Switch = 0;                     //Value of the switch ("HIGH" or "LOW")
unsigned long t_Manual_Switch = 0;             //Typically represents the current time of the switch
unsigned long t_0_Manual_Switch = 0;           //The time that we last passed through an interesting state
unsigned long bounce_delay_Manual_Switch = 20; //The delay to list for bouncing

//Switch Variables - "Reed_Switch"
int state_Reed_Switch = 0;                   //The actual ~state~ of the state machine
int pin_Reed_Switch = 3;                    //Input/Output (IO) pin for the switch
int value_Reed_Switch = 0;                     //Value of the switch ("HIGH" or "LOW")
unsigned long t_Reed_Switch = 0;             //Typically represents the current time of the switch
unsigned long t_0_Reed_Switch = 0;           //The time that we last passed through an interesting state
unsigned long bounce_delay_Reed_Switch = 5; //The delay to filter bouncing
unsigned int bin_counter = 0; //binary counter for reed switch
int virtual_Reed_Switch = VIRT_REED_SWITCH_OFF; // virtual switch

//SM "Volt Meter" variables
int state_Volt_Meter = 0;
unsigned long t_Volt_Meter = 0;
unsigned long t_0_Volt_Meter = 0;
unsigned long delay_Between_2_Measures = 300;

//Display variables
int state_display1 = 0;                   //The actual ~state~ of the state machine
int state_prev_display1 = 0;              //Remembers the previous state (useful to know when the state has changed)

//SM Counter variables
int state_counter1 = 0;                   //The actual ~state~ of the state machine
int prev_state_counter1 = 0;              //Remembers the previous state (useful to know when the state has changed)
int iSecCounter1 = -1;
int prev_iSecCounter1 = 0;
int iMinCounter1 = -1;
int prev_iMinCounter1 = 0;
unsigned long start_counter1 = 0;
unsigned long elapsed_counter1 = 0;

//SM Status LED Variables
int state_Status_Led = 0;
int state_prev_Status_Led = 0;
int pin_Status_Led = LED_BUILTIN;
int val_Status_Led = 0;
unsigned long t_Status_Led = 0;
unsigned long t_0_Status_Led = 0;
unsigned long on_delay_Status_Led = 500;
unsigned long off_delay_Status_Led = 500;
int beep_count_Status_Led = 0;
int beep_number_Status_Led = 2;

// Setup MAX7219 LED Matrix
// pin 12 is connected to the DataIn on the display
// pin 11 is connected to the CLK on the display
// pin 10 is connected to LOAD on the display
LedControl lc = LedControl(12, 11, 10, 1); //sets the 3 control pins as 12, 11 & 10 and then sets 1 display
byte intensity = 1;
byte bright_intensity = 8;
byte dimm_intensity = 1;

char TimeCounterStr[] = "00:00"; /** String to store time counter value, format: MM:SS */

//Set up OLED display
SSD1306AsciiWire oled_display;
//SSD1306AsciiAvrI2c oled_display; /* Use only when no other I2C devices are used! */

// Current and voltage sensor class
Adafruit_INA219 ina219_monitor;
#define INA219_VCC_PIN 3
#define INA219_GND_PIN 2

// INA219 sensor variables
float bus_Voltage_V;    /** Measured bus voltage in V*/
float bus_Voltage_mV;    /** Measured bus voltage in mV*/
char volt_String[] = "99999.9";         /** String to store measured voltage value in mV */

// Calculated temperature
float calc_Temperature_V1 = 0.0;
char temperature_String_V1[] = "  999.9";
float steinhart = 0.0;
float calc_Temperature_V2 = 0.0;
char temperature_String_V2[] = "  999.9";
float thermistor_Res = 0.00; // Thermistor calculated resistance

//HT16K33 LED Matrix display
//Define lpaseen library class
HT16K33 HT;
//Define Adafruit library class
//Adafruit_8x16minimatrix matrix = Adafruit_8x16minimatrix();
Adafruit_X_Mirrored_8x16minimatrix matrix = Adafruit_X_Mirrored_8x16minimatrix();

/* Function declarations */

void StateMachine_counter1();
void StateMachine_Manual_Switch();
void StateMachine_Reed_Switch();
void StateMachine_Status_Led();
void StateMachine_Volt_Meter();
void clear_Display_Max7219();
void bright_Display_Max7219(int dnum2disp);
void dimm_Display_Max7219(int dnum2disp);
void disp2digit_on_5x7(int d_address, int num2disp, int d_intensity);
void puttinydigit3x5l(int address, byte x, byte y, char c);
void disp_MinsAsColumn(int dispMinutes, byte dispCol);
void update_TimeCounterStr(int tMinutes, int tSeconds);
void display_on_oled(int dispMinutes, int dispSeconds);

void Gpio_Init(void);
void Ssd1306_Oled_Init(void);
void Max7219_Led_Matrix_Init(void);
void display_Timer_On_All(boolean need_Display_Clear,boolean need_Display_Stopped);
void display_Timer_On_Ssd1306(boolean need_Display_Clear,boolean need_Display_Stopped);
void display_Timer_On_Max7219(boolean need_Display_Clear,boolean need_Display_Stopped);

void ina219_Init(void);
void get_Voltage(void);
void calculate_Temperature_V2(void);

void Ht16k33_Led_Matrix_Init(void);
void Adafruit_Text_Display_Test(void);

/* Functions */

void setup() {
  //if DEBUG is turned on, intialize serial connection
  Serial.begin(115200);
  #ifdef SERIAL_DEBUG_ENABLED
  Serial.println(F("Debugging is ON"));
  #endif
  Gpio_Init();
  #ifdef SERIAL_MAX7219_ENABLED
  Max7219_Led_Matrix_Init();
  #endif
  Ssd1306_Oled_Init();
  ina219_Init();
  Ht16k33_Led_Matrix_Init();
  // SM inits
  StateMachine_counter1();
  StateMachine_Reed_Switch();
  StateMachine_Volt_Meter();

  Adafruit_Text_Display_Test();
  
}

void loop() {
    //Instruct all the state machines to proceed one step
  StateMachine_Reed_Switch();

  //Provide events that can force the state machines to change state
  switch (virtual_Reed_Switch) {
    case VIRT_REED_SWITCH_OFF:
      digitalWrite(pin_Status_Led, LOW);
      if (state_counter1 == COUNTER_STATE_COUNTING) {
        state_counter1 = COUNTER_STATE_STOP;
      }
      break;
    case VIRT_REED_SWITCH_ON:
      digitalWrite(pin_Status_Led, HIGH);
      if (state_counter1 == COUNTER_STATE_DISABLED) {
        state_counter1 = COUNTER_STATE_START;
      }
      break;

  }
  StateMachine_counter1();
  StateMachine_Volt_Meter();

}

/**
* @brief Counter 1 state machine - counts the seconds
*/
void StateMachine_counter1() {

  prev_state_counter1 = state_counter1;

  //State Machine Section
  switch (state_counter1) {
    case COUNTER_STATE_RESET:
      iSecCounter1 = 0;
      iMinCounter1 = 0;
      elapsed_counter1 = 0;
      state_counter1 = COUNTER_STATE_DISABLED;
      break;
    case COUNTER_STATE_DISABLED:
      //waiting for START event
      break;
    case COUNTER_STATE_START:
      iSecCounter1 = 0;
      iMinCounter1 = 0;
      elapsed_counter1 = 0;
      start_counter1 = millis();
      state_counter1 = COUNTER_STATE_COUNTING;
      display_Timer_On_All(DISPLAY_CLEAR_TRUE,DISPLAY_STOPPED_FALSE);
      break;
    case COUNTER_STATE_COUNTING:
      prev_iSecCounter1 = iSecCounter1;
      elapsed_counter1 = millis() - start_counter1;
      iSecCounter1 = int ((elapsed_counter1 / 1000) % 60);
      iMinCounter1 = int ((elapsed_counter1 / 1000) / 60);
      if (iSecCounter1 != prev_iSecCounter1) {
        // debug display
        #ifdef SERIAL_DEBUG_ENABLED
        Serial.print(F("iMinCounter1: "));
        Serial.print(iMinCounter1, DEC);
        Serial.print(F(" iSecCounter1: "));
        Serial.println(iSecCounter1, DEC);
        #endif        
        display_Timer_On_All(DISPLAY_CLEAR_FALSE,DISPLAY_STOPPED_FALSE);
      }
      break;
    case COUNTER_STATE_STOP:
      state_counter1 = COUNTER_STATE_DISABLED;
      display_Timer_On_All(DISPLAY_CLEAR_FALSE,DISPLAY_STOPPED_TRUE);
      break;
  }
  if (prev_state_counter1 == state_counter1)
  {
    //do nothing
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
void StateMachine_Manual_Switch() {

  //Common code for every state
  value_Manual_Switch = digitalRead(pin_Manual_Switch);
  state_prev_Manual_Switch = state_Manual_Switch;

  //State Machine Section
  switch (state_Manual_Switch) {
    case 0: //RESET!
      // state_Manual_Switch variable initialization
      state_Manual_Switch = 1;
      break;

    case 1: //OFF, wait for on
      //Wait for the pin to go low (switch=ON)
      if (value_Manual_Switch == SW_ON_LEVEL) {
        state_Manual_Switch = 2;
      }
      break;

    case 2: //OFF, arming to ON
      //Start debounce timer and proceed to state3, "OFF, armed to ON"
      t_0_Manual_Switch = millis();
      state_Manual_Switch = 3;
      break;

    case 3: //OFF, armed to ON
      //Check to see if debounce delay has passed and switch still ON
      t_Manual_Switch = millis();
      if (t_Manual_Switch - t_0_Manual_Switch > bounce_delay_Manual_Switch && value_Manual_Switch == SW_ON_LEVEL) {
        state_Manual_Switch = 4;
      }
      break;

    case 4: //Switched ON
      state_Manual_Switch = 5;
      break;

    case 5: //ON, wait for off
      //Wait for the pin to go high (switch=OFF)
      if (value_Manual_Switch == SW_OFF_LEVEL) {
        state_Manual_Switch = 6;
      }
      break;

    case 6: //ON, arming to OFF
      //Start debounce timer and proceed to state7, "OFF, armed to ON"
      t_0_Manual_Switch = millis();
      state_Manual_Switch = 7;
      break;

    case 7: //ON, armed to OFF
      //Check to see if debounce delay has passed and switch still OFF
      t_Manual_Switch = millis();
      if (t_Manual_Switch - t_0_Manual_Switch > bounce_delay_Manual_Switch && value_Manual_Switch == SW_OFF_LEVEL) {
        state_Manual_Switch = 8;
      }
      break;
    case 8: //Switched OFF
      //go back to State1
      state_Manual_Switch = 1;
      break;
  }

}

/**
* @brief Reed switch state machine, extended with a logic switch which handles the 50Hz switching of reed switch on the solenoid
*/
void StateMachine_Reed_Switch() {
  int prev_virtual_Reed_Switch;
  //State Machine Section
  switch (state_Reed_Switch) {
    case REED_SWITCH_STATE_RESET: //RESET!
      // variables initialization
      bin_counter = 0;
      virtual_Reed_Switch = VIRT_REED_SWITCH_OFF;
      value_Reed_Switch = SW_OFF_LEVEL;
      state_Reed_Switch = REED_SWITCH_STATE_START_TIMER;
      break;

    case REED_SWITCH_STATE_START_TIMER: //Start SW timer
      //Start debounce timer and proceed to state3, "OFF, armed to ON"
      t_0_Reed_Switch = millis();
      state_Reed_Switch = REED_SWITCH_STATE_STOP_TIMER;
      break;

    case REED_SWITCH_STATE_STOP_TIMER: //Timer stop
      //Check to see if debounce delay has passed
      t_Reed_Switch = millis();
      if (t_Reed_Switch - t_0_Reed_Switch > bounce_delay_Reed_Switch) {
        state_Reed_Switch = REED_SWITCH_STATE_READ_PIN;
      }
      break;

    case REED_SWITCH_STATE_READ_PIN: //Read Switch pin
      value_Reed_Switch = digitalRead(pin_Reed_Switch);
      state_Reed_Switch = REED_SWITCH_STATE_ROTATE_BIN_COUNTER;
      break;
    case REED_SWITCH_STATE_ROTATE_BIN_COUNTER: //Rotate binary counter
      bin_counter = bin_counter << 1;
      if (value_Reed_Switch == SW_ON_LEVEL) {
        bin_counter++ ;
      }
      state_Reed_Switch = REED_SWITCH_STATE_SET_LOGIC_SWITCH;
      break;

    case REED_SWITCH_STATE_SET_LOGIC_SWITCH:
      prev_virtual_Reed_Switch=virtual_Reed_Switch;
      if (bin_counter > 0) {
        if (prev_virtual_Reed_Switch == VIRT_REED_SWITCH_OFF) {
          virtual_Reed_Switch = VIRT_REED_SWITCH_ON;
          // debug display
          #ifdef SERIAL_DEBUG_ENABLED
          Serial.println(F("Virtual Reed switch ON"));
          #endif
        }
        
      }
      else
      {
        if (prev_virtual_Reed_Switch == VIRT_REED_SWITCH_ON) {
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
void StateMachine_Status_Led() {
  //Common code for every state
  state_prev_Status_Led = state_Status_Led;
  //State Machine Section
  switch (state_Status_Led) {
    case 0: //RESET
      //Set Beep Count to 0 then proceed to WAIT
      beep_count_Status_Led = 0;
      state_Status_Led = 1;
      break;
    case 1: //WAIT
      //Do nothing.  Only the top level loop can force us out of this state into state 2 "TURN ON"
      break;

    case 2: //TURNING ON
      digitalWrite(pin_Status_Led, HIGH);
      // debug display
      #ifdef SERIAL_DEBUG_ENABLED
      Serial.println(F(":: LED ON"));
      #endif
      t_0_Status_Led = millis();
      state_Status_Led = 3;
      break;
    case 3: //ON
      //Wait for time to elapse, then proceed to TURN OFF
      t_Status_Led = millis();
      if (t_Status_Led - t_0_Status_Led > on_delay_Status_Led) {
        state_Status_Led = 4;
      }
      break;
    case 4: //TURNING OFF
      //Increment the beep counter, proceed to OFF
      beep_count_Status_Led++;
      t_0_Status_Led = millis();
      digitalWrite(pin_Status_Led, LOW);
      // debug display
      #ifdef SERIAL_DEBUG_ENABLED
      Serial.println(F(":: LED off"));
      #endif
      state_Status_Led = 5;
      break;
    case 5: //OFF
      t_Status_Led = millis();
      if (t_Status_Led - t_0_Status_Led > off_delay_Status_Led) {
        state_Status_Led = 2;
      }
      if (beep_count_Status_Led >= beep_number_Status_Led) {
        state_Status_Led = 0;
      }
      break;
  }

}

/**
* @brief Clears the dot matrix display(s)
*/ 
void clear_Display_Max7219() {
  int devices = lc.getDeviceCount();
  for (byte address = 0; address < devices; address++) {
    lc.clearDisplay(address);
  }  
}

/**
* @brief Displays the number on the LED display with "bright_intensity" brightness
* @param dnum2disp : number to display
*/
void bright_Display_Max7219(int dnum2disp) {
  disp2digit_on_5x7(0, dnum2disp, bright_intensity);
  // debug display
  #ifdef SERIAL_DEBUG_ENABLED
  Serial.print(F("Bright: "));
  Serial.println(dnum2disp);
  #endif
}

/**
* @brief Displays the number on the LED display with "dimm_intensity" brightness
* @param dnum2disp : number to display
*/
void dimm_Display_Max7219(int dnum2disp) {
  disp2digit_on_5x7(0, dnum2disp, dimm_intensity);
  // debug display
  #ifdef SERIAL_DEBUG_ENABLED
  Serial.print(F("Dimm: "));
  Serial.println(dnum2disp);
  #endif
}

/** 
* @brief Displays a 2 digits number in landscape mode on a 5x7 matrix
* 
* @param d_address : address of the LED matrix display
* @param num2disp : number to display
* @param d_intensity : LED intensity (=brightness) level
*/
void disp2digit_on_5x7(int d_address, int num2disp, int d_intensity) {
  char disp_ch_tens, disp_ch_ones;
  num2disp = num2disp % 100;
  disp_ch_tens = 0x30 + num2disp / 10;
  disp_ch_ones = 0x30 + num2disp % 10;
  lc.setIntensity(d_address, d_intensity);
  puttinydigit3x5l(d_address, 0, 0, disp_ch_tens);
  puttinydigit3x5l(d_address, 4, 0, disp_ch_ones);

}

/** 
* @brief Display tiny 3x5 digit in landscape mode on a 5x7 matrix
* 
* @param address : address of the LED matrix display
* @param x : X coordinate
* @param y : no Y coordinate can be set! (permanently zeroed currently)
* @param c : character to display (numbers only)
*/
void puttinydigit3x5l(int address, byte x, byte y, char c)
{
  byte dots,dot_col,char_selector;
  y = 0;
  if (c >= '0' && c <= '9') {
    char_selector = (c - '0');
    for (dot_col = 0; dot_col < 3; dot_col++) {
      dots = pgm_read_byte_near(&digit3x5landscape[char_selector][dot_col]);
      lc.setRow(address, dot_col + x, dots << 3);

    }
  }
}

void disp_MinsAsColumn(int dispMinutes, byte dispCol) {
  int address=0;
  byte columnBits;
  dispMinutes = dispMinutes % 10;
  switch (dispMinutes) {
    case 0:
      columnBits=0;
      break;
    case 1:
      columnBits=B00000001;
      columnBits=B10000000;
      break;
    case 2:
      columnBits=B00000011;
      columnBits=B11000000;
      break;
    case 3:
      columnBits=B00000111;
      columnBits=B11100000;
      break;
    case 4:
      columnBits=B00001111;
      columnBits=B11110000;
      break;
    case 5:
      columnBits=B00011111;
      columnBits=B11111000;
      break;
    case 6:
      columnBits=B00010000;
      columnBits=B00001000;
      break;
    case 7:
      columnBits=B00011000;
      columnBits=B00011000;
      break;
    case 8:
      columnBits=B00011100;
      columnBits=B00111000;
      break;
    case 9:
      columnBits=B00011110;
      columnBits=B01111000;
      break;
  }
  lc.setRow(address,dispCol,columnBits);
}

/**
* @brief Converts the minutes and seconds to char and updates the TimeCounterStr string
* @param tMinutes : minutes value
* @param tSeconds : seconds value
*/ 
void update_TimeCounterStr(int tMinutes, int tSeconds) {
  TimeCounterStr[0] = (char) ((tMinutes / 10)+0x30);
  TimeCounterStr[1] = (char) ((tMinutes % 10)+0x30);
  TimeCounterStr[3] = (char) ((tSeconds / 10)+0x30);
  TimeCounterStr[4] = (char) ((tSeconds % 10)+0x30);
}

/**
* @brief Dsiplays the TimeCounterStr string on the OLED screen, format: MM:SS
* @param dispMinutes : minutes value
* @param dispSeconds : seconds value
*/ 
void display_on_oled(int dispMinutes, int dispSeconds) {
  update_TimeCounterStr(dispMinutes,dispSeconds);
  oled_display.setCol(0);
  oled_display.setRow(0);
  oled_display.println(TimeCounterStr);  
}

void Gpio_Init(void) {
  pinMode(pin_Manual_Switch, INPUT_PULLUP); //INPUT => reverse logic!
  pinMode(pin_Reed_Switch, INPUT_PULLUP); //INPUT => reverse logic!
  pinMode(pin_Status_Led, OUTPUT);
}

void Ssd1306_Oled_Init(void) {
  Wire.begin();
  oled_display.begin(&Adafruit128x32, OLED_I2C_ADDR);
  oled_display.clear();
  oled_display.setFont(fixed_bold10x15);
  oled_display.setRow(0);
  oled_display.println(F("Brew Timer "));
  delay(1500);
  oled_display.clear();
  display_on_oled(0,0);
}

void Max7219_Led_Matrix_Init(void) {
  //we have already set the number of devices when we created the LedControl
  int devices = lc.getDeviceCount();
  //we have to init all devices in a loop
  for (int address = 0; address < devices; address++) {
    /*The MAX72XX is in power-saving mode on startup*/
    lc.shutdown(address, false);
    /* Set the brightness to a medium values */
    lc.setIntensity(address, dimm_intensity);
    /* and clear the display */
    lc.clearDisplay(address);
  }
  lc.setIntensity(0, dimm_intensity);
  disp2digit_on_5x7(0, 0, dimm_intensity);
}

void display_Timer_On_All(boolean need_Display_Clear,boolean need_Display_Stopped) {
  #ifdef SERIAL_MAX7219_ENABLED
  display_Timer_On_Max7219(need_Display_Clear,need_Display_Stopped);
  #endif
  display_Timer_On_Ssd1306(need_Display_Clear,need_Display_Stopped);
}

void display_Timer_On_Ssd1306(boolean need_Display_Clear,boolean need_Display_Stopped) {
  if(need_Display_Clear) {
    oled_display.clear();
  }
  display_on_oled(iMinCounter1,iSecCounter1);
  if(need_Display_Stopped) {
    oled_display.print(F("stopped"));
  }          
}

void display_Timer_On_Max7219(boolean need_Display_Clear,boolean need_Display_Stopped) {
  if(need_Display_Clear) {
    clear_Display_Max7219();
  }
  if(need_Display_Stopped) {
    dimm_Display_Max7219(iSecCounter1);
  }
  else {
    bright_Display_Max7219(iSecCounter1);
  }
  disp_MinsAsColumn(iMinCounter1,3);
}

void ina219_Init(void)
{
  //pinMode(INA219_GND_PIN, OUTPUT);
  //pinMode(INA219_VCC_PIN, OUTPUT);
  //digitalWrite(INA219_GND_PIN, LOW);
  //digitalWrite(INA219_VCC_PIN, HIGH);
  delay(100);
  ina219_monitor.begin();
}

void get_Voltage(void)
{
  //measure voltage and current
  bus_Voltage_V = (ina219_monitor.getBusVoltage_V());
  bus_Voltage_mV = bus_Voltage_V*1000;
  //convert to text
  dtostrf(bus_Voltage_mV, 7, 1, volt_String);
  // debug display
  #ifdef SERIAL_DEBUG_ENABLED
  Serial.print(volt_String);
  Serial.println(F(" mV"));
  #endif
}

void calculate_Temperature_V2(void) {
  thermistor_Res = SERIESRESISTOR_V2 * (1/((LMREF / bus_Voltage_V) -1));
  steinhart = thermistor_Res / THERMISTORNOMINAL_V2;
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT_V2;                   // 1/B * ln(R/Ro)
  steinhart += (1.0 / (TEMPERATURENOMINAL + 273.15)); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  calc_Temperature_V2 = (float) steinhart - 273.15;                         // convert to C
  dtostrf(calc_Temperature_V2, 7, 1, temperature_String_V2);
  // debug display
  #ifdef SERIAL_DEBUG_ENABLED
  Serial.print(temperature_String_V1);
  Serial.println(F(" *C"));
  #endif
}

void StateMachine_Volt_Meter(void) {
  
  switch (state_Volt_Meter) {
    case VOLT_METER_STATE_RESET:
      state_Volt_Meter = VOLT_METER_STATE_START_TIMER;
      break;

    case VOLT_METER_STATE_START_TIMER:
      t_0_Volt_Meter = millis();
      state_Volt_Meter = VOLT_METER_STATE_STOP_TIMER;
      break;

    case VOLT_METER_STATE_STOP_TIMER:
      t_Volt_Meter = millis();
      if (t_Volt_Meter - t_0_Volt_Meter > delay_Between_2_Measures) {
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
      break;
  } 
}

void Ht16k33_Led_Matrix_Init(void) {
  HT.begin(0x70);
  HT.setBrightness(1);
  HT.clearAll();
  matrix.begin(0x70);
  matrix.setFont(&Picopixel);
}

void Adafruit_Text_Display_Test(void) {
  //int8_t x,y;
  matrix.setTextSize(1);
  matrix.setTextWrap(false);
  /**
  Serial.println(F("rotation=0"));
  matrix.setRotation(0);
  matrix.setTextColor(LED_ON);
  matrix.clear();
  matrix.setCursor(0,0);
  matrix.print("123");
  matrix.writeDisplay();
  delay(5000);
  */
  Serial.println(F("rotation=1"));
  matrix.setRotation(1);
  matrix.setTextColor(LED_ON);
  matrix.clear();
  matrix.setCursor(0,7);
  matrix.print("0:00");
  matrix.writeDisplay();
  delay(5000);
  matrix.clear();
  matrix.setCursor(0,7);
  matrix.print("1:00");
  matrix.writeDisplay();
  delay(5000);
  matrix.clear();
  matrix.setCursor(0,7);
  matrix.print("100.0*");
  matrix.writeDisplay();
  delay(5000);
  for(int i=50;i<120;i++) {
    matrix.clear();
    matrix.setCursor(0,7);
    matrix.print(i);
    matrix.writeDisplay();
    delay(200);    
  }
  matrix.setRotation(0);
  
}