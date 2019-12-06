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
* Libraries used:
*  - SSD1306Ascii by Bill Greiman - Copyright (c) 2019, Bill Greiman
*  - LedControl by wayoda - Copyright (c) 2012, Eberhard Fahle
*
* BSD license, all text here must be included in any redistribution.
*
*/

#include <Wire.h>
#include "LedControl.h"
#include "TinyDigit.h"
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
//#include "SSD1306AsciiAvrI2c.h" /* Use only when no other I2C devices are used! */

#define LED_VERT_OFFSET 2
#define LED_HOR_OFFSET 1
#define LED_DIGIT_OFFSET 4

#define SW_ON_LEVEL LOW  /* Switch on level */
#define SW_OFF_LEVEL HIGH  /* Switch off level */

#define OLED_I2C_ADDR 0x3C /* OLED module I2C address */

//Top Level Variables:
int DEBUG = 1;  //Set to 1 to enable serial monitor debugging info

//Switch Variables - "sw1"
int state_sw1 = 0;                   //The actual ~state~ of the state machine
int state_prev_sw1 = 0;              //Remembers the previous state (useful to know when the state has changed)
int pin_sw1 = 3;                    //Input/Output (IO) pin for the switch
int value_sw1 = 0;                     //Value of the switch ("HIGH" or "LOW")
unsigned long t_sw1 = 0;             //Typically represents the current time of the switch
unsigned long t_0_sw1 = 0;           //The time that we last passed through an interesting state
unsigned long bounce_delay_sw1 = 20; //The delay to list for bouncing

//Switch Variables - "Reed_Switch"
int state_Reed_Switch = 0;                   //The actual ~state~ of the state machine
int state_prev_Reed_Switch = 0;              //Remembers the previous state (useful to know when the state has changed)
int pin_Reed_Switch = 3;                    //Input/Output (IO) pin for the switch
int value_Reed_Switch = 0;                     //Value of the switch ("HIGH" or "LOW")
unsigned long t_Reed_Switch = 0;             //Typically represents the current time of the switch
unsigned long t_0_Reed_Switch = 0;           //The time that we last passed through an interesting state
unsigned long bounce_delay_Reed_Switch = 5; //The delay to list for bouncing
unsigned int bin_counter = 0; //binary counter
int logic_Reed_Switch = 0; // logic switch


//Display variables
int state_display1 = 0;                   //The actual ~state~ of the state machine
int state_prev_display1 = 0;              //Remembers the previous state (useful to know when the state has changed)

//Counter variables
int state_counter1 = 0;                   //The actual ~state~ of the state machine
int prev_state_counter1 = 0;              //Remembers the previous state (useful to know when the state has changed)
int iSecCounter1 = -1;
int prev_iSecCounter1 = 0;
int iMinCounter1 = -1;
int prev_iMinCounter1 = 0;
unsigned long start_counter1 = 0;
unsigned long elapsed_counter1 = 0;

//LED Variables (Variables similar to those above)
int state_led1 = 0;
int state_prev_led1 = 0;
int pin_led1 = LED_BUILTIN;
int val_led1 = 0;
unsigned long t_led1 = 0;
unsigned long t_0_led1 = 0;
unsigned long on_delay_led1 = 500;
unsigned long off_delay_led1 = 500;
int beep_count_led1 = 0;
int beep_number_led1 = 2;

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

/* Function declarations */

void StateMachine_counter1();
void StateMachine_sw1();
void StateMachine_Reed_Switch();
void StateMachine_led1();
void clear_display1();
void bright_display1(int dnum2disp);
void dimm_display1(int dnum2disp);
void disp2digit_on_5x7(int d_address, int num2disp, int d_intensity);
void puttinydigit3x5l(int address, byte x, byte y, char c);
void disp_MinsAsColumn(int dispMinutes, byte dispCol);
void update_TimeCounterStr(int tMinutes, int tSeconds);
void display_on_oled(int dispMinutes, int dispSeconds);

void Gpio_Init(void);
void Ssd1306_Oled_Init(void);
void Max7219_Led_Matrix_Init(void);

/* Functions */

void setup() {
  //if DEBUG is turned on, intialize serial connection
  if (DEBUG) {
    Serial.begin(115200);
    Serial.println("Debugging is ON");
  }
  Gpio_Init();
  Max7219_Led_Matrix_Init();
  Ssd1306_Oled_Init();
    
  // SM inits
  StateMachine_counter1();
  //StateMachine_sw1();
  StateMachine_Reed_Switch();
  
}

void loop() {
    //Instruct all the state machines to proceed one step
  //StateMachine_sw1();
  StateMachine_Reed_Switch();

  //Provide events that can force the state machines to change state
  switch (logic_Reed_Switch) {
    case 0:
      digitalWrite(pin_led1, LOW);
      if (state_counter1 == 3)
      {
        state_counter1 = 4; // Stop counter
      }
      break;
    case 1:
      digitalWrite(pin_led1, HIGH);
      if (state_counter1 == 1)
      {
        state_counter1 = 2; // Start counter
      }
      break;

  }
  StateMachine_counter1();
  
}

/**
* @brief Counter 1 state machine - counts the seconds
*/
void StateMachine_counter1() {

  prev_state_counter1 = state_counter1;

  //State Machine Section
  switch (state_counter1) {
    case 0: //RESET!
      iSecCounter1 = 0;
      iMinCounter1 = 0;
      elapsed_counter1 = 0;
      state_counter1 = 1;
      if (DEBUG) {
        //Serial.println(iSecCounter1);
      }
      break;
    case 1: //Disabled
      //state_counter1 = 2;
      break;
    case 2: //Start
      iSecCounter1 = 0;
      iMinCounter1 = 0;
      elapsed_counter1 = 0;
      start_counter1 = millis();
      state_counter1 = 3;
      clear_display1();
      bright_display1(iSecCounter1);
      disp_MinsAsColumn(iMinCounter1,3);
      oled_display.clear();
      display_on_oled(iMinCounter1,iSecCounter1); /* displaying 00:00 on OLED screen */
      break;
    case 3: //Counting
      prev_iSecCounter1 = iSecCounter1;
      elapsed_counter1 = millis() - start_counter1;
      iSecCounter1 = int ((elapsed_counter1 / 1000) % 60);
      iMinCounter1 = int ((elapsed_counter1 / 1000) / 60);
      if (iSecCounter1 != prev_iSecCounter1) {
        if (DEBUG) {
          Serial.print(F("iMinCounter1: "));
          Serial.print(iMinCounter1, DEC);
          Serial.print(F(" iSecCounter1: "));
          Serial.println(iSecCounter1, DEC);
        }
        bright_display1(iSecCounter1);
        disp_MinsAsColumn(iMinCounter1,3);
        display_on_oled(iMinCounter1,iSecCounter1);
      }
      break;
    case 4: //Stop
      state_counter1 = 1;
      dimm_display1(iSecCounter1);
      disp_MinsAsColumn(iMinCounter1,3);
      oled_display.print(F("stopped"));
      break;
  }
  if (prev_state_counter1 == state_counter1)
  {
    //do nothing
  }
  else
  {
    if (DEBUG) {
      Serial.print(F("state_counter1: "));
      Serial.println(state_counter1, DEC);
    }

  }
}

/**
* @brief Clears the dot matrix display(s)
*/ 
void clear_display1() {
  for (byte address = 0; address < 2; address++) {
    lc.clearDisplay(address);
  }
  if (DEBUG) {
    //Serial.println("Clear display1");
  }
}

/**
* @brief Displays the number on the LED display with "bright_intensity" brightness
* @param dnum2disp : number to display
*/
void bright_display1(int dnum2disp) {
  disp2digit_on_5x7(0, dnum2disp, bright_intensity);
  if (DEBUG) {
    //Serial.print(F("Bright: "));
    //Serial.println(dnum2disp);
  }
}

/**
* @brief Displays the number on the LED display with "dimm_intensity" brightness
* @param dnum2disp : number to display
*/
void dimm_display1(int dnum2disp) {
  disp2digit_on_5x7(0, dnum2disp, dimm_intensity);
  if (DEBUG) {
    //Serial.print(F("Dimm: "));
    //Serial.println(dnum2disp);
  }
}

/**
* @brief Switch-1 state machine
*/
void StateMachine_sw1() {

  //Common code for every state
  value_sw1 = digitalRead(pin_sw1);
  state_prev_sw1 = state_sw1;

  //State Machine Section
  switch (state_sw1) {
    case 0: //RESET!
      // state_sw1 variable initialization
      state_sw1 = 1;
      break;

    case 1: //OFF, wait for on
      //Wait for the pin to go low (switch=ON)
      if (value_sw1 == SW_ON_LEVEL) {
        state_sw1 = 2;
      }
      break;

    case 2: //OFF, arming to ON
      //Start debounce timer and proceed to state3, "OFF, armed to ON"
      t_0_sw1 = millis();
      state_sw1 = 3;
      break;

    case 3: //OFF, armed to ON
      //Check to see if debounce delay has passed and switch still ON
      t_sw1 = millis();
      if (t_sw1 - t_0_sw1 > bounce_delay_sw1 && value_sw1 == SW_ON_LEVEL) {
        state_sw1 = 4;
      }
      break;

    case 4: //Switched ON
      if (DEBUG) {
        Serial.println(F("State4, SW1 switched ON"));
      }
      state_sw1 = 5;
      break;

    case 5: //ON, wait for off
      //Wait for the pin to go high (switch=OFF)
      if (value_sw1 == SW_OFF_LEVEL) {
        state_sw1 = 6;
      }
      break;

    case 6: //ON, arming to OFF
      //Start debounce timer and proceed to state7, "OFF, armed to ON"
      t_0_sw1 = millis();
      state_sw1 = 7;
      break;

    case 7: //ON, armed to OFF
      //Check to see if debounce delay has passed and switch still OFF
      t_sw1 = millis();
      if (t_sw1 - t_0_sw1 > bounce_delay_sw1 && value_sw1 == SW_OFF_LEVEL) {
        state_sw1 = 8;
      }
      break;
    case 8: //Switched OFF
      if (DEBUG) {
        Serial.println(F("State8, SW1 switched OFF"));
      }
      //go back to State1
      state_sw1 = 1;
      break;
  }

}

/**
* @brief Switch-2 state machine, extended with a logic switch which handles the 50Hz switching of reed switch on the solenoid
*/
void StateMachine_Reed_Switch() {

  //Common code for every state
  state_prev_Reed_Switch = state_Reed_Switch;

  //State Machine Section
  switch (state_Reed_Switch) {
    case 0: //RESET!
      // variables initialization
      bin_counter = 0;
      logic_Reed_Switch = 0;
      value_Reed_Switch = SW_OFF_LEVEL;
      state_Reed_Switch = 1;
      break;

    case 1: //Start SW timer
      //Start debounce timer and proceed to state3, "OFF, armed to ON"
      t_0_Reed_Switch = millis();
      state_Reed_Switch = 2;
      break;

    case 2: //Timer stop
      //Check to see if debounce delay has passed
      t_Reed_Switch = millis();
      if (t_Reed_Switch - t_0_Reed_Switch > bounce_delay_Reed_Switch) {
        state_Reed_Switch = 3;
      }
      break;

    case 3: //Read SW pin
      value_Reed_Switch = digitalRead(pin_Reed_Switch);
      state_Reed_Switch = 4;
      break;
    case 4: //Rotate binary counter
      bin_counter = bin_counter << 1;
      if (value_Reed_Switch == SW_ON_LEVEL) {
        bin_counter++ ;
      }
      if (DEBUG) {
        //Serial.print(F("bin_counter: "));
        //Serial.println(bin_counter, BIN);
      }
      state_Reed_Switch = 5;
      break;

    case 5: //Logic SW
      if (bin_counter > 0) {
        logic_Reed_Switch = 1;
        if (DEBUG) {
          //Serial.println(F("SM-Reed_Switch: logic switch set ON"));
        }
      }
      else
      {
        logic_Reed_Switch = 0;
        if (DEBUG) {
          //Serial.println("SM-Reed_Switch: logic switch set OFF");
        }
      }
      state_Reed_Switch = 1;

      break;

  }

}

/**
* @brief LED-1 state machine
*/
void StateMachine_led1() {
  //Common code for every state
  state_prev_led1 = state_led1;
  //State Machine Section
  switch (state_led1) {
    case 0: //RESET
      //Set Beep Count to 0 then proceed to WAIT
      beep_count_led1 = 0;
      state_led1 = 1;
      break;
    case 1: //WAIT
      //Do nothing.  Only the top level loop can force us out of this state into state 2 "TURN ON"
      break;

    case 2: //TURNING ON
      digitalWrite(pin_led1, HIGH);
      if (DEBUG) {
        Serial.println(F(":: LED ON"));
      }
      t_0_led1 = millis();
      state_led1 = 3;
      break;
    case 3: //ON
      //Wait for time to elapse, then proceed to TURN OFF
      t_led1 = millis();
      if (t_led1 - t_0_led1 > on_delay_led1) {
        state_led1 = 4;
      }
      break;
    case 4: //TURNING OFF
      //Increment the beep counter, proceed to OFF
      beep_count_led1++;
      t_0_led1 = millis();
      digitalWrite(pin_led1, LOW);
      if (DEBUG) {
        Serial.println(F(":: LED off"));
      }
      state_led1 = 5;
      break;
    case 5: //OFF
      t_led1 = millis();
      if (t_led1 - t_0_led1 > off_delay_led1) {
        state_led1 = 2;
      }
      if (beep_count_led1 >= beep_number_led1) {
        state_led1 = 0;
      }
      break;
  }

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
  //oled_display.set2X();
  oled_display.setCol(0);
  oled_display.setRow(0);
  oled_display.println(TimeCounterStr);  
}

void Gpio_Init(void) {
  pinMode(pin_sw1, INPUT_PULLUP); //INPUT => reverse logic!
  pinMode(pin_Reed_Switch, INPUT_PULLUP); //INPUT => reverse logic!
  pinMode(pin_led1, OUTPUT);
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