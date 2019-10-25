/**
* Arduino Brew Timer for LaMarzocco Linea Mini espresso machine
*
*
* @author Robert Klajko
* @url https://github.com/klajkor/lineamini-brewtimer-pio.git
*
* Board: Arduino Pro Mini/Nano
*
* Extension modules and hw used:
*  - MAX7219, I2C
*  - SSD1306 OLED dsiplay, I2C
*  - 3x5 digits on 5x7 dot matrix
*  - reed switch, sensing magnet valve sc
* Libraries used:
*  - LedControl by wayoda - Copyright (c) 2012, Eberhard Fahle
*  - SSD1306Ascii by Bill Greiman - Copyright (c) 2019, Bill Greiman
*
* BSD license, all text here must be included in any redistribution.
*
*/

#include "LedControl.h"
#include "TinyDigit.h"

#define LED_VERT_OFFSET 2
#define LED_HOR_OFFSET 1
#define LED_DIGIT_OFFSET 4

#define SW_ON_LEVEL LOW  /* Switch on level */
#define SW_OFF_LEVEL HIGH  /* Switch off level */

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
unsigned long hold_delay_sw1 = 1000; //The time required to register a long press (a.k.a. "hold")

//Switch Variables - "sw2"
int state_sw2 = 0;                   //The actual ~state~ of the state machine
int state_prev_sw2 = 0;              //Remembers the previous state (useful to know when the state has changed)
int pin_sw2 = 3;                    //Input/Output (IO) pin for the switch
int value_sw2 = 0;                     //Value of the switch ("HIGH" or "LOW")
unsigned long t_sw2 = 0;             //Typically represents the current time of the switch
unsigned long t_0_sw2 = 0;           //The time that we last passed through an interesting state
unsigned long bounce_delay_sw2 = 5; //The delay to list for bouncing
unsigned int bin_counter = 0; //binary counter
int logic_sw2 = 0; // logic switch


//Display variables
int state_display1 = 0;                   //The actual ~state~ of the state machine
int state_prev_display1 = 0;              //Remembers the previous state (useful to know when the state has changed)

//Counter variables
int state_counter1 = 0;                   //The actual ~state~ of the state machine
int prev_state_counter1 = 0;              //Remembers the previous state (useful to know when the state has changed)
int icounter1 = -1;
int prev_icounter1 = 0;
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

// Setup LED Matrix
// pin 12 is connected to the DataIn on the display
// pin 11 is connected to the CLK on the display
// pin 10 is connected to LOAD on the display
LedControl lc = LedControl(12, 11, 10, 1); //sets the 3 pins as 12, 11 & 10 and then sets 4 displays (max is 8 displays)
byte intensity = 1;
byte bright_intensity = 8;
byte dimm_intensity = 1;

/* Function declarations */

void StateMachine_counter1();
void StateMachine_display1();
void StateMachine_sw1();
void StateMachine_sw2();
void StateMachine_led1();
void clear_display1();
void bright_display1(int dnum2disp);
void dimm_display1(int dnum2disp);
void disp2digit_on_5x7(int d_address, int num2disp, int d_intensity);
void puttinydigit3x5l(int address, byte x, byte y, char c);

/* Functions */

/**
* @brief Counter 1 state machine - counts the seconds
*/
void StateMachine_counter1() {

  prev_state_counter1 = state_counter1;

  //State Machine Section
  switch (state_counter1) {
    case 0: //RESET!
      icounter1 = 0;
      elapsed_counter1 = 0;
      state_counter1 = 1;
      if (DEBUG) {
        //Serial.println(icounter1);
      }
      break;
    case 1: //Disabled
      //state_counter1 = 2;
      break;
    case 2: //Start
      icounter1 = 0;
      elapsed_counter1 = 0;
      start_counter1 = millis();
      state_counter1 = 3;
      clear_display1();
      bright_display1(0);
      break;
    case 3: //Counting
      prev_icounter1 = icounter1;
      elapsed_counter1 = millis() - start_counter1;
      icounter1 = int ((elapsed_counter1 / 1000) % 60);
      if (icounter1 != prev_icounter1) {
        if (DEBUG) {
          Serial.print(F("icounter1: "));
          Serial.println(icounter1, DEC);
        }
        bright_display1(icounter1);
      }
      break;
    case 4: //Stop
      state_counter1 = 1;
      dimm_display1(icounter1);
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

void StateMachine_display1() {

  state_prev_display1 = state_display1;

  //State Machine Section
  switch (state_display1) {
    case 0: //RESET!
      state_display1 = 1;
      break;
    case 1: //Display clear
      //state_display1 = 2;
      break;
    case 2: //Bright display
      //state_display1 = 3;
      break;
    case 3: //Dimm display
      //state_display1 = 4;
      break;
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

void bright_display1(int dnum2disp) {
  disp2digit_on_5x7(0, dnum2disp, bright_intensity);
  if (DEBUG) {
    //Serial.print("Bright: ");
    //Serial.println(dnum2disp);
  }
}

void dimm_display1(int dnum2disp) {
  disp2digit_on_5x7(0, dnum2disp, dimm_intensity);
  if (DEBUG) {
    //Serial.print("Dimm: ");
    //Serial.println(dnum2disp);
  }
}




void StateMachine_sw1() {

  //Almost every state needs these lines, so I'll put it outside the State Machine
  value_sw1 = digitalRead(pin_sw1);
  state_prev_sw1 = state_sw1;

  //State Machine Section
  switch (state_sw1) {
    case 0: //RESET!
      //Catch all "home base" for the State Machine
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
        Serial.println("State4, switched ON");
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
        Serial.println("State8, switched OFF");
      }
      //go back to State1
      state_sw1 = 1;
      break;
  }

}

void StateMachine_sw2() {

  //Almost every state needs these lines, so I'll put it outside the State Machine
  state_prev_sw2 = state_sw2;

  //State Machine Section
  switch (state_sw2) {
    case 0: //RESET!
      bin_counter = 0;
      logic_sw2 = 0;
      value_sw2 = SW_OFF_LEVEL;
      state_sw2 = 1;
      break;

    case 1: //Start SW timer
      //Start debounce timer and proceed to state3, "OFF, armed to ON"
      t_0_sw2 = millis();
      state_sw2 = 2;
      break;

    case 2: //Timer stop
      //Check to see if debounce delay has passed
      t_sw2 = millis();
      if (t_sw2 - t_0_sw2 > bounce_delay_sw2) {
        state_sw2 = 3;

      }
      break;

    case 3: //Read SW pin
      value_sw2 = digitalRead(pin_sw2);
      state_sw2 = 4;
      break;
    case 4: //Rotate
      bin_counter = bin_counter << 1;
      if (value_sw2 == SW_ON_LEVEL) {
        bin_counter++ ;
      }
      if (DEBUG) {
        //Serial.print("bin_counter:::::: ");
        //Serial.println(bin_counter, BIN);
      }
      state_sw2 = 5;
      break;

    case 5: //Logic SW
      if (bin_counter > 0) {
        logic_sw2 = 1;
        if (DEBUG) {
          //Serial.println("SM-SW2: logic sw set ON");
        }
      }
      else
      {
        logic_sw2 = 0;
        if (DEBUG) {
          //Serial.println("SM-SW2: logic sw set OFF");
        }
      }
      state_sw2 = 1;

      break;

  }

}


void StateMachine_led1() {
  //Almost every state needs these lines so I'll put it outside the State Machine
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

    case 2: //TURN ON
      //Start buzzer, record time then proceed to ON,
      digitalWrite(pin_led1, HIGH);
      if (DEBUG) {
        Serial.println(":: LED ON");
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
    case 4: //TURN OFF
      //Increment the beep counter, turn off buzzer, proceed to OFF
      beep_count_led1++;
      t_0_led1 = millis();
      digitalWrite(pin_led1, LOW);
      if (DEBUG) {
        Serial.println(":: LED off");
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

void disp2digit_on_5x7(int d_address, int num2disp, int d_intensity) {
  char disp_ch_tens, disp_ch_ones;
  num2disp = num2disp % 100;
  disp_ch_tens = 0x30 + num2disp / 10;
  disp_ch_ones = 0x30 + num2disp % 10;
  lc.setIntensity(d_address, d_intensity);
  puttinydigit3x5l(d_address, 0, 0, disp_ch_tens);
  puttinydigit3x5l(d_address, 4, 0, disp_ch_ones);

}

// put tiny 3x5 digit on a 5x7 matrix in landscape mode
// no Y coordinate can be set! (permanently zeroed)
void puttinydigit3x5l(int address, byte x, byte y, char c)
{
  byte dots;
  y = 0;
  if (c >= '0' && c <= '9') {
    c = (c - '0');
    for (byte col = 0; col < 3; col++) {
      dots = pgm_read_byte_near(&digit3x5landscape[c][col]);
      lc.setRow(address, col + x, dots << 3);

    }
  }
}

void setup() {
  Serial.begin(115200);
  // initialize digital pins
  pinMode(pin_sw1, INPUT_PULLUP); //INPUT => reverse logic!
  pinMode(pin_sw2, INPUT_PULLUP); //INPUT => reverse logic!
  pinMode(pin_led1, OUTPUT);

  //initialize the 4 matrix panels
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
  //if DEBUG is turned on, intialize serial connection
  if (DEBUG) {
    Serial.println("Debugging is ON");
  }
  lc.setIntensity(0, 15);
  disp2digit_on_5x7(0, 3, bright_intensity);
  delay(1000);
  lc.setIntensity(0, dimm_intensity);
  disp2digit_on_5x7(0, 0, dimm_intensity);
  // SM inits
  StateMachine_counter1();
  //StateMachine_sw1();
  StateMachine_sw2();

}

void loop() {
  //Instruct all the state machines to proceed one step (sometimes called a "cycle)
  //StateMachine_sw1();
  StateMachine_sw2();

  //Provide events that can force the state machines to change state
  switch (logic_sw2) {
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
  StateMachine_display1();

}
