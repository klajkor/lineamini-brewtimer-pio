#pragma once

#include <Wire.h>
#include <math.h>

// uncomment the line below if you would like to have debug messages
//#define SERIAL_DEBUG_ENABLED 1

// uncomment the line below if you would like to use MAX7219 LED matrix
//#define MAX7219_ENABLED 1

// uncomment the line below if you would like to use HT16K33 LED matrix
#define HT16K33_ENABLED 1

// uncomment the line below if you would like to use SSD1306 OLED display
#define SSD1306_ENABLED 1

extern int  iSecCounter1;
extern int  iMinCounter1;
extern char TimeCounterStr[];
extern char SecondsCounterStr[];

/* Function declarations */

void setup(void);
void loop(void);
void StateMachine_counter1(void);
void StateMachine_Reed_Switch(void);
void StateMachine_Status_Led(void);
void update_TimeCounterStr(int tMinutes, int tSeconds);
void Gpio_Init(void);