#pragma once

#include <Arduino.h>

#include "main.h"
#include "temperature_meter.h"

#ifdef MAX7219_ENABLED
#include "display_max7219_ledmatrix.h"
#endif // #ifdef MAX7219_ENABLED

#ifdef SSD1306_ENABLED
#include "display_ssd1306.h"
#endif

#ifdef HT16K33_ENABLED
#include "display_ht16k33.h"
#endif

#define DISPLAY_CLEAR_TRUE true
#define DISPLAY_CLEAR_FALSE false
#define DISPLAY_STOPPED_TRUE true
#define DISPLAY_STOPPED_FALSE false

#define DISPLAY_STATE_RESET 0
#define DISPLAY_STATE_TIMER_RUNNING 1
#define DISPLAY_STATE_TIMER_STOPPED 2
#define DISPLAY_STATE_HOLD_TIMER_ON 3
#define DISPLAY_STATE_TEMPERATURE 4
#define DISPLAY_STATE_DO_NOTHING 5

// State Machine Display global variables
extern int state_Display;
// extern unsigned long t_Display;
// extern unsigned long t_0_Display;
// extern unsigned long delay_For_Stopped_Timer;

void Display_Init(void);
void StateMachine_Display(void);
void Display_Timer_On_All(boolean need_Display_Clear, boolean need_Display_Stopped);
void Display_Running_Timer(void);
void Display_Stopped_Timer(void);
void Display_Temperature(void);
void Display_Clear(void);
