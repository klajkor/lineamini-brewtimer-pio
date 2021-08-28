#include "display.h"

// State Machine Display Global variables
int state_Display = 0;

// State Machine Display Local variables
static unsigned long t_Display = DISPLAY_STATE_RESET;
static unsigned long t_0_Display = 0;
static unsigned long delay_For_Stopped_Timer = 5000; // millisec

void Display_Init(void)
{
#ifdef MAX7219_ENABLED
    max7219_led_matrix_init();
#endif
#ifdef SSD1306_ENABLED
    Ssd1306_Oled_Init();
#endif
#ifdef HT16K33_ENABLED
    Ht16k33_Led_Matrix_Init();
#endif
}

void Display_Timer_On_All(boolean need_Display_Clear, boolean need_Display_Stopped)
{
    update_TimeCounterStr(iMinCounter1, iSecCounter1);
#ifdef MAX7219_ENABLED
    display_Timer_On_Max7219(iSecCounter1, need_Display_Clear, need_Display_Stopped);
#endif
#ifdef SSD1306_ENABLED
    display_Timer_On_Ssd1306(SecondsCounterStr, need_Display_Clear, need_Display_Stopped);
#endif
#ifdef HT16K33_ENABLED
    display_Timer_On_Ht16k33(SecondsCounterStr, need_Display_Clear, need_Display_Stopped);
#endif
}

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
    Display_Timer_On_All(DISPLAY_CLEAR_FALSE, DISPLAY_STOPPED_FALSE);
}

void Display_Stopped_Timer(void)
{
    Display_Timer_On_All(DISPLAY_CLEAR_FALSE, DISPLAY_STOPPED_TRUE);
}

void Display_Temperature(void)
{
#ifdef SSD1306_ENABLED
    display_Temperature_On_Ssd1306(temperature_str_V2);
#endif
#ifdef HT16K33_ENABLED
    // display_Temperature_On_Ht16k33(temperature_str_V2_Led_Matrix);
#endif
}

void Display_Clear(void)
{
#ifdef SSD1306_ENABLED
    oled_ssd1306_display.clear();
#endif
}
