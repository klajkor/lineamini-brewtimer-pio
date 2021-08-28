#pragma once

#include "Adafruit_INA219.h"
#include "main.h"
#include <Arduino.h>

#define VOLT_METER_STATE_RESET 0
#define VOLT_METER_STATE_START_TIMER 1
#define VOLT_METER_STATE_STOP_TIMER 2
#define VOLT_METER_STATE_READ_VOLTAGE 3
#define VOLT_METER_STATE_CONVERT_TO_TEMPERATURE 4

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

// Global temperature strings
extern char *temperature_str_V2;
extern char *temperature_str_V2_Led_Matrix;

void  ina219_Init(void);
float get_Voltage(void);
void  StateMachine_Volt_Meter(void);
void  calculate_Temperature_V2(float thermistor_voltage);
