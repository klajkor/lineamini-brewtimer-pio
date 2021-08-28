#include "temperature_meter.h"

//#define SERIAL_DEBUG_ENABLED

// Current and voltage sensor class
static Adafruit_INA219 ina219_monitor;

// State Machine "Volt Meter" variables
static int           state_Volt_Meter = VOLT_METER_STATE_RESET;
static unsigned long t_Volt_Meter = 0;
static unsigned long t_0_Volt_Meter = 0;
static unsigned long delay_Between_2_Measures = 250;

// Calculated temperature
// static float calc_Temperature_V1 = 0.0;
// static char  temperature_String_V1[] = "  999.9";
static float steinhart = 0.0;
static float calc_Temperature_V2 = 0.0;
static float thermistor_Res = 0.00; // Thermistor calculated resistance

// Global temperature strings
char *temperature_str_V2 = "999.9";
char *temperature_str_V2_Led_Matrix = "99.9";

void ina219_Init(void)
{
    delay(10);
    ina219_monitor.begin();
}

float get_Voltage(void)
{
    float bus_Voltage_V;
    int   bus_Voltage_mV;
    char  volt_String[] = "99999"; /** String to store measured voltage value in mV */

    // measure voltage
    bus_Voltage_V = ina219_monitor.getBusVoltage_V();
    bus_Voltage_mV = (int)(bus_Voltage_V * 1000);
    // convert to text
    dtostrf(bus_Voltage_mV, 5, 0, volt_String);
// debug display
#ifdef SERIAL_DEBUG_ENABLED
    Serial.print(volt_String);
    Serial.println(F(" mV"));
#endif
    return bus_Voltage_V;
}

void StateMachine_Volt_Meter(void)
{
    float measured_voltage;

    switch (state_Volt_Meter)
    {
    case VOLT_METER_STATE_RESET:
        state_Volt_Meter = VOLT_METER_STATE_START_TIMER;
        measured_voltage = 0.0;
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
        measured_voltage = get_Voltage();
        state_Volt_Meter = VOLT_METER_STATE_CONVERT_TO_TEMPERATURE;
        break;

    case VOLT_METER_STATE_CONVERT_TO_TEMPERATURE:
        calculate_Temperature_V2(measured_voltage);
        state_Volt_Meter = VOLT_METER_STATE_START_TIMER;
        break;
    }
}

void calculate_Temperature_V2(float thermistor_voltage)
{
    thermistor_Res = SERIESRESISTOR_V2 * (1 / ((LMREF / thermistor_voltage) - 1));
    steinhart = thermistor_Res / THERMISTORNOMINAL_V2;
    steinhart = log(steinhart);                         // ln(R/Ro)
    steinhart /= BCOEFFICIENT_V2;                       // 1/B * ln(R/Ro)
    steinhart += (1.0 / (TEMPERATURENOMINAL + 273.15)); // + (1/To)
    steinhart = 1.0 / steinhart;                        // Invert
    calc_Temperature_V2 = (float)steinhart - 273.15;    // convert to C
    if (calc_Temperature_V2 <= -100)
    {
        calc_Temperature_V2 = -99.9;
    }
    if (calc_Temperature_V2 > 199.9)
    {
        calc_Temperature_V2 = 199.9;
    }
    dtostrf(calc_Temperature_V2, 5, 1, temperature_str_V2);
    if (calc_Temperature_V2 >= 100.0)
    {
        dtostrf(calc_Temperature_V2, 4, 0, temperature_str_V2_Led_Matrix);
    }
    else
    {
        dtostrf(calc_Temperature_V2, 4, 1, temperature_str_V2_Led_Matrix);
    }
// debug display
#ifdef SERIAL_DEBUG_ENABLED
    Serial.print(temperature_str_V2);
    Serial.println(F(" *C"));
#endif
}
