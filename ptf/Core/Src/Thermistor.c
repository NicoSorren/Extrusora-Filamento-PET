/*
 * Thermistor.c
 *
 *  Created on: Nov 10, 2025
 *      Author: Alper
 */

#include "Menu.h"
#include "st7789.h"
#include "fonts.h"
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include <math.h>
#include "Thermistor.h"

float Get_Temperature(uint16_t adc_val)
{
    // Constants for 100k NTC thermistor (B=3950)* with 10k pull-up
    const float VCC = 3.3f;
    const float R_PULLUP = 10000.0f;
    const float BETA = 3950.0f;
    const float R0 = 100000.0f;
    const float T0 = 298.15f; // 25°C in Kelvin

    // Convert ADC to voltage
    float Vout = ((float)adc_val / 4095.0f) * VCC;

    // Convert voltage to thermistor resistance
    float R_ntc = R_PULLUP * Vout / (VCC - Vout);

    // Calculate temperature in Kelvin using beta formula
    float T_kelvin = 1.0f / ((1.0f / BETA) * logf(R_ntc / R0) + 1.0f / T0);

    // Convert to Celsius
    float T_c = T_kelvin - 273.15f;

    return T_c;
}

uint16_t Read_ADC_Channel(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    uint16_t value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return value;
}
