/*
 * Thermistor.h
 *
 *  Created on: Nov 10, 2025
 *      Author: Alper
 */

#include <stdint.h>


#ifndef INC_THERMISTOR_H_
#define INC_THERMISTOR_H_

float Get_Temperature(uint16_t adc_val);
uint16_t Read_ADC_Channel(ADC_HandleTypeDef* hadc);



#endif /* INC_THERMISTOR_H_ */
