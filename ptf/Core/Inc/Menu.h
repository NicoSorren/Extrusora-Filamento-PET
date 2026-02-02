/*
 * Menu.h
 *
 *  Created on: Nov 5, 2025
 *      Author: Alper
 */

#include <stdint.h>

#ifndef INC_MENU_H_
#define INC_MENU_H_

void main_menu();
void mm_values(uint16_t Measured_Temp, uint16_t Target_Temp, uint16_t Speed);
void mm_selection_indicator(uint8_t selection);
void settings_menu();



extern volatile uint8_t page;
extern const uint16_t Fire_50x50[50][50];
extern const uint16_t Stepper_56x42[56][42];

#endif /* INC_MENU_H_ */
