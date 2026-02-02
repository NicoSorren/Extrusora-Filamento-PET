/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_Pin GPIO_PIN_2
#define SW_GPIO_Port GPIOA
#define SW_EXTI_IRQn EXTI2_3_IRQn
#define EndSW_Pin GPIO_PIN_3
#define EndSW_GPIO_Port GPIOA
#define ST7789_CS_Pin GPIO_PIN_0
#define ST7789_CS_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_8
#define EN_GPIO_Port GPIOA
#define TIM1_CH2_STEP_Pin GPIO_PIN_9
#define TIM1_CH2_STEP_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_10
#define DIR_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_4
#define Buzzer_GPIO_Port GPIOB
#define ST7789_MOSI_Pin GPIO_PIN_5
#define ST7789_MOSI_GPIO_Port GPIOB
#define ST7789_RESET_Pin GPIO_PIN_6
#define ST7789_RESET_GPIO_Port GPIOB
#define ST7789_DC_Pin GPIO_PIN_7
#define ST7789_DC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern uint8_t Buzzer_State;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
