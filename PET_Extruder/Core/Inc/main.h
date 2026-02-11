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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void save_config_to_flash(void);
void load_config_from_flash(void);
void restore_defaults(void);
void send_config_report(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIR_Pin GPIO_PIN_1
#define DIR_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOA
#define RGB_B_Pin GPIO_PIN_3
#define RGB_B_GPIO_Port GPIOA
#define START_BTN_Pin GPIO_PIN_4
#define START_BTN_GPIO_Port GPIOA
#define MOTOR_EN_Pin GPIO_PIN_5
#define MOTOR_EN_GPIO_Port GPIOA
#define RGB_R_Pin GPIO_PIN_0
#define RGB_R_GPIO_Port GPIOB
#define RGB_G_Pin GPIO_PIN_1
#define RGB_G_GPIO_Port GPIOB
#define HEATER_BTN_Pin GPIO_PIN_11
#define HEATER_BTN_GPIO_Port GPIOB
#define STOP_BTN_Pin GPIO_PIN_12
#define STOP_BTN_GPIO_Port GPIOB
#define ENC_SW_Pin GPIO_PIN_13
#define ENC_SW_GPIO_Port GPIOB
#define ENC_SW_EXTI_IRQn EXTI15_10_IRQn
#define ENC_CLK_Pin GPIO_PIN_14
#define ENC_CLK_GPIO_Port GPIOB
#define ENC_CLK_EXTI_IRQn EXTI15_10_IRQn
#define ENC_DT_Pin GPIO_PIN_15
#define ENC_DT_GPIO_Port GPIOB
#define DRV_FAULT_Pin GPIO_PIN_11
#define DRV_FAULT_GPIO_Port GPIOA
#define DRV_FAULT_EXTI_IRQn EXTI15_10_IRQn
#define HEATER_EN_Pin GPIO_PIN_8
#define HEATER_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// Moved from main.c for visibility in menu.c
typedef enum {
  ERROR_NONE = 0,
  ERROR_NTC_DISCONNECTED,
  ERROR_OVERTEMP,
  ERROR_HEATING_TIMEOUT,
  ERROR_MOTOR_FAULT,
  ERROR_EMERGENCY_STOP
} SystemError_t;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
