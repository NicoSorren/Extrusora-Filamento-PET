/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* --- Pines DIR/STEP --- */
#define DIR_PIN    GPIO_PIN_1
#define DIR_PORT   GPIOA
#define STEP_PIN   GPIO_PIN_2
#define STEP_PORT  GPIOA

/* --- Botones (pull-up) --- */
#define STOP_BTN_PIN            GPIO_PIN_3   // PA3: STOP
#define STOP_BTN_PORT           GPIOA
#define START_BTN_PIN           GPIO_PIN_4   // PA4: START
#define START_BTN_PORT          GPIOA
#define BTN_PRESSED_STATE       GPIO_PIN_RESET  // con pull-up: 0 = presionado

/* Delay en us usando TIM1 a 1 MHz */
void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

/* Pulso STEP (alto y bajo del mismo ancho) */
static inline void step_pulse(uint16_t d_us){
  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
  microDelay(d_us);
  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
  microDelay(d_us);
}

/* Lectura con “debounce” simple: confirma transición a presionado */
static inline uint8_t button_pressed(GPIO_TypeDef* port, uint16_t pin){
  if (HAL_GPIO_ReadPin(port, pin) == BTN_PRESSED_STATE){
    HAL_Delay(20); // debounce
    if (HAL_GPIO_ReadPin(port, pin) == BTN_PRESSED_STATE) return 1;
  }
  return 0;
}

/* Espera a que el botón dado se presione y suelte (ambas con debounce) */
static void wait_press_and_release(GPIO_TypeDef* port, uint16_t pin){
  // esperar PRESION
  while (!button_pressed(port, pin)) { HAL_Delay(5); }
  // esperar SOLTAR
  while (HAL_GPIO_ReadPin(port, pin) == BTN_PRESSED_STATE) { HAL_Delay(5); }
  HAL_Delay(20); // debounce de soltado
}

/* Acelera hasta d_target, mantiene crucero en d_target y
   frena cuando se presiona el botón de STOP. */
void run_extrusion_until_stop(uint16_t d_start, uint16_t d_target, uint32_t accel_steps)
{
  if (d_start < d_target) { uint16_t t=d_start; d_start=d_target; d_target=t; }
  if (accel_steps == 0) accel_steps = 1;
  int32_t delta = (int32_t)d_start - (int32_t)d_target;

  /* --- Aceleración --- (toca d_target en el último paso) */
  for (uint32_t i = 0; i < accel_steps; i++){
    uint16_t d = d_start - (uint16_t)((delta * (int32_t)(i+1)) / (int32_t)accel_steps);
    step_pulse(d);
    // Si ya apretaron STOP durante la rampa, terminá de frenar igual:
    if (HAL_GPIO_ReadPin(STOP_BTN_PORT, STOP_BTN_PIN) == BTN_PRESSED_STATE){
      break;
    }
  }

  /* --- Crucero: delay fijo d_target hasta STOP --- */
  while (HAL_GPIO_ReadPin(STOP_BTN_PORT, STOP_BTN_PIN) != BTN_PRESSED_STATE){
    step_pulse(d_target);
  }

  /* Debounce y espera a que suelte STOP */
  HAL_Delay(20);
  while (HAL_GPIO_ReadPin(STOP_BTN_PORT, STOP_BTN_PIN) == BTN_PRESSED_STATE){
    step_pulse(d_target); // mantener suave mientras está presionado
  }
  HAL_Delay(20);

  /* --- Deceleración simétrica --- */
  for (uint32_t i = 0; i < accel_steps; i++){
    uint16_t d = d_target + (uint16_t)((delta * (int32_t)(i+1)) / (int32_t)accel_steps);
    step_pulse(d);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);   // TIM1 a 1 MHz (1 us/tick)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1) Esperar orden de ARRANQUE (START) */
    wait_press_and_release(START_BTN_PORT, START_BTN_PIN);

    /* 2) Elegir sentido (si querés fijo, dejá uno solo) */
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);

    /* 3) Acelera -> CRUCERO constante -> FRENA cuando se presione STOP */
    run_extrusion_until_stop(3000, 400, 600);

    /* 4) Al terminar de frenar, volver al estado de espera de START */
    HAL_Delay(100); // pequeño respiro antes de volver a escuchar START
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;                 // 72MHz/(71+1) = 1 MHz
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* DIR/STEP en 0 al inicio */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /* PA1 (DIR) y PA2 (STEP) como salida */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   // si querés, poné HIGH para STEP
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA3 (STOP) entrada con pull-up */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA4 (START) entrada con pull-up */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
}
#endif /* USE_FULL_ASSERT */
