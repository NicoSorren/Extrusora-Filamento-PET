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
#include "tim.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void step_set_period_us(uint32_t d_us);
static uint8_t button_pressed(GPIO_TypeDef* port, uint16_t pin);
static void wait_press_and_release(GPIO_TypeDef* port, uint16_t pin);
static void run_until_stop_with_ramp(uint32_t d_start, uint32_t d_target, uint32_t ramp_ms); // ← NUEVO
static void run_until_stop_scurve_freq(uint32_t d_start, uint32_t d_target, uint32_t ramp_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  wait_press_and_release(START_BTN_GPIO_Port, START_BTN_Pin);
	  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	  step_set_period_us(3000);                      // d_start seguro
	  __HAL_TIM_SET_COUNTER(&htim2, 0);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	  run_until_stop_scurve_freq(3000, 400, 1500);   // ↑ probá 1200–2000 ms para más “progresivo”
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Ajusta período (y 50% de duty) sin bloquear: periodo = 2*d_us
// Acelera linealmente de d_start -> d_target en ramp_ms (ms), mantiene crucero,
// y frena simétrico (d_target -> d_start en ramp_ms) cuando se presiona STOP.
static void run_until_stop_with_ramp(uint32_t d_start, uint32_t d_target, uint32_t ramp_ms)
{
  if (d_start < 2) d_start = 2;
  if (d_target < 2) d_target = 2;
  if (ramp_ms == 0) ramp_ms = 1;

  // --- ACELERACIÓN: d = d_start -> d_target en ramp_ms ---
  uint32_t t0 = HAL_GetTick();
  while (1) {
    // ¿se pidió STOP? (pull-up: 0 = presionado) — si sí, saltá a frenar
    if (HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port, STOP_BTN_Pin) == GPIO_PIN_RESET) break;

    uint32_t elapsed = HAL_GetTick() - t0;
    if (elapsed >= ramp_ms) {
      step_set_period_us(d_target);   // alcanzó la meta
      break;
    } else {
      // Interpolación entera: d = d_start - (d_start - d_target) * (elapsed / ramp_ms)
      int32_t delta = (int32_t)d_start - (int32_t)d_target;
      uint32_t d = d_start - (uint32_t)((delta * (int32_t)elapsed) / (int32_t)ramp_ms);
      step_set_period_us(d);
    }
    // Pequeño respiro (no crítico, el timer sigue generando STEP solo)
    HAL_Delay(1);
  }

  // --- CRUCERO: fijo en d_target hasta STOP ---
  while (HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port, STOP_BTN_Pin) != GPIO_PIN_RESET) {
    HAL_Delay(1); // hacé otras tareas aquí; esto es solo "demo"
  }

  // debounce simple de STOP y esperar soltar
  HAL_Delay(20);
  while (HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port, STOP_BTN_Pin) == GPIO_PIN_RESET) { HAL_Delay(5); }
  HAL_Delay(20);

  // --- FRENADO: d = d_target -> d_start en ramp_ms ---
  t0 = HAL_GetTick();
  while (1) {
    uint32_t elapsed = HAL_GetTick() - t0;
    if (elapsed >= ramp_ms) {
      step_set_period_us(d_start);
      break;
    } else {
      int32_t delta = (int32_t)d_start - (int32_t)d_target;  // mismo delta
      uint32_t d = d_target + (uint32_t)((delta * (int32_t)elapsed) / (int32_t)ramp_ms);
      step_set_period_us(d);
    }
    HAL_Delay(1);
  }
}


static void step_set_period_us(uint32_t d_us)
{
  if (d_us < 2) d_us = 2;                    // asegura tiempos alto/bajo mínimos
  uint32_t arr = 2U * d_us - 1U;             // ARR+1 = 2*d_us  (1 tick = 1us)
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, d_us);  // 50%
  HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE); // aplica ya
}

// Devuelve 1 si detecta pulsación estable (pull-up: 0 = presionado)
static uint8_t button_pressed(GPIO_TypeDef* port, uint16_t pin)
{
  if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) {
    HAL_Delay(20); // debounce
    if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) return 1;
  }
  return 0;
}

// Espera presionar y soltar el botón, ambas con debounce
static void wait_press_and_release(GPIO_TypeDef* port, uint16_t pin)
{
  while (!button_pressed(port, pin)) { HAL_Delay(5); }         // esperar PRESION
  while (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) { HAL_Delay(5); } // esperar SOLTAR
  HAL_Delay(20);
}

static void step_set_freq_hz(float f_hz){
  if (f_hz < 1.0f) f_hz = 1.0f;
  float d_us_f = 1e6f / (2.0f * f_hz);           // T = 1/f → d_us = T/2
  uint32_t d_us = (uint32_t)(d_us_f + 0.5f);
  step_set_period_us(d_us);
}

// Suavizado cúbico (S-curve): 0→1 con jerk ~0 en extremos
static float scurve(float u){
  if (u <= 0.f) return 0.f;
  if (u >= 1.f) return 1.f;
  return u*u*(3.f - 2.f*u);
}

static void run_until_stop_scurve_freq(uint32_t d_start, uint32_t d_target, uint32_t ramp_ms)
{
  if (d_start < 2) d_start = 2;
  if (d_target < 2) d_target = 2;
  if (ramp_ms == 0) ramp_ms = 1;

  // Convierte tus d_us a frecuencias (steps/s)
  float f_start  = 1e6f / (2.0f * (float)d_start);
  float f_target = 1e6f / (2.0f * (float)d_target);

  // --- Aceleración: f = S-curve(f_start→f_target) ---
  uint32_t t0 = HAL_GetTick();
  while (1) {
    if (HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port, STOP_BTN_Pin) == GPIO_PIN_RESET) break;
    uint32_t elapsed = HAL_GetTick() - t0;
    float u = (elapsed >= ramp_ms) ? 1.0f : (elapsed / (float)ramp_ms);
    float f_now = f_start + (f_target - f_start) * scurve(u);
    step_set_freq_hz(f_now);
    if (elapsed >= ramp_ms) break;
    HAL_Delay(1); // paso de actualización (podés usar 1–2 ms)
  }

  // --- Crucero ---
  while (HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port, STOP_BTN_Pin) != GPIO_PIN_RESET) { HAL_Delay(1); }

  // Debounce STOP
  HAL_Delay(20);
  while (HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port, STOP_BTN_Pin) == GPIO_PIN_RESET) { HAL_Delay(5); }
  HAL_Delay(20);

  // --- Frenado: f = S-curve(f_target→f_start) ---
  t0 = HAL_GetTick();
  while (1) {
    uint32_t elapsed = HAL_GetTick() - t0;
    float u = (elapsed >= ramp_ms) ? 1.0f : (elapsed / (float)ramp_ms);
    float f_now = f_target + (f_start - f_target) * scurve(u);
    step_set_freq_hz(f_now);
    if (elapsed >= ramp_ms) break;
    HAL_Delay(1);
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
