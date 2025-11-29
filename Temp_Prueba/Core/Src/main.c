/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c  (IOC-managed GPIO / Control calefactor integrado)
  * @brief          : Subsistema térmico con TIM3 (sin delays en runtime)
  *                   + control ON/OFF del calentador (IRFZ44N via BC337/BC327)
  *                   usando el pin configurado en el .ioc (ej.: PB5 = HEATER_EN).
  *
  *  Notas:
  *   - No inicializa el GPIO del heater aquí: lo hace MX_GPIO_Init() generado
  *     por CubeMX a partir del .ioc (User Label = HEATER_EN).
  *   - Usa macros HEATER_EN_Pin y HEATER_EN_GPIO_Port generadas en main.h.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- NTC y divisor (pull-up: Rfix a 3.3V, NTC a GND) ---
#define VREF    3.3f
#define ADCMAX  4095.0f
#define RFIX    10000.0f      // 10kΩ a 3.3V
#define R0_NTC  100000.0f     // 100kΩ @ 25°C
#define T0_K    298.15f       // 25°C
#define BETA    3950.0f       // típico B3950

// --- Parámetros de control térmico ---
#define T_SET_C     245.0f    // Setpoint (°C)
#define T_HYST_C      2.0f    // Histéresis ± (°C)
#define T_MAX_C     270.0f    // Corte por seguridad (°C)

// --- Configuración DMA ---
#define ADC_SAMPLES 32        // Cantidad de muestras por lectura
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static inline float adc_to_voltage(uint16_t adc) { return (adc * VREF) / ADCMAX; }
static inline void heater_set(bool on) {
  HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t flag_10ms = 0, flag_100ms = 0, flag_1s = 0;   // generadas por TIM3
static float v_filt = 0.0f;                                     // EMA de voltaje
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint16_t adc_read_once(void);
static uint16_t adc_read_avg(uint8_t n);
static float    ntc_temp_c_from_adc(uint16_t adc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Lee 1 vez ADC
static uint16_t adc_read_once(void) {
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return v;
}

// Promedio robusto con descarte min/max
static uint16_t adc_read_avg(uint8_t n) {
  uint32_t acc = 0; uint16_t vmin = 0xFFFF, vmax = 0;
  for (uint8_t i = 0; i < n; i++) {
    uint16_t v = adc_read_once();
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
    acc += v;
  }
  if (n > 2) { acc -= vmin + vmax; return (uint16_t)(acc / (n - 2)); }
  return (uint16_t)(acc / n);
}

// ADC -> Voltaje -> R_NTC -> Temperatura (modelo Beta)
static float ntc_temp_c_from_adc(uint16_t adc) {
  float v = adc_to_voltage(adc);
  if (v <= 0.0f || v >= VREF) return NAN;         // fuera de rango → sensor inválido
  float r_ntc = RFIX * (v / (VREF - v));
  float invT  = (1.0f / T0_K) + (1.0f / BETA) * logf(r_ntc / R0_NTC);
  float Tkelvin = 1.0f / invT;
  return Tkelvin - 273.15f;
}
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Calibración del ADC (F103): una vez antes de medir
  HAL_ADCEx_Calibration_Start(&hadc1);

  // LCD
  lcd_init();
  lcd_clear();
  lcd_put_cur(0,0); lcd_send_string("NTC Bluepill");
  lcd_put_cur(1,0); lcd_send_string("Init...");
  HAL_Delay(800);               // único delay de arranque
  lcd_clear();

  // Timer base con interrupción (tick 1 ms)
  HAL_TIM_Base_Start_IT(&htim3);

  char line1[17], line2[17];
  static float tc_filt = NAN;   // EMA temperatura
  static float tc_disp = NAN;   // último valor mostrado
  static uint32_t t_last = 0;   // último refresh LCD
  static bool heater_on = false;
  v_filt = 0.0f;

  while (1)
  {
    if (flag_100ms) {
      flag_100ms = 0;

      // 1) Lectura estable del ADC
      uint16_t adc = adc_read_avg(32);

      // 2) Voltaje + EMA (presentación)
      float v  = adc_to_voltage(adc);
      const float alpha_V = 0.10f;          // 0.08–0.12
      v_filt = (1.0f - alpha_V) * v_filt + alpha_V * v;

      // 3) Convertir a °C
      float tc = ntc_temp_c_from_adc(adc);

      // 4) EMA de temperatura
      const float alpha_T = 0.10f;          // 0.08 si querés más calma
      if (isnan(tc_filt)) tc_filt = tc; else tc_filt += alpha_T * (tc - tc_filt);

      // 5) Control ON/OFF con histéresis + seguridad
      bool sensor_ok = !isnan(tc_filt);
      bool overtemp  = sensor_ok && (tc_filt > T_MAX_C);
      if (!sensor_ok || overtemp) {
        heater_on = false;                  // seguridad
      } else {
        if (!heater_on && (tc_filt <= (T_SET_C - T_HYST_C))) heater_on = true;
        else if (heater_on && (tc_filt >= (T_SET_C + T_HYST_C))) heater_on = false;
      }
      heater_set(heater_on);

      // 6) Display con deadband ±0.10 °C y rate-limit 1 s
      uint32_t now = HAL_GetTick();
      bool debe_refrescar = isnan(tc_disp)
                         || fabsf(tc_filt - tc_disp) >= 0.10f
                         || (now - t_last) >= 1000;
      if (debe_refrescar) {
        tc_disp = tc_filt; t_last = now;
        if (isnan(tc_disp)) snprintf(line1, sizeof(line1), "T:  ---.- C   ");
        else                snprintf(line1, sizeof(line1), "T:%7.1f C   ", tc_disp);
        snprintf(line2, sizeof(line2), "ADC:%4u  %.2fV", adc, v_filt);
        lcd_put_cur(0,0); lcd_send_string(line1);
        lcd_put_cur(1,0); lcd_send_string(line2);
      }
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {        // tick = 1 ms (PSC=7199, ARR=9 con 72 MHz)
    static uint16_t d10=0, d100=0, d1000=0;
    if (++d10   >= 10)   { d10=0;   flag_10ms  = 1; }  // 10 ms
    if (++d100  >= 100)  { d100=0;  flag_100ms = 1; }  // 100 ms
    if (++d1000 >= 1000) { d1000=0; flag_1s    = 1; }  // 1 s
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
