/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PET Extruder - Thermal + Stepper Control
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h> // <--- AGREGAR ESTO (Para atof)
#include <string.h> // <--- AGREGAR ESTO
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  STATE_IDLE = 0,
  STATE_ACCEL,
  STATE_RUN,
  STATE_DECEL
} MotorState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ========== THERMAL SUBSYSTEM CONSTANTS ==========
#define VREF            3.3f    
#define ADCMAX          4095.0f

#define R_SERIES        1000.0f    
#define R_NTC_25C       100500.0f  

// --- ESTRATEGIA BIMODAL (Dual Beta) ---
// Beta 3950: Precisión en ambiente y arranque (25°C - 150°C)
#define BETA_LOW        3950.0f    

// Beta 4800: Corrección para Alta Temperatura (>150°C). 
// Esto hará que la lectura en pantalla baje, forzando al PID a calentar más.
#define BETA_HIGH       4400.0f

// Punto de cambio: ~180°C (Resistencia aprox 1200-1500 ohms)
#define R_CROSSOVER     1500.0f    

#define T_25C_K         298.15f

#define T_SET_C         250.0f   
#define T_HYST_C        2.0f
#define T_MAX_C         275.0f   

#define ADC_BUF_SIZE    64
#define ALPHA_TEMP      0.1f

// ========== MOTOR SUBSYSTEM CONSTANTS ==========
#define FREQ_MIN_HZ     50      
#define ACCEL_STEPS     2000    
#define DECEL_STEPS     2000    

#define TEMP_SAFE_EXTRUSION  100.0f 

#define MOTOR_STEPS_PER_REV  200.0f
#define MICROSTEPPING        32.0f    // ⬅️ CAMBIO: Era 16, ahora 32

// CORRECCIÓN FINAL: Relación Real Calculada
// Etapa 1: 8→32 (4:1)
// Etapa 2: 12→36 (3:1)
// Etapa 3: 11→55 (5:1)
// Total: 4 × 3 × 5 = 60:1
#define GEAR_RATIO           60.0f    

#define ROLLER_DIAMETER_MM   70.8f    
#define PI                   3.14159265f
#define E_STEPS_PER_MM       ((MOTOR_STEPS_PER_REV * MICROSTEPPING * GEAR_RATIO) / (PI * ROLLER_DIAMETER_MM))
/* USER CODE END PD */

// ========== NUEVAS CONSTANTES DE PROTECCIÓN ==========
#define HEATING_TIMEOUT_MS      300000UL    // 5 minutos
#define TEMP_STARTUP_THRESHOLD  100.0f
#define ADC_DISCONNECTED_LOW    50
#define ADC_DISCONNECTED_HIGH   4045

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static inline float adc_to_voltage(uint16_t adc) { 
  return (adc * VREF) / ADCMAX; 
}

static inline float ntc_temp_c_from_adc(uint16_t adc) {
  float v_ntc = adc_to_voltage(adc);
  
  if (v_ntc < 0.05f || v_ntc > (VREF - 0.05f)) return NAN; 

  float r_ntc = R_SERIES / ((VREF / v_ntc) - 1.0f);
  if (r_ntc <= 0.0f) return NAN; 

  // --- SELECCIÓN DINÁMICA DE BETA ---
  float current_beta;
  
  // Si R > 1500 (estamos fríos, <180°C), usa Beta Normal
  if (r_ntc > R_CROSSOVER) {
      current_beta = BETA_LOW;
  } 
  // Si R < 1500 (estamos calientes, >180°C), usa Beta Alto
  else {
      current_beta = BETA_HIGH;
  }

  float inv_t = (1.0f / T_25C_K) + (1.0f / current_beta) * logf(r_ntc / R_NTC_25C);
  return (1.0f / inv_t) - 273.15f;
}

static inline void heater_set(bool on) {
  HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void motor_enable(bool en) {
  // Activar con 0V (RESET). Desactivar con 3.3V (SET)
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void set_motor_speed(uint16_t freq_hz) {
  if (freq_hz < FREQ_MIN_HZ) freq_hz = FREQ_MIN_HZ;
  // Límite superior de seguridad (20kHz)
  if (freq_hz > 20000) freq_hz = 20000;
  
  uint32_t arr = (1000000UL / freq_hz) - 1;
  __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
  
  // ERROR ORIGINAL: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, arr / 2);
  // CORRECCIÓN: Usar CHANNEL_1
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr / 2); 
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// ========== THERMAL SUBSYSTEM VARIABLES ==========
static uint16_t adc_buffer[ADC_BUF_SIZE];
static volatile bool adc_conversion_complete = false;
static volatile bool flag_100ms = false;
static volatile bool flag_1ms = false; 

static float tc_filt = NAN;
static bool heater_on = false;

// ========== MOTOR SUBSYSTEM VARIABLES ==========
static MotorState_t motor_state = STATE_IDLE;
static uint32_t step_count = 0;
static uint16_t current_freq = FREQ_MIN_HZ;
static bool direction_cw = true;
static bool motor_running = false;

// ========== TELEMETRY BUFFER (DMA) ========== 
// <<< CAMBIO DMA: Buffer para enviar datos sin bloquear la CPU
static char tx_buffer[64]; 

// ========== PID CONTROL VARIABLES ==========
typedef struct {
    float Kp; 
    float Ki; 
    float Kd; 
    
    float prevError;
    float integral;
    float output;
} PID_Config;

// TUNING ESTABLE (Oscilación mínima)
PID_Config pid = {
    .Kp = 4.0f,     // Subimos de 0.8 a 5.0 (Fuerza moderada)
    .Ki = 0.15f,     // Valor estándar para corrección fina
    .Kd = 10.0f,    // Freno suave
    .prevError = 0.0f,
    .integral = 0.0f,
    .output = 0.0f
};

// Variable para el PWM por software
uint32_t pwm_window_start = 0;

// NUEVO: Variable de velocidad global para que el LCD la vea
float target_speed_mm_s = 3.0f ;

// VARIABLES PARA RECEPCIÓN UART
uint8_t rx_byte;           // Byte temporal
char rx_buffer[10];        // Buffer para el comando (ej: "v2.5")
uint8_t rx_index = 0;      // Índice del buffer
/* USER CODE END PV */

// ========== VARIABLES DE PROTECCIÓN ==========
typedef enum {
  ERROR_NONE = 0,
  ERROR_NTC_DISCONNECTED,
  ERROR_OVERTEMP,
  ERROR_HEATING_TIMEOUT,
  ERROR_MOTOR_FAULT,
  ERROR_EMERGENCY_STOP
} SystemError_t;

static SystemError_t system_error = ERROR_NONE;
static uint32_t heating_start_time = 0;
static bool heating_phase_active = false;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void process_adc_buffer(void);
static void motor_state_machine(void);
static void update_lcd(void);
float PID_Compute(PID_Config *pid, float setpoint, float measured_value, float dt_seconds);
uint32_t speed_mm_s_to_hz(float speed_mm_s); 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ========== FUNCIÓN DE RAMPA S-CURVE (Curva Sigmoidal) ==========
// Genera una transición suave usando una función seno
// Entrada: progress (0.0 a 1.0)
// Salida: factor de velocidad (0.0 a 1.0) con aceleración suave
static float s_curve_profile(float progress) {
    // Limitar rango de entrada
    if (progress <= 0.0f) return 0.0f;
    if (progress >= 1.0f) return 1.0f;
    
    // Fórmula: (1 - cos(π * progress)) / 2
    // Esto crea una curva suave en forma de S
    return (1.0f - cosf(PI * progress)) / 2.0f;
}

// --- FUNCIÓN QUE FALTABA ---
uint32_t speed_mm_s_to_hz(float speed_mm_s) {
    if (speed_mm_s <= 0.0f) return 0;
    float hz = speed_mm_s * E_STEPS_PER_MM;
    if (hz > 20000.0f) hz = 20000.0f;
    return (uint32_t)hz;
}

// ========== ADC DMA Callback (Thermal) ==========
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    adc_conversion_complete = true;
  }
}

// ========== TIM3 Interrupt (1ms Timebase) ==========
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    // --- LÓGICA DE TIEMPO (No tocar) ---
    flag_1ms = true; 
    static uint16_t ms_counter = 0;
    ms_counter++;
    if (ms_counter >= 100) {
      ms_counter = 0;
      flag_100ms = true;
    }

    // --- NUEVA LÓGICA: PWM 50 Hz (Silencioso) ---
    // Periodo: 20 ticks de 1ms = 20ms (50 Hz)
    // Resolución: 5% (Suficiente para calentador)
    
    static uint16_t pwm_tick = 0;
    pwm_tick++;
    if (pwm_tick >= 20) pwm_tick = 0; // Reiniciar cada 20ms

    // Escalar PID (0-100) al nuevo rango (0-20)
    // Dividimos por 5: Si PID es 50%, (50/5) = 10 ticks encendido.
    if (pwm_tick < (uint16_t)(pid.output / 5.0f)) {
        HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, GPIO_PIN_RESET);
    }
  }
}

// ========== Procesar Buffer ADC (Thermal) ==========
static void process_adc_buffer(void) {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_BUF_SIZE; i++) {
    sum += adc_buffer[i];
  }
  uint16_t adc_avg = sum / ADC_BUF_SIZE;
  
  float tc_raw = ntc_temp_c_from_adc(adc_avg);
  
  if (!isnan(tc_raw)) {
    if (isnan(tc_filt)) {
      tc_filt = tc_raw;
    } else {
      tc_filt = ALPHA_TEMP * tc_raw + (1.0f - ALPHA_TEMP) * tc_filt;
    }
  }
}

// ========== Motor State Machine (VERSIÓN CON S-CURVE + DEBUG) ==========
static void motor_state_machine(void) {
  bool btn_start = (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) == GPIO_PIN_RESET);
  bool btn_stop = (HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port, STOP_BTN_Pin) == GPIO_PIN_RESET);
  
  bool sensor_ok = !isnan(tc_filt);
  bool temp_safe = sensor_ok && (tc_filt >= TEMP_SAFE_EXTRUSION);
  
  // Calcular frecuencia objetivo
  uint32_t target_hz = speed_mm_s_to_hz(target_speed_mm_s);
  if (target_hz < FREQ_MIN_HZ) target_hz = FREQ_MIN_HZ;

  // --- DEBUG: Enviar datos solo cada 100 ciclos (reducir spam) ---
  static uint16_t debug_counter = 0;
  debug_counter++;
  bool send_debug = (debug_counter >= 100); // Debug cada 100ms
  if (send_debug) debug_counter = 0;

  switch (motor_state) {
    
    case STATE_IDLE:
      motor_enable(false);
      motor_running = false;
      step_count = 0;
      current_freq = 0; // ⬅️ CAMBIO: Forzar a 0 en IDLE (no FREQ_MIN_HZ)
      
      if (btn_start && temp_safe) {
        motor_state = STATE_ACCEL;
        motor_running = true;
        motor_enable(true);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        current_freq = FREQ_MIN_HZ;
        set_motor_speed(FREQ_MIN_HZ);
        step_count = 0;
        
        // DEBUG: Notificar arranque
        if (huart1.gState == HAL_UART_STATE_READY) {
          HAL_UART_Transmit(&huart1, (uint8_t*)"[DEBUG] ACCEL START\r\n", 21, 100);
        }
      }
      break;
      
    case STATE_ACCEL:
      motor_enable(true);
      motor_running = true;
      step_count++;
      
      // RAMPA S-CURVE (Suave al arrancar y al llegar)
      if (step_count < ACCEL_STEPS) {
        float progress = (float)step_count / (float)ACCEL_STEPS;
        float smooth_factor = s_curve_profile(progress); // ⬅️ NUEVO
        current_freq = FREQ_MIN_HZ + (uint32_t)((target_hz - FREQ_MIN_HZ) * smooth_factor);
        set_motor_speed(current_freq);
        
        // DEBUG: Mostrar progreso cada 100ms
        if (send_debug && huart1.gState == HAL_UART_STATE_READY) {
          char dbg[50];
          sprintf(dbg, "[ACCEL] Step:%lu Hz:%u\r\n", step_count, current_freq);
          HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
        }
      } else {
        // Fin de aceleración
        motor_state = STATE_RUN;
        current_freq = target_hz;
        set_motor_speed(current_freq);
        step_count = 0;
        
        // DEBUG
        if (huart1.gState == HAL_UART_STATE_READY) {
          HAL_UART_Transmit(&huart1, (uint8_t*)"[DEBUG] RUN START\r\n", 19, 100);
        }
      }
      
      if (btn_stop || !temp_safe) {
        motor_state = STATE_DECEL;
        step_count = 0;
        
        // DEBUG
        if (huart1.gState == HAL_UART_STATE_READY) {
          HAL_UART_Transmit(&huart1, (uint8_t*)"[DEBUG] DECEL START\r\n", 21, 100);
        }
      }
      break;
      
    case STATE_RUN:
      motor_enable(true);
      motor_running = true;
      
      // Rampa dinámica para cambios de velocidad en vuelo
      int32_t freq_error = (int32_t)target_hz - (int32_t)current_freq;
      
      const uint32_t RAMP_STEP_HZ = 5; // ⬅️ CAMBIO: Más suave (era 10)
      
      if (abs(freq_error) > RAMP_STEP_HZ) {
        if (freq_error > 0) {
          current_freq += RAMP_STEP_HZ;
        } else {
          current_freq -= RAMP_STEP_HZ;
        }
        set_motor_speed(current_freq);
        
        // DEBUG: Solo si hay cambio significativo
        if (send_debug && abs(freq_error) > 100 && huart1.gState == HAL_UART_STATE_READY) {
          char dbg[50];
          sprintf(dbg, "[RUN] Target:%lu Current:%u\r\n", target_hz, current_freq);
          HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
        }
      } else if (abs(freq_error) > 0) {
        current_freq = target_hz;
        set_motor_speed(current_freq);
      }
      
      if (btn_stop || !temp_safe) {
        motor_state = STATE_DECEL;
        step_count = 0;
        
        // DEBUG
        if (huart1.gState == HAL_UART_STATE_READY) {
          HAL_UART_Transmit(&huart1, (uint8_t*)"[DEBUG] DECEL START\r\n", 21, 100);
        }
      }
      break;
      
    case STATE_DECEL:
      motor_enable(true);
      motor_running = true;
      step_count++;
      
      // RAMPA S-CURVE INVERSA (Igual de suave que ACCEL)
      if (step_count < DECEL_STEPS) {
        float progress = (float)step_count / (float)DECEL_STEPS;
        float smooth_factor = s_curve_profile(progress); // ⬅️ NUEVO
        
        // Guardar la frecuencia inicial de desaceleración
        static uint32_t decel_start_freq = 0;
        if (step_count == 1) {
          decel_start_freq = current_freq; // Capturar velocidad al entrar
        }
        
        current_freq = decel_start_freq - (uint32_t)((decel_start_freq - FREQ_MIN_HZ) * smooth_factor);
        set_motor_speed(current_freq);
        
        // DEBUG
        if (send_debug && huart1.gState == HAL_UART_STATE_READY) {
          char dbg[50];
          sprintf(dbg, "[DECEL] Step:%lu Hz:%u\r\n", step_count, current_freq);
          HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
        }
      } else {
        // Fin de desaceleración
        motor_state = STATE_IDLE;
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        motor_enable(false);
        motor_running = false;
        current_freq = 0; // ⬅️ CAMBIO: Forzar a 0
        
        // DEBUG
        if (huart1.gState == HAL_UART_STATE_READY) {
          HAL_UART_Transmit(&huart1, (uint8_t*)"[DEBUG] IDLE (Motor OFF)\r\n", 26, 100);
        }
      }
      break;
  }
}

// ========== Actualizar Display LCD (CON FRECUENCIA) ==========
static void update_lcd(void) {
  char lcd_line1[17];
  char lcd_line2[17];

  // Línea 1: Temperatura
  if (isnan(tc_filt)) {
      snprintf(lcd_line1, sizeof(lcd_line1), "T: Err / %.0f C", T_SET_C);
  } else {
      snprintf(lcd_line1, sizeof(lcd_line1), "T:%.1f/%.0f C%c", 
               tc_filt, T_SET_C, heater_on ? '*' : ' ');
  }
  
  // Línea 2: Motor
  if (motor_state == STATE_IDLE) {
      // ⬅️ CAMBIO: Mostrar explícitamente "0 Hz" cuando está detenido
      if (tc_filt < TEMP_SAFE_EXTRUSION) {
          snprintf(lcd_line2, sizeof(lcd_line2), "Cold - 0 Hz"); 
      } else {
          snprintf(lcd_line2, sizeof(lcd_line2), "Ready - 0 Hz");
      }
  } else {
      snprintf(lcd_line2, sizeof(lcd_line2), "%.1fmm/s %uHz", 
               target_speed_mm_s, current_freq);
  }
  
  lcd_put_cur(0, 0);
  lcd_send_string(lcd_line1);
  lcd_put_cur(1, 0); 
  lcd_send_string(lcd_line2);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  
  // ========== INICIALIZACIÓN LCD ==========
  lcd_init();
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("PET Extruder");
  lcd_put_cur(1, 0);
  lcd_send_string("System v1.0");
  HAL_Delay(2000);
  lcd_clear();

  // >>> BORRADO: TMC2208_Init(); 
  
  // ========== INICIALIZACIÓN ADC + DMA (Thermal) ==========
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_SIZE);
  
  // ========== INICIALIZACIÓN TIM3 (Control PWM Térmico) ==========
  HAL_TIM_Base_Start_IT(&htim3);
  
  // ========== ESTADO INICIAL ==========
  // Para DRV8825: ENABLE LOW = ON, HIGH = OFF.
  // Empezamos apagados.
  motor_enable(false); 
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, direction_cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
  
  heater_set(false);
  
  // ACTIVAR RECEPCIÓN POR INTERRUPCIÓN (Comandos)
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // ⬅️ NUEVO: Refrescar Watchdog (OBLIGATORIO cada <4 segundos)
    HAL_IWDG_Refresh(&hiwdg);
    
    // ========== TASK 1: ADC Conversion cada 100ms ==========
    if (flag_100ms) {
      flag_100ms = false;
      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_SIZE);
    }
    
    // ========== TASK 2: Control Térmico + Protecciones ==========
    if (adc_conversion_complete) {
      adc_conversion_complete = false;
      process_adc_buffer();
      
      // ⬅️ NUEVO: Verificar Condiciones de Seguridad
      check_safety_conditions();
      
      // PID (igual que antes)
      bool sensor_ok = !isnan(tc_filt);
      bool overtemp = sensor_ok && (tc_filt > T_MAX_C);
      
      if (!sensor_ok || overtemp) {
        pid.output = 0.0f; 
      } else {
        pid.output = PID_Compute(&pid, T_SET_C, tc_filt, 0.1f);
      }
      
      heater_on = (pid.output > 1.0f); 
      
      // Telemetría (igual que antes)
      if (huart1.gState == HAL_UART_STATE_READY) {
          float error = T_SET_C - tc_filt;
          float actual_speed_mm_s = (float)current_freq / E_STEPS_PER_MM;
          
          int len = sprintf(tx_buffer, "%lu,%.2f,%.2f,%d,%.2f,%.2f\r\n", 
                 HAL_GetTick(), tc_filt, T_SET_C, (int)pid.output, 
                 actual_speed_mm_s, error);
          HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, len, 100);
      }
      
      update_lcd();
    }
    
    // ========== TASK 3: Motor State Machine ==========
    if (flag_1ms) {
      flag_1ms = false;
      motor_state_machine();
    }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

// ========== CALLBACK DE RECEPCIÓN UART (Parser de Comandos) ==========
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    
    // --- CASO 1: Recibimos Enter (Fin del Comando) ---
    if (rx_byte == '\n' || rx_byte == '\r') {
      rx_buffer[rx_index] = '\0'; // Terminar el string con NULL
      
      // --- PARSEAR COMANDO ---
      if (rx_index > 0) { // Solo si hay algo en el buffer
        
        // COMANDO 'v': Cambiar Velocidad
        // Ejemplo: "v3.5" → target_speed_mm_s = 3.5
        if (rx_buffer[0] == 'v' || rx_buffer[0] == 'V') {
          float new_speed = atof(&rx_buffer[1]); // Convertir string a float
          
          // Validar rango de seguridad (0.5 a 10 mm/s)
          if (new_speed >= 0.5f && new_speed <= 5.0f) {
            target_speed_mm_s = new_speed;
            
            // Enviar confirmación al PC
            char ack[32];
            sprintf(ack, "OK: Speed = %.2f mm/s\r\n", target_speed_mm_s);
            HAL_UART_Transmit(&huart1, (uint8_t*)ack, strlen(ack), 100);
          } else {
            // Error: Fuera de rango
            HAL_UART_Transmit(&huart1, (uint8_t*)"ERR: Speed out of range\r\n", 25, 100);
          }
        }
        
        // COMANDO 's': Stop Suave (CORREGIDO)
        else if (rx_buffer[0] == 's' || rx_buffer[0] == 'S') {
          if (motor_state != STATE_IDLE) {
            // CLAVE: Guardar la frecuencia actual antes de desacelerar
            // Esto permite que STATE_DECEL use la rampa desde donde está ahora
            motor_state = STATE_DECEL;
            step_count = 0; // Reiniciar contador de desaceleración
            HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Stopping\r\n", 14, 100);
          }
        }
        
        // COMANDO 'r': Resume (Arrancar)
        else if (rx_buffer[0] == 'r' || rx_buffer[0] == 'R') {
          if (motor_state == STATE_IDLE && tc_filt >= TEMP_SAFE_EXTRUSION) {
            motor_state = STATE_ACCEL;
            motor_running = true;
            motor_enable(true);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            set_motor_speed(FREQ_MIN_HZ);
            HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Running\r\n", 13, 100);
          } else {
            HAL_UART_Transmit(&huart1, (uint8_t*)"ERR: Temp too low\r\n", 19, 100);
          }
        }
        
        // COMANDO DESCONOCIDO
        else {
          HAL_UART_Transmit(&huart1, (uint8_t*)"ERR: Unknown command\r\n", 22, 100);
        }
      }
      
      rx_index = 0; // Reiniciar buffer para el próximo comando
    }
    
    // --- CASO 2: Recibimos un Carácter Normal ---
    else {
      // Acumular en el buffer si hay espacio
      if (rx_index < (sizeof(rx_buffer) - 1)) {
        rx_buffer[rx_index++] = rx_byte;
      } else {
        // Overflow: Buffer lleno sin Enter
        rx_index = 0; // Reset forzado
        HAL_UART_Transmit(&huart1, (uint8_t*)"ERR: Buffer overflow\r\n", 22, 100);
      }
    }
    
    // --- REACTIVAR RECEPCIÓN (Obligatorio) ---
    // Sin esta línea, solo recibirías 1 byte y luego se bloquearía.
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

// --- FUNCIÓN PID (MANTENER ESTA) ---
float PID_Compute(PID_Config *pid, float setpoint, float measured_value, float dt_seconds) {
    float error = setpoint - measured_value;
    
    // Término Proporcional
    float P = pid->Kp * error;
    
    // Término Integral (con Anti-Windup simple)
    if (fabs(error) < 15.0f) { 
        pid->integral += error * dt_seconds; 
    } else { 
        pid->integral = 0.0f; // Reset si estamos muy lejos
    }
    
    float I_term = pid->Ki * pid->integral; 
    
    // Limits (Clamp)
    if (I_term > 100.0f) { 
        I_term = 100.0f; 
        pid->integral = 100.0f / pid->Ki; 
    }
    else if (I_term < -20.0f) { 
        I_term = -20.0f; 
        pid->integral = -20.0f / pid->Ki; 
    }
    
    // Término Derivativo
    float derivative = (error - pid->prevError) / dt_seconds;
    float D = pid->Kd * derivative;
    
    pid->prevError = error;
    
    // Suma final
    float output = P + I_term + D;
    
    // Limitar Salida (0% a 100%)
    if (output > 100.0f) output = 100.0f;
    if (output < 0.0f) output = 0.0f; 
    
    return output;
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
      // El microcontrolador se queda aquí si algo falla gravemente
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
