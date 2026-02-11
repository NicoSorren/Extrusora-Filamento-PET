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
#include "encoder.h"
#include "menu.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// SystemError_t moved to main.h
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VREF            3.3f    
#define ADCMAX          4095.0f

#define R_SERIES        1000.0f    
#define R_NTC_25C       100500.0f  

#define BETA_LOW        3950.0f    
#define BETA_HIGH       4410.0f
#define R_CROSSOVER     1500.0f    
#define T_25C_K         298.15f

//#define T_SET_C         250.0f   

#define T_HYST_C        2.0f
#define T_MAX_C         270.0f   

#define ADC_BUF_SIZE    64
#define ALPHA_TEMP      0.1f

#define FREQ_MIN_HZ     50      
#define ACCEL_STEPS     2000    
#define DECEL_STEPS     2000    

#define TEMP_SAFE_EXTRUSION  230.0f 

#define MOTOR_STEPS_PER_REV  200.0f
#define MICROSTEPPING        32.0f
#define GEAR_RATIO           60.0f    
#define ROLLER_DIAMETER_MM   70.8f    
#define PI                   3.14159265f
#define E_STEPS_PER_MM       ((MOTOR_STEPS_PER_REV * MICROSTEPPING * GEAR_RATIO) / (PI * ROLLER_DIAMETER_MM))

#define HEATING_TIMEOUT_MS      300000UL
#define TEMP_STARTUP_THRESHOLD  100.0f
// Rango ADC más estricto para asegurar disparo de alarma antes que T:Err
#define ADC_DISCONNECTED_LOW    100     // Antes 50
#define ADC_DISCONNECTED_HIGH   4000    // Antes 4045

// --- FLASH MEMORY SETTINGS (Page 63 for STM32F103C8 - 64KB) ---
#define FLASH_USER_START_ADDR   0x0800FC00 
#define CONFIG_MAGIC_NUMBER     0xCAFEBABE // Para saber si ya guardamos algo valido antes

typedef struct {
    float saved_target_temp;
    float saved_target_speed;
    float saved_kp;
    float saved_ki;
    float saved_kd;
    uint32_t magic;
} ExtruderConfig;
/* USER CODE END PD */

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

  float current_beta = (r_ntc > R_CROSSOVER) ? BETA_LOW : BETA_HIGH;

  float inv_t = (1.0f / T_25C_K) + (1.0f / current_beta) * logf(r_ntc / R_NTC_25C);
  return (1.0f / inv_t) - 273.15f;
}



// Control PWM del heater (duty cycle 0-100%)
static inline void heater_set_pwm(float power_percent) {
    if (power_percent < 0.0f) power_percent = 0.0f;
    if (power_percent > 100.0f) power_percent = 100.0f;
    uint32_t duty = (uint32_t)(power_percent * 100.0f); // 0-100% -> 0-9999
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty);
}

static inline void motor_enable(bool en) {
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void set_motor_speed(uint16_t freq_hz) {
  if (freq_hz < FREQ_MIN_HZ) freq_hz = FREQ_MIN_HZ;
  if (freq_hz > 20000) freq_hz = 20000;
  
  uint32_t arr = (1000000UL / freq_hz) - 1;
  __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr / 2); 
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t error_count = 0;
float target_temp = 245.0f;  // Temperatura inicial consistente con defaults
static uint16_t adc_buffer[ADC_BUF_SIZE];
volatile bool adc_conversion_complete = false;  
volatile bool flag_100ms = false;              
volatile bool flag_1ms = false;                

float tc_filt = NAN;            // Removed static
bool heater_on = false;                        

static MotorState motor_state = MOTOR_IDLE;      
static ThermalState thermal_state = THERMAL_IDLE;
static uint32_t step_count = 0;
uint16_t current_freq = FREQ_MIN_HZ; // Removed static
static bool direction_cw = true;
static bool motor_running = false;

static char tx_buffer[64]; 

PID_Config pid = {  
    .Kp = 1.0f,   // Balanceado: suficiente ganancia sin overshoot extremo
    .Ki = 0.15f,  // Mantener integral suave
    .Kd = 60.0f,  // Alto para buena anticipación de overshoot
    .prevError = 0.0f,
    .integral = 0.0f,
    .output = 0.0f
};

uint32_t pwm_window_start = 0;  
float target_speed_mm_s = 3.0f;

uint8_t rx_byte;
char rx_buffer[10];
uint8_t rx_index = 0;

SystemError_t system_error = ERROR_NONE; // Removed static
static uint32_t heating_start_time = 0;
static bool heating_phase_active = false;

// VARIABLES DE GESTIÓN DE BOTONES FÍSICOS
// Start (Motor) local en function
// Stop (Emergency Stop) local en function

// Nuevo Botón Heater (Toggle PB11)
static bool last_heater_btn_state = true; // Asumimos Pull-up (1 = suelto, 0 = apretado)
static uint32_t heater_btn_timer = 0;

// Flags de eventos (Comandos que se pasan al main loop)
bool cmd_motor_start = false;
bool cmd_motor_stop = false;
bool cmd_heater_start = false;
bool cmd_heater_stop = false;

// ⬇️ NUEVAS BANDERAS PARA UART (Mantén las de heater y agrega las de motor) ⬇️
volatile bool uart_cmd_heater_start = false;
volatile bool uart_cmd_heater_stop = false;
volatile bool uart_cmd_motor_start = false; // <--- AGREGAR
volatile bool uart_cmd_motor_stop = false;  // <--- AGREGAR

// --- BUZZER TIMEOUT ---
uint32_t buzzer_stop_time = 0;

// --- COLD PROTECT MESSAGE TIMEOUT ---
static uint32_t cold_protect_message_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void handle_physical_buttons(void);
static void process_adc_buffer(void);
static void motor_state_machine(void);
static void update_lcd(void);
float PID_Compute(PID_Config *pid, float setpoint, float measured_value, float dt_seconds);
uint32_t speed_mm_s_to_hz(float speed_mm_s); 
static void check_safety_conditions(void);
void save_config_to_flash(void);
void load_config_from_flash(void);

// --- AUXILIARY FUNCTIONS ---
void beep(uint32_t duration_ms) {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    buzzer_stop_time = HAL_GetTick() + duration_ms;
}

void set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void save_config_to_flash(void) {
    HAL_FLASH_Unlock();
    
    // 1. Borrar la página antes de escribir
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages     = 1;
    
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        HAL_FLASH_Lock();
        return; // Error al borrar
    }

    // 2. Preparar datos
    ExtruderConfig cfg;
    cfg.saved_target_temp = target_temp;
    cfg.saved_target_speed = target_speed_mm_s;
    cfg.saved_kp = pid.Kp;
    cfg.saved_ki = pid.Ki;
    cfg.saved_kd = pid.Kd;
    cfg.magic = CONFIG_MAGIC_NUMBER;
    
    // 3. Escribir datos (palabra por palabra de 32 bits)
    uint32_t *source_addr = (uint32_t *)&cfg;
    uint32_t dest_addr = FLASH_USER_START_ADDR;
    
    for (int i = 0; i < sizeof(ExtruderConfig) / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dest_addr, *source_addr) == HAL_OK) {
            dest_addr += 4;
            source_addr++;
        } else {
            break; // Error escritura
        }
    }
    
    HAL_FLASH_Lock();
}

void load_config_from_flash(void) {
    ExtruderConfig *cfg = (ExtruderConfig *)FLASH_USER_START_ADDR;
    
    if (cfg->magic == CONFIG_MAGIC_NUMBER) {
        // Datos validos encontrados
        target_temp = cfg->saved_target_temp;
        target_speed_mm_s = cfg->saved_target_speed;
        pid.Kp = cfg->saved_kp;
        pid.Ki = cfg->saved_ki;
        pid.Kd = cfg->saved_kd;
    } 
    // Si no coincide el magic number, mantenemos los defaults hardcodeados
}

void restore_defaults(void) {
    target_temp = 245.0f;
    target_speed_mm_s = 3.0f;
    pid.Kp = 1.0f;   // Balanceado
    pid.Ki = 0.15f;
    pid.Kd = 60.0f;  // Alto para anticipación
    save_config_to_flash(); // Guardar defaults inmediatamente
}

void send_config_report(void) {
  // Enviar configuración actual como mensaje especial "PAR:..."
  // PAR:T=250.0,V=3.0,P=4.00,I=0.15,D=10.00
  char buf[100];
  
  // Manual float conversion for safety inside interrupt context or restricted printfs
  int t_int = (int)target_temp; int t_dec = (int)(fabs(target_temp - t_int)*10);
  int v_int = (int)target_speed_mm_s; int v_dec = (int)(fabs(target_speed_mm_s - v_int)*10);
  
  // PID (2 decimals)
  int p_int = (int)pid.Kp; int p_dec = (int)(fabs(pid.Kp - p_int)*100);
  int i_int = (int)pid.Ki; int i_dec = (int)(fabs(pid.Ki - i_int)*100);
  int d_int = (int)pid.Kd; int d_dec = (int)(fabs(pid.Kd - d_int)*100);

  int len = sprintf(buf, "PAR:T=%d.%d,V=%d.%d,P=%d.%02d,I=%d.%02d,D=%d.%02d\r\n", 
          t_int, t_dec, v_int, v_dec, 
          p_int, p_dec, i_int, i_dec, d_int, d_dec);
          
  HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
}

static float s_curve_profile(float progress) {
    if (progress <= 0.0f) return 0.0f;
    if (progress >= 1.0f) return 1.0f;
    return (1.0f - cosf(PI * progress)) / 2.0f;
}

uint32_t speed_mm_s_to_hz(float speed_mm_s) {
    if (speed_mm_s <= 0.0f) return 0;
    float hz = speed_mm_s * E_STEPS_PER_MM;
    if (hz > 20000.0f) hz = 20000.0f;
    return (uint32_t)hz;
}

static void check_safety_conditions(void) {
    uint32_t sum = 0;
    for (int i = 0; i < ADC_BUF_SIZE; i++) {
        sum += adc_buffer[i];
    }
    uint16_t adc_avg = sum / ADC_BUF_SIZE;
    
    // Check NTC desconectado
    // Si ya estamos en error de desconexión, no es necesario re-trigger continuamente
    // MODIFICADO: Agregamos isnan(tc_filt) como condición de seguridad adicional
    if (adc_avg < ADC_DISCONNECTED_LOW || adc_avg > ADC_DISCONNECTED_HIGH || isnan(tc_filt)) {
        if (system_error != ERROR_NTC_DISCONNECTED) {
            system_error = ERROR_NTC_DISCONNECTED;
            thermal_state = THERMAL_ERROR; 
            pid.output = 0.0f;
            heater_set_pwm(0.0f);
            motor_state = MOTOR_IDLE;
            motor_enable(false);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        }
        return;
    }
    
    // Check Sobretemperatura
    if (!isnan(tc_filt) && tc_filt > T_MAX_C) {
        if (system_error != ERROR_OVERTEMP) {
            system_error = ERROR_OVERTEMP;
            thermal_state = THERMAL_ERROR;
            pid.output = 0.0f;
            heater_set_pwm(0.0f);
            motor_state = MOTOR_IDLE;
            motor_enable(false);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        }
        return;
    }
    
    // Check Timeout Calentamiento
    if (heating_phase_active) {
        if (tc_filt >= TEMP_STARTUP_THRESHOLD) {
            heating_phase_active = false;
        } else if ((HAL_GetTick() - heating_start_time) > HEATING_TIMEOUT_MS) {
            if (system_error != ERROR_HEATING_TIMEOUT) {
                system_error = ERROR_HEATING_TIMEOUT;
                thermal_state = THERMAL_ERROR;
                pid.output = 0.0f;
                heater_set_pwm(0.0f);
            }
            return;
        }
    }
}

static void process_adc_buffer(void) {
    uint32_t sum = 0;
    for (int i = 0; i < ADC_BUF_SIZE; i++) {
        sum += adc_buffer[i];
    }
    uint16_t adc_avg = sum / ADC_BUF_SIZE;
    
    float tc_raw = ntc_temp_c_from_adc(adc_avg);
    
    if (isnan(tc_raw)) {
        tc_filt = NAN;
    } else {
        if (isnan(tc_filt)) {
            tc_filt = tc_raw;
        } else {
            tc_filt = ALPHA_TEMP * tc_raw + (1.0f - ALPHA_TEMP) * tc_filt;
        }
    }
}

static void motor_state_machine(void) {
  // ELIMINADAS lecturas directas de pines (ahora usamos cmd_motor_start/stop)
  
  bool sensor_ok = !isnan(tc_filt);
  bool temp_safe = sensor_ok && (tc_filt >= TEMP_SAFE_EXTRUSION);
  
  uint32_t target_hz = speed_mm_s_to_hz(target_speed_mm_s);
  if (target_hz < FREQ_MIN_HZ) target_hz = FREQ_MIN_HZ;

  switch (motor_state) {
    
    case MOTOR_IDLE:
      motor_enable(false);
      motor_running = false;
      current_freq = 0;
      
      // CAMBIO AQUÍ: Chequear flag física O flag UART
      if ((cmd_motor_start || uart_cmd_motor_start) && temp_safe) {
        motor_state = MOTOR_ACCEL;
        motor_running = true;
        motor_enable(true);
        current_freq = FREQ_MIN_HZ;
        set_motor_speed(current_freq);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        step_count = 0;
        
        // CONSUMIR BANDERAS
        uart_cmd_motor_start = false; 
        cmd_motor_start = false;

        update_lcd(); // Actualizar LCD inmediatamente
      } 
      // Si intentan arrancar en frío, mostrar aviso
      else if ((cmd_motor_start || uart_cmd_motor_start) && !temp_safe) {
          lcd_put_cur(1, 0);
          lcd_send_string("Cold Protect!   ");
          cold_protect_message_time = HAL_GetTick(); // Marca tiempo del mensaje
          
          // CONSUMIR BANDERAS (Aviso mostrado)
          uart_cmd_motor_start = false; 
          cmd_motor_start = false;
      }
      
      // Limpieza de seguridad: Si hay orden de stop en IDLE, la limpiamos
      if (cmd_motor_stop) cmd_motor_stop = false; 
      if (uart_cmd_motor_stop) uart_cmd_motor_stop = false;
      break;
      
    case MOTOR_ACCEL:
      motor_enable(true);
      motor_running = true;
      
      if (step_count < ACCEL_STEPS) {
        float progress = (float)step_count / (float)ACCEL_STEPS;
        float curve_factor = s_curve_profile(progress);
        current_freq = FREQ_MIN_HZ + (uint32_t)((target_hz - FREQ_MIN_HZ) * curve_factor);
        set_motor_speed(current_freq);
        step_count++;
      } else {
        motor_state = MOTOR_RUN;
        current_freq = target_hz;
        set_motor_speed(current_freq);
      }
      
      // CAMBIO AQUÍ: Chequear Stop Físico O UART
      if (cmd_motor_stop || uart_cmd_motor_stop || !temp_safe) {
        motor_state = MOTOR_DECEL;
        step_count = 0;
        
        // CONSUMIR BANDERAS
        uart_cmd_motor_start = false; // Cancel start if pending
        cmd_motor_start = false;
        uart_cmd_motor_stop = false;
        cmd_motor_stop = false;
      }
      break;
      
    case MOTOR_RUN:
      motor_enable(true);
      motor_running = true;
      
      int32_t freq_error = (int32_t)target_hz - (int32_t)current_freq;
      const uint32_t RAMP_STEP_HZ = 5;
      
      if (abs(freq_error) > RAMP_STEP_HZ) {
        if (freq_error > 0) current_freq += RAMP_STEP_HZ;
        else current_freq -= RAMP_STEP_HZ;
        set_motor_speed(current_freq);
      } else if (abs(freq_error) > 0) {
        current_freq = target_hz;
        set_motor_speed(current_freq);
      }
      
      // CAMBIO AQUÍ: Chequear Stop Físico O UART
      if (cmd_motor_stop || uart_cmd_motor_stop || !temp_safe) {
        motor_state = MOTOR_DECEL;
        step_count = 0;
        
        // CONSUMIR BANDERAS
        uart_cmd_motor_start = false; // Cancel start if pending
        cmd_motor_start = false;
        uart_cmd_motor_stop = false;
        cmd_motor_stop = false;
      }
      break;
      
    case MOTOR_DECEL:
      if (step_count < DECEL_STEPS) {
        float progress = (float)step_count / (float)DECEL_STEPS;
        float curve_factor = 1.0f - s_curve_profile(progress);
        current_freq = FREQ_MIN_HZ + (uint32_t)((target_hz - FREQ_MIN_HZ) * curve_factor);
        set_motor_speed(current_freq);
        step_count++;
      } else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        motor_state = MOTOR_IDLE;
        motor_enable(false);
        motor_running = false;
        current_freq = 0;
      }
      break;
      
    default:
        motor_enable(false);
        break;
  }
}

static void update_lcd(void) {
  char lcd_line1[17];
  char lcd_line2[17];

  // 1. Manejo de Errores (Máx 16 chars)
  if (system_error != ERROR_NONE) {
      if (system_error == ERROR_NTC_DISCONNECTED) 
          snprintf(lcd_line1, sizeof(lcd_line1), "ERROR: NTC DISC ");
      else if (system_error == ERROR_OVERTEMP) 
          snprintf(lcd_line1, sizeof(lcd_line1), "ERROR: OVERTEMP ");
      else 
          snprintf(lcd_line1, sizeof(lcd_line1), "ERROR: CODE %-4d", system_error);
  } 
  // 2. Error de Lectura
  else if (isnan(tc_filt)) {
      snprintf(lcd_line1, sizeof(lcd_line1), "T:Err / %.0fC     ", target_temp);
  } 
  // 3. Funcionamiento Normal
  else {
      char status_char;
      switch (thermal_state) {
          case THERMAL_IDLE:    status_char = '-'; break;
          case THERMAL_HEATING: status_char = '^'; break;
          case THERMAL_READY:   status_char = '*'; break;
          default:              status_char = '?'; break;
      }
      
      char buffer_temp[17];
      snprintf(buffer_temp, sizeof(buffer_temp), "T:%.1f/%.0f %c %c", 
               tc_filt, target_temp, status_char, heater_on ? 'H' : ' ');
      snprintf(lcd_line1, sizeof(lcd_line1), "%-16.16s", buffer_temp);
  }
  
  // Línea 2 (Motor y Estado)
  if (motor_state == MOTOR_IDLE) {
      if (thermal_state == THERMAL_IDLE) {
          // Solo pedimos START si está todo apagado
          snprintf(lcd_line2, sizeof(lcd_line2), "Press START     "); // 16 chars con espacios
      } 
      else if (thermal_state == THERMAL_HEATING) {
          // Si está calentando, mostramos progreso
          snprintf(lcd_line2, sizeof(lcd_line2), "Heating... %3d%% ", (int)pid.output);
      }
      else if (thermal_state == THERMAL_READY) {
          snprintf(lcd_line2, sizeof(lcd_line2), "Ready - 0 Hz    ");
      }
      else {
          snprintf(lcd_line2, sizeof(lcd_line2), "Wait...         ");
      }
  } else {
      char buff_speed[32];
      snprintf(buff_speed, sizeof(buff_speed), "%.1fmm/s %uHz", 
               target_speed_mm_s, current_freq);
      snprintf(lcd_line2, sizeof(lcd_line2), "%-16.16s", buff_speed);
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  
  lcd_init();
  lcd_clear();
  encoder_init();
  menu_init();

  // CARGAR CONFIGURACION GUARDADA
  load_config_from_flash();

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_SIZE);
  HAL_TIM_Base_Start_IT(&htim3);
  
  motor_enable(false); 
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, direction_cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
  heater_set_pwm(0.0f);
  
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_IWDG_Refresh(&hiwdg);
    
    // 1. BUZZER TIMEOUT: Apagar beeps temporales expirados
    if (buzzer_stop_time > 0 && HAL_GetTick() > buzzer_stop_time) {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        buzzer_stop_time = 0;
    }
    
    // 2. ALARMA Y LED (Prioridad 1 - Feedback de seguridad inmediato)
    // ERROR: Rojo + Alarma sonora
    if (system_error != ERROR_NONE) {
        uint32_t now = HAL_GetTick();
        // Patrón de Alarma: 500ms ON / 500ms OFF sincronizado
        if ((now / 500) % 2 == 0) {
           set_rgb(1, 0, 0); // Rojo ON
           HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        } else {
           set_rgb(0, 0, 0); // LED OFF
           HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        }
        buzzer_stop_time = 0; // Desactivar lógica normal de beeps
    } 
    // Prioridad 2: Funcionamiento Normal
    else {
        // LED RGB según estado térmico
        // Verde si está a ±5°C del target (independiente del estado)
        if (!isnan(tc_filt) && fabs(tc_filt - target_temp) <= 5.0f && thermal_state != THERMAL_IDLE) {
            set_rgb(0,1,0); // Verde
        }
        else if (thermal_state == THERMAL_HEATING) {
            // Cyan si falta menos de 20°C, Azul si falta más
            if(tc_filt > (target_temp - 20.0f)) set_rgb(0,1,1); else set_rgb(0,0,1);
        }
        else set_rgb(1,1,0); // AMARILLO (Sistema en Reposo - Listo para empezar)
        
        // Apagar Buzzer si no hay beep pendiente
        if (buzzer_stop_time == 0) {
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        }
    }
    
    // 3. INPUTS: Leer estado de controles físicos y encoder
    encoder_update();
    handle_physical_buttons();
    
    // 4. MOTOR: Ejecutar solo si pasó 1 milisegundo (Control de Rampa)
    if (flag_1ms) {
        motor_state_machine();
        flag_1ms = false;
    }
    
    if (menu_is_active()) {
      menu_update();
      menu_render();
      if (encoder_button_pressed()) menu_handle_button_short();
      if (encoder_button_long_press()) menu_handle_button_long();
      
    } 
    else {
      
      if (encoder_button_long_press()) {
        menu_enter();
        continue;
      }
      
      // Iniciar conversión ADC cada 100ms
      if (flag_100ms) {
        flag_100ms = false;
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_SIZE);
      }
      
      // 3. SENSORES Y SEGURIDAD (Solo si hay nuevos datos)
      if (adc_conversion_complete) {
        process_adc_buffer();
        check_safety_conditions(); // Puede cambiar thermal_state a ERROR
      }
        
      // 5. LÓGICA TÉRMICA (Ejecutar SIEMPRE para detectar botones)
      switch (thermal_state) {
          
          case THERMAL_IDLE:
            pid.output = 0.0f;
            heater_on = false;
            heating_phase_active = false;  // ✅ Asegurar que está limpio en IDLE
            
            // Limpiar comandos de stop si llegan estando ya apagado
            if (cmd_heater_stop) cmd_heater_stop = false;
            if (uart_cmd_heater_stop) uart_cmd_heater_stop = false;
            
            // ⬇️ MODIFICADO: Aceptar botón Físico (cmd) O comando UART (uart_cmd)
            if ((cmd_heater_start || uart_cmd_heater_start) && !isnan(tc_filt)) {
                thermal_state = THERMAL_HEATING;
                heating_phase_active = true;
                heating_start_time = HAL_GetTick();
                
                // Limpiar banderas
                cmd_heater_start = false;
                uart_cmd_heater_start = false; 
                
                if (huart1.gState == HAL_UART_STATE_READY) HAL_UART_Transmit(&huart1, (uint8_t*)"[TH] Start\r\n", 12, 100);
            }
            break;
            
          case THERMAL_HEATING:
            // Limpiar comandos de start si llegan estando ya calentando
            if (cmd_heater_start) cmd_heater_start = false;
            if (uart_cmd_heater_start) uart_cmd_heater_start = false;
            
            // Calcular PID solo si hay datos nuevos (cada 100ms)
            if (adc_conversion_complete) {
                pid.output = PID_Compute(&pid, target_temp, tc_filt, 0.1f);
                
                // Umbral simple: 3% de potencia PID
                // El PID con Kd alto previene overshoot naturalmente
                heater_on = (pid.output > 3.0f);
                
                // Chequear si llegamos al target
                if (!isnan(tc_filt) && fabs(tc_filt - target_temp) < 5.0f) {
                  thermal_state = THERMAL_READY;
                  heating_phase_active = false;
                  if (huart1.gState == HAL_UART_STATE_READY) HAL_UART_Transmit(&huart1, (uint8_t*)"[TH] Ready\r\n", 12, 100);
                }
            }
            
            // ⬇️ MODIFICADO: Apagado manual Físico o UART
            if (cmd_heater_stop || uart_cmd_heater_stop) {
                thermal_state = THERMAL_IDLE;
                heating_phase_active = false;  // ✅ Limpiar timer de timeout
                cmd_heater_stop = false;
                uart_cmd_heater_stop = false;
                if (huart1.gState == HAL_UART_STATE_READY) HAL_UART_Transmit(&huart1, (uint8_t*)"[TH] Stop\r\n", 12, 100);
            }
            
            if (system_error != ERROR_NONE) thermal_state = THERMAL_ERROR;
            break;
            
          case THERMAL_READY:
             // Limpiar comandos de start si llegan estando ya listo
             if (cmd_heater_start) cmd_heater_start = false;
             if (uart_cmd_heater_start) uart_cmd_heater_start = false;
             
             if (adc_conversion_complete) {
                pid.output = PID_Compute(&pid, target_temp, tc_filt, 0.1f);
                heater_on = (pid.output > 3.0f);  // Umbral simple
             }
            
            // ⬇️ MODIFICADO: Apagado manual Físico o UART
            if (cmd_heater_stop || uart_cmd_heater_stop) {
                thermal_state = THERMAL_IDLE;
                pid.output = 0.0f;
                heater_on = false;
                heating_phase_active = false;  // ✅ Limpiar timer de timeout
                cmd_heater_stop = false;
                uart_cmd_heater_stop = false;
                if (huart1.gState == HAL_UART_STATE_READY) HAL_UART_Transmit(&huart1, (uint8_t*)"[TH] Stop\r\n", 12, 100);
            }
            
            if (system_error != ERROR_NONE) thermal_state = THERMAL_ERROR;
            break;
            
          case THERMAL_ERROR:
            pid.output = 0.0f;
            heater_on = false;
            
            // "Acknowledge": Solo el botón START limpia el error (Requisito de seguridad)
            // Esto asegura que el usuario intencionadamente reinicia el sistema.
            if (cmd_motor_start) {
              system_error = ERROR_NONE;
              thermal_state = THERMAL_IDLE;
              
              // Consumir el comando de arranque para que no arranque el motor inmediatamente.
              // El usuario deberá iniciar calentamiento y luego motor.
              cmd_motor_start = false;
              
              // Limpiar otros comandos pendientes por seguridad
              uart_cmd_motor_start = false;
              cmd_heater_start = false;
              
              lcd_clear();
            }
            
            // Limpiar resto de banderas para evitar comportamientos extraños al salir del error
            cmd_motor_stop = false;
            cmd_heater_stop = false;
            cmd_heater_start = false;
            // No limpiamos las de UART por si el usuario quiere resetear por UART (aunque el if solo mira cmd_motor_start)
            // Si quieres permitir UART reset, cambia el if a: if (cmd_motor_start || uart_cmd_motor_start)
            if (uart_cmd_motor_start) {
                 // Permitimos Reset por UART también por comodidad en dashboard
                  system_error = ERROR_NONE;
                  thermal_state = THERMAL_IDLE;
                  uart_cmd_motor_start = false;
                  lcd_clear();
            }
            break;
      }
      
      // 6. SALIDA FÍSICA (CRÍTICO: APLICAR EL VALOR AL PIN)
      heater_set_pwm((thermal_state == THERMAL_IDLE) ? 0.0f : pid.output);
      
      // CAMBIO AQUI: Quitamos la condición restrictiva huart1.gState.
      // Solo chequeamos si hay nuevos datos (adc_conversion_complete)
      if (adc_conversion_complete) {
          float error = target_temp - tc_filt;
          
          int pwm_out = (thermal_state == THERMAL_IDLE) ? 0 : (int)pid.output;
          float actual_speed_mm_s = (float)current_freq / E_STEPS_PER_MM;
          
          // FIX: Manual formatting for floats (avoids linker flags issues)
          // Temp Actual
          int t_curr_int = (int)tc_filt;
          int t_curr_dec = (int)(fabs(tc_filt - t_curr_int) * 100);
          // Target
          int t_targ_int = (int)target_temp;
          int t_targ_dec = (int)(fabs(target_temp - t_targ_int) * 100);
          // Speed
          int spd_int = (int)actual_speed_mm_s;
          int spd_dec = (int)(fabs(actual_speed_mm_s - spd_int) * 100);
          // Error (PID)
          int err_int = (int)error;
          int err_dec = (int)(fabs(error - err_int) * 100);

          // STATUS FLAGS
          // Bit 0: Heater On
          // Bit 1: Motor Running
          // Bit 2: Temp Safe (Cold Protect: 0=Cold, 1=Safe)
          bool is_temp_safe = (!isnan(tc_filt) && tc_filt >= TEMP_SAFE_EXTRUSION);
          uint8_t status_flags = 0;
          if (heater_on)      status_flags |= (1 << 0);
          if (motor_running)  status_flags |= (1 << 1);
          if (is_temp_safe)   status_flags |= (1 << 2);

          // AÑADIDO: system_error y status_flags al final del CSV
          int len = sprintf(tx_buffer, "%lu,%d.%02d,%d.%02d,%d,%d.%02d,%d.%02d,%d,%d\r\n", 
                 HAL_GetTick(), 
                 t_curr_int, t_curr_dec,
                 t_targ_int, t_targ_dec,
                 pwm_out, 
                 spd_int, spd_dec,
                 err_int, err_dec,
                 (int)system_error,
                 status_flags); // <--- Columna 8: Flags
                 
          // Timeout de 50ms suficiente para enviar sin bloquear todo si está ocupado
          HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, len, 50);
      }
      
      // Limpiar bandera de ADC al FINAL
      if (adc_conversion_complete) {
          adc_conversion_complete = false;
          update_lcd();
      }
      
      // Restaurar LCD después de mensaje "Cold Protect"
      if (cold_protect_message_time > 0 && (HAL_GetTick() - cold_protect_message_time) > 800) {
          cold_protect_message_time = 0;
          update_lcd();
      }
    }
    
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

// ========== CALLBACK DE RECEPCIÓN UART (INTERRUPCIÓN) ==========
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    if (rx_byte == '\n' || rx_byte == '\r') {
      rx_buffer[rx_index] = '\0'; 
      if (rx_index > 0) {
        char cmd = rx_buffer[0];
        
        // --- T: SET TEMPERATURE ---
        if (cmd == 't' || cmd == 'T') {
          float new_temp = atof((char*)&rx_buffer[1]);
          if (new_temp >= 0.0f && new_temp <= T_MAX_C) {
             target_temp = new_temp;
             HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Temp Updated\r\n", 18, 100);
          }
        }
        // --- H: HEATER ON/OFF ---
        else if (cmd == 'h' || cmd == 'H') {
           if (rx_buffer[1] == '1') {
               uart_cmd_heater_start = true; 
               HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Heater ON\r\n", 15, 100);
           } else {
               uart_cmd_heater_stop = true; 
               HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Heater OFF\r\n", 16, 100);
           }
        }
        // --- V: VELOCIDAD ---
        else if (cmd == 'v' || cmd == 'V') {
          float new_speed = atof((char*)&rx_buffer[1]);
          if (new_speed >= 0.1f && new_speed <= 15.0f) { 
            target_speed_mm_s = new_speed;
            char ack[32]; sprintf(ack, "OK: Speed=%.1f\r\n", target_speed_mm_s);
            HAL_UART_Transmit(&huart1, (uint8_t*)ack, strlen(ack), 100);
          }
        }
        // --- S/R: STOP/RUN ---
        else if (cmd == 's' || cmd == 'S') {
            uart_cmd_motor_stop = true; // <--- USAR NUEVA VARIABLE
            HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Motor Stop\r\n", 16, 100);
        }
        else if (cmd == 'r' || cmd == 'R') {
            uart_cmd_motor_start = true; // <--- USAR NUEVA VARIABLE
            HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Motor Start\r\n", 17, 100);
        }
        // --- PID: KP ---
        else if (cmd == 'p' || cmd == 'P') {
            float val = atof((char*)&rx_buffer[1]);
            // Permitimos 0.0
            if (val >= 0.0f) {
                pid.Kp = val;
                HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Kp Updated\r\n", 16, 100);
            }
        }
        // --- PID: KI ---
        else if (cmd == 'i' || cmd == 'I') {
            float val = atof((char*)&rx_buffer[1]);
            if (val >= 0.0f) {
                pid.Ki = val;
                pid.integral = 0.0f; // Reset integral to avoid windup jumps
                HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Ki Updated\r\n", 16, 100);
            }
        }
        // --- PID: KD ---
        else if (cmd == 'd' || cmd == 'D') {
            float val = atof((char*)&rx_buffer[1]);
            if (val >= 0.0f) {
                pid.Kd = val;
                HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Kd Updated\r\n", 16, 100);
            }
        }
        // --- W: WRITE (SAVE) TO FLASH ---
        else if (cmd == 'w' || cmd == 'W') {
            save_config_to_flash();
            HAL_UART_Transmit(&huart1, (uint8_t*)"OK: CONFIG SAVED\r\n", 18, 100);
        }
        // --- X: FACTORY RESET ---
        else if (cmd == 'x' || cmd == 'X') {
            restore_defaults();
            send_config_report();
            HAL_UART_Transmit(&huart1, (uint8_t*)"OK: DEFAULTS RESTORED\r\n", 23, 100);
        }
        // --- ?: QUERY CONFIG ---
        else if (cmd == '?') {
            send_config_report();
        }
        else {
             HAL_UART_Transmit(&huart1, (uint8_t*)"ERR: Unknown\r\n", 14, 100);
        }
      }
      rx_index = 0; 
    } else {
      if (rx_index < (sizeof(rx_buffer) - 1)) rx_buffer[rx_index++] = rx_byte;
      else rx_index = 0; 
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

void handle_physical_buttons(void) {
    // NOTA: NO LIMPIAMOS LOS FLAGS AQUÍ PARA EVITAR PERDER LA ORDEN
    // SI EL LOOP CORRE MÁS RÁPIDO QUE EL TIMER DE 1MS.
    // LOS CONSUMIDORES (motor_state_machine, etc) DEBEN LIMPIARLOS.
    
    uint32_t now = HAL_GetTick();

    // --- START BUTTON (PA4) - MOTOR START ---
    static bool last_start_state = true;
    static uint32_t start_timer = 0;
    bool is_start_pressed = (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) == GPIO_PIN_RESET);
    
    if (is_start_pressed && last_start_state == true) {
        if ((now - start_timer) > 200) { // Debounce 200ms
            beep(100);
            cmd_motor_start = true;
            start_timer = now;
            last_start_state = false;
        }
    } else if (!is_start_pressed) {
        last_start_state = true;
    }

    // --- STOP BUTTON (PB12) - MOTOR STOP ---
    static bool last_stop_state = true;
    static uint32_t stop_timer = 0;
    bool is_stop_pressed = (HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port, STOP_BTN_Pin) == GPIO_PIN_RESET);
    
    if (is_stop_pressed && last_stop_state == true) {
        if ((now - stop_timer) > 200) { // Debounce 200ms
            beep(100);
            cmd_motor_stop = true;
            stop_timer = now;
            last_stop_state = false;
        }
    } else if (!is_stop_pressed) {
        last_stop_state = true;
    }

    // Toggle Heater (PB11)
    bool is_heater_pressed = (HAL_GPIO_ReadPin(HEATER_BTN_GPIO_Port, HEATER_BTN_Pin) == GPIO_PIN_RESET);
    if (is_heater_pressed && last_heater_btn_state == true) { 
        if ((now - heater_btn_timer) > 300) {
            beep(50);
            // Toggle Logic
            if (thermal_state == THERMAL_IDLE || thermal_state == THERMAL_ERROR) {
                cmd_heater_start = true;
            } else {
                cmd_heater_stop = true; 
            }
            heater_btn_timer = now;
        }
        last_heater_btn_state = false; 
    } else if (!is_heater_pressed) {
        last_heater_btn_state = true; 
    }
}

float PID_Compute(PID_Config *pid, float setpoint, float measured_value, float dt_seconds) {
    float error = setpoint - measured_value;
    float P = pid->Kp * error;
    if (fabs(error) < 15.0f) pid->integral += error * dt_seconds; else pid->integral = 0.0f;
    float I_term = pid->Ki * pid->integral; 
    if (I_term > 100.0f) { I_term = 100.0f; pid->integral = 100.0f / pid->Ki; }
    else if (I_term < -20.0f) { I_term = -20.0f; pid->integral = -20.0f / pid->Ki; }
    float derivative = (error - pid->prevError) / dt_seconds;
    float D = pid->Kd * derivative;
    pid->prevError = error;
    float output = P + I_term + D;
    if (output > 100.0f) output = 100.0f;
    if (output < 0.0f) output = 0.0f; 
    return output;
}

// ============================================
// TIMER CALLBACK: EL CORAZÓN DEL SISTEMA
// Maneja Rampa Motor (1ms) y Datos/Gráficos (100ms)
// ============================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    static uint32_t counter_100ms = 0;
    
    // Bandera para Rampa de Motor (1ms)
    flag_1ms = true; 
    
    counter_100ms++;
    if (counter_100ms >= 100) {
      flag_100ms = true;
      
      // IMPLEMENTACIÓN IDEAL:
      // Ya NO activamos adc_conversion_complete aquí.
      // Esperamos a que el callback del ADC DMA lo active cuando realmente termine (~1.3ms después)
      
      counter_100ms = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
