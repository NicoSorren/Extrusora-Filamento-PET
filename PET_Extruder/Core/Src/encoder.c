#include "encoder.h"

// ========== VARIABLES PRIVADAS ==========
static volatile int32_t encoder_count = 0;
static volatile EncoderDir_t last_direction = ENCODER_DIR_NONE;

// Flags de eventos para el main
static volatile bool event_button_short = false;
static volatile bool event_button_long = false;

// Variables de estado interno del botón
static uint32_t button_press_start_time = 0;
static bool button_is_pressed_physically = false;
static bool long_press_handled = false; // ⬅️ LA CLAVE: Evita re-disparos

static uint32_t last_clk_time = 0;
static bool last_clk_state = 1;

// Contadores de debug
static volatile uint32_t irq_count_clk = 0;
static volatile uint32_t irq_count_sw = 0;

// ========== INICIALIZACIÓN ==========
void encoder_init(void) {
  encoder_count = 0;
  last_direction = ENCODER_DIR_NONE;
  event_button_short = false;
  event_button_long = false;
  
  button_press_start_time = 0;
  button_is_pressed_physically = false;
  long_press_handled = false;
  
  // Leer estado inicial de CLK
  last_clk_state = HAL_GPIO_ReadPin(ENC_CLK_GPIO_Port, ENC_CLK_Pin);
}

// ========== FUNCIONES PÚBLICAS ==========
int32_t encoder_get_count(void) {
  return encoder_count;
}

void encoder_reset_count(void) {
  encoder_count = 0;
}

EncoderDir_t encoder_get_direction(void) {
  return last_direction;
}

// Devuelve true UNA VEZ por evento
bool encoder_button_pressed(void) {
  if (event_button_short) {
    event_button_short = false;
    return true;
  }
  return false;
}

// Devuelve true UNA VEZ por evento
bool encoder_button_long_press(void) {
  if (event_button_long) {
    event_button_long = false;
    return true;
  }
  return false;
}

uint32_t encoder_get_irq_count_clk(void) {
  return irq_count_clk;
}

uint32_t encoder_get_irq_count_sw(void) {
  return irq_count_sw;
}

// ========== CALLBACK DE INTERRUPCIÓN (ROTACIÓN) ==========
void encoder_irq_handler(void) {
  irq_count_clk++;
  uint32_t now = HAL_GetTick();
  
  if ((now - last_clk_time) < ENCODER_DEBOUNCE_MS) {
    return;
  }
  last_clk_time = now;
  
  bool clk = HAL_GPIO_ReadPin(ENC_CLK_GPIO_Port, ENC_CLK_Pin);
  bool dt = HAL_GPIO_ReadPin(ENC_DT_GPIO_Port, ENC_DT_Pin);
  
  if (clk != last_clk_state) {
    if (clk == dt) {
      encoder_count--;
      last_direction = ENCODER_DIR_CCW;
    } else {
      encoder_count++;
      last_direction = ENCODER_DIR_CW;
    }
    last_clk_state = clk;
  }
}

// ========== CALLBACK DEL BOTÓN (GPIO EXTI) ==========
// Ahora solo registramos que "algo pasó", la lógica la hacemos en update()
// para evitar rebotes y lógica compleja dentro de la interrupción.
void encoder_button_irq_handler(void) {
    // Solo necesitamos despertar o registrar actividad si el micro duerme.
    // Como usamos polling en update(), dejamos esto mínimo o vacío
    // para no conflictos. La lectura real la hace encoder_update.
    irq_count_sw++;
}

// ========== ACTUALIZACIÓN (LLAMAR EN EL LOOP) ==========
void encoder_update(void) {
  uint32_t now = HAL_GetTick();
  
  // Leemos el pin físico (0 = presionado, 1 = suelto)
  bool pin_state = HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin);
  bool is_pressed = (pin_state == 0); 
  
  // --- DETECCIÓN DE FLANCOS Y ESTADO ---
  
  if (is_pressed && !button_is_pressed_physically) {
    // FLANCO DE BAJADA (Acaba de presionar)
    button_is_pressed_physically = true;
    button_press_start_time = now;
    long_press_handled = false; // Resetear flag de "ya atendido"
  
  } else if (!is_pressed && button_is_pressed_physically) {
    // FLANCO DE SUBIDA (Acaba de soltar)
    button_is_pressed_physically = false;
    
    // Si soltamos rápido (< 2000ms) Y NO se manejó como largo... es CORTO
    if (!long_press_handled) {
       uint32_t duration = now - button_press_start_time;
       if (duration > 50) { // Debounce mínimo de 50ms para ignorar ruido
           event_button_short = true; 
       }
    }
  }
  
  // --- LÓGICA DE PULSACIÓN LARGA (DURANTE LA PRESIÓN) ---
  if (button_is_pressed_physically && !long_press_handled) {
      uint32_t duration = now - button_press_start_time;
      
      if (duration >= ENCODER_LONG_PRESS_MS) {
          event_button_long = true;  // ¡EVENTO CLICK LARGO!
          long_press_handled = true; // 🔒 TRABAR: No volver a disparar hasta soltar
      }
  }
}