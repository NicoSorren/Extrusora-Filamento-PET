#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

// ========== CONFIGURACIÓN DEL ENCODER ==========
#define ENCODER_DEBOUNCE_MS    2   // Ignorar pulsos más rápidos que 1 ms
#define ENCODER_LONG_PRESS_MS  1500 // Pulsación larga = 2 segundos

// ========== DIRECCIÓN DE ROTACIÓN ==========
typedef enum {
  ENCODER_DIR_NONE = 0,  // Sin movimiento
  ENCODER_DIR_CW,        // Clockwise (horario) → Incrementar valor
  ENCODER_DIR_CCW        // Counter-clockwise (antihorario) → Decrementar
} EncoderDir_t;

// ========== FUNCIONES PÚBLICAS ==========

/**
 * @brief Inicializar el encoder (llamar 1 vez en main, después de HAL_Init)
 */
void encoder_init(void);

/**
 * @brief Obtener contador acumulado (cantidad de clics desde el inicio)
 * @return int32_t - Positivo = CW, Negativo = CCW
 */
int32_t encoder_get_count(void);

/**
 * @brief Resetear contador a 0 (útil al cambiar de menú)
 */
void encoder_reset_count(void);

/**
 * @brief Obtener la última dirección detectada
 * @return EncoderDir_t - CW, CCW o NONE
 */
EncoderDir_t encoder_get_direction(void);

/**
 * @brief Detectar pulsación corta del botón (presionar y soltar < 2s)
 * @return true si fue presionado, false si no
 */
bool encoder_button_pressed(void);

/**
 * @brief Detectar pulsación larga del botón (mantener > 2s)
 * @return true si se mantuvo presionado > 2s
 */
bool encoder_button_long_press(void);

/**
 * @brief Callback de interrupción (llamada automáticamente por EXTI)
 * NO LLAMAR MANUALMENTE - El hardware la ejecuta al girar el encoder
 */
void encoder_irq_handler(void);

void encoder_update(void);

// ========== CALLBACKS DE INTERRUPCIÓN ==========
void encoder_button_irq_handler(void);  // ⬅️ AGREGAR ESTA LÍNEA

// ========== FUNCIONES DE DEBUG ==========
uint32_t encoder_get_irq_count_clk(void);
uint32_t encoder_get_irq_count_sw(void);

#endif // ENCODER_H