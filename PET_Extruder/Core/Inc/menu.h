#ifndef MENU_H
#define MENU_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

// ========== DECLARACIÓN FORWARD DE TIPOS (desde main.c) ==========
typedef struct {
    float Kp; 
    float Ki; 
    float Kd; 
    float prevError;
    float integral;
    float output;
} PID_Config;

typedef enum {
  MOTOR_IDLE = 0,
  MOTOR_ACCEL,   // ⬅️ AGREGADO
  MOTOR_RUN,     // ⬅️ AGREGADO (era RUNNING)
  MOTOR_DECEL,   // ⬅️ AGREGADO (era STOPPED)
  MOTOR_ERROR
} MotorState;

typedef enum {
  THERMAL_IDLE = 0,
  THERMAL_HEATING,
  THERMAL_READY,
  THERMAL_ERROR
} ThermalState;

// ========== CONFIGURACIÓN DEL MENÚ ==========
#define MENU_MAX_ITEMS       20   // Máximo número de items totales
#define MENU_TIMEOUT_MS      30000 // 30s sin actividad → salir del menú
#define MENU_BLINK_MS        500  // Velocidad de parpadeo en modo edición

// ========== TIPOS DE ITEMS DEL MENÚ ==========
typedef enum {
  MENU_ITEM_SUBMENU,   // Entra a otro menú (ej: "Config Motor")
  MENU_ITEM_ACTION,    // Ejecuta una función (ej: "Resetear Stats")
  MENU_ITEM_VALUE_INT, // Editar entero (ej: Setpoint temperatura)
  MENU_ITEM_VALUE_FLOAT, // Editar float (ej: Velocidad motor)
  MENU_ITEM_TOGGLE,    // Toggle ON/OFF (ej: Sentido motor)
  MENU_ITEM_BACK       // Volver al menú anterior
} MenuItemType_t;

// ========== ESTRUCTURA DE UN ITEM DEL MENÚ ==========
typedef struct MenuItem_t {
  const char *label;          // Texto mostrado en LCD (max 16 chars)
  MenuItemType_t type;        // Tipo de item
  
  union {
    struct MenuItem_t *submenu; // Puntero a submenú (si type == SUBMENU)
    void (*action)(void);       // Función a ejecutar (si type == ACTION)
    
    struct {
      void *value_ptr;          // Puntero a la variable a editar
      int32_t min;              // Valor mínimo
      int32_t max;              // Valor máximo
      int32_t step;             // Incremento por clic
      const char *unit;         // Unidad (ej: "mm/s", "°C")
    } value;
  } data;
  
  struct MenuItem_t *parent;  // Puntero al menú padre (para volver atrás)
} MenuItem_t;

// ========== ESTADOS DEL MENÚ ==========
typedef enum {
  MENU_STATE_IDLE,      // Pantalla principal (no en menú)
  MENU_STATE_BROWSE,    // Navegando por el menú
  MENU_STATE_EDIT       // Editando un valor
} MenuState_t;

// ========== ESTRUCTURA DE CONTEXTO DEL MENÚ ==========
typedef struct {
  MenuState_t state;
  MenuItem_t *current_menu;    // Menú actual
  uint8_t selected_index;      // Índice del item seleccionado
  uint8_t item_count;          // Cantidad de items en el menú actual
  
  uint32_t last_activity;      // Timestamp de última actividad
  bool blink_state;            // Estado del parpadeo (edición)
  uint32_t last_blink;         // Timestamp del último parpadeo
  
  int32_t edit_value_backup;   // Backup del valor antes de editar
} MenuContext_t;

// ========== FUNCIONES PÚBLICAS ==========

/**
 * @brief Inicializar sistema de menú
 */
void menu_init(void);

/**
 * @brief Actualizar menú (llamar en el loop principal cada 50ms)
 */
void menu_update(void);

/**
 * @brief Renderizar menú en LCD
 */
void menu_render(void);

/**
 * @brief Entrar al menú principal (llamar al hacer long-press del encoder)
 */
void menu_enter(void);

/**
 * @brief Salir del menú (volver a pantalla principal)
 */
void menu_exit(void);

/**
 * @brief Manejar rotación del encoder (llamar cuando encoder_get_count() cambie)
 * @param delta: +1 para CW, -1 para CCW
 */
void menu_handle_encoder(int32_t delta);

/**
 * @brief Manejar pulsación corta del encoder (seleccionar/confirmar)
 */
void menu_handle_button_short(void);

/**
 * @brief Manejar pulsación larga del encoder (cancelar/salir)
 */
void menu_handle_button_long(void);

/**
 * @brief Comprobar si el menú está activo
 */
bool menu_is_active(void);

// ========== MENÚS EXTERNOS (definidos en menu.c) ==========
extern MenuItem_t menu_main[];
extern MenuItem_t menu_view_status[];
extern MenuItem_t menu_config_motor[];
extern MenuItem_t menu_config_thermal[];
extern MenuItem_t menu_stats[];

#endif // MENU_H