#include "menu.h"
#include "encoder.h"
#include "i2c-lcd.h"
#include <stdio.h>
#include <string.h>

// ========== VARIABLES EXTERNAS (de main.c) ==========
extern float target_speed_mm_s;
extern float target_temp;
extern PID_Config pid;
extern MotorState motor_state;
extern ThermalState thermal_state;
extern bool heater_on;
extern uint32_t error_count;
// --- EXPUESTAS PARA LIVE VIEW ---
extern float tc_filt;
extern uint16_t current_freq;
extern SystemError_t system_error;

// ========== VARIABLES PRIVADAS DEL MENÚ ==========
static MenuContext_t menu_ctx;
static int32_t last_encoder_count = 0;

// Variables editables (copias locales para edición)
static int32_t edit_temp_setpoint = 230;   // °C
static float edit_motor_speed = 2.5f;      // mm/s
static float edit_pid_kp = 2.0f;
static float edit_pid_ki = 0.1f;
static float edit_pid_kd = 1.0f;
static bool edit_motor_direction = true;   // true=CW, false=CCW

// Buffers para Etiquetas Dinámicas
static char str_temp_live[17];
static char str_speed_live[17];
static char str_uptime_live[17];
static char str_error_live[17];

// ========== DECLARACIÓN FORWARD DE ACCIONES ==========
static void action_reset_stats(void);
static void action_apply_motor_config(void);
static void action_apply_thermal_config(void);
static void action_save_config_flash(void);
static void action_restore_defaults(void);

// ========== DEFINICIÓN DE MENÚS ==========

// --- MENÚ: Ver Estado ---
MenuItem_t menu_view_status[] = {
  {
    .label = "T: ---.- / ---", // Se sobrescribe dinámicamente
    .type = MENU_ITEM_BACK, // BACK permite salir clickeando
    .parent = menu_main
  },
  {
    .label = "V: ---.- mm/s", // Se sobrescribe dinámicamente
    .type = MENU_ITEM_BACK,
    .parent = menu_main
  },
  {
    .label = "<< Volver",
    .type = MENU_ITEM_BACK,
    .parent = menu_main
  },
  { .label = NULL }  // Terminador
};

// --- MENÚ: Configurar Motor ---
MenuItem_t menu_config_motor[] = {
  {
    .label = "Velocidad",
    .type = MENU_ITEM_VALUE_FLOAT,
    .data.value = {
      .value_ptr = &edit_motor_speed,
      .min = 5,      // 0.5 mm/s (× 10)
      .max = 50,     // 5.0 mm/s (× 10)
      .step = 1,     // 0.1 mm/s
      .unit = "mm/s"
    },
    .parent = menu_main
  },
  {
    .label = "Sentido",
    .type = MENU_ITEM_TOGGLE,
    .data.value = {
      .value_ptr = &edit_motor_direction,
      .unit = ""
    },
    .parent = menu_main
  },
  {
    .label = "Aplicar",
    .type = MENU_ITEM_ACTION,
    .data.action = action_apply_motor_config,
    .parent = menu_main
  },
  {
    .label = "Volver",
    .type = MENU_ITEM_BACK,
    .parent = menu_main
  },
  { .label = NULL }
};

// --- MENÚ: Configurar Térmica ---
MenuItem_t menu_config_thermal[] = {
  {
    .label = "Setpoint",
    .type = MENU_ITEM_VALUE_INT,
    .data.value = {
      .value_ptr = &edit_temp_setpoint,
      .min = 200,
      .max = 260,
      .step = 5,
      .unit = "\xDF""C"  // Símbolo de grado (°C)
    },
    .parent = menu_main
  },
  {
    .label = "PID - Kp",
    .type = MENU_ITEM_VALUE_FLOAT,
    .data.value = {
      .value_ptr = &edit_pid_kp,
      .min = 0,         // Sin límite inferior
      .max = 10000,     // Max 100.0 (10000 * 0.01)
      .step = 10,       // Incremento 0.1 (10 * 0.01)
      .unit = ""
    },
    .parent = menu_main
  },
  {
    .label = "PID - Ki",
    .type = MENU_ITEM_VALUE_FLOAT,
    .data.value = {
      .value_ptr = &edit_pid_ki,
      .min = 0,         // Sin límite inferior
      .max = 10000,     // Max 100.0 (10000 * 0.01)
      .step = 10,       // Incremento 0.1 (10 * 0.01)
      .unit = ""
    },
    .parent = menu_main
  },
  {
    .label = "PID - Kd",
    .type = MENU_ITEM_VALUE_FLOAT,
    .data.value = {
      .value_ptr = &edit_pid_kd,
      .min = 0,         // Sin límite inferior
      .max = 10000,     // Max 100.0 (10000 * 0.01)
      .step = 10,       // Incremento 0.1 (10 * 0.01)
      .unit = ""
    },
    .parent = menu_main
  },
  {
    .label = "Aplicar",
    .type = MENU_ITEM_ACTION,
    .data.action = action_apply_thermal_config,
    .parent = menu_main
  },
  {
    .label = "Volver",
    .type = MENU_ITEM_BACK,
    .parent = menu_main
  },
  { .label = NULL }
};

// --- MENÚ: Estadísticas ---
MenuItem_t menu_stats[] = {
  {
    .label = "Up: -----s",
    .type = MENU_ITEM_BACK, // BACK permite salir clickeando
    .parent = menu_main
  },
  {
    .label = "Err: - Code: -",
    .type = MENU_ITEM_BACK,
    .parent = menu_main
  },
  {
    .label = "Resetear Stats",
    .type = MENU_ITEM_ACTION,
    .data.action = action_reset_stats,
    .parent = menu_main
  },
  {
    .label = "<< Volver",
    .type = MENU_ITEM_BACK,
    .parent = menu_main
  },
  { .label = NULL }
};

// --- MENÚ PRINCIPAL ---
MenuItem_t menu_main[] = {
  {
    .label = "Ver Estado",
    .type = MENU_ITEM_SUBMENU,
    .data.submenu = menu_view_status,
    .parent = NULL
  },
  {
    .label = "Config Motor",
    .type = MENU_ITEM_SUBMENU,
    .data.submenu = menu_config_motor,
    .parent = NULL
  },
  {
    .label = "Config Termica",
    .type = MENU_ITEM_SUBMENU,
    .data.submenu = menu_config_thermal,
    .parent = NULL
  },
  {
    .label = "Estadisticas",
    .type = MENU_ITEM_SUBMENU,
    .data.submenu = menu_stats,
    .parent = NULL
  },
  {
    .label = "Guardar Config",
    .type = MENU_ITEM_ACTION,
    .data.action = action_save_config_flash,
    .parent = NULL
  },
  {
    .label = "Rest. Fabrica",
    .type = MENU_ITEM_ACTION,
    .data.action = action_restore_defaults,
    .parent = NULL
  },
  {
    .label = "Salir",
    .type = MENU_ITEM_BACK,
    .parent = NULL
  },
  { .label = NULL }  // Terminador
};

// ========== FUNCIONES DE ACCIÓN ==========

static void action_save_config_flash(void) {
    lcd_clear();
    lcd_put_cur(0, 0);
    lcd_send_string("Guardando...");
    
    // Llamar a la función real de main.c
    save_config_to_flash();
    
    HAL_Delay(500);
    lcd_put_cur(1, 0);
    lcd_send_string("Guardado OK!");
    HAL_Delay(1000);
}

static void action_reset_stats(void) {
  error_count = 0;
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("Stats Reseteadas");
  HAL_Delay(1500);
}

static void action_apply_motor_config(void) {
  target_speed_mm_s = edit_motor_speed;
  // TODO: Aplicar sentido del motor (si implementaste dirección en hardware)
  send_config_report(); // Notificar UART
  
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("Config Aplicada");
  HAL_Delay(1500);
}

static void action_apply_thermal_config(void) {
  target_temp = (float)edit_temp_setpoint;
  pid.Kp = edit_pid_kp;
  pid.Ki = edit_pid_ki;
  pid.Kd = edit_pid_kd;
  send_config_report(); // Notificar UART
  
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("Config Aplicada");
  HAL_Delay(1500);
}

static void action_restore_defaults(void) {
    lcd_clear();
    lcd_put_cur(0, 0);
    lcd_send_string("Reset Defaults..");
    restore_defaults();
    send_config_report(); // Notificar UART
    HAL_Delay(1000);
    lcd_put_cur(1, 0);
    lcd_send_string("OK! Reiniciando.");
    HAL_Delay(1000);
}

// ========== FUNCIONES AUXILIARES ==========

static void menu_refresh_dynamic_labels(void) {
    // Si estamos en "Ver Estado"
    if (menu_ctx.current_menu == menu_view_status) {
        // Temp
        snprintf(str_temp_live, 16, "T:%.1f / %.0f", tc_filt, target_temp);
        menu_view_status[0].label = str_temp_live;
        
        // Velocidad
        // Calculamos velocidad real aprox basada en freq
        // O mejor mostramos Target vs Real freq?
        // Simplificación: Mostramos Target Speed
        snprintf(str_speed_live, 16, "V:%.1f mm/s", target_speed_mm_s);
        menu_view_status[1].label = str_speed_live;
    }
    // Si estamos en "Estadisticas"
    else if (menu_ctx.current_menu == menu_stats) {
        // Uptime
        uint32_t uptime_sec = HAL_GetTick() / 1000;
        snprintf(str_uptime_live, 16, "Up: %lums", HAL_GetTick()); // ms cabe mejor o sec
        if(uptime_sec < 60) snprintf(str_uptime_live, 16, "Up: %lus", uptime_sec);
        else snprintf(str_uptime_live, 16, "Up: %lum", uptime_sec/60);
        menu_stats[0].label = str_uptime_live;
        
        // Errors
        snprintf(str_error_live, 16, "E:%lu C:%d", error_count, system_error);
        menu_stats[1].label = str_error_live;
    }
}

static uint8_t menu_count_items(MenuItem_t *menu) {
  uint8_t count = 0;
  while (menu[count].label != NULL) {
    count++;
  }
  return count;
}

// ========== FUNCIONES PÚBLICAS ==========

void menu_init(void) {
  menu_ctx.state = MENU_STATE_IDLE;
  menu_ctx.current_menu = NULL;
  menu_ctx.selected_index = 0;
  menu_ctx.item_count = 0;
  menu_ctx.last_activity = 0;
  menu_ctx.blink_state = false;
  menu_ctx.last_blink = 0;
  
  last_encoder_count = encoder_get_count();
  
  // Cargar valores actuales en las variables de edición
  edit_temp_setpoint = (int32_t)target_temp;
  edit_motor_speed = target_speed_mm_s;
  edit_pid_kp = pid.Kp;
  edit_pid_ki = pid.Ki;
  edit_pid_kd = pid.Kd;
}

void menu_enter(void) {
  if (menu_ctx.state == MENU_STATE_IDLE) {
    menu_ctx.state = MENU_STATE_BROWSE;
    menu_ctx.current_menu = menu_main;
    menu_ctx.selected_index = 0;
    menu_ctx.item_count = menu_count_items(menu_main);
    menu_ctx.last_activity = HAL_GetTick();
    
    // REFRESCAR VALORES AL ENTRAR (Fix Sync UUID/LCD)
    edit_temp_setpoint = (int32_t)target_temp;
    edit_motor_speed = target_speed_mm_s;
    edit_pid_kp = pid.Kp;
    edit_pid_ki = pid.Ki;
    edit_pid_kd = pid.Kd;
    
    encoder_reset_count();
    last_encoder_count = 0;
    
    lcd_clear();
  }
}

void menu_exit(void) {
  menu_ctx.state = MENU_STATE_IDLE;
  menu_ctx.current_menu = NULL;
  lcd_clear();
}

void menu_handle_encoder(int32_t delta) {
  menu_ctx.last_activity = HAL_GetTick();
  
  if (menu_ctx.state == MENU_STATE_BROWSE) {
    // Navegar por el menú
    int32_t new_index = (int32_t)menu_ctx.selected_index + delta;
    
    if (new_index < 0) {
      new_index = menu_ctx.item_count - 1;  // Wrap al final
    } else if (new_index >= menu_ctx.item_count) {
      new_index = 0;  // Wrap al inicio
    }
    
    menu_ctx.selected_index = (uint8_t)new_index;
    
  } else if (menu_ctx.state == MENU_STATE_EDIT) {
    // Editar valor
    MenuItem_t *item = &menu_ctx.current_menu[menu_ctx.selected_index];
    
    if (item->type == MENU_ITEM_VALUE_INT) {
      int32_t *val = (int32_t *)item->data.value.value_ptr;
      int32_t new_val = *val + (delta * item->data.value.step);
      
      if (new_val < item->data.value.min) new_val = item->data.value.min;
      if (new_val > item->data.value.max) new_val = item->data.value.max;
      
      *val = new_val;
      
    } else if (item->type == MENU_ITEM_VALUE_FLOAT) {
      float *val = (float *)item->data.value.value_ptr;
      // Incremento de 0.01 tal como solicitado
      float new_val = *val + (delta * item->data.value.step * 0.01f);
      
      if (new_val < item->data.value.min * 0.01f) new_val = item->data.value.min * 0.01f;
      if (new_val > item->data.value.max * 0.01f) new_val = item->data.value.max * 0.01f;
      
      *val = new_val;
      
    } else if (item->type == MENU_ITEM_TOGGLE) {
      bool *val = (bool *)item->data.value.value_ptr;
      *val = !(*val);
    }
  }
}

void menu_handle_button_short(void) {
  menu_ctx.last_activity = HAL_GetTick();
  
  if (menu_ctx.state == MENU_STATE_IDLE) {
    return;  // No hacer nada si no estamos en el menú
  }
  
  if (menu_ctx.state == MENU_STATE_BROWSE) {
    MenuItem_t *item = &menu_ctx.current_menu[menu_ctx.selected_index];
    
    switch (item->type) {
      case MENU_ITEM_SUBMENU:
        menu_ctx.current_menu = item->data.submenu;
        menu_ctx.selected_index = 0;
        menu_ctx.item_count = menu_count_items(item->data.submenu);
        encoder_reset_count();
        last_encoder_count = 0;
        lcd_clear();
        break;
        
      case MENU_ITEM_ACTION:
        if (item->data.action != NULL) {
          item->data.action();  // Ejecutar acción
        }
        break;
        
      case MENU_ITEM_VALUE_INT:
      case MENU_ITEM_VALUE_FLOAT:
      case MENU_ITEM_TOGGLE:
        menu_ctx.state = MENU_STATE_EDIT;
        menu_ctx.edit_value_backup = *(int32_t *)item->data.value.value_ptr;
        encoder_reset_count();
        last_encoder_count = 0;
        lcd_clear();
        break;
        
      case MENU_ITEM_BACK:
        if (item->parent != NULL) {
          menu_ctx.current_menu = item->parent;
          menu_ctx.selected_index = 0;
          menu_ctx.item_count = menu_count_items(item->parent);
        } else {
          menu_exit();
        }
        encoder_reset_count();
        last_encoder_count = 0;
        lcd_clear();
        break;
    }
    
  } else if (menu_ctx.state == MENU_STATE_EDIT) {
    // Guardar y salir del modo edición
    menu_ctx.state = MENU_STATE_BROWSE;
    encoder_reset_count();
    last_encoder_count = 0;
    lcd_clear();
  }
}

void menu_handle_button_long(void) {
  if (menu_ctx.state == MENU_STATE_EDIT) {
    // Cancelar edición (restaurar valor anterior)
    MenuItem_t *item = &menu_ctx.current_menu[menu_ctx.selected_index];
    *(int32_t *)item->data.value.value_ptr = menu_ctx.edit_value_backup;
    
    menu_ctx.state = MENU_STATE_BROWSE;
    encoder_reset_count();
    last_encoder_count = 0;
    lcd_clear();
    
  } else if (menu_ctx.state == MENU_STATE_BROWSE) {
    // Volver al menú anterior o salir
    MenuItem_t *current = menu_ctx.current_menu;
    if (current->parent != NULL) {
      menu_ctx.current_menu = current->parent;
      menu_ctx.selected_index = 0;
      menu_ctx.item_count = menu_count_items(current->parent);
    } else {
      menu_exit();
    }
    encoder_reset_count();
    last_encoder_count = 0;
    lcd_clear();
  }
}

void menu_update(void) {
  // Refrescar etiquetas dinámicas antes de chequear estado IDLE
  // (Para que si entras, los textos ya esten listos)
  if (menu_ctx.state != MENU_STATE_IDLE) {
      menu_refresh_dynamic_labels();
  }

  if (menu_ctx.state == MENU_STATE_IDLE) {
    return;  // No actualizar si no estamos en el menú
  }
  
  // Timeout: salir del menú si no hay actividad en 30 segundos
  if ((HAL_GetTick() - menu_ctx.last_activity) > MENU_TIMEOUT_MS) {
    menu_exit();
    return;
  }
  
  // Actualizar parpadeo (modo edición)
  if (menu_ctx.state == MENU_STATE_EDIT) {
    if ((HAL_GetTick() - menu_ctx.last_blink) > MENU_BLINK_MS) {
      menu_ctx.blink_state = !menu_ctx.blink_state;
      menu_ctx.last_blink = HAL_GetTick();
    }
  }
  
  // Detectar cambios en el encoder
  int32_t current_count = encoder_get_count();  // Dividir por 2 para compensar
  int32_t delta = current_count - last_encoder_count;
  
  if (delta != 0) {
    menu_handle_encoder(delta);
    last_encoder_count = current_count;
  }
}

void menu_render(void) {
  if (menu_ctx.state == MENU_STATE_IDLE) {
    return;  // No renderizar si no estamos en el menú
  }
  
  MenuItem_t *item = &menu_ctx.current_menu[menu_ctx.selected_index];
  
  if (menu_ctx.state == MENU_STATE_BROWSE) {
    // Mostrar item actual del menú
    lcd_put_cur(0, 0);
    char line1[17];
    snprintf(line1, sizeof(line1), ">%-15s", item->label);
    lcd_send_string(line1);
    
    // Mostrar item siguiente (si existe)
    lcd_put_cur(1, 0);
    if (menu_ctx.selected_index + 1 < menu_ctx.item_count) {
      MenuItem_t *next_item = &menu_ctx.current_menu[menu_ctx.selected_index + 1];
      char line2[17];
      snprintf(line2, sizeof(line2), " %-15s", next_item->label);
      lcd_send_string(line2);
    } else {
      lcd_send_string("                ");  // Limpiar línea
    }
    
  } else if (menu_ctx.state == MENU_STATE_EDIT) {
    // Modo edición
    lcd_put_cur(0, 0);
    lcd_send_string(item->label);
    lcd_send_string(":");
    
    lcd_put_cur(1, 0);
    char line2[17];
    
    if (item->type == MENU_ITEM_VALUE_INT) {
      int32_t val = *(int32_t *)item->data.value.value_ptr;
      snprintf(line2, sizeof(line2), "  %ld %s", val, item->data.value.unit);
    } else if (item->type == MENU_ITEM_VALUE_FLOAT) {
      float val = *(float *)item->data.value.value_ptr;
      snprintf(line2, sizeof(line2), "  %.1f %s", val, item->data.value.unit);
    } else if (item->type == MENU_ITEM_TOGGLE) {
      bool val = *(bool *)item->data.value.value_ptr;
      snprintf(line2, sizeof(line2), "  %s", val ? "SI" : "NO");
    }
    
    // Parpadeo del cursor
    if (menu_ctx.blink_state) {
      lcd_send_string(line2);
    } else {
      lcd_send_string("                ");  // Ocultar durante parpadeo
    }
  }
}

bool menu_is_active(void) {
  return (menu_ctx.state != MENU_STATE_IDLE);
}

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */