#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

static const char *TAG = "HMI_CLIMATIZADOR";

// ============================================================================
// 1. CONFIGURAÇÕES
// ============================================================================
#define LED_ON           0
#define LED_OFF          1
#define BTN_PRESSED      0
#define LONG_PRESS_MS    1500

#define F_MIN            10
#define F_MAX            60

// Botões
static const gpio_num_t BTN_PINS[] = {
    GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27,
    GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_4,  GPIO_NUM_2
};
#define BTN_COUNT (sizeof(BTN_PINS) / sizeof(BTN_PINS[0]))

// LEDs
static const gpio_num_t LED_PINS[] = {
    GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23
};
#define LED_COUNT (sizeof(LED_PINS) / sizeof(LED_PINS[0]))

typedef enum { MODE_OP, MODE_MENU } HmiState_t;

typedef enum {
    ID_MAIS = 0, ID_MENOS, ID_CLIMATIZAR, ID_VENTILAR, ID_DRENO,
    ID_SWING, ID_EXAUSTAO, ID_ONOFF, ID_SET, ID_RESET_WIFI
} ButtonIdx_t;

typedef struct {
    uint32_t press_start_tick;
    bool is_pressed;
    bool short_click_pending;
    bool long_press_pending;
} ButtonHandler_t;

// ============================================================================
// 2. MENU DE PARÂMETROS
// ============================================================================
typedef enum {
    MENU_FMIN = 0,
    MENU_FMAX,
    MENU_DRENO_TIME,
    MENU_EXAUST_TIME,
    MENU_COUNT
} MenuItem_t;

static MenuItem_t menu_item = MENU_FMIN;

static int param_fmin = F_MIN;
static int param_fmax = F_MAX;
static int param_dreno_time = 60;
static int param_exaust_time = 120;

// ============================================================================
// 3. VARIÁVEIS GLOBAIS
// ============================================================================
static HmiState_t hmi_mode = MODE_OP;
static bool system_on = false;
static int current_freq = F_MIN;

static bool bomba_on = false;
static bool swing_on = false;
static bool dreno_active = false;
static bool exaustao_on = false;

static ButtonHandler_t btns[BTN_COUNT];

// ============================================================================
// 4. FUNÇÕES AUXILIARES MENU
// ============================================================================
static int* menu_get_value_ptr(MenuItem_t item) {
    switch (item) {
        case MENU_FMIN:        return &param_fmin;
        case MENU_FMAX:        return &param_fmax;
        case MENU_DRENO_TIME:  return &param_dreno_time;
        case MENU_EXAUST_TIME: return &param_exaust_time;
        default: return NULL;
    }
}

static void menu_get_limits(MenuItem_t item, int *min, int *max) {
    switch (item) {
        case MENU_FMIN:        *min = 5;   *max = 30;  break;
        case MENU_FMAX:        *min = 30;  *max = 80;  break;
        case MENU_DRENO_TIME:  *min = 10;  *max = 300; break;
        case MENU_EXAUST_TIME: *min = 10;  *max = 300; break;
        default: *min = 0; *max = 0; break;
    }
}

// ============================================================================
// 5. COMUNICAÇÃO MI (SIMULAÇÃO)
// ============================================================================
void enviar_comando_MI(const char* comando, int valor) {
    ESP_LOGI("MI_COMM", ">> %s | %d", comando, valor);
}

// ============================================================================
// 6. HARDWARE
// ============================================================================
void init_hw(void) {
    uint64_t led_mask = 0, btn_mask = 0;

    for (int i = 0; i < LED_COUNT; i++) led_mask |= (1ULL << LED_PINS[i]);
    for (int i = 0; i < BTN_COUNT; i++) btn_mask |= (1ULL << BTN_PINS[i]);

    gpio_config_t led_conf = {
        .pin_bit_mask = led_mask,
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&led_conf);

    gpio_config_t btn_conf = {
        .pin_bit_mask = btn_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1
    };
    gpio_config(&btn_conf);

    memset(btns, 0, sizeof(btns));
}

// ============================================================================
// 7. LEITURA DE BOTÕES (ORIGINAL)
// ============================================================================
void monitor_buttons(void) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (int i = 0; i < BTN_COUNT; i++) {
        int level = gpio_get_level(BTN_PINS[i]);

        if (level == BTN_PRESSED) {
            if (!btns[i].is_pressed) {
                btns[i].is_pressed = true;
                btns[i].press_start_tick = now;
            } else if ((now - btns[i].press_start_tick > LONG_PRESS_MS)
                        && !btns[i].long_press_pending) {
                btns[i].long_press_pending = true;
            }
        } else {
            if (btns[i].is_pressed) {
                if (!btns[i].long_press_pending)
                    btns[i].short_click_pending = true;
                btns[i].is_pressed = false;
                btns[i].long_press_pending = false;
            }
        }
    }
}

// ============================================================================
// 8. LÓGICA PRINCIPAL (CORRIGIDA)
// ============================================================================
void process_logic(void) {

    // RESET WIFI
    if (btns[ID_RESET_WIFI].short_click_pending) {
        ESP_LOGW(TAG, "Reset Wi-Fi");
        nvs_flash_erase();
        esp_restart();
    }

    // ON / OFF (sempre ativo)
    if (btns[ID_ONOFF].short_click_pending) {
        system_on = !system_on;
        if (!system_on) {
            enviar_comando_MI("OFF", 0);
            bomba_on = swing_on = exaustao_on = dreno_active = false;
        } else {
            enviar_comando_MI("ON", 1);
        }
        btns[ID_ONOFF].short_click_pending = false;
    }

    if (!system_on) {
        for (int i = 0; i < BTN_COUNT; i++)
            btns[i].short_click_pending = btns[i].long_press_pending = false;
        return;
    }

    // ===== SET (menu) =====
    if (btns[ID_SET].short_click_pending) {
        if (hmi_mode == MODE_OP) {
            hmi_mode = MODE_MENU;
            menu_item = MENU_FMIN;
            ESP_LOGI(TAG, "MENU: entrar");
        } else {
            menu_item++;
            if (menu_item >= MENU_COUNT)
                menu_item = MENU_FMIN;
        }
        btns[ID_SET].short_click_pending = false;
    }

    if (btns[ID_SET].long_press_pending) {
        if (hmi_mode == MODE_MENU) {
            hmi_mode = MODE_OP;
            ESP_LOGI(TAG, "MENU: sair");
        }
        btns[ID_SET].long_press_pending = false;
    }

    // ===== MENU =====
    if (hmi_mode == MODE_MENU) {
        int *val = menu_get_value_ptr(menu_item);
        int min, max;
        menu_get_limits(menu_item, &min, &max);

        if (btns[ID_MAIS].short_click_pending) {
            if (*val < max) (*val)++;
            btns[ID_MAIS].short_click_pending = false;
        }
        if (btns[ID_MENOS].short_click_pending) {
            if (*val > min) (*val)--;
            btns[ID_MENOS].short_click_pending = false;
        }

        // bloqueia operação, mas NÃO retorna da função inteira
        return;
    }

    // ===== OPERAÇÃO NORMAL =====
    if (btns[ID_MAIS].short_click_pending) {
        if (current_freq < param_fmax) current_freq++;
        enviar_comando_MI("FREQ", current_freq);
        btns[ID_MAIS].short_click_pending = false;
    }

    if (btns[ID_MENOS].short_click_pending) {
        if (current_freq > param_fmin) current_freq--;
        enviar_comando_MI("FREQ", current_freq);
        btns[ID_MENOS].short_click_pending = false;
    }

    if (btns[ID_CLIMATIZAR].short_click_pending) {
        bomba_on = true; exaustao_on = false;
        enviar_comando_MI("BOMBA", 1);
        btns[ID_CLIMATIZAR].short_click_pending = false;
    }

    if (btns[ID_VENTILAR].short_click_pending) {
        bomba_on = false; exaustao_on = false;
        enviar_comando_MI("BOMBA", 0);
        btns[ID_VENTILAR].short_click_pending = false;
    }

    if (btns[ID_EXAUSTAO].short_click_pending) {
        exaustao_on = true; bomba_on = false;
        enviar_comando_MI("EXAUSTAO", 1);
        btns[ID_EXAUSTAO].short_click_pending = false;
    }

    if (btns[ID_SWING].short_click_pending) {
        swing_on = !swing_on;
        enviar_comando_MI("SWING", swing_on);
        btns[ID_SWING].short_click_pending = false;
    }

    if (btns[ID_DRENO].short_click_pending) {
        dreno_active = !dreno_active;
        enviar_comando_MI("DRENO", dreno_active);
        btns[ID_DRENO].short_click_pending = false;
    }
}

// ============================================================================
// 9. LEDS
// ============================================================================
void update_leds(void) {
    if (!system_on) {
        for (int i = 0; i < LED_COUNT; i++)
            gpio_set_level(LED_PINS[i], LED_OFF);
        return;
    }

    gpio_set_level(LED_PINS[0], swing_on ? LED_ON : LED_OFF);
    gpio_set_level(LED_PINS[1], dreno_active ? LED_ON : LED_OFF);
    gpio_set_level(LED_PINS[2], bomba_on ? LED_ON : LED_OFF);
    gpio_set_level(LED_PINS[3], (!bomba_on && !exaustao_on) ? LED_ON : LED_OFF);
    gpio_set_level(LED_PINS[4], exaustao_on ? LED_ON : LED_OFF);
}

// ============================================================================
// 10. MAIN
// ============================================================================
void app_main(void) {
    nvs_flash_init();
    init_hw();
    ESP_LOGI(TAG, "HMI pronta");

    while (1) {
        monitor_buttons();
        process_logic();
        update_leds();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
