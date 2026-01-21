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
// 1. CONFIGURAÇÕES E PINAGEM (Ajustado para Botão no GND)
// ============================================================================
#define LED_ON           0  // Active Low (Catodo no pino acende com 0)
#define LED_OFF          1
#define BTN_PRESSED      0  // AGORA É 0: Botão no GND derruba a tensão para 0V
#define LONG_PRESS_MS    1500 

#define F_MIN            10 
#define F_MAX            60 

// Pinos dos Botões (MAIS, MENOS, CLIMATIZAR, VENTILAR, DRENO, SWING, EXAUSTAO, ONOFF, SET, RESET_WIFI)
static const gpio_num_t BTN_PINS[] = {
    GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27,
    GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_4,  GPIO_NUM_2
};
#define BTN_COUNT (sizeof(BTN_PINS) / sizeof(BTN_PINS[0]))

// Pinos dos LEDs (SWING, DRENO, CLIMATIZAR, VENTILAR, EXAUSTAO)
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

// Variáveis Globais
HmiState_t hmi_mode = MODE_OP;
bool system_on = false;
int current_freq = F_MIN;
bool bomba_on = false;
bool swing_on = false;
bool dreno_active = false;
bool exaustao_on = false;
ButtonHandler_t btns[BTN_COUNT];

// ============================================================================
// 2. COMUNICAÇÃO COM O MÓDULO INVERSOR (Simulação)
// ============================================================================
void enviar_comando_MI(const char* comando, int valor) {
    ESP_LOGI("MI_COMM", ">> ENVIO: %s | VALOR: %d", comando, valor);
}

// ============================================================================
// 3. INICIALIZAÇÃO DE HARDWARE (Ajustado para Pull-UP)
// ============================================================================
void init_hw() {
    // Configura LEDs (Igual: Saída)
    uint64_t led_mask = 0;
    for (int i = 0; i < LED_COUNT; i++) led_mask |= (1ULL << LED_PINS[i]);
    
    gpio_config_t conf_led = {
        .pin_bit_mask = led_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&conf_led);

    // Configura Botões com PULL-UP INTERNO
    uint64_t btn_mask = 0;
    for (int i = 0; i < BTN_COUNT; i++) btn_mask |= (1ULL << BTN_PINS[i]);

    gpio_config_t conf_btn = {
        .pin_bit_mask = btn_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,    // <-- ATIVA RESISTOR PARA 3.3V
        .pull_down_en = 0,  // Desativa Pull-down
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&conf_btn);
}

// ============================================================================
// 4. LÓGICA DE TRATAMENTO
// ============================================================================
void monitor_buttons() {
    for (int i = 0; i < BTN_COUNT; i++) {
        int level = gpio_get_level(BTN_PINS[i]);
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Se nível for 0 (BTN_PRESSED), significa que o botão conectou o pino ao GND
        if (level == BTN_PRESSED) {
            if (!btns[i].is_pressed) {
                btns[i].is_pressed = true;
                btns[i].press_start_tick = now;
            } else if ((now - btns[i].press_start_tick > LONG_PRESS_MS) && !btns[i].long_press_pending) {
                btns[i].long_press_pending = true;
            }
        } else {
            if (btns[i].is_pressed) {
                if (!btns[i].long_press_pending) btns[i].short_click_pending = true;
                btns[i].is_pressed = false;
                btns[i].long_press_pending = false;
            }
        }
    }
}

void process_logic() {
    if (btns[ID_RESET_WIFI].short_click_pending) {
        ESP_LOGW(TAG, "Limpando NVS e Reiniciando...");
        nvs_flash_erase();
        esp_restart();
    }

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
        for(int i=0; i<BTN_COUNT; i++) btns[i].short_click_pending = btns[i].long_press_pending = false;
        return;
    }

    // SET (Curto e Longo)
    if (btns[ID_SET].short_click_pending) {
        if (hmi_mode == MODE_OP) hmi_mode = MODE_MENU;
        btns[ID_SET].short_click_pending = false;
    }
    if (btns[ID_SET].long_press_pending) {
        if (hmi_mode == MODE_MENU) hmi_mode = MODE_OP;
        btns[ID_SET].long_press_pending = false;
    }

    // MODOS
    if (hmi_mode == MODE_OP) {
        if (btns[ID_MAIS].short_click_pending) {
            if (current_freq < F_MAX) current_freq++;
            enviar_comando_MI("FREQ", current_freq);
            btns[ID_MAIS].short_click_pending = false;
        }
        if (btns[ID_MENOS].short_click_pending) {
            if (current_freq > F_MIN) current_freq--;
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

void update_leds() {
    if (!system_on) {
        for (int i = 0; i < LED_COUNT; i++) gpio_set_level(LED_PINS[i], LED_OFF);
        return;
    }
    gpio_set_level(LED_PINS[0], swing_on ? LED_ON : LED_OFF);
    gpio_set_level(LED_PINS[1], dreno_active ? LED_ON : LED_OFF);
    gpio_set_level(LED_PINS[2], bomba_on ? LED_ON : LED_OFF);
    gpio_set_level(LED_PINS[3], (!bomba_on && !exaustao_on) ? LED_ON : LED_OFF);
    gpio_set_level(LED_PINS[4], exaustao_on ? LED_ON : LED_OFF);
}

// ============================================================================
// 5. MAIN
// ============================================================================
void app_main() {
    nvs_flash_init();
    init_hw();
    ESP_LOGI(TAG, "HMI Pronto (Hardware: BTN->GND)");

    while (1) {
        monitor_buttons();
        process_logic();
        update_leds();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}