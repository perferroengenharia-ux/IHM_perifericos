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
#define BTN_PRESSED      0  // Botão leva ao GND
#define BTN_RELEASED     1

// Tempos
#define DEBOUNCE_MS      50   // Filtro de ruído
#define LONG_PRESS_MS    1500 // Tempo para Set Longo (Sair do menu)
#define REPEAT_DELAY_MS  600  // Tempo segurando MAIS/MENOS antes de começar a repetir
#define REPEAT_RATE_MS   150  // Velocidade da repetição (ms entre incrementos)

#define F_MIN            10 
#define F_MAX            60 

// Pinos
static const gpio_num_t BTN_PINS[] = {
    GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27,
    GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_4,  GPIO_NUM_5
};
#define BTN_COUNT (sizeof(BTN_PINS) / sizeof(BTN_PINS[0]))

static const gpio_num_t LED_PINS[] = {
    GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23
};
#define LED_COUNT (sizeof(LED_PINS) / sizeof(LED_PINS[0]))

// Índices
typedef enum {
    ID_MAIS = 0, ID_MENOS, ID_CLIMATIZAR, ID_VENTILAR, ID_DRENO,
    ID_SWING, ID_EXAUSTAO, ID_ONOFF, ID_SET, ID_RESET_WIFI
} ButtonIdx_t;

typedef enum { MODE_OP, MODE_MENU } HmiState_t;

// Estrutura de Botão Aprimorada
typedef struct {
    bool last_stable_state;     // Estado após debounce
    uint32_t last_debounce_time;
    bool is_pressed;            // Se está fisicamente pressionado
    uint32_t press_start_time;  // Momento que foi pressionado
    uint32_t last_repeat_time;  // Para controlar a velocidade do auto-repeat
    bool action_trigger;        // Dispara ação imediata (Click curto ou Repetição)
    bool long_press_trigger;    // Dispara ação longa (Apenas SET)
    bool ignore_release;        // Se já repetiu, ignora o evento de soltar
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
// 2. FUNÇÕES AUXILIARES
// ============================================================================
void enviar_comando_MI(const char* comando, int valor) {
    ESP_LOGI("MI_COMM", "CMD: %s | VAL: %d", comando, valor);
}

void init_hw() {
    // LEDs
    uint64_t led_mask = 0;
    for (int i = 0; i < LED_COUNT; i++) led_mask |= (1ULL << LED_PINS[i]);
    gpio_config_t conf_led = { .pin_bit_mask = led_mask, .mode = GPIO_MODE_OUTPUT, .pull_up_en = 0, .pull_down_en = 0 };
    gpio_config(&conf_led);

    // Botões (Input Pull-up)
    uint64_t btn_mask = 0;
    for (int i = 0; i < BTN_COUNT; i++) btn_mask |= (1ULL << BTN_PINS[i]);
    gpio_config_t conf_btn = { .pin_bit_mask = btn_mask, .mode = GPIO_MODE_INPUT, .pull_up_en = 1, .pull_down_en = 0 };
    gpio_config(&conf_btn);

    for (int i = 0; i < LED_COUNT; i++) gpio_set_level(LED_PINS[i], LED_OFF);
}

// ============================================================================
// 3. MONITORAMENTO INTELIGENTE (COM AUTO-REPEAT)
// ============================================================================
void monitor_buttons() {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (int i = 0; i < BTN_COUNT; i++) {
        int reading = gpio_get_level(BTN_PINS[i]);

        // 1. Debounce
        if (reading != btns[i].last_stable_state) {
            btns[i].last_debounce_time = now;
            btns[i].last_stable_state = reading;
        }

        if ((now - btns[i].last_debounce_time) > DEBOUNCE_MS) {
            // Se detectou mudança estável
            if (reading == BTN_PRESSED) {
                if (!btns[i].is_pressed) {
                    // --- Borda de Descida (Pressionou) ---
                    btns[i].is_pressed = true;
                    btns[i].press_start_time = now;
                    btns[i].last_repeat_time = now + REPEAT_DELAY_MS; // Delay inicial
                    btns[i].ignore_release = false;
                    btns[i].long_press_trigger = false; 
                } 
                else {
                    // --- Mantendo Pressionado (HOLD) ---
                    
                    // Lógica para MAIS e MENOS (Auto-Repeat)
                    if (i == ID_MAIS || i == ID_MENOS) {
                        if (now > btns[i].last_repeat_time) {
                            btns[i].action_trigger = true; // Dispara ação repetida
                            btns[i].last_repeat_time = now + REPEAT_RATE_MS; // Próximo tiro
                            btns[i].ignore_release = true; // Não disparar de novo ao soltar
                        }
                    }
                    
                    // Lógica para SET (Long Press)
                    if (i == ID_SET && !btns[i].ignore_release) {
                        if ((now - btns[i].press_start_time) > LONG_PRESS_MS) {
                            btns[i].long_press_trigger = true;
                            btns[i].ignore_release = true; // Trava para executar só uma vez
                        }
                    }
                }
            } 
            else { // BTN_RELEASED
                if (btns[i].is_pressed) {
                    // --- Borda de Subida (Soltou) ---
                    btns[i].is_pressed = false;
                    
                    // Se não foi um evento de repetição ou long press tratado, é um clique curto
                    if (!btns[i].ignore_release) {
                        btns[i].action_trigger = true;
                    }
                }
            }
        }
    }
}

// ============================================================================
// 4. LÓGICA DO SISTEMA
// ============================================================================
void process_logic() {
    // 1. ONOFF
    if (btns[ID_ONOFF].action_trigger) {
        system_on = !system_on;
        if (!system_on) {
            ESP_LOGW(TAG, "SISTEMA DESLIGADO");
            bomba_on = swing_on = exaustao_on = dreno_active = false;
            enviar_comando_MI("STOP", 0);
        } else {
            ESP_LOGW(TAG, "SISTEMA LIGADO");
            enviar_comando_MI("START", 1);
        }
        btns[ID_ONOFF].action_trigger = false;
    }

    // 2. RESET WIFI
    if (btns[ID_RESET_WIFI].action_trigger) {
        ESP_LOGE(TAG, "RESET WIFI");
        nvs_flash_erase();
        esp_restart();
        btns[ID_RESET_WIFI].action_trigger = false;
    }

    // Se desligado, limpa tudo e retorna
    if (!system_on) {
        for(int i=0; i<BTN_COUNT; i++) {
            btns[i].action_trigger = false;
            btns[i].long_press_trigger = false;
        }
        return;
    }

    // --- SISTEMA LIGADO ---

    // 3. SET (Menu)
    if (btns[ID_SET].long_press_trigger) { // Longo: Sai do Menu
        if (hmi_mode == MODE_MENU) {
            hmi_mode = MODE_OP;
            ESP_LOGI(TAG, "SAIU DO MENU");
        }
        btns[ID_SET].long_press_trigger = false;
    }
    else if (btns[ID_SET].action_trigger) { // Curto: Entra/Confirma
        if (hmi_mode == MODE_OP) {
            hmi_mode = MODE_MENU;
            ESP_LOGI(TAG, "ENTROU MENU");
        } else {
            ESP_LOGI(TAG, "MENU: CONFIRMAR");
        }
        btns[ID_SET].action_trigger = false;
    }

    // 4. MAIS e MENOS (Com Auto-Repeat)
    if (btns[ID_MAIS].action_trigger) {
        if (hmi_mode == MODE_OP) {
            if (current_freq < F_MAX) {
                current_freq++;
                enviar_comando_MI("VELOCIDADE", current_freq);
            }
        } else {
            ESP_LOGI(TAG, "MENU: UP");
        }
        btns[ID_MAIS].action_trigger = false;
    }

    if (btns[ID_MENOS].action_trigger) {
        if (hmi_mode == MODE_OP) {
            if (current_freq > F_MIN) {
                current_freq--;
                enviar_comando_MI("VELOCIDADE", current_freq);
            }
        } else {
            ESP_LOGI(TAG, "MENU: DOWN");
        }
        btns[ID_MENOS].action_trigger = false;
    }

    // 5. MODOS DE OPERAÇÃO
    if (btns[ID_CLIMATIZAR].action_trigger) {
        bomba_on = true;
        exaustao_on = false; 
        enviar_comando_MI("BOMBA", 1);
        btns[ID_CLIMATIZAR].action_trigger = false;
    }

    if (btns[ID_VENTILAR].action_trigger) {
        bomba_on = false;
        exaustao_on = false;
        enviar_comando_MI("BOMBA", 0);
        btns[ID_VENTILAR].action_trigger = false;
    }

    if (btns[ID_EXAUSTAO].action_trigger) {
        exaustao_on = true;
        bomba_on = false;
        enviar_comando_MI("EXAUSTAO", 1);
        btns[ID_EXAUSTAO].action_trigger = false;
    }

    // 6. TOGGLES
    if (btns[ID_SWING].action_trigger) {
        swing_on = !swing_on;
        enviar_comando_MI("SWING", swing_on);
        btns[ID_SWING].action_trigger = false;
    }

    if (btns[ID_DRENO].action_trigger) {
        dreno_active = !dreno_active;
        enviar_comando_MI("DRENO", dreno_active);
        btns[ID_DRENO].action_trigger = false;
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

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    init_hw();
    ESP_LOGI(TAG, "SISTEMA INICIADO. PRESSIONE ON/OFF.");

    while (1) {
        monitor_buttons();
        process_logic();
        update_leds();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}