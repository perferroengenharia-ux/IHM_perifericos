#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
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
#define BTN_RELEASED     1

#define DEBOUNCE_MS        50   
#define LONG_PRESS_MS      1500 
#define WIFI_RESET_TIME_MS 5000 
#define REPEAT_DELAY_MS    600  
#define REPEAT_RATE_MS     150  

#define F_MIN            10 
#define F_MAX            60 

#define UART_SIM         UART_NUM_0
#define BUF_SIZE         1024

static const gpio_num_t BTN_PINS[] = {
    GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27,
    GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_4,  GPIO_NUM_5 
};
#define BTN_COUNT (sizeof(BTN_PINS) / sizeof(BTN_PINS[0]))

static const gpio_num_t LED_PINS[] = {
    GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23
};
#define LED_COUNT (sizeof(LED_PINS) / sizeof(LED_PINS[0]))

typedef enum {
    ID_MAIS = 0, ID_MENOS, ID_CLIMATIZAR, ID_VENTILAR, ID_DRENO,
    ID_SWING, ID_EXAUSTAO, ID_ONOFF, ID_SET, ID_RESET_WIFI
} ButtonIdx_t;

typedef enum { MODE_OP, MODE_MENU } HmiState_t;
typedef enum { DRENO_IDLE, DRENO_AGUARDANDO_LED, DRENO_EM_CURSO } DrenoState_t;

typedef struct {
    bool last_stable_state;     
    uint32_t last_debounce_time;
    bool is_pressed;            
    uint32_t press_start_time;  
    uint32_t last_repeat_time;  
    bool action_trigger;        
    bool long_press_trigger;    
    bool ignore_release;        
} ButtonHandler_t;

// Variáveis Globais
HmiState_t hmi_mode = MODE_OP;
bool system_on = false;
int current_freq = F_MIN;

// Estados Funcionais
bool bomba_on = false;
bool swing_on = false;
bool exaustao_on = false;
bool saved_bomba_on = false;

// Estado do Dreno
DrenoState_t dreno_status = DRENO_IDLE;
// Obs: Removemos 'dreno_on' e 'dreno_led_on' redundantes, usaremos o 'dreno_status'

ButtonHandler_t btns[BTN_COUNT];

// ============================================================================
// 2. COMUNICAÇÃO E HARDWARE
// ============================================================================
void enviar_comando_MI(const char* comando, int valor) {
    ESP_LOGI("MI_COMM", "CMD: %s | VAL: %d", comando, valor);
}

void check_serial_simulation() {
    uint8_t data[BUF_SIZE];
    int len = uart_read_bytes(UART_SIM, data, BUF_SIZE - 1, 0); 
    if (len > 0) {
        char ch = (char)data[0];
        
        // Se MI confirmar LED Dreno (Simulando resposta 'L')
        if (dreno_status == DRENO_AGUARDANDO_LED && (ch == 'l' || ch == 'L')) {
            ESP_LOGW(TAG, "MI -> CONFIRMACAO INICIO DRENO");
            dreno_status = DRENO_EM_CURSO;
        }
        // Se MI confirmar FIM Dreno (Simulando resposta 'F')
        else if (dreno_status == DRENO_EM_CURSO && (ch == 'f' || ch == 'F')) {
            ESP_LOGE(TAG, "MI -> FIM DO DRENO. DESLIGANDO TUDO.");
            dreno_status = DRENO_IDLE;
            system_on = false; 
            enviar_comando_MI("STOP", 0);
        }
    }
}

void init_hw() {
    uint64_t led_mask = 0;
    for (int i = 0; i < LED_COUNT; i++) led_mask |= (1ULL << LED_PINS[i]);
    gpio_config_t conf_led = { .pin_bit_mask = led_mask, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&conf_led);

    uint64_t btn_mask = 0;
    for (int i = 0; i < BTN_COUNT; i++) btn_mask |= (1ULL << BTN_PINS[i]);
    gpio_config_t conf_btn = { .pin_bit_mask = btn_mask, .mode = GPIO_MODE_INPUT, .pull_up_en = 1 };
    gpio_config(&conf_btn);

    uart_config_t uart_config = {
        .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_SIM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_SIM, &uart_config);

    for (int i = 0; i < LED_COUNT; i++) gpio_set_level(LED_PINS[i], LED_OFF);
}

// ============================================================================
// 3. MONITORAMENTO DE BOTÕES
// ============================================================================
void monitor_buttons() {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    for (int i = 0; i < BTN_COUNT; i++) {
        int reading = gpio_get_level(BTN_PINS[i]);
        if (reading != btns[i].last_stable_state) {
            btns[i].last_debounce_time = now;
            btns[i].last_stable_state = reading;
        }
        if ((now - btns[i].last_debounce_time) > DEBOUNCE_MS) {
            if (reading == BTN_PRESSED) {
                if (!btns[i].is_pressed) {
                    btns[i].is_pressed = true;
                    btns[i].press_start_time = now;
                    btns[i].last_repeat_time = now + REPEAT_DELAY_MS;
                    btns[i].ignore_release = false;
                    btns[i].long_press_trigger = false; 
                } else {
                    if (i == ID_RESET_WIFI) {
                        if (!btns[i].ignore_release && (now - btns[i].press_start_time >= WIFI_RESET_TIME_MS)) {
                            btns[i].action_trigger = true; 
                            btns[i].ignore_release = true; 
                        }
                    } else if (i == ID_MAIS || i == ID_MENOS) {
                        if (now > btns[i].last_repeat_time) {
                            btns[i].action_trigger = true;
                            btns[i].last_repeat_time = now + REPEAT_RATE_MS;
                            btns[i].ignore_release = true;
                        }
                    } else if (i == ID_SET && !btns[i].ignore_release) {
                        if ((now - btns[i].press_start_time) > LONG_PRESS_MS) {
                            btns[i].long_press_trigger = true;
                            btns[i].ignore_release = true;
                        }
                    }
                }
            } else { 
                if (btns[i].is_pressed) {
                    btns[i].is_pressed = false;
                    if (!btns[i].ignore_release && i != ID_RESET_WIFI) {
                        btns[i].action_trigger = true;
                    }
                }
            }
        }
    }
}

// ============================================================================
// 4. LÓGICA DO SISTEMA (COM BLOQUEIO NO DRENO)
// ============================================================================
void process_logic() {
    
    // 1. ONOFF (PRIORIDADE MÁXIMA - Funciona mesmo no Dreno)
    if (btns[ID_ONOFF].action_trigger) {
        system_on = !system_on;
        if (!system_on) {
            // Desligar tudo
            bomba_on = swing_on = exaustao_on = false;
            dreno_status = DRENO_IDLE; // Reseta dreno se forçado o OFF
            enviar_comando_MI("STOP", 0);
        } else {
            enviar_comando_MI("START", 1);
        }
        btns[ID_ONOFF].action_trigger = false;
    }

    if (!system_on) {
        // Limpa triggers residuais
        for(int i=0; i<BTN_COUNT; i++) { btns[i].action_trigger = false; btns[i].long_press_trigger = false; }
        return;
    }

    // 2. LÓGICA DE BLOQUEIO DO DRENO
    // Se o dreno estiver em qualquer estado ativo (Aguardando ou Em Curso)
    if (dreno_status != DRENO_IDLE) {
        // Limpa qualquer clique que o usuário tente dar em outros botões
        for(int i=0; i<BTN_COUNT; i++) {
            if(i != ID_ONOFF) { // ONOFF já foi tratado acima
                btns[i].action_trigger = false;
                btns[i].long_press_trigger = false;
            }
        }
        
        // Verifica a Serial (Pois precisamos receber o 'F' para sair deste modo)
        check_serial_simulation();
        
        // RETORNA AQUI: Nenhuma outra lógica abaixo é executada
        return; 
    }

    // --- DAQUI PARA BAIXO, SÓ EXECUTA SE DRENO NÃO ESTIVER ATIVO ---

    if (btns[ID_RESET_WIFI].action_trigger) {
        nvs_flash_erase(); esp_restart();
        btns[ID_RESET_WIFI].action_trigger = false;
    }

    // MENU / SET
    if (btns[ID_SET].long_press_trigger) {
        if (hmi_mode == MODE_MENU) { hmi_mode = MODE_OP; ESP_LOGI(TAG, "SAIU DO MENU"); }
        btns[ID_SET].long_press_trigger = false;
    }
    else if (btns[ID_SET].action_trigger) {
        if (hmi_mode == MODE_OP) { hmi_mode = MODE_MENU; ESP_LOGI(TAG, "ENTROU MENU"); }
        else { ESP_LOGI(TAG, "MENU: CONFIRMAR"); }
        btns[ID_SET].action_trigger = false;
    }

    // MAIS / MENOS
    if (btns[ID_MAIS].action_trigger) {
        if (hmi_mode == MODE_OP) {
            if (current_freq < F_MAX) {
                current_freq++;
                enviar_comando_MI("VELOCIDADE", current_freq);
            }
            } else ESP_LOGI(TAG, "MENU: UP");
            btns[ID_MAIS].action_trigger = false;
    }

    if (btns[ID_MENOS].action_trigger) {
        if (hmi_mode == MODE_OP) {
            if (current_freq > F_MIN) {
                current_freq--;
                enviar_comando_MI("VELOCIDADE", current_freq);
            }
        } else ESP_LOGI(TAG, "MENU: DOWN");
                    btns[ID_MENOS].action_trigger = false;
    }

    // EXAUSTÃO
    if (btns[ID_EXAUSTAO].action_trigger) {
        if (!exaustao_on) {
            saved_bomba_on = bomba_on; exaustao_on = true; bomba_on = false;
            enviar_comando_MI("EXAUSTAO", 1);
        } else {
            exaustao_on = false; bomba_on = saved_bomba_on;
            enviar_comando_MI("EXAUSTAO", 0);
            enviar_comando_MI("BOMBA", bomba_on);
        }
        btns[ID_EXAUSTAO].action_trigger = false;
    }

    // CLIMATIZAR / VENTILAR
    if (btns[ID_CLIMATIZAR].action_trigger) {
        if (exaustao_on) { exaustao_on = false; enviar_comando_MI("EXAUSTAO", 0); }
        bomba_on = true; enviar_comando_MI("BOMBA", 1);
        btns[ID_CLIMATIZAR].action_trigger = false;
    }
    if (btns[ID_VENTILAR].action_trigger) {
        if (exaustao_on) { exaustao_on = false; enviar_comando_MI("EXAUSTAO", 0); }
        bomba_on = false; enviar_comando_MI("BOMBA", 0);
        btns[ID_VENTILAR].action_trigger = false;
    }

    // SWING
    if (btns[ID_SWING].action_trigger) {
        swing_on = !swing_on; enviar_comando_MI("SWING", swing_on);
        btns[ID_SWING].action_trigger = false;
    }

    // DRENO (INÍCIO DO PROCESSO)
    if (btns[ID_DRENO].action_trigger) {
        // Só aceita comando se estiver IDLE (proteção extra)
        if (dreno_status == DRENO_IDLE) {
            ESP_LOGW(TAG, ">>> INICIANDO CICLO DE DRENO <<<");
            
            // 1. Envia comando
            enviar_comando_MI("SOLICITAR_DRENO", 1);
            
            // 2. Derruba todos os estados lógicos imediatamente
            bomba_on = false;
            swing_on = false;
            exaustao_on = false;
            
            // 3. Muda estado para bloquear o sistema na próxima volta do loop
            dreno_status = DRENO_AGUARDANDO_LED;
        }
        btns[ID_DRENO].action_trigger = false;
    }

    check_serial_simulation();
}

// ============================================================================
// 5. ATUALIZAÇÃO DOS LEDS (PRIORIDADE PARA O DRENO)
// ============================================================================
void update_leds() {
    if (!system_on) {
        for (int i = 0; i < LED_COUNT; i++) gpio_set_level(LED_PINS[i], LED_OFF);
        return;
    }

    // Lógica Prioritária de LEDs do Dreno
    if (dreno_status != DRENO_IDLE) {
        // Apenas LED DRENO (índice 1) ligado
        gpio_set_level(LED_PINS[0], LED_OFF); // Swing
        gpio_set_level(LED_PINS[1], LED_ON);  // DRENO
        gpio_set_level(LED_PINS[2], LED_OFF); // Climatizar
        gpio_set_level(LED_PINS[3], LED_OFF); // Ventilar
        gpio_set_level(LED_PINS[4], LED_OFF); // Exaustao
        return; // Sai da função, ignorando o resto
    }

    // LEDs Normais (Só executa se não for Dreno)
    gpio_set_level(LED_PINS[0], swing_on ? LED_ON : LED_OFF);
    gpio_set_level(LED_PINS[1], LED_OFF); // Dreno desligado
    gpio_set_level(LED_PINS[2], (bomba_on && !exaustao_on) ? LED_ON : LED_OFF);
    gpio_set_level(LED_PINS[3], (!bomba_on && !exaustao_on) ? LED_ON : LED_OFF);
    gpio_set_level(LED_PINS[4], exaustao_on ? LED_ON : LED_OFF);
}

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }
    init_hw();
    while (1) {
        monitor_buttons();
        process_logic();
        update_leds();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}