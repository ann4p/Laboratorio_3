#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"


#define ADC_LM335_CH    ADC1_CHANNEL_7
#define ADC_LDR_CH      ADC1_CHANNEL_6
#define PIN_RELAY       GPIO_NUM_26
#define PIN_LED_PWM     GPIO_NUM_25
#define PIN_STEP_IN1    GPIO_NUM_16
#define PIN_STEP_IN2    GPIO_NUM_17
#define PIN_STEP_IN3    GPIO_NUM_18
#define PIN_STEP_IN4    GPIO_NUM_19

#define ADC_VREF        3.1f
#define TEMP_OFFSET_C   35.0f

#define LEDC_TIMER_NUM  LEDC_TIMER_0
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_N  LEDC_CHANNEL_0
#define LEDC_FREQ_HZ    5000
#define LEDC_DUTY_RES   LEDC_TIMER_8_BIT

#define UART_PORT_NUM   UART_NUM_0
#define UART_BAUD_RATE  115200

#define STEP_SEQ_LEN 8
static const uint8_t STEP_SEQ[STEP_SEQ_LEN][4] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

#define LDR_RAW_MIN 1147.0f
#define LDR_RAW_MAX 4055.0f

static SemaphoreHandle_t  g_mutex         = NULL;
static volatile float     g_target_temp   = 25.0f;
static volatile int       g_step_dir      = 1;
static volatile uint8_t   g_led_duty      = 0;
static int                g_step_index    = 0;
static volatile int       g_step_speed    = 0;
static esp_timer_handle_t g_stepper_timer = NULL;
static volatile int       g_system_ready  = 0;

static void  init_gpio(void);
static void  init_adc(void);
static void  init_ledc(void);
static void  init_uart(void);
static void  init_stepper_timer(void);
static float read_temperature_celsius(void);
static float read_light_percent(void);
static void  control_temperature(float T, float Tc);
static void  control_lighting(float light_pct);
static void  apply_stepper_outputs(int index);
static void  stop_stepper_outputs(void);
static void  update_stepper_timer(int steps_per_sec);
static void  stepper_timer_callback(void *arg);
static void  set_led_duty(uint8_t duty);
static void  uart_send_str(const char *str);
static int   process_command(const char *line);
static void  control_task(void *arg);
static void  serial_task(void *arg);

void app_main(void) {
    g_mutex = xSemaphoreCreateMutex();
    init_gpio();
    init_adc();
    init_ledc();
    init_uart();
    init_stepper_timer();
    uart_send_str("\r\n\r\n========================================\r\n");
    uart_send_str("   SISTEMA DOMOTICO - LABORATORIO 3\r\n");
    uart_send_str("========================================\r\n");
    uart_send_str("FASE DE CONFIGURACION INICIAL\r\n");
    uart_send_str("Ingrese la temperatura de control:\r\n");
    uart_send_str("  SET_TEMP:XX  (rango: 10 a 40 grados C)\r\n");
    uart_send_str("Una vez configurada, el sistema arranca.\r\n");
    uart_send_str("----------------------------------------\r\n\r\n");
    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
    xTaskCreate(serial_task,  "serial_task",  4096, NULL, 4, NULL);
}

static void init_gpio(void) {
    gpio_config_t relay_cfg = {
        .pin_bit_mask = 1ULL << PIN_RELAY,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&relay_cfg);
    gpio_set_level(PIN_RELAY, 0);
    gpio_config_t step_cfg = {
        .pin_bit_mask = (1ULL << PIN_STEP_IN1) | (1ULL << PIN_STEP_IN2) |
                        (1ULL << PIN_STEP_IN3) | (1ULL << PIN_STEP_IN4),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&step_cfg);
    stop_stepper_outputs();
}

static void init_adc(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_LM335_CH, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC_LDR_CH,   ADC_ATTEN_DB_11);
}

static void init_ledc(void) {
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER_NUM,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);
    ledc_channel_config_t c = {
        .gpio_num   = PIN_LED_PWM,
        .speed_mode = LEDC_SPEED_MODE,
        .channel    = LEDC_CHANNEL_N,
        .timer_sel  = LEDC_TIMER_NUM,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&c);
}

static void init_uart(void) {
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT_NUM, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &cfg);
}

static void init_stepper_timer(void) {
    esp_timer_create_args_t args = {
        .callback = stepper_timer_callback,
        .name     = "stepper"
    };
    esp_timer_create(&args, &g_stepper_timer);
}

static float read_temperature_celsius(void) {
    int sum = 0;
    for (int i = 0; i < 16; i++) sum += adc1_get_raw(ADC_LM335_CH);
    float v = (sum / 16) * (ADC_VREF / 4095.0f);
    return (v / 0.010f) - 273.15f + TEMP_OFFSET_C;
}

static float read_light_percent(void) {
    int sum = 0;
    for (int i = 0; i < 16; i++) sum += adc1_get_raw(ADC_LDR_CH);
    float raw = (float)(sum / 16);
    float pct = (raw - LDR_RAW_MIN) / (LDR_RAW_MAX - LDR_RAW_MIN) * 100.0f;
    pct = 100.0f - pct;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

static void control_temperature(float T, float Tc) {
    if (T >= (Tc - 1.0f) && T <= (Tc + 1.0f)) {
        gpio_set_level(PIN_RELAY, 0);
        update_stepper_timer(0);
    } else if (T < (Tc - 1.0f)) {
        gpio_set_level(PIN_RELAY, 1);
        g_step_dir = 1;
        update_stepper_timer(100);
    } else if (T > (Tc + 1.0f) && T < (Tc + 3.0f)) {
        gpio_set_level(PIN_RELAY, 0);
        g_step_dir = -1;
        update_stepper_timer(100);
    } else if (T >= (Tc + 3.0f) && T <= (Tc + 5.0f)) {
        gpio_set_level(PIN_RELAY, 0);
        g_step_dir = -1;
        update_stepper_timer(300);
    } else {
        gpio_set_level(PIN_RELAY, 0);
        g_step_dir = -1;
        update_stepper_timer(600);
    }
}

static void control_lighting(float light_pct) {
    uint8_t duty;
    if      (light_pct < 20.0f) duty = 255;
    else if (light_pct < 30.0f) duty = 204;
    else if (light_pct < 40.0f) duty = 153;
    else if (light_pct < 60.0f) duty = 128;
    else if (light_pct < 80.0f) duty = 76;
    else                        duty = 0;
    set_led_duty(duty);
    g_led_duty = duty;
}

static void apply_stepper_outputs(int index) {
    gpio_set_level(PIN_STEP_IN1, STEP_SEQ[index][0]);
    gpio_set_level(PIN_STEP_IN2, STEP_SEQ[index][1]);
    gpio_set_level(PIN_STEP_IN3, STEP_SEQ[index][2]);
    gpio_set_level(PIN_STEP_IN4, STEP_SEQ[index][3]);
}

static void stop_stepper_outputs(void) {
    gpio_set_level(PIN_STEP_IN1, 0);
    gpio_set_level(PIN_STEP_IN2, 0);
    gpio_set_level(PIN_STEP_IN3, 0);
    gpio_set_level(PIN_STEP_IN4, 0);
}

static void stepper_timer_callback(void *arg) {
    g_step_index = (g_step_index + g_step_dir + STEP_SEQ_LEN) % STEP_SEQ_LEN;
    apply_stepper_outputs(g_step_index);
}

static void update_stepper_timer(int steps_per_sec) {
    esp_timer_stop(g_stepper_timer);
    g_step_speed = steps_per_sec;
    if (steps_per_sec > 0)
        esp_timer_start_periodic(g_stepper_timer, 1000000ULL / steps_per_sec);
    else
        stop_stepper_outputs();
}

static void set_led_duty(uint8_t duty) {
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_N, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_N);
}

static void uart_send_str(const char *str) {
    uart_write_bytes(UART_PORT_NUM, str, strlen(str));
}

static int process_command(const char *line) {
    if (strlen(line) > 9 && strncmp(line, "SET_TEMP:", 9) == 0) {
        float nt = atof(line + 9);
        if (nt >= 10.0f && nt <= 40.0f) {
            if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_target_temp = nt;
                xSemaphoreGive(g_mutex);
            }
            char r[64];
            snprintf(r, sizeof(r), "\r\n>> Temperatura de control: %.1f C\r\n", nt);
            uart_send_str(r);
            return 1;
        } else {
            uart_send_str("\r\n>> ERROR: valor fuera de rango (10-40 C)\r\n");
        }
    } else {
        uart_send_str("\r\n>> Comando no reconocido. Use: SET_TEMP:XX\r\n");
    }
    return 0;
}

static void control_task(void *arg) {
    while (!g_system_ready) vTaskDelay(pdMS_TO_TICKS(200));
    uart_send_str("\r\n[SISTEMA] Control activo.\r\n");
    uart_send_str("[SISTEMA] Puede cambiar Tc en cualquier momento con SET_TEMP:XX\r\n\r\n");
    char buf[160];
    while (1) {
        float T = read_temperature_celsius();
        float L = read_light_percent();
        float Tc;
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            Tc = g_target_temp;
            xSemaphoreGive(g_mutex);
        } else {
            Tc = 25.0f;
        }
        control_temperature(T, Tc);
        control_lighting(L);
        uint8_t duty_pct = (uint8_t)(((uint32_t)g_led_duty * 100) / 255);
        snprintf(buf, sizeof(buf),
                 "[ESTADO] Tc=%.1fC | T=%.2fC | Luz=%.1f%% | LED=%d%% | Rele=%s | Motor=%s %d steps/s\r\n",
                 Tc, T, L, duty_pct,
                 gpio_get_level(PIN_RELAY) ? "ON" : "OFF",
                 g_step_dir == 1 ? "CW" : "CCW",
                 g_step_speed);
        uart_send_str(buf);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void serial_task(void *arg) {
    uint8_t data[256];
    char    line[64];
    int     idx = 0;
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;
        for (int i = 0; i < len; i++) {
            char c = (char)data[i];
            uart_write_bytes(UART_PORT_NUM, &c, 1);
            if (c == '\r' || c == '\n') {
                line[idx] = '\0';
                if (idx > 0) {
                    int ok = process_command(line);
                    if (!g_system_ready && ok) g_system_ready = 1;
                }
                idx = 0;
            } else if (idx < 63) {
                line[idx++] = c;
            }
        }
    }
}