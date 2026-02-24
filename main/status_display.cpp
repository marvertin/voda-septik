#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "tm1637.h"
#include "esp_log.h"

#ifdef __cplusplus
}
#endif

#include "status_display.h"
#include "tm1637_startup_animation.h"

#include "pins.h"
#include "app_error_check.h"

static const char *TAG = "status_display";
static constexpr TickType_t STATUS_DISPLAY_TASK_PERIOD = pdMS_TO_TICKS(100);

static tm1637_config_t s_tm1637_config = {
    .clk_pin = TM_CLK,
    .dio_pin = TM_DIO,
    .bit_delay_us = 100,
};

static tm1637_handle_t s_tm1637_display = nullptr;
static bool s_tm1637_available = false;
static bool s_error_latched = false;
static bool s_text_latched = false;

static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static system_network_level_t s_network_level = SYS_NET_DOWN;

static void errorled_fallback_signal(void);

//
static constexpr struct {
    uint32_t on_ms;
    uint32_t off_ms;
    uint32_t blink_count;
    uint32_t gap_ms;
} s_network_level_display_config[] = {
    [SYS_NET_DOWN]        = { 80,  80,  3, 200},
    [SYS_NET_WIFI_ONLY]   = {200,  80,  2, 300},
    [SYS_NET_IP_ONLY]     = {300, 100,  1, 500},
    [SYS_NET_MQTT_READY]  = {1000,  10,  1, 0},
    [SYS_NET_AP_CONFIG]   = {400,  200, 20, 500},
};

static uint8_t svitici_segmenty[] = { 0, 0, 0, 0 };

void set_segments(const uint8_t segments, uint8_t position, bool on) 
{
    if (position < 4) {
        svitici_segmenty[position] = on ? (svitici_segmenty[position] | segments) : (svitici_segmenty[position] & ~segments);
        if (s_tm1637_available && s_tm1637_display != nullptr) {
            esp_err_t write_result = tm1637_set_segments(s_tm1637_display, svitici_segmenty, 4, 0);
            if (write_result != ESP_OK) {
                ESP_LOGE(TAG, "TM1637 set_segments selhal: %s", esp_err_to_name(write_result));
                s_tm1637_available = false;
                errorled_fallback_signal();
            }
        }
    }
}

static void set_colon(bool colon_on)
{
    set_segments(TM1637_SEG_DP, 1, colon_on);
}

static void blink_pattern_blocking(system_network_level_t level)
{
    const auto& config = s_network_level_display_config[level];
    
    if (config.on_ms == 0) {
        return;
    }
    
    for (uint32_t i = 0; i < config.blink_count; ++i) {
        set_colon(true);
        vTaskDelay(pdMS_TO_TICKS(config.on_ms));
        set_colon(false);
        vTaskDelay(pdMS_TO_TICKS(config.off_ms));
    }
    vTaskDelay(pdMS_TO_TICKS(config.gap_ms));
}



static void status_display_task(void *pvParameters)
{
    (void)pvParameters;


    /*
    set_segments(TM1637_SEG_A, 0, true);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 5; ++j) {
            blink_pattern_blocking(static_cast<system_network_level_t>(i));
        }
    }
        */
    while (true) {
        if (s_tm1637_available && s_tm1637_display != nullptr && !s_error_latched && !s_text_latched) {
            system_network_level_t level_snapshot;

            taskENTER_CRITICAL(&s_status_mux);
            level_snapshot = s_network_level;
            taskEXIT_CRITICAL(&s_status_mux);

            for (int i = 0; i < 2; ++i) {
                blink_pattern_blocking(level_snapshot);
            }
        } else {
            vTaskDelay(STATUS_DISPLAY_TASK_PERIOD);
        }
    }
}

static void errorled_fallback_signal(void)
{
    gpio_reset_pin(ERRORLED_PIN);
    gpio_set_direction(ERRORLED_PIN, GPIO_MODE_OUTPUT);

    for (int i = 0; i < 3; ++i) {
        gpio_set_level(ERRORLED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
        gpio_set_level(ERRORLED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

static void show_error_code_on_tm1637(const char *error_code)
{
    if (error_code == nullptr || !s_tm1637_available || s_tm1637_display == nullptr) {
        errorled_fallback_signal();
        return;
    }

    char display_text[5] = {' ', ' ', ' ', ' ', '\0'};
    for (size_t index = 0; index < 4 && error_code[index] != '\0'; ++index) {
        display_text[index] = error_code[index];
    }

    tm1637_handle_t display = s_tm1637_display;
    s_tm1637_available = false; // zabrani dalsim pokusum o pouziti displeje, kdyz už na nem bude číslo chyby, bude stejně abort a chyba na displeji musí zůstat vidět
    s_tm1637_display = nullptr; // zabrani dalsim pokusum o pouziti displeje, kdyz už na nem bude číslo chyby, bude stejně abort a chyba na displeji musí zůstat vidět
    esp_err_t write_result = tm1637_write_string(display, display_text);
    if (write_result != ESP_OK) {
        ESP_LOGE(TAG, "TM1637 write selhal: %s", esp_err_to_name(write_result));
        s_tm1637_available = false;
        errorled_fallback_signal();
    }
}

static void app_error_code_log_handler(const char *error_code)
{
    ESP_LOGE(TAG, "Error code: %s", (error_code != nullptr) ? error_code : "(null)");
    show_error_code_on_tm1637(error_code);
}

void status_display_init(void)
{
    esp_err_t init_result = tm1637_init(&s_tm1637_config, &s_tm1637_display);
    if (init_result != ESP_OK) {
        s_tm1637_display = nullptr;
        s_tm1637_available = false;
        ESP_LOGE(TAG, "TM1637 init selhal, displej nebude pouzit: %s", esp_err_to_name(init_result));
        errorled_fallback_signal();
    } else {
        esp_err_t brightness_result = tm1637_set_brightness(s_tm1637_display, 7, true);
        if (brightness_result != ESP_OK) {
            s_tm1637_available = false;
            ESP_LOGE(TAG, "TM1637 brightness selhal, displej nebude pouzit: %s", esp_err_to_name(brightness_result));
            errorled_fallback_signal();
        } else {
            esp_err_t startup_result = tm1637_startup_animation_play_preset(s_tm1637_display, TM1637_STARTUP_ANIMATION_FAST);
            if (startup_result != ESP_OK) {
                s_tm1637_available = false;
                ESP_LOGE(TAG, "TM1637 startup sekvence selhala, displej nebude pouzit: %s", esp_err_to_name(startup_result));
                errorled_fallback_signal();
            } else {
                s_tm1637_available = true;
                xTaskCreate(status_display_task, "status_display", 3072, NULL, 3, NULL);
            }
        }
    }

    app_error_check_set_handler(app_error_code_log_handler);
}

void status_display_set_network_state(const network_event_t *event)
{
    if (event == nullptr) {
        return;
    }

    taskENTER_CRITICAL(&s_status_mux);
    s_network_level = event->level;
    taskEXIT_CRITICAL(&s_status_mux);
}

void status_display_ap_mode()
{
    if (!s_tm1637_available || s_tm1637_display == nullptr) {
        return;
    }

    s_text_latched = true;

    static constexpr uint8_t ALL_SEGMENTS = static_cast<uint8_t>(
        TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_D |
        TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G | TM1637_SEG_DP);
    static constexpr uint8_t DASH_SEGMENTS = TM1637_SEG_G;
    static constexpr uint8_t A_SEGMENTS = static_cast<uint8_t>(TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G);
    static constexpr uint8_t P_SEGMENTS = static_cast<uint8_t>(TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G);

    for (uint8_t position = 0; position < 4; ++position) {
        set_segments(ALL_SEGMENTS, position, false);
    }

    set_segments(DASH_SEGMENTS, 0, true);
    set_segments(A_SEGMENTS, 1, true);
    set_segments(P_SEGMENTS, 2, true);
    set_segments(DASH_SEGMENTS, 3, true);
}
