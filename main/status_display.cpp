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

static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static system_network_level_t s_network_level = SYS_NET_DOWN;
static char s_base_text[5] = {' ', ' ', ' ', ' ', '\0'};

static void errorled_fallback_signal(void);

static bool should_colon_be_on(uint32_t phase_ms, uint32_t cycle_ms, uint32_t on_ms)
{
    if (cycle_ms == 0 || on_ms == 0) {
        return false;
    }
    return phase_ms < on_ms;
}

static bool should_show_colon(system_network_level_t level, uint32_t now_ms)
{
    switch (level) {
        case SYS_NET_DOWN:
            return should_colon_be_on(now_ms % 1200, 1200, 90);
        case SYS_NET_WIFI_ONLY:
            return should_colon_be_on(now_ms % 900, 900, 120);
        case SYS_NET_IP_ONLY: {
            const uint32_t phase = now_ms % 700;
            return (phase < 110) || (phase >= 250 && phase < 360);
        }
        case SYS_NET_MQTT_READY:
            return true;
        case SYS_NET_AP_CONFIG:
            return should_colon_be_on(now_ms % 400, 400, 200);
        default:
            return false;
    }
}

static void compose_display_text(char *out_text, bool colon_on)
{
    char local_base[5];

    taskENTER_CRITICAL(&s_status_mux);
    memcpy(local_base, s_base_text, sizeof(local_base));
    taskEXIT_CRITICAL(&s_status_mux);

    if (!colon_on) {
        memcpy(out_text, local_base, 5);
        return;
    }

    out_text[0] = local_base[0];
    out_text[1] = local_base[1];
    out_text[2] = '.';
    out_text[3] = local_base[2];
    out_text[4] = local_base[3];
    out_text[5] = '\0';
}

static void status_display_task(void *pvParameters)
{
    (void)pvParameters;

    char rendered_text[6] = {'\0'};

    while (true) {
        if (s_tm1637_available && s_tm1637_display != nullptr && !s_error_latched) {
            system_network_level_t level_snapshot;

            taskENTER_CRITICAL(&s_status_mux);
            level_snapshot = s_network_level;
            taskEXIT_CRITICAL(&s_status_mux);

            uint32_t now_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
            bool colon_on = should_show_colon(level_snapshot, now_ms);

            char target_text[6] = {'\0'};
            compose_display_text(target_text, colon_on);

            if (strncmp(target_text, rendered_text, sizeof(rendered_text)) != 0) {
                esp_err_t write_result = tm1637_write_string(s_tm1637_display, target_text);
                if (write_result == ESP_OK) {
                    memcpy(rendered_text, target_text, sizeof(rendered_text));
                } else {
                    ESP_LOGE(TAG, "TM1637 write selhal: %s", esp_err_to_name(write_result));
                    s_tm1637_available = false;
                    errorled_fallback_signal();
                }
            }
        }

        vTaskDelay(STATUS_DISPLAY_TASK_PERIOD);
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
            s_tm1637_available = true;
            xTaskCreate(status_display_task, "status_display", 3072, NULL, 3, NULL);
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

void status_display_set_text(const char *text)
{
    char normalized[5] = {' ', ' ', ' ', ' ', '\0'};
    if (text != nullptr) {
        for (size_t i = 0; i < 4 && text[i] != '\0'; ++i) {
            normalized[i] = text[i];
        }
    }

    taskENTER_CRITICAL(&s_status_mux);
    memcpy(s_base_text, normalized, sizeof(s_base_text));
    taskEXIT_CRITICAL(&s_status_mux);
}
