#ifdef __cplusplus
extern "C" {
#endif

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

static tm1637_config_t s_tm1637_config = {
    .clk_pin = TM_CLK,
    .dio_pin = TM_DIO,
    .bit_delay_us = 100,
};

static tm1637_handle_t s_tm1637_display = nullptr;
static bool s_tm1637_available = false;

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
        }
    }

    app_error_check_set_handler(app_error_code_log_handler);
}
