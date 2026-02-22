#include "boot_button.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static TaskHandle_t s_boot_button_task_handle = nullptr;
static boot_button_press_callback_t s_callback = nullptr;
static void *s_callback_ctx = nullptr;
static gpio_num_t s_pin = GPIO_NUM_NC;

static void boot_button_task(void *pvParameters)
{
    (void)pvParameters;

    gpio_config_t button_cfg = {
        .pin_bit_mask = 1ULL << s_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&button_cfg) != ESP_OK) {
        s_boot_button_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    bool last_pressed = false;
    while (true) {
        bool pressed = (gpio_get_level(s_pin) == 0);

        if (pressed && !last_pressed) {
            vTaskDelay(pdMS_TO_TICKS(40));
            pressed = (gpio_get_level(s_pin) == 0);
        }

        if (pressed && !last_pressed && s_callback != nullptr) {
            s_callback(s_callback_ctx);
        }

        last_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t boot_button_start(gpio_num_t pin, boot_button_press_callback_t callback, void *ctx)
{
    if (pin == GPIO_NUM_NC || pin < 0 || pin >= GPIO_NUM_MAX || callback == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_boot_button_task_handle != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    s_pin = pin;
    s_callback = callback;
    s_callback_ctx = ctx;

    BaseType_t task_ok = xTaskCreate(boot_button_task,
                                     "boot_button",
                                     configMINIMAL_STACK_SIZE * 2,
                                     nullptr,
                                     5,
                                     &s_boot_button_task_handle);
    if (task_ok != pdPASS) {
        s_pin = GPIO_NUM_NC;
        s_callback = nullptr;
        s_callback_ctx = nullptr;
        s_boot_button_task_handle = nullptr;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
