#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*boot_button_press_callback_t)(void *ctx);

esp_err_t boot_button_start(gpio_num_t pin, boot_button_press_callback_t callback, void *ctx);

#ifdef __cplusplus
}
#endif
