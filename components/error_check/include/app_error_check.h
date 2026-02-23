#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*app_error_code_handler_t)(const char *error_code);

void app_error_check_set_handler(app_error_code_handler_t handler);
void app_error_check_report(const char *error_code);

#define APP_ERROR_CHECK(error_code_literal, expr)                  \
    do {                                                           \
        esp_err_t __app_err_rc = (expr);                           \
        if (__app_err_rc != ESP_OK) {                              \
            app_error_check_report((error_code_literal));          \
            ESP_ERROR_CHECK(__app_err_rc);                         \
        }                                                          \
    } while (0)

#ifdef __cplusplus
}
#endif
