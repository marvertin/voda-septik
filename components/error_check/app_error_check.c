#include "app_error_check.h"

#include "esp_log.h"

static const char *TAG = "error_check";
static app_error_code_handler_t s_error_code_handler = NULL;

void app_error_check_set_handler(app_error_code_handler_t handler)
{
    s_error_code_handler = handler;
}

void app_error_check_report(const char *error_code)
{
    const char *code_to_log = (error_code != NULL) ? error_code : "E000";

    if (s_error_code_handler != NULL) {
        s_error_code_handler(code_to_log);
        return;
    }

    ESP_LOGE(TAG, "Error code: %s", code_to_log);
}
