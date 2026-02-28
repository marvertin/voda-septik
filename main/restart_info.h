#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "esp_system.h"

typedef struct {
    uint32_t boot_count;
    esp_reset_reason_t last_reason;
    int64_t last_restart_unix;
} app_restart_info_t;

esp_err_t app_restart_info_load(app_restart_info_t *out_info);
esp_err_t app_restart_info_update_and_load(app_restart_info_t *out_info);
