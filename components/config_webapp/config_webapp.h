#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint32_t boot_count;
    int32_t last_reason;
    int64_t last_restart_unix;
} config_webapp_restart_info_t;

typedef struct {
    bool is_ap_mode;
    const char *active_ssid;
} config_webapp_network_info_t;

esp_err_t config_webapp_prepare(const char *nvs_namespace);

esp_err_t config_webapp_start(uint16_t http_port,
                              const config_webapp_restart_info_t *restart_info,
                              const config_webapp_network_info_t *network_info);
esp_err_t config_webapp_stop(void);

esp_err_t config_webapp_get_i32(const char *key, int32_t *value);
esp_err_t config_webapp_get_float(const char *key, float *value);
esp_err_t config_webapp_get_bool(const char *key, bool *value);
esp_err_t config_webapp_get_string(const char *key, char *buffer, size_t buffer_len);

esp_err_t config_webapp_set_i32(const char *key, int32_t value);
esp_err_t config_webapp_set_float(const char *key, float value);
esp_err_t config_webapp_set_bool(const char *key, bool value);
esp_err_t config_webapp_set_string(const char *key, const char *value);
