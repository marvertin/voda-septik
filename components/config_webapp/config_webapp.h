#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    CONFIG_VALUE_STRING = 0,
    CONFIG_VALUE_INT32,
    CONFIG_VALUE_FLOAT,
    CONFIG_VALUE_BOOL,
} config_value_type_t;

typedef struct {
    const char *key;
    const char *label;
    const char *description;
    config_value_type_t type;

    const char *default_string;
    int32_t default_int;
    float default_float;
    bool default_bool;

    size_t max_string_len;
    int32_t min_int;
    int32_t max_int;
    float min_float;
    float max_float;
} config_item_t;

esp_err_t config_webapp_start(const char *nvs_namespace,
                              const config_item_t *items,
                              size_t item_count,
                              uint16_t http_port);

esp_err_t config_webapp_get_i32(const char *key, int32_t *value);
esp_err_t config_webapp_get_float(const char *key, float *value);
esp_err_t config_webapp_get_bool(const char *key, bool *value);
esp_err_t config_webapp_get_string(const char *key, char *buffer, size_t buffer_len);

esp_err_t config_webapp_set_i32(const char *key, int32_t value);
esp_err_t config_webapp_set_float(const char *key, float value);
esp_err_t config_webapp_set_bool(const char *key, bool value);
esp_err_t config_webapp_set_string(const char *key, const char *value);
