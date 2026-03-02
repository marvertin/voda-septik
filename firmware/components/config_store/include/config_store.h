#pragma once

#include "config_types.h"
#include "esp_err.h"

esp_err_t config_store_prepare(const char *nvs_namespace);
esp_err_t config_store_begin_section(const char *section_name);
esp_err_t config_store_register_item(const config_item_t *item);

bool config_store_is_ready(void);
const config_item_t *config_store_find_item(const char *key);
size_t config_store_item_count(void);
const config_item_t *config_store_item_at(size_t index);
const char *config_store_section_for_item_at(size_t index);

int32_t config_store_get_i32_item(const config_item_t *item);
float config_store_get_float_item(const config_item_t *item);
bool config_store_get_bool_item(const config_item_t *item);
void config_store_get_string_item(const config_item_t *item, char *buffer, size_t buffer_len);

esp_err_t config_store_set_i32_item(const config_item_t *item, int32_t value);
esp_err_t config_store_set_float_item(const config_item_t *item, float value);
esp_err_t config_store_set_bool_item(const config_item_t *item, bool value);
esp_err_t config_store_set_string_item(const config_item_t *item, const char *value);

int32_t config_store_get_i32(const char *key);
float config_store_get_float(const char *key);
bool config_store_get_bool(const char *key);
void config_store_get_string(const char *key, char *buffer, size_t buffer_len);

esp_err_t config_store_set_i32(const char *key, int32_t value);
esp_err_t config_store_set_float(const char *key, float value);
esp_err_t config_store_set_bool(const char *key, bool value);
esp_err_t config_store_set_string(const char *key, const char *value);
