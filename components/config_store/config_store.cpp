#include "config_store.h"

#include <algorithm>
#include <cstring>
#include <string>

#include "app_error_check.h"
#include "nvs.h"

namespace {

static constexpr size_t CONFIG_STORE_MAX_ITEMS = 64;

struct config_store_ctx_t {
    const config_item_t *items[CONFIG_STORE_MAX_ITEMS];
    size_t item_count;
    char nvs_namespace[16];
};

config_store_ctx_t s_ctx = {};

bool is_valid_item(const config_item_t &item)
{
    return item.key != nullptr && strlen(item.key) > 0 && strlen(item.key) <= 15;
}

esp_err_t open_nvs(nvs_open_mode_t mode, nvs_handle_t *out_handle)
{
    if (!config_store_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    return nvs_open(s_ctx.nvs_namespace, mode, out_handle);
}

esp_err_t nvs_set_float(nvs_handle_t handle, const char *key, float value)
{
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_get_float(nvs_handle_t handle, const char *key, float *value)
{
    size_t size = sizeof(*value);
    return nvs_get_blob(handle, key, value, &size);
}

int32_t clamp_i32(const config_item_t *item, int32_t value)
{
    return std::max(item->min_int, std::min(item->max_int, value));
}

float clamp_float(const config_item_t *item, float value)
{
    return std::max(item->min_float, std::min(item->max_float, value));
}

void check_or_abort(const char *error_code, esp_err_t result)
{
    APP_ERROR_CHECK(error_code, result);
}

} // namespace

esp_err_t config_store_prepare(const char *nvs_namespace)
{
    if (nvs_namespace == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(nvs_namespace) > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    strncpy(s_ctx.nvs_namespace, nvs_namespace, sizeof(s_ctx.nvs_namespace) - 1);

    return ESP_OK;
}

esp_err_t config_store_register_item(const config_item_t *item)
{
    if (!config_store_is_ready() || item == nullptr || !is_valid_item(*item)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t index = 0; index < s_ctx.item_count; ++index) {
        const config_item_t *existing = s_ctx.items[index];
        if (existing != nullptr && strcmp(existing->key, item->key) == 0) {
            return ESP_OK;
        }
    }

    if (s_ctx.item_count >= CONFIG_STORE_MAX_ITEMS) {
        return ESP_ERR_NO_MEM;
    }

    s_ctx.items[s_ctx.item_count] = item;
    s_ctx.item_count += 1;
    return ESP_OK;
}

bool config_store_is_ready(void)
{
    return s_ctx.nvs_namespace[0] != '\0';
}

const config_item_t *config_store_find_item(const char *key)
{
    if (key == nullptr || !config_store_is_ready()) {
        return nullptr;
    }

    for (size_t index = 0; index < s_ctx.item_count; ++index) {
        const config_item_t *item = s_ctx.items[index];
        if (item != nullptr && strcmp(item->key, key) == 0) {
            return item;
        }
    }
    return nullptr;
}

size_t config_store_item_count(void)
{
    return config_store_is_ready() ? s_ctx.item_count : 0;
}

const config_item_t *config_store_item_at(size_t index)
{
    if (!config_store_is_ready() || index >= s_ctx.item_count) {
        return nullptr;
    }
    return s_ctx.items[index];
}

int32_t config_store_get_i32_item(const config_item_t *item)
{
    check_or_abort("E650", (item != nullptr && item->type == CONFIG_VALUE_INT32) ? ESP_OK : ESP_ERR_INVALID_ARG);
    check_or_abort("E651", config_store_find_item(item->key) != nullptr ? ESP_OK : ESP_ERR_NOT_FOUND);

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    check_or_abort("E652", result);

    int32_t value = 0;
    result = nvs_get_i32(nvs_handle, item->key, &value);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        value = clamp_i32(item, item->default_int);
        result = nvs_set_i32(nvs_handle, item->key, value);
        if (result == ESP_OK) {
            result = nvs_commit(nvs_handle);
        }
    }

    nvs_close(nvs_handle);
    check_or_abort("E653", result);
    return value;
}

float config_store_get_float_item(const config_item_t *item)
{
    check_or_abort("E654", (item != nullptr && item->type == CONFIG_VALUE_FLOAT) ? ESP_OK : ESP_ERR_INVALID_ARG);
    check_or_abort("E655", config_store_find_item(item->key) != nullptr ? ESP_OK : ESP_ERR_NOT_FOUND);

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    check_or_abort("E656", result);

    float value = 0.0f;
    result = nvs_get_float(nvs_handle, item->key, &value);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        value = clamp_float(item, item->default_float);
        result = nvs_set_float(nvs_handle, item->key, value);
        if (result == ESP_OK) {
            result = nvs_commit(nvs_handle);
        }
    }

    nvs_close(nvs_handle);
    check_or_abort("E657", result);
    return value;
}

bool config_store_get_bool_item(const config_item_t *item)
{
    check_or_abort("E658", (item != nullptr && item->type == CONFIG_VALUE_BOOL) ? ESP_OK : ESP_ERR_INVALID_ARG);
    check_or_abort("E659", config_store_find_item(item->key) != nullptr ? ESP_OK : ESP_ERR_NOT_FOUND);

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    check_or_abort("E660", result);

    bool value = false;
    uint8_t raw = 0;
    result = nvs_get_u8(nvs_handle, item->key, &raw);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        raw = item->default_bool ? 1 : 0;
        result = nvs_set_u8(nvs_handle, item->key, raw);
        if (result == ESP_OK) {
            result = nvs_commit(nvs_handle);
        }
    }

    nvs_close(nvs_handle);
    check_or_abort("E661", result);
    value = (raw != 0);
    return value;
}

void config_store_get_string_item(const config_item_t *item, char *buffer, size_t buffer_len)
{
    check_or_abort("E662", (item != nullptr && item->type == CONFIG_VALUE_STRING) ? ESP_OK : ESP_ERR_INVALID_ARG);
    check_or_abort("E663", (buffer != nullptr && buffer_len > 0) ? ESP_OK : ESP_ERR_INVALID_ARG);
    check_or_abort("E664", config_store_find_item(item->key) != nullptr ? ESP_OK : ESP_ERR_NOT_FOUND);

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    check_or_abort("E665", result);

    size_t required_size = buffer_len;
    result = nvs_get_str(nvs_handle, item->key, buffer, &required_size);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        std::string normalized = item->default_string != nullptr ? item->default_string : "";
        if (item->max_string_len > 0 && normalized.size() > item->max_string_len) {
            normalized = normalized.substr(0, item->max_string_len);
        }

        check_or_abort("E666", (normalized.size() + 1 <= buffer_len) ? ESP_OK : ESP_ERR_NVS_INVALID_LENGTH);

        result = nvs_set_str(nvs_handle, item->key, normalized.c_str());
        if (result == ESP_OK) {
            result = nvs_commit(nvs_handle);
        }
        if (result == ESP_OK) {
            memcpy(buffer, normalized.c_str(), normalized.size() + 1);
        }
    }

    nvs_close(nvs_handle);
    check_or_abort("E667", result);
}

esp_err_t config_store_set_i32_item(const config_item_t *item, int32_t value)
{
    if (item == nullptr || item->type != CONFIG_VALUE_INT32) {
        return ESP_ERR_INVALID_ARG;
    }

    const int32_t clamped = clamp_i32(item, value);

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    result = nvs_set_i32(nvs_handle, item->key, clamped);
    if (result == ESP_OK) {
        result = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return result;
}

esp_err_t config_store_set_float_item(const config_item_t *item, float value)
{
    if (item == nullptr || item->type != CONFIG_VALUE_FLOAT) {
        return ESP_ERR_INVALID_ARG;
    }

    const float clamped = clamp_float(item, value);

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    result = nvs_set_float(nvs_handle, item->key, clamped);
    if (result == ESP_OK) {
        result = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return result;
}

esp_err_t config_store_set_bool_item(const config_item_t *item, bool value)
{
    if (item == nullptr || item->type != CONFIG_VALUE_BOOL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    result = nvs_set_u8(nvs_handle, item->key, value ? 1 : 0);
    if (result == ESP_OK) {
        result = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return result;
}

esp_err_t config_store_set_string_item(const config_item_t *item, const char *value)
{
    if (item == nullptr || value == nullptr || item->type != CONFIG_VALUE_STRING) {
        return ESP_ERR_INVALID_ARG;
    }

    std::string normalized = value;
    if (item->max_string_len > 0 && normalized.size() > item->max_string_len) {
        normalized = normalized.substr(0, item->max_string_len);
    }

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    result = nvs_set_str(nvs_handle, item->key, normalized.c_str());
    if (result == ESP_OK) {
        result = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return result;
}

int32_t config_store_get_i32(const char *key)
{
    return config_store_get_i32_item(config_store_find_item(key));
}

float config_store_get_float(const char *key)
{
    return config_store_get_float_item(config_store_find_item(key));
}

bool config_store_get_bool(const char *key)
{
    return config_store_get_bool_item(config_store_find_item(key));
}

void config_store_get_string(const char *key, char *buffer, size_t buffer_len)
{
    config_store_get_string_item(config_store_find_item(key), buffer, buffer_len);
}

esp_err_t config_store_set_i32(const char *key, int32_t value)
{
    return config_store_set_i32_item(config_store_find_item(key), value);
}

esp_err_t config_store_set_float(const char *key, float value)
{
    return config_store_set_float_item(config_store_find_item(key), value);
}

esp_err_t config_store_set_bool(const char *key, bool value)
{
    return config_store_set_bool_item(config_store_find_item(key), value);
}

esp_err_t config_store_set_string(const char *key, const char *value)
{
    return config_store_set_string_item(config_store_find_item(key), value);
}
