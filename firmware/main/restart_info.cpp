#include "restart_info.h"

#include <ctime>

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "restart_info";
static const char *SYS_NAMESPACE = "sys_meta";
static const char *SYS_BOOT_COUNT_KEY = "boot_count_v2";
static const char *SYS_LAST_REASON_KEY = "last_reason";
static const char *SYS_LAST_TIME_KEY = "last_time";

static esp_err_t set_and_check(esp_err_t result, nvs_handle_t nvs_handle)
{
    if (result != ESP_OK) {
        nvs_close(nvs_handle);
    }
    return result;
}

esp_err_t app_restart_info_load(app_restart_info_t *out_info)
{
    if (out_info == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t result = nvs_open(SYS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        out_info->boot_count = 0;
        out_info->last_reason = ESP_RST_UNKNOWN;
        out_info->last_restart_unix = 0;
        return ESP_OK;
    }
    if (result != ESP_OK) {
        return result;
    }

    uint32_t boot_count = 0;
    int32_t last_reason = (int32_t)ESP_RST_UNKNOWN;
    int64_t last_time = 0;

    result = nvs_get_u32(nvs_handle, SYS_BOOT_COUNT_KEY, &boot_count);
    if (result != ESP_OK && result != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs_handle);
        return result;
    }

    result = nvs_get_i32(nvs_handle, SYS_LAST_REASON_KEY, &last_reason);
    if (result != ESP_OK && result != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs_handle);
        return result;
    }

    result = nvs_get_i64(nvs_handle, SYS_LAST_TIME_KEY, &last_time);
    if (result != ESP_OK && result != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs_handle);
        return result;
    }

    nvs_close(nvs_handle);

    out_info->boot_count = boot_count;
    out_info->last_reason = (esp_reset_reason_t)last_reason;
    out_info->last_restart_unix = last_time;
    return ESP_OK;
}

esp_err_t app_restart_info_update_and_load(app_restart_info_t *out_info)
{
    if (out_info == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t result = nvs_open(SYS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    uint32_t boot_count = 0;
    result = nvs_get_u32(nvs_handle, SYS_BOOT_COUNT_KEY, &boot_count);
    if (result != ESP_OK && result != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs_handle);
        return result;
    }
    boot_count++;

    esp_reset_reason_t reason = esp_reset_reason();
    int64_t now = static_cast<int64_t>(time(nullptr));
    if (now < 1609459200) {
        now = 0;
    }

    result = set_and_check(nvs_set_u32(nvs_handle, SYS_BOOT_COUNT_KEY, boot_count), nvs_handle);
    if (result != ESP_OK) {
        return result;
    }
    result = set_and_check(nvs_set_i32(nvs_handle, SYS_LAST_REASON_KEY, static_cast<int32_t>(reason)), nvs_handle);
    if (result != ESP_OK) {
        return result;
    }
    result = set_and_check(nvs_set_i64(nvs_handle, SYS_LAST_TIME_KEY, now), nvs_handle);
    if (result != ESP_OK) {
        return result;
    }
    result = set_and_check(nvs_commit(nvs_handle), nvs_handle);
    if (result != ESP_OK) {
        return result;
    }
    nvs_close(nvs_handle);

    out_info->boot_count = boot_count;
    out_info->last_reason = reason;
    out_info->last_restart_unix = now;

    ESP_LOGI(TAG,
             "Restart metadata updated: count=%lu reason=%d time=%lld",
             static_cast<unsigned long>(boot_count),
             static_cast<int>(reason),
             static_cast<long long>(now));

    return ESP_OK;
}
