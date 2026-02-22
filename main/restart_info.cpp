#include "restart_info.h"

#include <ctime>

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "restart_info";
static const char *SYS_NAMESPACE = "sys_meta";
static const char *SYS_BOOT_COUNT_KEY = "boot_count";
static const char *SYS_LAST_REASON_KEY = "last_reason";
static const char *SYS_LAST_TIME_KEY = "last_time";

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

    ESP_ERROR_CHECK(nvs_set_u32(nvs_handle, SYS_BOOT_COUNT_KEY, boot_count));
    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, SYS_LAST_REASON_KEY, static_cast<int32_t>(reason)));
    ESP_ERROR_CHECK(nvs_set_i64(nvs_handle, SYS_LAST_TIME_KEY, now));
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
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
