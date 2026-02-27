#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>

#ifdef __cplusplus
}
#endif

#include "adc_shared.h"

static const char *TAG = "adc_shared";

static adc_oneshot_unit_handle_t s_adc_handle = nullptr;
static bool s_initialized = false;
static adc_unit_t s_adc_unit = ADC_UNIT_1;

static StaticSemaphore_t s_mutex_buffer;
static SemaphoreHandle_t s_mutex = nullptr;
static portMUX_TYPE s_init_mux = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t ensure_mutex(void)
{
    if (s_mutex != nullptr) {
        return ESP_OK;
    }

    taskENTER_CRITICAL(&s_init_mux);
    if (s_mutex == nullptr) {
        s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_buffer);
    }
    taskEXIT_CRITICAL(&s_init_mux);

    return (s_mutex != nullptr) ? ESP_OK : ESP_ERR_NO_MEM;
}

static bool lock_mutex(TickType_t timeout_ticks)
{
    if (s_mutex == nullptr) {
        return false;
    }
    return xSemaphoreTake(s_mutex, timeout_ticks) == pdTRUE;
}

static void unlock_mutex(void)
{
    if (s_mutex != nullptr) {
        (void)xSemaphoreGive(s_mutex);
    }
}

esp_err_t adc_shared_init(adc_unit_t unit)
{
    esp_err_t mutex_result = ensure_mutex();
    if (mutex_result != ESP_OK) {
        return mutex_result;
    }

    if (!lock_mutex(pdMS_TO_TICKS(200))) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_initialized) {
        const esp_err_t result = (s_adc_unit == unit) ? ESP_OK : ESP_ERR_INVALID_STATE;
        unlock_mutex();
        return result;
    }

    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = unit;

    esp_err_t init_result = adc_oneshot_new_unit(&init_config, &s_adc_handle);
    if (init_result == ESP_OK) {
        s_adc_unit = unit;
        s_initialized = true;
    } else {
        ESP_LOGE(TAG, "adc_oneshot_new_unit selhalo: %s", esp_err_to_name(init_result));
    }

    unlock_mutex();
    return init_result;
}

esp_err_t adc_shared_config_channel(adc_channel_t channel,
                                    adc_bitwidth_t bitwidth,
                                    adc_atten_t atten)
{
    esp_err_t mutex_result = ensure_mutex();
    if (mutex_result != ESP_OK) {
        return mutex_result;
    }

    if (!lock_mutex(pdMS_TO_TICKS(200))) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_initialized || s_adc_handle == nullptr) {
        unlock_mutex();
        return ESP_ERR_INVALID_STATE;
    }

    adc_oneshot_chan_cfg_t config = {};
    config.bitwidth = bitwidth;
    config.atten = atten;

    esp_err_t config_result = adc_oneshot_config_channel(s_adc_handle, channel, &config);
    unlock_mutex();
    return config_result;
}

esp_err_t adc_shared_read(adc_channel_t channel, int *raw_value)
{
    if (raw_value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t mutex_result = ensure_mutex();
    if (mutex_result != ESP_OK) {
        return mutex_result;
    }

    if (!lock_mutex(pdMS_TO_TICKS(200))) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_initialized || s_adc_handle == nullptr) {
        unlock_mutex();
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t read_result = adc_oneshot_read(s_adc_handle, channel, raw_value);
    unlock_mutex();
    return read_result;
}
