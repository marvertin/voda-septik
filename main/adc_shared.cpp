#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
#include "app_error_check.h"

#ifdef __cplusplus
}
#endif

#include "adc_shared.h"

static const char *TAG = "adc_shared";

static adc_oneshot_unit_handle_t s_adc_handle = nullptr;
static StaticSemaphore_t s_adc_read_mutex_buffer;
static SemaphoreHandle_t s_adc_read_mutex = nullptr;

void adc_shared_init()
{
    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;

    APP_ERROR_CHECK("E519", adc_oneshot_new_unit(&init_config, &s_adc_handle));
    s_adc_read_mutex = xSemaphoreCreateMutexStatic(&s_adc_read_mutex_buffer);
    APP_ERROR_CHECK("E519", s_adc_read_mutex != nullptr ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_LOGI(TAG, "ADC jednotka inicializovana (unit=%d)", (int)ADC_UNIT_1);
}

void adc_channel_init(adc_channel_t channel)
{

     adc_oneshot_chan_cfg_t chan_cfg = {
       .atten = ADC_ATTEN_DB_0,              // nejlepší přesnost do ~1.1V
       .bitwidth = ADC_BITWIDTH_12,      // 12 bit na ESP32
    };  

    esp_err_t cfg_result = adc_oneshot_config_channel(s_adc_handle, channel, &chan_cfg);

    APP_ERROR_CHECK("E519", cfg_result);
    ESP_LOGI(TAG, "ADC kanal nakonfigurovan (channel=%d bitwidth=%d atten=%d)", (int)channel, (int)ADC_BITWIDTH_12, (int)ADC_ATTEN_DB_0    );
}   

int  adc_read(adc_channel_t channel)
{
    APP_ERROR_CHECK("E536", s_adc_read_mutex != nullptr ? ESP_OK : ESP_ERR_INVALID_STATE);
    APP_ERROR_CHECK("E536", xSemaphoreTake(s_adc_read_mutex, portMAX_DELAY) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT);

    int raw = 0;
    esp_err_t result = adc_oneshot_read(s_adc_handle, channel, &raw);
    (void)xSemaphoreGive(s_adc_read_mutex);

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Cteni ADC selhalo (channel=%d): %s", (int)channel, esp_err_to_name(result));
    }
    APP_ERROR_CHECK("E536", result);        
    return raw  ;
}
