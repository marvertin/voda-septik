#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#ifdef __cplusplus
}
#endif

#include "trimmed_mean.hpp"
#include "adc_shared.h"
#include "pins.h"
#include "sensor_events.h"
#include "config_webapp.h"
#include "debug_mqtt.h"

#define TAG "TLAK"

namespace {

static const config_item_t PRESSURE_CONFIG_ITEMS[] = {
    {
        .key = "tlk_raw_4ma",
        .label = "Tlak RAW pro 4 mA",
        .description = "ADC RAW hodnota odpovidajici vstupu 4 mA.",
        .type = CONFIG_VALUE_INT32,
        .default_string = nullptr,
        .default_int = 745,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 4095,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "tlk_raw_20ma",
        .label = "Tlak RAW pro 20 mA",
        .description = "ADC RAW hodnota odpovidajici vstupu 20 mA.",
        .type = CONFIG_VALUE_INT32,
        .default_string = nullptr,
        .default_int = 3722,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 1,
        .max_int = 4095,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "tlk_p_min",
        .label = "Tlak min [bar]",
        .description = "Tlak odpovidajici 4 mA.",
        .type = CONFIG_VALUE_FLOAT,
        .default_string = nullptr,
        .default_int = 0,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 0,
        .min_float = -1.0f,
        .max_float = 100.0f,
    },
    {
        .key = "tlk_p_max",
        .label = "Tlak max [bar]",
        .description = "Tlak odpovidajici 20 mA.",
        .type = CONFIG_VALUE_FLOAT,
        .default_string = nullptr,
        .default_int = 0,
        .default_float = 10.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 0,
        .min_float = -1.0f,
        .max_float = 100.0f,
    },
    {
        .key = "tlk_dp_100",
        .label = "dP pro 100% zaneseni [bar]",
        .description = "Rozdil tlaku, ktery odpovida 100% zanesenosti filtru.",
        .type = CONFIG_VALUE_FLOAT,
        .default_string = nullptr,
        .default_int = 0,
        .default_float = 1.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.01f,
        .max_float = 20.0f,
    },
};

typedef struct {
    int32_t raw_at_4ma;
    int32_t raw_at_20ma;
    float pressure_min_bar;
    float pressure_max_bar;
    float dp_100_percent_bar;
} pressure_calibration_config_t;

static pressure_calibration_config_t g_pressure_config = {
    .raw_at_4ma = 745,
    .raw_at_20ma = 3722,
    .pressure_min_bar = 0.0f,
    .pressure_max_bar = 10.0f,
    .dp_100_percent_bar = 1.0f,
};

static TrimmedMean<31, 5> pressure_before_filter;
static TrimmedMean<31, 5> pressure_after_filter;

static void load_pressure_calibration_config(void)
{
    esp_err_t ret = config_webapp_get_i32("tlk_raw_4ma", &g_pressure_config.raw_at_4ma);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Konfigurace tlk_raw_4ma neni dostupna (%s), pouzivam default", esp_err_to_name(ret));
    }

    ret = config_webapp_get_i32("tlk_raw_20ma", &g_pressure_config.raw_at_20ma);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Konfigurace tlk_raw_20ma neni dostupna (%s), pouzivam default", esp_err_to_name(ret));
    }

    ret = config_webapp_get_float("tlk_p_min", &g_pressure_config.pressure_min_bar);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Konfigurace tlk_p_min neni dostupna (%s), pouzivam default", esp_err_to_name(ret));
    }

    ret = config_webapp_get_float("tlk_p_max", &g_pressure_config.pressure_max_bar);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Konfigurace tlk_p_max neni dostupna (%s), pouzivam default", esp_err_to_name(ret));
    }

    ret = config_webapp_get_float("tlk_dp_100", &g_pressure_config.dp_100_percent_bar);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Konfigurace tlk_dp_100 neni dostupna (%s), pouzivam default", esp_err_to_name(ret));
    }

    if (g_pressure_config.raw_at_20ma <= g_pressure_config.raw_at_4ma) {
        g_pressure_config.raw_at_20ma = g_pressure_config.raw_at_4ma + 1;
        ESP_LOGW(TAG, "Neplatna kalibrace RAW (20mA <= 4mA), upravuji tlk_raw_20ma na %ld",
                 (long)g_pressure_config.raw_at_20ma);
    }

    if (g_pressure_config.pressure_max_bar <= g_pressure_config.pressure_min_bar) {
        g_pressure_config.pressure_max_bar = g_pressure_config.pressure_min_bar + 1.0f;
        ESP_LOGW(TAG, "Neplatny rozsah tlaku (max <= min), upravuji tlk_p_max na %.3f",
                 (double)g_pressure_config.pressure_max_bar);
    }

    if (g_pressure_config.dp_100_percent_bar <= 0.0f) {
        g_pressure_config.dp_100_percent_bar = 1.0f;
        ESP_LOGW(TAG, "Neplatne tlk_dp_100, pouzivam 1.0 bar");
    }

    ESP_LOGI(TAG,
             "Kalibrace tlaku: raw4=%ld raw20=%ld p_min=%.3f p_max=%.3f dp100=%.3f",
             (long)g_pressure_config.raw_at_4ma,
             (long)g_pressure_config.raw_at_20ma,
             (double)g_pressure_config.pressure_min_bar,
             (double)g_pressure_config.pressure_max_bar,
             (double)g_pressure_config.dp_100_percent_bar);
}

static esp_err_t adc_init(void)
{
    esp_err_t ret = adc_shared_init(PRESSURE_SENSOR_ADC_UNIT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nelze inicializovat ADC jednotku: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = adc_shared_config_channel(PRESSURE_SENSOR_BEFORE_ADC_CHANNEL,
                                    PRESSURE_SENSOR_ADC_BITWIDTH,
                                    PRESSURE_SENSOR_ADC_ATTENUATION);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nelze nakonfigurovat kanal pred filtrem: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = adc_shared_config_channel(PRESSURE_SENSOR_AFTER_ADC_CHANNEL,
                                    PRESSURE_SENSOR_ADC_BITWIDTH,
                                    PRESSURE_SENSOR_ADC_ATTENUATION);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nelze nakonfigurovat kanal za filtrem: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

static bool read_raw_with_filter(adc_channel_t channel,
                                 TrimmedMean<31, 5> &filter,
                                 uint32_t *raw_unfiltered,
                                 uint32_t *raw_filtered)
{
    int raw = 0;
    if (adc_shared_read(channel, &raw) != ESP_OK) {
        ESP_LOGW(TAG, "Cteni ADC selhalo na kanalu %d", (int)channel);
        if (raw_unfiltered != nullptr) {
            *raw_unfiltered = 0;
        }
        if (raw_filtered != nullptr) {
            *raw_filtered = 0;
        }
        return false;
    }

    if (raw_unfiltered != nullptr) {
        *raw_unfiltered = (uint32_t)raw;
    }

    filter.insert(raw);
    if (raw_filtered != nullptr) {
        *raw_filtered = filter.getValue();
    }

    return true;
}

static float raw_to_current_ma(uint32_t raw)
{
    const float raw_span = (float)(g_pressure_config.raw_at_20ma - g_pressure_config.raw_at_4ma);
    const float normalized = ((float)raw - (float)g_pressure_config.raw_at_4ma) / raw_span;
    return 4.0f + normalized * 16.0f;
}

static float current_ma_to_pressure_bar(float current_ma)
{
    float normalized = (current_ma - 4.0f) / 16.0f;
    if (normalized < 0.0f) {
        normalized = 0.0f;
    }
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }

    return g_pressure_config.pressure_min_bar + normalized * (g_pressure_config.pressure_max_bar - g_pressure_config.pressure_min_bar);
}

static float pressure_diff_to_clogging_percent(float pressure_diff_bar)
{
    float normalized = pressure_diff_bar / g_pressure_config.dp_100_percent_bar;
    if (normalized < 0.0f) {
        normalized = 0.0f;
    }
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }
    return normalized * 100.0f;
}

static void tlak_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Spoustim mereni tlaku (pred/za filtrem)...");

    if (adc_init() != ESP_OK) {
        ESP_LOGE(TAG, "Inicializace ADC pro tlak selhala");
        vTaskDelete(nullptr);
        return;
    }

    const size_t prefill = pressure_before_filter.getBufferSize();
    for (size_t i = 0; i < prefill; ++i) {
        (void)read_raw_with_filter(PRESSURE_SENSOR_BEFORE_ADC_CHANNEL,
                                   pressure_before_filter,
                                   nullptr,
                                   nullptr);
        (void)read_raw_with_filter(PRESSURE_SENSOR_AFTER_ADC_CHANNEL,
                                   pressure_after_filter,
                                   nullptr,
                                   nullptr);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    while (true) {
        uint32_t raw_before_unfiltered = 0;
        uint32_t raw_after_unfiltered = 0;
        uint32_t raw_before_filtered = 0;
        uint32_t raw_after_filtered = 0;

        (void)read_raw_with_filter(PRESSURE_SENSOR_BEFORE_ADC_CHANNEL,
                                   pressure_before_filter,
                                   &raw_before_unfiltered,
                                   &raw_before_filtered);
        (void)read_raw_with_filter(PRESSURE_SENSOR_AFTER_ADC_CHANNEL,
                                   pressure_after_filter,
                                   &raw_after_unfiltered,
                                   &raw_after_filtered);

        const float current_before_ma = raw_to_current_ma(raw_before_filtered);
        const float current_after_ma = raw_to_current_ma(raw_after_filtered);

        const float pressure_before_bar = current_ma_to_pressure_bar(current_before_ma);
        const float pressure_after_bar = current_ma_to_pressure_bar(current_after_ma);
        const float pressure_drop_bar = pressure_before_bar - pressure_after_bar;
        const float filter_clogging_percent = pressure_diff_to_clogging_percent(pressure_drop_bar);

        app_event_t event = {
            .event_type = EVT_SENSOR,
            .timestamp_us = esp_timer_get_time(),
            .data = {
                .sensor = {
                    .sensor_type = SENSOR_EVENT_PRESSURE,
                    .data = {
                        .pressure = {
                            .pressure_before_bar = pressure_before_bar,
                            .pressure_after_bar = pressure_after_bar,
                            .pressure_diff_bar = pressure_drop_bar,
                            .filter_clogging_percent = filter_clogging_percent,
                        },
                    },
                },
            },
        };

        bool queued = sensor_events_publish(&event, pdMS_TO_TICKS(20));
        if (!queued) {
            ESP_LOGW(TAG, "Fronta sensor eventu je plna, tlak zahozen");
        }

        ESP_LOGD(TAG,
             "pred: raw_pre=%lu raw_post=%lu i=%.2f mA p=%.3f bar | za: raw_pre=%lu raw_post=%lu i=%.2f mA p=%.3f bar | dP=%.3f bar clog=%.1f%% queued=%d",
             (unsigned long)raw_before_unfiltered,
             (unsigned long)raw_before_filtered,
                 (double)current_before_ma,
                 (double)pressure_before_bar,
             (unsigned long)raw_after_unfiltered,
             (unsigned long)raw_after_filtered,
                 (double)current_after_ma,
                 (double)pressure_after_bar,
                 (double)pressure_drop_bar,
                 (double)filter_clogging_percent,
                 queued ? 1 : 0);

        DEBUG_PUBLISH("tlak",
                  "queued=%d ts=%lld before_raw_pre=%lu before_raw_post=%lu after_raw_pre=%lu after_raw_post=%lu p_before=%.3f p_after=%.3f dp=%.3f clog=%.1f",
                  queued ? 1 : 0,
                  (long long)event.timestamp_us,
                  (unsigned long)raw_before_unfiltered,
                  (unsigned long)raw_before_filtered,
                  (unsigned long)raw_after_unfiltered,
                  (unsigned long)raw_after_filtered,
                  (double)pressure_before_bar,
                  (double)pressure_after_bar,
                  (double)pressure_drop_bar,
                  (double)filter_clogging_percent);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

} // namespace

void tlak_init(void)
{
    load_pressure_calibration_config();
    xTaskCreate(tlak_task, TAG, configMINIMAL_STACK_SIZE * 6, nullptr, 5, nullptr);
}

config_group_t tlak_get_config_group(void)
{
    config_group_t group = {
        .items = PRESSURE_CONFIG_ITEMS,
        .item_count = sizeof(PRESSURE_CONFIG_ITEMS) / sizeof(PRESSURE_CONFIG_ITEMS[0]),
    };
    return group;
}
