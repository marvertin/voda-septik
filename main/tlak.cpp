#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>

#ifdef __cplusplus
}
#endif

#include "trimmed_mean.hpp"
#include "adc_shared.h"
#include "pins.h"
#include "sensor_events.h"
#include "config_store.h"
#include "debug_mqtt.h"
#include "app_error_check.h"

#define TAG "tlak"

namespace {

static const config_item_t PRESSURE_RAW_4MA_ITEM = {
    .key = "tlk_raw_4ma", .label = "Tlak RAW pro 4 mA", .description = "ADC RAW hodnota odpovidajici vstupu 4 mA.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = 745, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 4095, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t PRESSURE_RAW_20MA_ITEM = {
    .key = "tlk_raw_20ma", .label = "Tlak RAW pro 20 mA", .description = "ADC RAW hodnota odpovidajici vstupu 20 mA.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = 3722, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 1, .max_int = 4095, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t PRESSURE_MIN_ITEM = {
    .key = "tlk_p_min", .label = "Tlak min [bar]", .description = "Tlak odpovidajici 4 mA.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = -1.0f, .max_float = 100.0f,
};
static const config_item_t PRESSURE_MAX_ITEM = {
    .key = "tlk_p_max", .label = "Tlak max [bar]", .description = "Tlak odpovidajici 20 mA.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = 10.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = -1.0f, .max_float = 100.0f,
};
static const config_item_t PRESSURE_DP100_ITEM = {
    .key = "tlk_dp_100", .label = "dP pro 100% zaneseni [bar]", .description = "Rozdil tlaku, ktery odpovida 100% zanesenosti filtru.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = 1.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 0.01f, .max_float = 20.0f,
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

typedef struct {
    uint32_t raw_unfiltered;
    uint32_t raw_filtered;
    float current_ma;
    float pressure_bar;
} pressure_sensor_sample_t;

typedef struct {
    const char *name;
    adc_channel_t channel;
    TrimmedMean<31, 5> *filter;
} pressure_sensor_ctx_t;

static void load_pressure_calibration_config(void)
{
    g_pressure_config.raw_at_4ma = config_store_get_i32_item(&PRESSURE_RAW_4MA_ITEM);
    g_pressure_config.raw_at_20ma = config_store_get_i32_item(&PRESSURE_RAW_20MA_ITEM);
    g_pressure_config.pressure_min_bar = config_store_get_float_item(&PRESSURE_MIN_ITEM);
    g_pressure_config.pressure_max_bar = config_store_get_float_item(&PRESSURE_MAX_ITEM);
    g_pressure_config.dp_100_percent_bar = config_store_get_float_item(&PRESSURE_DP100_ITEM);

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
    ESP_LOGI(TAG,
             "ADC init: unit=%d bitwidth=%d atten=%d pred(gpio=%d,ch=%d) za(gpio=%d,ch=%d)",
             (int)PRESSURE_SENSOR_ADC_UNIT,
             (int)PRESSURE_SENSOR_ADC_BITWIDTH,
             (int)PRESSURE_SENSOR_ADC_ATTENUATION,
             32,
             (int)PRESSURE_SENSOR_BEFORE_ADC_CHANNEL,
             33,
             (int)PRESSURE_SENSOR_AFTER_ADC_CHANNEL);

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

static pressure_sensor_sample_t measure_pressure_sensor(adc_channel_t channel,
                                                        TrimmedMean<31, 5> &filter)
{
    pressure_sensor_sample_t sample = {};

    (void)read_raw_with_filter(channel,
                               filter,
                               &sample.raw_unfiltered,
                               &sample.raw_filtered);

    sample.current_ma = raw_to_current_ma(sample.raw_filtered);
    sample.pressure_bar = current_ma_to_pressure_bar(sample.current_ma);
    return sample;
}

static pressure_sensor_sample_t measure_pressure_sensor(const pressure_sensor_ctx_t &ctx)
{
    return measure_pressure_sensor(ctx.channel, *ctx.filter);
}

static void prefill_pressure_sensor(const pressure_sensor_ctx_t &ctx)
{
    (void)read_raw_with_filter(ctx.channel,
                               *ctx.filter,
                               nullptr,
                               nullptr);
}

static void tlak_task(void *pvParameters)
{
    (void)pvParameters;
    APP_ERROR_CHECK("E538", esp_task_wdt_add(nullptr));

    ESP_LOGI(TAG, "Spoustim mereni tlaku (pred/za filtrem)...");

    const pressure_sensor_ctx_t sensors[] = {
        {.name = "pred", .channel = PRESSURE_SENSOR_BEFORE_ADC_CHANNEL, .filter = &pressure_before_filter},
        {.name = "za", .channel = PRESSURE_SENSOR_AFTER_ADC_CHANNEL, .filter = &pressure_after_filter},
    };

    APP_ERROR_CHECK("E521", adc_init());

    const size_t prefill = pressure_before_filter.getBufferSize();
    for (size_t i = 0; i < prefill; ++i) {
        for (const pressure_sensor_ctx_t &sensor : sensors) {
            prefill_pressure_sensor(sensor);
        }
        APP_ERROR_CHECK("E539", esp_task_wdt_reset());
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    while (true) {
        pressure_sensor_sample_t samples[sizeof(sensors) / sizeof(sensors[0])] = {};
        for (size_t i = 0; i < (sizeof(sensors) / sizeof(sensors[0])); ++i) {
            samples[i] = measure_pressure_sensor(sensors[i]);
        }

        const pressure_sensor_sample_t &pred_filtrem_sensor = samples[0];
        const pressure_sensor_sample_t &za_filtrem_sensor = samples[1];

        const float pred_filtrem = pred_filtrem_sensor.pressure_bar;
        const float za_filtrem = za_filtrem_sensor.pressure_bar;
        const float rozdil_filtru = pred_filtrem - za_filtrem;
        const float zanesenost_filtru = pressure_diff_to_clogging_percent(rozdil_filtru);

        app_event_t event = {
            .event_type = EVT_SENSOR,
            .timestamp_us = esp_timer_get_time(),
            .data = {
                .sensor = {
                    .sensor_type = SENSOR_EVENT_PRESSURE,
                    .data = {
                        .pressure = {
                            .pred_filtrem = pred_filtrem,
                            .za_filtrem = za_filtrem,
                            .rozdil_filtru = rozdil_filtru,
                            .zanesenost_filtru = zanesenost_filtru,
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
             (unsigned long)pred_filtrem_sensor.raw_unfiltered,
             (unsigned long)pred_filtrem_sensor.raw_filtered,
                 (double)pred_filtrem_sensor.current_ma,
                 (double)pred_filtrem,
             (unsigned long)za_filtrem_sensor.raw_unfiltered,
             (unsigned long)za_filtrem_sensor.raw_filtered,
                 (double)za_filtrem_sensor.current_ma,
                 (double)za_filtrem,
                 (double)rozdil_filtru,
                 (double)zanesenost_filtru,
                 queued ? 1 : 0);

        DEBUG_PUBLISH("tlak",
                  "queued=%d ts=%lld before_raw_pre=%lu before_raw_post=%lu after_raw_pre=%lu after_raw_post=%lu p_before=%.3f p_after=%.3f dp=%.3f clog=%.1f",
                  queued ? 1 : 0,
                  (long long)event.timestamp_us,
                  (unsigned long)pred_filtrem_sensor.raw_unfiltered,
                  (unsigned long)pred_filtrem_sensor.raw_filtered,
                  (unsigned long)za_filtrem_sensor.raw_unfiltered,
                  (unsigned long)za_filtrem_sensor.raw_filtered,
                  (double)pred_filtrem,
                  (double)za_filtrem,
                  (double)rozdil_filtru,
                  (double)zanesenost_filtru);

      DEBUG_PUBLISH("tlak2",
                  "pred=%lu za=%lu",
                  (unsigned long)pred_filtrem_sensor.raw_unfiltered,
                  (unsigned long)za_filtrem_sensor.raw_unfiltered);

        APP_ERROR_CHECK("E540", esp_task_wdt_reset());

                  
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

} // namespace

void tlak_register_config_items(void)
{
    APP_ERROR_CHECK("E685", config_store_register_item(&PRESSURE_RAW_4MA_ITEM));
    APP_ERROR_CHECK("E686", config_store_register_item(&PRESSURE_RAW_20MA_ITEM));
    APP_ERROR_CHECK("E687", config_store_register_item(&PRESSURE_MIN_ITEM));
    APP_ERROR_CHECK("E688", config_store_register_item(&PRESSURE_MAX_ITEM));
    APP_ERROR_CHECK("E689", config_store_register_item(&PRESSURE_DP100_ITEM));
}

void tlak_init(void)
{
    load_pressure_calibration_config();
    APP_ERROR_CHECK("E523",
                    xTaskCreate(tlak_task, TAG, configMINIMAL_STACK_SIZE * 6, nullptr, 5, nullptr) == pdPASS
                        ? ESP_OK
                        : ESP_FAIL);
}
