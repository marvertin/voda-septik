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

#include <cmath>

#include "trimmed_mean.hpp"
#include "adc_shared.h"
#include "pins.h"
#include "sensor_events.h"
#include "config_store.h"
#include "debug_mqtt.h"
#include "app_error_check.h"

#define TAG "tlak"

namespace {

static constexpr int32_t PRESSURE_DEFAULT_RAW_4MA = 745;
static constexpr int32_t PRESSURE_DEFAULT_RAW_20MA = 3722;
static constexpr float PRESSURE_DEFAULT_MIN_BAR = 0.0f;
static constexpr float PRESSURE_DEFAULT_MAX_BAR = 10.0f;
static constexpr float PRESSURE_DEFAULT_DP100_BAR = 1.0f;
static constexpr float PRESSURE_DEFAULT_EMA_ALPHA = 0.55f;
static constexpr float PRESSURE_DEFAULT_HYST_BAR = 0.02f;
static constexpr int32_t PRESSURE_DEFAULT_SAMPLE_MS = 100;
static constexpr int32_t PRESSURE_DEFAULT_ROUND_DECIMALS = 2;

static constexpr float PRESSURE_MIN_BAR_LIMIT = -1.0f;
static constexpr float PRESSURE_MAX_BAR_LIMIT = 16.0f;
static constexpr int32_t PRESSURE_MIN_SAMPLE_MS = 10;
static constexpr int32_t PRESSURE_MAX_SAMPLE_MS = 1000;
static constexpr int32_t PRESSURE_MIN_ROUND_DECIMALS = 1;
static constexpr int32_t PRESSURE_MAX_ROUND_DECIMALS = 3;
static constexpr int64_t PRESSURE_CFG_DEBUG_PERIOD_US = 10LL * 1000LL * 1000LL;
static constexpr int32_t PRESSURE_RAW_SANITY_MIN = 0;
static constexpr int32_t PRESSURE_RAW_SANITY_MAX = 4095;
static constexpr int32_t PRESSURE_RAW_SANITY_MIN_MARGIN = 80;

static const config_item_t PRESSURE_BEFORE_RAW_4MA_ITEM = {
    .key = "tlk_b_raw_4ma", .label = "Tlak pred filtrem RAW pro 4 mA", .description = "ADC RAW hodnota (pred filtrem) odpovidajici vstupu 4 mA.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = PRESSURE_DEFAULT_RAW_4MA, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 4095, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t PRESSURE_BEFORE_RAW_20MA_ITEM = {
    .key = "tlk_b_raw_20ma", .label = "Tlak pred filtrem RAW pro 20 mA", .description = "ADC RAW hodnota (pred filtrem) odpovidajici vstupu 20 mA.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = PRESSURE_DEFAULT_RAW_20MA, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 1, .max_int = 4095, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t PRESSURE_BEFORE_MIN_ITEM = {
    .key = "tlk_b_p_min", .label = "Tlak pred filtrem min [bar]", .description = "Tlak pred filtrem odpovidajici 4 mA.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = PRESSURE_DEFAULT_MIN_BAR, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = PRESSURE_MIN_BAR_LIMIT, .max_float = PRESSURE_MAX_BAR_LIMIT,
};
static const config_item_t PRESSURE_BEFORE_MAX_ITEM = {
    .key = "tlk_b_p_max", .label = "Tlak pred filtrem max [bar]", .description = "Tlak pred filtrem odpovidajici 20 mA.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = PRESSURE_DEFAULT_MAX_BAR, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = PRESSURE_MIN_BAR_LIMIT, .max_float = PRESSURE_MAX_BAR_LIMIT,
};
static const config_item_t PRESSURE_AFTER_RAW_4MA_ITEM = {
    .key = "tlk_a_raw_4ma", .label = "Tlak za filtrem RAW pro 4 mA", .description = "ADC RAW hodnota (za filtrem) odpovidajici vstupu 4 mA.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = PRESSURE_DEFAULT_RAW_4MA, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 4095, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t PRESSURE_AFTER_RAW_20MA_ITEM = {
    .key = "tlk_a_raw_20ma", .label = "Tlak za filtrem RAW pro 20 mA", .description = "ADC RAW hodnota (za filtrem) odpovidajici vstupu 20 mA.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = PRESSURE_DEFAULT_RAW_20MA, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 1, .max_int = 4095, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t PRESSURE_AFTER_MIN_ITEM = {
    .key = "tlk_a_p_min", .label = "Tlak za filtrem min [bar]", .description = "Tlak za filtrem odpovidajici 4 mA.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = PRESSURE_DEFAULT_MIN_BAR, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = PRESSURE_MIN_BAR_LIMIT, .max_float = PRESSURE_MAX_BAR_LIMIT,
};
static const config_item_t PRESSURE_AFTER_MAX_ITEM = {
    .key = "tlk_a_p_max", .label = "Tlak za filtrem max [bar]", .description = "Tlak za filtrem odpovidajici 20 mA.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = PRESSURE_DEFAULT_MAX_BAR, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = PRESSURE_MIN_BAR_LIMIT, .max_float = PRESSURE_MAX_BAR_LIMIT,
};
static const config_item_t PRESSURE_EMA_ALPHA_ITEM = {
    .key = "tlk_ema_alpha", .label = "Tlak EMA alpha", .description = "Spolecny koeficient EMA filtru tlaku (0-1).",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = PRESSURE_DEFAULT_EMA_ALPHA, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 0.01f, .max_float = 1.0f,
};
static const config_item_t PRESSURE_HYST_BAR_ITEM = {
    .key = "tlk_hyst_bar", .label = "Tlak hystereze [bar]", .description = "Spolecne mrtve pasmo hystereze tlaku v barech.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = PRESSURE_DEFAULT_HYST_BAR, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 0.0f, .max_float = 1.0f,
};
static const config_item_t PRESSURE_SAMPLE_MS_ITEM = {
    .key = "tlk_sample_ms", .label = "Tlak perioda mereni [ms]", .description = "Spolecna perioda cteni obou tlakovych cidel.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = PRESSURE_DEFAULT_SAMPLE_MS, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = PRESSURE_MIN_SAMPLE_MS, .max_int = PRESSURE_MAX_SAMPLE_MS, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t PRESSURE_ROUND_DECIMALS_ITEM = {
    .key = "tlk_round_dec", .label = "Tlak zaokrouhleni desetinna mista", .description = "Spolecny pocet desetinnych mist pro publikovany tlak (1-3).",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = PRESSURE_DEFAULT_ROUND_DECIMALS, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = PRESSURE_MIN_ROUND_DECIMALS, .max_int = PRESSURE_MAX_ROUND_DECIMALS, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t PRESSURE_DP100_ITEM = {
    .key = "tlk_dp_100", .label = "dP pro 100% zaneseni [bar]", .description = "Rozdil tlaku, ktery odpovida 100% zanesenosti filtru.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = PRESSURE_DEFAULT_DP100_BAR, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 0.01f, .max_float = 20.0f,
};

typedef struct {
    int32_t raw_at_4ma;
    int32_t raw_at_20ma;
    float pressure_min_bar;
    float pressure_max_bar;
} pressure_sensor_calibration_t;

typedef struct {
    pressure_sensor_calibration_t before;
    pressure_sensor_calibration_t after;
    float ema_alpha;
    float hyst_bar;
    int32_t sample_ms;
    int32_t round_decimals;
    float dp_100_percent_bar;
} pressure_runtime_config_t;

static pressure_runtime_config_t g_pressure_config = {
    .before = {
        .raw_at_4ma = PRESSURE_DEFAULT_RAW_4MA,
        .raw_at_20ma = PRESSURE_DEFAULT_RAW_20MA,
        .pressure_min_bar = PRESSURE_DEFAULT_MIN_BAR,
        .pressure_max_bar = PRESSURE_DEFAULT_MAX_BAR,
    },
    .after = {
        .raw_at_4ma = PRESSURE_DEFAULT_RAW_4MA,
        .raw_at_20ma = PRESSURE_DEFAULT_RAW_20MA,
        .pressure_min_bar = PRESSURE_DEFAULT_MIN_BAR,
        .pressure_max_bar = PRESSURE_DEFAULT_MAX_BAR,
    },
    .ema_alpha = PRESSURE_DEFAULT_EMA_ALPHA,
    .hyst_bar = PRESSURE_DEFAULT_HYST_BAR,
    .sample_ms = PRESSURE_DEFAULT_SAMPLE_MS,
    .round_decimals = PRESSURE_DEFAULT_ROUND_DECIMALS,
    .dp_100_percent_bar = PRESSURE_DEFAULT_DP100_BAR,
};

static TrimmedMean<31, 5> pressure_before_filter;
static TrimmedMean<31, 5> pressure_after_filter;

static float s_before_ema_value = 0.0f;
static bool s_before_ema_initialized = false;
static float s_after_ema_value = 0.0f;
static bool s_after_ema_initialized = false;

static float s_before_hyst_value = 0.0f;
static bool s_before_hyst_initialized = false;
static float s_after_hyst_value = 0.0f;
static bool s_after_hyst_initialized = false;

static int64_t s_last_cfg_debug_publish_us = 0;

typedef struct {
    const char *name;
    adc_channel_t channel;
    TrimmedMean<31, 5> *filter;
    pressure_sensor_calibration_t *calibration;
    float *ema_value;
    bool *ema_initialized;
    float *hyst_value;
    bool *hyst_initialized;
} pressure_sensor_ctx_t;

typedef struct {
    uint32_t raw_unfiltered;
    uint32_t raw_filtered;
    float pressure_raw;
    float pressure_ema;
    float pressure_hyst;
    float pressure_rounded;
} pressure_sensor_sample_t;

static float clamp01(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

static void publish_config_debug(void)
{
    DEBUG_PUBLISH("tlak_cfg",
                  "b:r4=%ld r20=%ld pmin=%.3f pmax=%.3f a:r4=%ld r20=%ld pmin=%.3f pmax=%.3f ema=%.3f hy=%.4f sm=%ld rd=%ld dp100=%.3f",
                  (long)g_pressure_config.before.raw_at_4ma,
                  (long)g_pressure_config.before.raw_at_20ma,
                  (double)g_pressure_config.before.pressure_min_bar,
                  (double)g_pressure_config.before.pressure_max_bar,
                  (long)g_pressure_config.after.raw_at_4ma,
                  (long)g_pressure_config.after.raw_at_20ma,
                  (double)g_pressure_config.after.pressure_min_bar,
                  (double)g_pressure_config.after.pressure_max_bar,
                  (double)g_pressure_config.ema_alpha,
                  (double)g_pressure_config.hyst_bar,
                  (long)g_pressure_config.sample_ms,
                  (long)g_pressure_config.round_decimals,
                  (double)g_pressure_config.dp_100_percent_bar);
}

static void publish_config_debug_periodic(int64_t now_us)
{
    if (s_last_cfg_debug_publish_us != 0
        && (now_us - s_last_cfg_debug_publish_us) < PRESSURE_CFG_DEBUG_PERIOD_US) {
        return;
    }

    publish_config_debug();
    s_last_cfg_debug_publish_us = now_us;
}

static void sanitize_sensor_calibration(pressure_sensor_calibration_t *calibration,
                                        const char *sensor_name)
{
    if (calibration->raw_at_20ma <= calibration->raw_at_4ma) {
        calibration->raw_at_20ma = calibration->raw_at_4ma + 1;
        ESP_LOGW(TAG, "[%s] Neplatna kalibrace RAW (20mA <= 4mA), upravuji RAW_20mA na %ld",
                 sensor_name,
                 (long)calibration->raw_at_20ma);
    }

    if (calibration->pressure_max_bar <= calibration->pressure_min_bar) {
        calibration->pressure_max_bar = calibration->pressure_min_bar + 1.0f;
        ESP_LOGW(TAG, "[%s] Neplatny rozsah tlaku (max <= min), upravuji p_max na %.3f",
                 sensor_name,
                 (double)calibration->pressure_max_bar);
    }
}

static void load_pressure_calibration_config(void)
{
    g_pressure_config.before.raw_at_4ma = config_store_get_i32_item(&PRESSURE_BEFORE_RAW_4MA_ITEM);
    g_pressure_config.before.raw_at_20ma = config_store_get_i32_item(&PRESSURE_BEFORE_RAW_20MA_ITEM);
    g_pressure_config.before.pressure_min_bar = config_store_get_float_item(&PRESSURE_BEFORE_MIN_ITEM);
    g_pressure_config.before.pressure_max_bar = config_store_get_float_item(&PRESSURE_BEFORE_MAX_ITEM);

    g_pressure_config.after.raw_at_4ma = config_store_get_i32_item(&PRESSURE_AFTER_RAW_4MA_ITEM);
    g_pressure_config.after.raw_at_20ma = config_store_get_i32_item(&PRESSURE_AFTER_RAW_20MA_ITEM);
    g_pressure_config.after.pressure_min_bar = config_store_get_float_item(&PRESSURE_AFTER_MIN_ITEM);
    g_pressure_config.after.pressure_max_bar = config_store_get_float_item(&PRESSURE_AFTER_MAX_ITEM);

    g_pressure_config.ema_alpha = config_store_get_float_item(&PRESSURE_EMA_ALPHA_ITEM);
    g_pressure_config.hyst_bar = config_store_get_float_item(&PRESSURE_HYST_BAR_ITEM);
    g_pressure_config.sample_ms = config_store_get_i32_item(&PRESSURE_SAMPLE_MS_ITEM);
    g_pressure_config.round_decimals = config_store_get_i32_item(&PRESSURE_ROUND_DECIMALS_ITEM);
    g_pressure_config.dp_100_percent_bar = config_store_get_float_item(&PRESSURE_DP100_ITEM);

    sanitize_sensor_calibration(&g_pressure_config.before, "pred");
    sanitize_sensor_calibration(&g_pressure_config.after, "za");

    if (g_pressure_config.ema_alpha <= 0.0f || g_pressure_config.ema_alpha > 1.0f) {
        g_pressure_config.ema_alpha = PRESSURE_DEFAULT_EMA_ALPHA;
        ESP_LOGW(TAG, "Neplatna tlk_ema_alpha, pouzivam default %.3f", (double)g_pressure_config.ema_alpha);
    }

    if (g_pressure_config.hyst_bar < 0.0f) {
        g_pressure_config.hyst_bar = PRESSURE_DEFAULT_HYST_BAR;
        ESP_LOGW(TAG, "Neplatna tlk_hyst_bar, pouzivam default %.4f bar", (double)g_pressure_config.hyst_bar);
    }

    if (g_pressure_config.sample_ms < PRESSURE_MIN_SAMPLE_MS || g_pressure_config.sample_ms > PRESSURE_MAX_SAMPLE_MS) {
        g_pressure_config.sample_ms = PRESSURE_DEFAULT_SAMPLE_MS;
        ESP_LOGW(TAG, "Neplatna tlk_sample_ms, pouzivam default %ld ms", (long)g_pressure_config.sample_ms);
    }

    if (g_pressure_config.round_decimals < PRESSURE_MIN_ROUND_DECIMALS || g_pressure_config.round_decimals > PRESSURE_MAX_ROUND_DECIMALS) {
        g_pressure_config.round_decimals = PRESSURE_DEFAULT_ROUND_DECIMALS;
        ESP_LOGW(TAG, "Neplatna tlk_round_dec, pouzivam default %ld", (long)g_pressure_config.round_decimals);
    }

    if (g_pressure_config.dp_100_percent_bar <= 0.0f) {
        g_pressure_config.dp_100_percent_bar = PRESSURE_DEFAULT_DP100_BAR;
        ESP_LOGW(TAG, "Neplatne tlk_dp_100, pouzivam %.3f bar", (double)g_pressure_config.dp_100_percent_bar);
    }

    ESP_LOGI(TAG,
             "Kalibrace tlaku: pred(raw4=%ld raw20=%ld p_min=%.3f p_max=%.3f) za(raw4=%ld raw20=%ld p_min=%.3f p_max=%.3f) ema=%.3f hyst=%.4f sm=%ld rd=%ld dp100=%.3f",
             (long)g_pressure_config.before.raw_at_4ma,
             (long)g_pressure_config.before.raw_at_20ma,
             (double)g_pressure_config.before.pressure_min_bar,
             (double)g_pressure_config.before.pressure_max_bar,
             (long)g_pressure_config.after.raw_at_4ma,
             (long)g_pressure_config.after.raw_at_20ma,
             (double)g_pressure_config.after.pressure_min_bar,
             (double)g_pressure_config.after.pressure_max_bar,
             (double)g_pressure_config.ema_alpha,
             (double)g_pressure_config.hyst_bar,
             (long)g_pressure_config.sample_ms,
             (long)g_pressure_config.round_decimals,
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

static bool pressure_raw_is_plausible(const pressure_sensor_calibration_t &calibration, uint32_t raw_value)
{
    if (raw_value > (uint32_t)PRESSURE_RAW_SANITY_MAX) {
        return false;
    }

    const int32_t raw_min = calibration.raw_at_4ma;
    const int32_t raw_max = calibration.raw_at_20ma;
    const int32_t span = (raw_max >= raw_min) ? (raw_max - raw_min) : (raw_min - raw_max);
    int32_t margin = span / 4;
    if (margin < PRESSURE_RAW_SANITY_MIN_MARGIN) {
        margin = PRESSURE_RAW_SANITY_MIN_MARGIN;
    }

    int32_t plausible_min = ((raw_min < raw_max) ? raw_min : raw_max) - margin;
    int32_t plausible_max = ((raw_min > raw_max) ? raw_min : raw_max) + margin;

    if (plausible_min < PRESSURE_RAW_SANITY_MIN) {
        plausible_min = PRESSURE_RAW_SANITY_MIN;
    }
    if (plausible_max > PRESSURE_RAW_SANITY_MAX) {
        plausible_max = PRESSURE_RAW_SANITY_MAX;
    }

    const int32_t raw_signed = (int32_t)raw_value;
    return raw_signed >= plausible_min && raw_signed <= plausible_max;
}

static bool adc_read_raw(adc_channel_t channel, uint32_t *raw_value)
{
    if (raw_value == nullptr) {
        return false;
    }

    int raw = 0;
    esp_err_t result = adc_shared_read(channel, &raw);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Cteni ADC selhalo na kanalu %d: %s", (int)channel, esp_err_to_name(result));
        return false;
    }

    if (raw < PRESSURE_RAW_SANITY_MIN || raw > PRESSURE_RAW_SANITY_MAX) {
        ESP_LOGW(TAG, "ADC vratilo nesmyslnou RAW hodnotu na kanalu %d: %d", (int)channel, raw);
        return false;
    }

    *raw_value = (uint32_t)raw;
    return true;
}

static uint32_t adc_filter_trimmed_mean(TrimmedMean<31, 5> &filter, uint32_t raw_value)
{
    filter.insert((int)raw_value);
    return filter.getValue();
}

static float adc_raw_to_pressure_bar(const pressure_sensor_calibration_t &calibration, uint32_t raw_value)
{
    const int32_t raw_span = calibration.raw_at_20ma - calibration.raw_at_4ma;
    if (raw_span == 0) {
        return calibration.pressure_min_bar;
    }

    return calibration.pressure_min_bar +
           ((float)((int32_t)raw_value - calibration.raw_at_4ma) *
            (calibration.pressure_max_bar - calibration.pressure_min_bar) /
            (float)raw_span);
}

static float pressure_filter_ema(const pressure_sensor_ctx_t &ctx, float pressure_raw_bar)
{
    if (!(*ctx.ema_initialized)) {
        *ctx.ema_value = pressure_raw_bar;
        *ctx.ema_initialized = true;
    } else {
        const float alpha = g_pressure_config.ema_alpha;
        *ctx.ema_value = alpha * pressure_raw_bar + (1.0f - alpha) * (*ctx.ema_value);
    }

    return *ctx.ema_value;
}

static float pressure_apply_hysteresis(const pressure_sensor_ctx_t &ctx, float pressure_bar)
{
    if (!(*ctx.hyst_initialized)) {
        *ctx.hyst_value = pressure_bar;
        *ctx.hyst_initialized = true;
        return *ctx.hyst_value;
    }

    const float delta = pressure_bar - *ctx.hyst_value;
    if (delta >= g_pressure_config.hyst_bar || delta <= -g_pressure_config.hyst_bar) {
        *ctx.hyst_value = pressure_bar;
    }

    return *ctx.hyst_value;
}

static float round_to_decimals(float value, int32_t decimals)
{
    if (decimals <= 1) {
        return std::roundf(value * 10.0f) / 10.0f;
    }
    if (decimals >= 3) {
        return std::roundf(value * 1000.0f) / 1000.0f;
    }
    return std::roundf(value * 100.0f) / 100.0f;
}

static float pressure_diff_to_clogging_percent(float pressure_diff_bar)
{
    const float normalized = clamp01(pressure_diff_bar / g_pressure_config.dp_100_percent_bar);
    return normalized * 100.0f;
}

static bool measure_pressure_sensor(const pressure_sensor_ctx_t &ctx, pressure_sensor_sample_t *sample)
{
    if (sample == nullptr) {
        return false;
    }

    sample->raw_unfiltered = 0;
    sample->raw_filtered = 0;
    sample->pressure_raw = NAN;
    sample->pressure_ema = NAN;
    sample->pressure_hyst = NAN;
    sample->pressure_rounded = NAN;

    if (!adc_read_raw(ctx.channel, &sample->raw_unfiltered)) {
        return false;
    }

    if (!pressure_raw_is_plausible(*ctx.calibration, sample->raw_unfiltered)) {
        ESP_LOGW(TAG, "[%s] ADC RAW mimo ocekavany rozsah: %lu",
                 ctx.name,
                 (unsigned long)sample->raw_unfiltered);
        return false;
    }

    sample->raw_filtered = adc_filter_trimmed_mean(*ctx.filter, sample->raw_unfiltered);
    sample->pressure_raw = adc_raw_to_pressure_bar(*ctx.calibration, sample->raw_filtered);
    sample->pressure_ema = pressure_filter_ema(ctx, sample->pressure_raw);
    sample->pressure_hyst = pressure_apply_hysteresis(ctx, sample->pressure_ema);
    sample->pressure_rounded = round_to_decimals(sample->pressure_hyst, g_pressure_config.round_decimals);
    return true;
}

static void prefill_pressure_sensor(const pressure_sensor_ctx_t &ctx)
{
    pressure_sensor_sample_t sample = {};
    (void)measure_pressure_sensor(ctx, &sample);
}

static void warmup_filters(const pressure_sensor_ctx_t *sensors, size_t sensor_count)
{
    const size_t buffer_size = pressure_before_filter.getBufferSize();
    ESP_LOGI(TAG, "Prebiha nabiti bufferu tlaku (%zu mereni)...", buffer_size);

    for (size_t index = 0; index < buffer_size; ++index) {
        for (size_t sensor_index = 0; sensor_index < sensor_count; ++sensor_index) {
            prefill_pressure_sensor(sensors[sensor_index]);
        }
        APP_ERROR_CHECK("E539", esp_task_wdt_reset());
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGI(TAG, "Buffer tlaku nabit, zacinam publikovat vysledky");
}

static void publish_invalid_pressure_measurement(int64_t timestamp_us,
                                                const pressure_sensor_sample_t *samples,
                                                size_t sample_count)
{
    app_event_t invalid_event = {
        .event_type = EVT_SENSOR,
        .timestamp_us = timestamp_us,
        .data = {
            .sensor = {
                .sensor_type = SENSOR_EVENT_PRESSURE,
                .data = {
                    .pressure = {
                        .pred_filtrem = NAN,
                        .za_filtrem = NAN,
                        .rozdil_filtru = NAN,
                        .zanesenost_filtru = NAN,
                    },
                },
            },
        },
    };

    const unsigned long pred_raw = (sample_count > 0) ? (unsigned long)samples[0].raw_unfiltered : 0UL;
    const unsigned long za_raw = (sample_count > 1) ? (unsigned long)samples[1].raw_unfiltered : 0UL;

    bool queued = sensor_events_publish(&invalid_event, pdMS_TO_TICKS(20));
    DEBUG_PUBLISH("tlak_dyn",
                  "q=%d ts=%lld invalid=1 pred_raw=%lu za_raw=%lu",
                  queued ? 1 : 0,
                  (long long)timestamp_us,
                  pred_raw,
                  za_raw);
}

static void tlak_task(void *pvParameters)
{
    (void)pvParameters;
    APP_ERROR_CHECK("E538", esp_task_wdt_add(nullptr));

    ESP_LOGI(TAG, "Spoustim mereni tlaku (pred/za filtrem)...");

    const pressure_sensor_ctx_t sensors[] = {
        {
            .name = "pred",
            .channel = PRESSURE_SENSOR_BEFORE_ADC_CHANNEL,
            .filter = &pressure_before_filter,
            .calibration = &g_pressure_config.before,
            .ema_value = &s_before_ema_value,
            .ema_initialized = &s_before_ema_initialized,
            .hyst_value = &s_before_hyst_value,
            .hyst_initialized = &s_before_hyst_initialized,
        },
        {
            .name = "za",
            .channel = PRESSURE_SENSOR_AFTER_ADC_CHANNEL,
            .filter = &pressure_after_filter,
            .calibration = &g_pressure_config.after,
            .ema_value = &s_after_ema_value,
            .ema_initialized = &s_after_ema_initialized,
            .hyst_value = &s_after_hyst_value,
            .hyst_initialized = &s_after_hyst_initialized,
        },
    };

    APP_ERROR_CHECK("E521", adc_init());
    warmup_filters(sensors, sizeof(sensors) / sizeof(sensors[0]));

    while (true) {
        int64_t timestamp_us = esp_timer_get_time();
        pressure_sensor_sample_t samples[sizeof(sensors) / sizeof(sensors[0])] = {};
        bool sensors_valid = true;
        for (size_t i = 0; i < (sizeof(sensors) / sizeof(sensors[0])); ++i) {
            if (!measure_pressure_sensor(sensors[i], &samples[i])) {
                sensors_valid = false;
            }
        }

        if (!sensors_valid) {
            publish_invalid_pressure_measurement(timestamp_us, samples, sizeof(samples) / sizeof(samples[0]));
        } else {
            const pressure_sensor_sample_t &pred_filtrem_sensor = samples[0];
            const pressure_sensor_sample_t &za_filtrem_sensor = samples[1];

            const float pred_filtrem = pred_filtrem_sensor.pressure_rounded;
            const float za_filtrem = za_filtrem_sensor.pressure_rounded;
            const float rozdil_filtru = pred_filtrem - za_filtrem;
            const float zanesenost_filtru = pressure_diff_to_clogging_percent(rozdil_filtru);

            app_event_t event = {
                .event_type = EVT_SENSOR,
                .timestamp_us = timestamp_us,
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

            DEBUG_PUBLISH("tlak_dyn",
                          "q=%d ts=%lld pred:r=%lu rt=%lu p=%.3f pe=%.3f ph=%.3f pr=%.3f za:r=%lu rt=%lu p=%.3f pe=%.3f ph=%.3f pr=%.3f dp=%.3f clog=%.1f",
                          queued ? 1 : 0,
                          (long long)event.timestamp_us,
                          (unsigned long)pred_filtrem_sensor.raw_unfiltered,
                          (unsigned long)pred_filtrem_sensor.raw_filtered,
                          (double)pred_filtrem_sensor.pressure_raw,
                          (double)pred_filtrem_sensor.pressure_ema,
                          (double)pred_filtrem_sensor.pressure_hyst,
                          (double)pred_filtrem_sensor.pressure_rounded,
                          (unsigned long)za_filtrem_sensor.raw_unfiltered,
                          (unsigned long)za_filtrem_sensor.raw_filtered,
                          (double)za_filtrem_sensor.pressure_raw,
                          (double)za_filtrem_sensor.pressure_ema,
                          (double)za_filtrem_sensor.pressure_hyst,
                          (double)za_filtrem_sensor.pressure_rounded,
                          (double)rozdil_filtru,
                          (double)zanesenost_filtru);
        }

        publish_config_debug_periodic(timestamp_us);

        APP_ERROR_CHECK("E540", esp_task_wdt_reset());
        vTaskDelay(pdMS_TO_TICKS(g_pressure_config.sample_ms));
    }
}

} // namespace

void tlak_register_config_items(void)
{
    APP_ERROR_CHECK("E685", config_store_register_item(&PRESSURE_BEFORE_RAW_4MA_ITEM));
    APP_ERROR_CHECK("E686", config_store_register_item(&PRESSURE_BEFORE_RAW_20MA_ITEM));
    APP_ERROR_CHECK("E687", config_store_register_item(&PRESSURE_BEFORE_MIN_ITEM));
    APP_ERROR_CHECK("E688", config_store_register_item(&PRESSURE_BEFORE_MAX_ITEM));
    APP_ERROR_CHECK("E689", config_store_register_item(&PRESSURE_AFTER_RAW_4MA_ITEM));
    APP_ERROR_CHECK("E690", config_store_register_item(&PRESSURE_AFTER_RAW_20MA_ITEM));
    APP_ERROR_CHECK("E691", config_store_register_item(&PRESSURE_AFTER_MIN_ITEM));
    APP_ERROR_CHECK("E692", config_store_register_item(&PRESSURE_AFTER_MAX_ITEM));
    APP_ERROR_CHECK("E693", config_store_register_item(&PRESSURE_EMA_ALPHA_ITEM));
    APP_ERROR_CHECK("E694", config_store_register_item(&PRESSURE_HYST_BAR_ITEM));
    APP_ERROR_CHECK("E695", config_store_register_item(&PRESSURE_SAMPLE_MS_ITEM));
    APP_ERROR_CHECK("E696", config_store_register_item(&PRESSURE_ROUND_DECIMALS_ITEM));
    APP_ERROR_CHECK("E697", config_store_register_item(&PRESSURE_DP100_ITEM));
}

void tlak_init(void)
{
    load_pressure_calibration_config();
    APP_ERROR_CHECK("E523",
                    xTaskCreate(tlak_task, TAG, configMINIMAL_STACK_SIZE * 6, nullptr, 5, nullptr) == pdPASS
                        ? ESP_OK
                        : ESP_FAIL);
}
