#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>

#ifdef __cplusplus
}
#endif

#include <cmath>

#include "trimmed_mean.hpp"
#include "adc_shared.h"
#include "pins.h"
#include "config_store.h"
#include "sensor_events.h"
#include "debug_mqtt.h"
#include "app_error_check.h"

#define TAG "zasoba"

// --- Modul zasoba: konfigurace, filtrace a publikace objemu/vysky ---
// ADC konfigurace pro senzor hladiny je centralizovana v pins.h.

// Vychozi kalibracni hodnoty a limity (jedine misto, kde jsou cisla definovana)
static constexpr int32_t LEVEL_DEFAULT_RAW_MIN = 540;
static constexpr int32_t LEVEL_DEFAULT_RAW_MAX = 950;
static constexpr float LEVEL_DEFAULT_HEIGHT_MIN_M = 0.0f;
static constexpr float LEVEL_DEFAULT_HEIGHT_MAX_M = 0.290f;
static constexpr float LEVEL_DEFAULT_TANK_AREA_M2 = 5.4f;
static constexpr float LEVEL_DEFAULT_EMA_ALPHA = 0.25f;
static constexpr float LEVEL_DEFAULT_HYST_M = 0.002f;
static constexpr int32_t LEVEL_DEFAULT_SAMPLE_MS = 20;
static constexpr int32_t LEVEL_DEFAULT_ROUND_DECIMALS = 2;

static constexpr float LEVEL_MIN_HEIGHT_M = 0.0f;
static constexpr float LEVEL_MAX_HEIGHT_M = 5.0f;
static constexpr float LEVEL_MIN_TANK_AREA_M2 = 0.1f;
static constexpr float LEVEL_MAX_TANK_AREA_M2 = 50.0f;
static constexpr float LEVEL_MIN_EMA_ALPHA = 0.01f;
static constexpr float LEVEL_MAX_EMA_ALPHA = 1.0f;
static constexpr float LEVEL_MIN_HYST_M = 0.0f;
static constexpr float LEVEL_MAX_HYST_M = 0.05f;
static constexpr int32_t LEVEL_MIN_SAMPLE_MS = 10;
static constexpr int32_t LEVEL_MAX_SAMPLE_MS = 1000;
static constexpr int32_t LEVEL_MIN_ROUND_DECIMALS = 1;
static constexpr int32_t LEVEL_MAX_ROUND_DECIMALS = 3;
static constexpr int64_t LEVEL_CFG_DEBUG_PERIOD_US = 10LL * 1000LL * 1000LL;
static constexpr uint32_t LEVEL_RAW_SANITY_MIN = 0;
static constexpr uint32_t LEVEL_RAW_SANITY_MAX = 4095;
static constexpr uint32_t LEVEL_RAW_SANITY_MIN_MARGIN = 80;

static const config_item_t LEVEL_RAW_MIN_ITEM = {
    .key = "lvl_raw_min", .label = "Hladina RAW min", .description = "ADC RAW hodnota odpovidajici minimalni hladine.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = LEVEL_DEFAULT_RAW_MIN, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 4095, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t LEVEL_RAW_MAX_ITEM = {
    .key = "lvl_raw_max", .label = "Hladina RAW max", .description = "ADC RAW hodnota odpovidajici maximalni hladine.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = LEVEL_DEFAULT_RAW_MAX, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 1, .max_int = 4095, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t LEVEL_H_MIN_ITEM = {
    .key = "lvl_h_min", .label = "Hladina vyska min [m]", .description = "Vyska hladiny pro minimalni hodnotu senzoru.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = LEVEL_DEFAULT_HEIGHT_MIN_M, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = LEVEL_MIN_HEIGHT_M, .max_float = LEVEL_MAX_HEIGHT_M,
};
static const config_item_t LEVEL_H_MAX_ITEM = {
    .key = "lvl_h_max", .label = "Hladina vyska max [m]", .description = "Vyska hladiny pro maximalni hodnotu senzoru.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = LEVEL_DEFAULT_HEIGHT_MAX_M, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = LEVEL_MIN_HEIGHT_M, .max_float = LEVEL_MAX_HEIGHT_M,
};
static const config_item_t LEVEL_TANK_AREA_ITEM = {
    .key = "tank_area_m2", .label = "Plocha nadrze [m2]", .description = "Pudorysna plocha nadrze pouzita pro prepocet vysky na objem.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = LEVEL_DEFAULT_TANK_AREA_M2, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = LEVEL_MIN_TANK_AREA_M2, .max_float = LEVEL_MAX_TANK_AREA_M2,
};
static const config_item_t LEVEL_EMA_ALPHA_ITEM = {
    .key = "lvl_ema_alpha", .label = "Hladina EMA alpha", .description = "Koeficient EMA filtru na vysce hladiny (0-1).",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = LEVEL_DEFAULT_EMA_ALPHA, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = LEVEL_MIN_EMA_ALPHA, .max_float = LEVEL_MAX_EMA_ALPHA,
};
static const config_item_t LEVEL_HYST_M_ITEM = {
    .key = "lvl_hyst_m", .label = "Hladina hystereze [m]", .description = "Mrtve pasmo hystereze vysky hladiny v metrech.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = LEVEL_DEFAULT_HYST_M, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = LEVEL_MIN_HYST_M, .max_float = LEVEL_MAX_HYST_M,
};
static const config_item_t LEVEL_SAMPLE_MS_ITEM = {
    .key = "lvl_sample_ms", .label = "Hladina perioda mereni [ms]", .description = "Perioda cteni senzoru hladiny a publikace hodnot.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = LEVEL_DEFAULT_SAMPLE_MS, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = LEVEL_MIN_SAMPLE_MS, .max_int = LEVEL_MAX_SAMPLE_MS, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t LEVEL_ROUND_DECIMALS_ITEM = {
    .key = "lvl_round_dec", .label = "Hladina zaokrouhleni desetinna mista", .description = "Pocet desetinnych mist pro publikovany objem (1-3; 1=desetiny, 2=setiny, 3=tisiciny).",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = LEVEL_DEFAULT_ROUND_DECIMALS, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = LEVEL_MIN_ROUND_DECIMALS, .max_int = LEVEL_MAX_ROUND_DECIMALS, .min_float = 0.0f, .max_float = 0.0f,
};

void zasoba_register_config_items(void)
{
    APP_ERROR_CHECK("E680", config_store_register_item(&LEVEL_RAW_MIN_ITEM));
    APP_ERROR_CHECK("E681", config_store_register_item(&LEVEL_RAW_MAX_ITEM));
    APP_ERROR_CHECK("E682", config_store_register_item(&LEVEL_H_MIN_ITEM));
    APP_ERROR_CHECK("E683", config_store_register_item(&LEVEL_H_MAX_ITEM));
    APP_ERROR_CHECK("E684", config_store_register_item(&LEVEL_TANK_AREA_ITEM));
    APP_ERROR_CHECK("E685", config_store_register_item(&LEVEL_EMA_ALPHA_ITEM));
    APP_ERROR_CHECK("E686", config_store_register_item(&LEVEL_HYST_M_ITEM));
    APP_ERROR_CHECK("E687", config_store_register_item(&LEVEL_SAMPLE_MS_ITEM));
    APP_ERROR_CHECK("E688", config_store_register_item(&LEVEL_ROUND_DECIMALS_ITEM));
}

typedef struct {
    int32_t adc_raw_min;
    int32_t adc_raw_max;
    float height_min;
    float height_max;
    float tank_area_m2;
    float ema_alpha;
    float hyst_m;
    int32_t sample_ms;
    int32_t round_decimals;
} level_calibration_config_t;

static level_calibration_config_t g_level_config = {
    .adc_raw_min = LEVEL_DEFAULT_RAW_MIN,
    .adc_raw_max = LEVEL_DEFAULT_RAW_MAX,
    .height_min = LEVEL_DEFAULT_HEIGHT_MIN_M,
    .height_max = LEVEL_DEFAULT_HEIGHT_MAX_M,
    .tank_area_m2 = LEVEL_DEFAULT_TANK_AREA_M2,
    .ema_alpha = LEVEL_DEFAULT_EMA_ALPHA,
    .hyst_m = LEVEL_DEFAULT_HYST_M,
    .sample_ms = LEVEL_DEFAULT_SAMPLE_MS,
    .round_decimals = LEVEL_DEFAULT_ROUND_DECIMALS,
};

// Stav filtrace mereni hladiny (31 prvku, 5 orezanych z obou stran)
static TrimmedMean<31, 5> level_filter;
static float s_height_ema_value = 0.0f;
static bool s_height_ema_initialized = false;
static float s_height_hysteresis_value = 0.0f;
static bool s_height_hysteresis_initialized = false;
static int64_t s_last_cfg_debug_publish_us = 0;

static void publish_config_debug(void)
{
    DEBUG_PUBLISH("zasoba_cfg",
                  "rmn=%ld rmx=%ld hmn=%.3f hmx=%.3f a=%.3f e=%.3f hy=%.4f sm=%ld rd=%ld",
                  (long)g_level_config.adc_raw_min,
                  (long)g_level_config.adc_raw_max,
                  (double)g_level_config.height_min,
                  (double)g_level_config.height_max,
                  (double)g_level_config.tank_area_m2,
                  (double)g_level_config.ema_alpha,
                  (double)g_level_config.hyst_m,
                  (long)g_level_config.sample_ms,
                  (long)g_level_config.round_decimals);
}

static void publish_config_debug_periodic(int64_t now_us)
{
    if (s_last_cfg_debug_publish_us != 0
        && (now_us - s_last_cfg_debug_publish_us) < LEVEL_CFG_DEBUG_PERIOD_US) {
        return;
    }

    publish_config_debug();
    s_last_cfg_debug_publish_us = now_us;
}

static void load_level_calibration_config(void)
{
    g_level_config.adc_raw_min = config_store_get_i32_item(&LEVEL_RAW_MIN_ITEM);
    g_level_config.adc_raw_max = config_store_get_i32_item(&LEVEL_RAW_MAX_ITEM);
    g_level_config.height_min = config_store_get_float_item(&LEVEL_H_MIN_ITEM);
    g_level_config.height_max = config_store_get_float_item(&LEVEL_H_MAX_ITEM);
    g_level_config.tank_area_m2 = config_store_get_float_item(&LEVEL_TANK_AREA_ITEM);
    g_level_config.ema_alpha = config_store_get_float_item(&LEVEL_EMA_ALPHA_ITEM);
    g_level_config.hyst_m = config_store_get_float_item(&LEVEL_HYST_M_ITEM);
    g_level_config.sample_ms = config_store_get_i32_item(&LEVEL_SAMPLE_MS_ITEM);
    g_level_config.round_decimals = config_store_get_i32_item(&LEVEL_ROUND_DECIMALS_ITEM);

    if (g_level_config.tank_area_m2 <= 0.0f) {
        g_level_config.tank_area_m2 = LEVEL_DEFAULT_TANK_AREA_M2;
        ESP_LOGW(TAG, "Neplatna plocha nadrze, pouzivam default %.3f m2", (double)g_level_config.tank_area_m2);
    }

    if (g_level_config.ema_alpha <= 0.0f || g_level_config.ema_alpha > 1.0f) {
        g_level_config.ema_alpha = LEVEL_DEFAULT_EMA_ALPHA;
        ESP_LOGW(TAG, "Neplatna ema_alpha, pouzivam default %.3f", (double)g_level_config.ema_alpha);
    }

    if (g_level_config.hyst_m < 0.0f) {
        g_level_config.hyst_m = LEVEL_DEFAULT_HYST_M;
        ESP_LOGW(TAG, "Neplatna hyst_m, pouzivam default %.4f m", (double)g_level_config.hyst_m);
    }

    if (g_level_config.sample_ms < 1) {
        g_level_config.sample_ms = LEVEL_DEFAULT_SAMPLE_MS;
        ESP_LOGW(TAG, "Neplatna sample_ms, pouzivam default %ld ms", (long)g_level_config.sample_ms);
    }

    if (g_level_config.round_decimals < LEVEL_MIN_ROUND_DECIMALS || g_level_config.round_decimals > LEVEL_MAX_ROUND_DECIMALS) {
        g_level_config.round_decimals = LEVEL_DEFAULT_ROUND_DECIMALS;
        ESP_LOGW(TAG, "Neplatna round_decimals, pouzivam default %ld", (long)g_level_config.round_decimals);
    }

    ESP_LOGI(TAG,
             "Nactena kalibrace objemu: raw_min=%ld raw_max=%ld h_min=%.3f m h_max=%.3f m area=%.3f m2 ema=%.3f hyst=%.4f sm=%ld rd=%ld",
             (long)g_level_config.adc_raw_min,
             (long)g_level_config.adc_raw_max,
             g_level_config.height_min,
             g_level_config.height_max,
             g_level_config.tank_area_m2,
             g_level_config.ema_alpha,
             g_level_config.hyst_m,
             (long)g_level_config.sample_ms,
             (long)g_level_config.round_decimals);

}

static bool level_raw_is_plausible(uint32_t raw_value)
{
    return (raw_value <= LEVEL_RAW_SANITY_MAX - LEVEL_RAW_SANITY_MIN_MARGIN
         && raw_value >= LEVEL_RAW_SANITY_MIN + LEVEL_RAW_SANITY_MIN_MARGIN);
}

/**
 * Inicializuje ADC pro čtení senzoru hladiny
 */
static esp_err_t adc_init(void)
{
    ESP_LOGI(TAG,
             "ADC init: gpio=%d channel=%d",
             34,
             (int)LEVEL_SENSOR_ADC_CHANNEL);

    adc_channel_init(LEVEL_SENSOR_ADC_CHANNEL);

    return ESP_OK;
}

/**
 * Čte surovou hodnotu z ADC
 * @return RAW hodnota ADC
 */
static bool adc_read_raw(uint32_t *raw_value)
{
    if (raw_value == nullptr) {
        return false;
    }

    int raw = adc_read(LEVEL_SENSOR_ADC_CHANNEL);

    if (raw < LEVEL_RAW_SANITY_MIN || raw > LEVEL_RAW_SANITY_MAX) {
        ESP_LOGW(TAG, "ADC vratilo nesmyslnou RAW hodnotu: %d", raw);
        return false;
    }

    *raw_value = (uint32_t)raw;
    return true;
}

/**
 * Prožene RAW hodnotu trimmed mean filtrem a vrátí filtrovanou RAW hodnotu
 * @param raw_value RAW hodnota z ADC
 * @return filtrovaná RAW hodnota ADC po oříznutí extrémů
 */
static uint32_t adc_filter_trimmed_mean(uint32_t raw_value)
{
    level_filter.insert((int)raw_value);
    return level_filter.getValue();
}

/**
 * Převede RAW ADC hodnotu na výšku hladiny v metrech
 * @param raw_value RAW hodnota z ADC
 * @return výška hladiny v metrech
 */
static float adc_raw_to_height(uint32_t raw_value)
{
    // Linearni interpolace mezi kalibracnimi body RAW -> vyska.
    const int32_t raw_span = g_level_config.adc_raw_max - g_level_config.adc_raw_min;
    if (raw_span == 0) {
        return g_level_config.height_min;
    }

    float height = g_level_config.height_min + (float)((int)raw_value - g_level_config.adc_raw_min) *
                   (g_level_config.height_max - g_level_config.height_min) /
                   (float)raw_span;

    return height;
}

/**
 * Aplikuje EMA filtr na výšku hladiny
 * @param height_raw_m nefiltrovaná výška hladiny [m]
 * @return filtrovaná výška hladiny [m]
 */
static float height_filter_ema(float height_raw_m)
{
    if (!s_height_ema_initialized) {
        s_height_ema_value = height_raw_m;
        s_height_ema_initialized = true;
    } else {
        const float alpha = g_level_config.ema_alpha;
        s_height_ema_value = alpha * height_raw_m
                           + (1.0f - alpha) * s_height_ema_value;
    }

    return s_height_ema_value;
}

/**
 * Aplikuje hysterezi na výšku hladiny
 * @param height_m výška hladiny [m]
 * @return výška hladiny po aplikaci hystereze [m]
 */
static float height_apply_hysteresis(float height_m)
{
    if (!s_height_hysteresis_initialized) {
        s_height_hysteresis_value = height_m;
        s_height_hysteresis_initialized = true;
        return s_height_hysteresis_value;
    }

    const float delta = height_m - s_height_hysteresis_value;
    if (delta >= g_level_config.hyst_m || delta <= -g_level_config.hyst_m) {
        s_height_hysteresis_value = height_m;
    }

    return s_height_hysteresis_value;
}

static float height_to_volume_m3(float height_m)
{
    if (height_m < 0.0f) {
        height_m = 0.0f;
    }

    return height_m * g_level_config.tank_area_m2;
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

static void publish_invalid_level_measurement(int64_t timestamp_us, bool adc_ok, uint32_t raw_value)
{
    app_event_t invalid_event = {
        .event_type = EVT_SENSOR,
        .timestamp_us = timestamp_us,
        .data = {
            .sensor = {
                .sensor_type = SENSOR_EVENT_ZASOBA,
                .data = {
                    .zasoba = {
                        .objem = NAN,
                        .hladina = NAN,
                    },
                },
            },
        },
    };

    bool queued = sensor_events_publish(&invalid_event, pdMS_TO_TICKS(20));
    DEBUG_PUBLISH("zasoba_dyn",
                  "q=%d ts=%lld invalid=1 adc_ok=%d raw=%lu",
                  queued ? 1 : 0,
                  (long long)timestamp_us,
                  adc_ok ? 1 : 0,
                  (unsigned long)(adc_ok ? raw_value : 0));
}

// Prednabi filtry tak, aby prvni publikovane hodnoty nebyly zkreslene rozbehem.
static void warmup_filters(void)
{
    uint32_t raw_value = 0;
    uint32_t raw_trimmed_value = 0;
    float height_raw = 0.0f;
    float height_ema = 0.0f;

    const size_t buffer_size = level_filter.getBufferSize();
    ESP_LOGI(TAG, "Prebiha nabiti bufferu (%zu mereni)...", buffer_size);
    for (size_t index = 0; index < buffer_size; ++index) {
        if (!adc_read_raw(&raw_value)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        raw_trimmed_value = adc_filter_trimmed_mean(raw_value);
        height_raw = adc_raw_to_height(raw_trimmed_value);
        height_ema = height_filter_ema(height_raw);
        (void)height_apply_hysteresis(height_ema);
    }

    ESP_LOGI(TAG, "Buffer nabit, zacinam publikovat vysledky");
}

static void zasoba_task(void *pvParameters)
{
    (void)pvParameters;
    APP_ERROR_CHECK("E535", esp_task_wdt_add(nullptr));

    ESP_LOGI(TAG, "Spousteni cteni hladiny...");
    warmup_filters();

    uint32_t raw_value;
    uint32_t raw_trimmed_value;
    float hladina_raw;
    float hladina_ema;
    float hladina_hyst;
    float objem_m3_raw;
    float objem_m3_rounded;

    while (1) {
        int64_t timestamp_us = esp_timer_get_time();

        // 1) Nacteni surove ADC hodnoty
        const bool adc_ok = adc_read_raw(&raw_value);
        const bool raw_plausible = adc_ok && level_raw_is_plausible(raw_value);

        if (!raw_plausible) {
            publish_invalid_level_measurement(timestamp_us, adc_ok, raw_value);
        } else {
            // 2) Trimmed-mean na RAW
            raw_trimmed_value = adc_filter_trimmed_mean(raw_value);
            // 3) Prevod RAW -> vyska hladiny [m]
            hladina_raw = adc_raw_to_height(raw_trimmed_value);
            // 4) EMA filtr na vysce
            hladina_ema = height_filter_ema(hladina_raw);
            // 5) Hystereze na vysce
            hladina_hyst = height_apply_hysteresis(hladina_ema);
            // 6) Prevod vyska -> objem [m3]
            objem_m3_raw = height_to_volume_m3(hladina_hyst);
            // 7) Zaokrouhleni na konfigurovany pocet desetinnych mist pro publikaci
            objem_m3_rounded = round_to_decimals(objem_m3_raw, g_level_config.round_decimals);

            app_event_t event = {
                .event_type = EVT_SENSOR,
                .timestamp_us = timestamp_us,
                .data = {
                    .sensor = {
                        .sensor_type = SENSOR_EVENT_ZASOBA,
                        .data = {
                            .zasoba = {
                                .objem = objem_m3_rounded,
                                .hladina = hladina_hyst,
                            },
                        },
                    },
                },
            };

            bool queued = sensor_events_publish(&event, pdMS_TO_TICKS(20));

            DEBUG_PUBLISH("zasoba_dyn",
                          "q=%d ts=%lld r=%lu rt=%lu h=%.4f he=%.4f hh=%.4f v=%.4f v2=%.3f",
                          queued ? 1 : 0,
                          (long long)event.timestamp_us,
                          (unsigned long)raw_value,
                          (unsigned long)raw_trimmed_value,
                          (double)hladina_raw,
                          (double)hladina_ema,
                          (double)hladina_hyst,
                          (double)objem_m3_raw,
                          (double)objem_m3_rounded);
        }

        publish_config_debug_periodic(timestamp_us);

        APP_ERROR_CHECK("E537", esp_task_wdt_reset());
        vTaskDelay(pdMS_TO_TICKS(g_level_config.sample_ms));
    }
}

void zasoba_init(void)
{
    load_level_calibration_config();

    APP_ERROR_CHECK("E520", adc_init());

    APP_ERROR_CHECK("E522",
                    xTaskCreate(zasoba_task, TAG, configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_FAIL);
}
