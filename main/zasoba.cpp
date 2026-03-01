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

// ADC konfigurace pro senzor hladiny je centralizovana v pins.h

static const config_item_t LEVEL_RAW_MIN_ITEM = {
    .key = "lvl_raw_min", .label = "Hladina RAW min", .description = "ADC RAW hodnota odpovidajici minimalni hladine.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = 540, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 4095, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t LEVEL_RAW_MAX_ITEM = {
    .key = "lvl_raw_max", .label = "Hladina RAW max", .description = "ADC RAW hodnota odpovidajici maximalni hladine.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = 950, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 1, .max_int = 4095, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t LEVEL_H_MIN_ITEM = {
    .key = "lvl_h_min", .label = "Hladina vyska min [m]", .description = "Vyska hladiny pro minimalni hodnotu senzoru.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 0.0f, .max_float = 5.0f,
};
static const config_item_t LEVEL_H_MAX_ITEM = {
    .key = "lvl_h_max", .label = "Hladina vyska max [m]", .description = "Vyska hladiny pro maximalni hodnotu senzoru.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = 0.290f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 0.0f, .max_float = 5.0f,
};
static const config_item_t LEVEL_TANK_AREA_ITEM = {
    .key = "tank_area_m2", .label = "Plocha nadrze [m2]", .description = "Pudorysna plocha nadrze pouzita pro prepocet vysky na objem.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = 5.4f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 0.1f, .max_float = 50.0f,
};

void zasoba_register_config_items(void)
{
    APP_ERROR_CHECK("E680", config_store_register_item(&LEVEL_RAW_MIN_ITEM));
    APP_ERROR_CHECK("E681", config_store_register_item(&LEVEL_RAW_MAX_ITEM));
    APP_ERROR_CHECK("E682", config_store_register_item(&LEVEL_H_MIN_ITEM));
    APP_ERROR_CHECK("E683", config_store_register_item(&LEVEL_H_MAX_ITEM));
    APP_ERROR_CHECK("E684", config_store_register_item(&LEVEL_TANK_AREA_ITEM));
}

typedef struct {
    int32_t adc_raw_min;
    int32_t adc_raw_max;
    float height_min;
    float height_max;
    float tank_area_m2;
} level_calibration_config_t;

static level_calibration_config_t g_level_config = {
    .adc_raw_min = 540,
    .adc_raw_max = 950,
    .height_min = 0.0f,
    .height_max = 0.290f,
    .tank_area_m2 = 5.4f,
};

// Vytvoříme instanci filtrů pro měření hladiny (31 prvků, 5 oříznutých z obou stran)
static TrimmedMean<31, 5> level_filter;
static constexpr float LEVEL_HEIGHT_EMA_ALPHA = 0.25f;
static constexpr float LEVEL_HEIGHT_HYSTERESIS_M = 0.002f;
static float s_height_ema_value = 0.0f;
static bool s_height_ema_initialized = false;
static float s_height_hysteresis_value = 0.0f;
static bool s_height_hysteresis_initialized = false;

static void load_level_calibration_config(void)
{
    g_level_config.adc_raw_min = config_store_get_i32_item(&LEVEL_RAW_MIN_ITEM);
    g_level_config.adc_raw_max = config_store_get_i32_item(&LEVEL_RAW_MAX_ITEM);
    g_level_config.height_min = config_store_get_float_item(&LEVEL_H_MIN_ITEM);
    g_level_config.height_max = config_store_get_float_item(&LEVEL_H_MAX_ITEM);
    g_level_config.tank_area_m2 = config_store_get_float_item(&LEVEL_TANK_AREA_ITEM);

    if (g_level_config.tank_area_m2 <= 0.0f) {
        g_level_config.tank_area_m2 = 5.4f;
        ESP_LOGW(TAG, "Neplatna plocha nadrze, pouzivam default %.3f m2", (double)g_level_config.tank_area_m2);
    }

    ESP_LOGI(TAG,
             "Nactena kalibrace objemu: raw_min=%ld raw_max=%ld h_min=%.3f m h_max=%.3f m area=%.3f m2",
             (long)g_level_config.adc_raw_min,
             (long)g_level_config.adc_raw_max,
             g_level_config.height_min,
             g_level_config.height_max,
             g_level_config.tank_area_m2);
}

/**
 * Inicializuje ADC pro čtení senzoru hladiny
 */
static esp_err_t adc_init(void)
{
    ESP_LOGI(TAG,
             "ADC init: gpio=%d unit=%d channel=%d bitwidth=%d atten=%d",
             34,
             (int)LEVEL_SENSOR_ADC_UNIT,
             (int)LEVEL_SENSOR_ADC_CHANNEL,
             (int)LEVEL_SENSOR_ADC_BITWIDTH,
             (int)LEVEL_SENSOR_ADC_ATTENUATION);

    esp_err_t ret = adc_shared_init(LEVEL_SENSOR_ADC_UNIT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Chyba: Nelze inicializovat ADC jednotku (%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = adc_shared_config_channel(LEVEL_SENSOR_ADC_CHANNEL,
                                    LEVEL_SENSOR_ADC_BITWIDTH,
                                    LEVEL_SENSOR_ADC_ATTENUATION);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Chyba: Nelze nakonfigurovat ADC kanál (%s)", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * Čte surovou hodnotu z ADC
 * @return RAW hodnota ADC
 */
static uint32_t adc_read_raw(void)
{
    int raw_value = 0;
    APP_ERROR_CHECK("E536", adc_shared_read(LEVEL_SENSOR_ADC_CHANNEL, &raw_value));
    return (uint32_t)raw_value;
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
    // Lineární interpolace
    float height = g_level_config.height_min + (float)((int)raw_value - g_level_config.adc_raw_min) *
                   (g_level_config.height_max - g_level_config.height_min) /
                   (float)(g_level_config.adc_raw_max - g_level_config.adc_raw_min);
    
    // Omezení na rozsah
    //if (height < HEIGHT_MIN) height = HEIGHT_MIN;
    //if (height > HEIGHT_MAX) height = HEIGHT_MAX;
    
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
        s_height_ema_value = LEVEL_HEIGHT_EMA_ALPHA * height_raw_m
                           + (1.0f - LEVEL_HEIGHT_EMA_ALPHA) * s_height_ema_value;
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
    if (delta >= LEVEL_HEIGHT_HYSTERESIS_M || delta <= -LEVEL_HEIGHT_HYSTERESIS_M) {
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

static float round_to_2_decimals(float value)
{
    return std::roundf(value * 100.0f) / 100.0f;
}

static void zasoba_task(void *pvParameters)
{
    (void)pvParameters;
    APP_ERROR_CHECK("E535", esp_task_wdt_add(nullptr));

    ESP_LOGI(TAG, "Spousteni cteni hladiny...");
    
    // Inicializace ADC
    APP_ERROR_CHECK("E520", adc_init());
    
    uint32_t raw_value;
    uint32_t raw_trimmed_value;
    float hladina_raw;
    float hladina_ema;
    float hladina_hyst;
    float objem_m3_raw;
    float objem_m3_rounded;

    // Nabití bufferu na začátku - přečteme tolik měření, jaká je velikost bufferu
    // aby se zabránilo zkresleným údajům na začátku
    size_t buffer_size = level_filter.getBufferSize();
    ESP_LOGI(TAG, "Prebiha nabiti bufferu (%zu mereni)...", buffer_size);
    for (size_t i = 0; i < buffer_size; i++) {
        raw_value = adc_read_raw();
        raw_trimmed_value = adc_filter_trimmed_mean(raw_value);
        hladina_raw = adc_raw_to_height(raw_trimmed_value);
        hladina_ema = height_filter_ema(hladina_raw);
        height_apply_hysteresis(hladina_ema);  // Jen plníme filtry bez publikování
    }
    ESP_LOGI(TAG, "Buffer nabit, zacinam publikovat vysledky");
    
    while (1)
    {
        // 1) Získám surovou ADC hodnotu
        raw_value = adc_read_raw();
        // 2) Proženu ji trimmed mean filtrem (stále RAW)
        raw_trimmed_value = adc_filter_trimmed_mean(raw_value);
        // 3) Převedu RAW na výšku hladiny
        hladina_raw = adc_raw_to_height(raw_trimmed_value);
        // 4) Aplikuju EMA filtr na výšku hladiny
        hladina_ema = height_filter_ema(hladina_raw);
        // 5) Aplikuju hysterezi na výšku hladiny
        hladina_hyst = height_apply_hysteresis(hladina_ema);
        // 6) Přepočet na objem [m3]
        objem_m3_raw = height_to_volume_m3(hladina_hyst);
        // 7) Zaokrouhlení objemu na dvě desetinná místa
        objem_m3_rounded = round_to_2_decimals(objem_m3_raw);
        
        app_event_t event = {
            .event_type = EVT_SENSOR,
            .timestamp_us = esp_timer_get_time(),
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
        if (!queued) {
            ESP_LOGW(TAG, "Fronta sensor eventu je plna, hladina zahozena");
        }

        DEBUG_PUBLISH("zasoba",
                      "q=%d ts=%lld r=%lu rt=%lu h=%.4f he=%.4f hh=%.4f v=%.4f v2=%.2f",
                      queued ? 1 : 0,
                      (long long)event.timestamp_us,
                      (unsigned long)raw_value,
                      (unsigned long)raw_trimmed_value,
                      (double)hladina_raw,
                      (double)hladina_ema,
                      (double)hladina_hyst,
                      (double)objem_m3_raw,
                      (double)objem_m3_rounded);

        APP_ERROR_CHECK("E537", esp_task_wdt_reset());
        
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void zasoba_init(void)
{
    load_level_calibration_config();

    APP_ERROR_CHECK("E522",
                    xTaskCreate(zasoba_task, TAG, configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_FAIL);
}
