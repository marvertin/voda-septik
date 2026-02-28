#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>

#ifdef __cplusplus
}
#endif

#include "trimmed_mean.hpp"
#include "adc_shared.h"
#include "pins.h"
#include "config_webapp.h"
#include "sensor_events.h"
#include "debug_mqtt.h"

#define TAG "OBJEM"

// ADC konfigurace pro senzor hladiny je centralizovana v pins.h

static const config_item_t LEVEL_CONFIG_ITEMS[] = {
    {
        .key = "lvl_raw_min",
        .label = "Hladina RAW min",
        .description = "ADC RAW hodnota odpovidajici minimalni hladine.",
        .type = CONFIG_VALUE_INT32,
        .default_string = nullptr,
        .default_int = 540,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 4095,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "lvl_raw_max",
        .label = "Hladina RAW max",
        .description = "ADC RAW hodnota odpovidajici maximalni hladine.",
        .type = CONFIG_VALUE_INT32,
        .default_string = nullptr,
        .default_int = 950,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 1,
        .max_int = 4095,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "lvl_h_min",
        .label = "Hladina vyska min [m]",
        .description = "Vyska hladiny pro minimalni hodnotu senzoru.",
        .type = CONFIG_VALUE_FLOAT,
        .default_string = nullptr,
        .default_int = 0,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.0f,
        .max_float = 5.0f,
    },
    {
        .key = "lvl_h_max",
        .label = "Hladina vyska max [m]",
        .description = "Vyska hladiny pro maximalni hodnotu senzoru.",
        .type = CONFIG_VALUE_FLOAT,
        .default_string = nullptr,
        .default_int = 0,
        .default_float = 0.290f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.0f,
        .max_float = 5.0f,
    },
    {
        .key = "obj_tank_area_m2",
        .label = "Plocha nadrze [m2]",
        .description = "Pudorysna plocha nadrze pouzita pro prepocet vysky na objem.",
        .type = CONFIG_VALUE_FLOAT,
        .default_string = nullptr,
        .default_int = 0,
        .default_float = 5.4f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.1f,
        .max_float = 50.0f,
    },
};

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

static void load_level_calibration_config(void)
{
    esp_err_t ret = config_webapp_get_i32("lvl_raw_min", &g_level_config.adc_raw_min);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Konfigurace lvl_raw_min neni dostupna (%s), pouzivam default", esp_err_to_name(ret));
    }

    ret = config_webapp_get_i32("lvl_raw_max", &g_level_config.adc_raw_max);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Konfigurace lvl_raw_max neni dostupna (%s), pouzivam default", esp_err_to_name(ret));
    }

    ret = config_webapp_get_float("lvl_h_min", &g_level_config.height_min);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Konfigurace lvl_h_min neni dostupna (%s), pouzivam default", esp_err_to_name(ret));
    }

    ret = config_webapp_get_float("lvl_h_max", &g_level_config.height_max);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Konfigurace lvl_h_max neni dostupna (%s), pouzivam default", esp_err_to_name(ret));
    }

    ret = config_webapp_get_float("obj_tank_area_m2", &g_level_config.tank_area_m2);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Konfigurace obj_tank_area_m2 neni dostupna (%s), pouzivam default", esp_err_to_name(ret));
    }

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
 * Čte průměrnou hodnotu z ADC
 * @return průměrná RAW hodnota ADC po oříznutí extrémů
 */
static uint32_t adc_read_average(void)
{
    int raw_value = 0;
    if (adc_shared_read(LEVEL_SENSOR_ADC_CHANNEL, &raw_value) != ESP_OK) {
        ESP_LOGE(TAG, "Chyba pri cteni ADC");
        return 0;
    }
    
    // Vložíme hodnotu do filtru
    level_filter.insert(raw_value);
    vTaskDelay(pdMS_TO_TICKS(10));  // Krátká pauza mezi vzorky
    
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

static float height_to_volume_liters(float height_m)
{
    if (height_m < 0.0f) {
        height_m = 0.0f;
    }

    const float volume_m3 = height_m * g_level_config.tank_area_m2;
    return volume_m3 * 1000.0f;
}

static void volume_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Spousteni cteni hladiny...");
    
    // Inicializace ADC
    if (adc_init() != ESP_OK) {
        ESP_LOGE(TAG, "Chyba při inicializaci ADC");
        vTaskDelete(NULL);
        return;
    }
    
    // Nabití bufferu na začátku - přečteme tolik měření, jaká je velikost bufferu
    // aby se zabránilo zkresleným údajům na začátku
    size_t buffer_size = level_filter.getBufferSize();
    ESP_LOGI(TAG, "Prebíhá nabití bufferu (%zu měření)...", buffer_size);
    for (size_t i = 0; i < buffer_size; i++) {
        adc_read_average();  // Jen vkládáme bez publikování
    }
    ESP_LOGI(TAG, "Buffer nabití, začínáme publikovat výsledky");
    
    uint32_t raw_value;
    float hladina;
    float objem;
    
    while (1)
    {
        // Čtení průměru z ADC
        raw_value = adc_read_average();
        
        // Převod na výšku
        hladina = adc_raw_to_height(raw_value);
        objem = height_to_volume_liters(hladina);
        
        // Výstup do logu
        //ESP_LOGI(TAG, "Surová hodnota: %lu | Výška hladiny: %.3f m", raw_value, height);
        
        app_event_t event = {
            .event_type = EVT_SENSOR,
            .timestamp_us = esp_timer_get_time(),
            .data = {
                .sensor = {
                    .sensor_type = SENSOR_EVENT_LEVEL,
                    .data = {
                        .level = {
                            .objem = objem,
                            .hladina = hladina,
                        },
                    },
                },
            },
        };

        bool queued = sensor_events_publish(&event, pdMS_TO_TICKS(20));
        if (!queued) {
            ESP_LOGW(TAG, "Fronta sensor eventu je plna, hladina zahozena");
        }

        DEBUG_PUBLISH("objem",
                      "queued=%d ts=%lld raw=%lu hladina_m=%.6f objem_l=%.3f area_m2=%.3f raw_min=%ld raw_max=%ld h_min=%.3f h_max=%.3f",
                      queued ? 1 : 0,
                      (long long)event.timestamp_us,
                      (unsigned long)raw_value,
                      (double)hladina,
                      (double)objem,
                      (double)g_level_config.tank_area_m2,
                      (long)g_level_config.adc_raw_min,
                      (long)g_level_config.adc_raw_max,
                      (double)g_level_config.height_min,
                      (double)g_level_config.height_max);
        
        // Čtení každou sekundu
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void objem_init(void)
{
    load_level_calibration_config();

    xTaskCreate(volume_task, TAG, configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}

config_group_t objem_get_config_group(void)
{
    config_group_t group = {
        .items = LEVEL_CONFIG_ITEMS,
        .item_count = sizeof(LEVEL_CONFIG_ITEMS) / sizeof(LEVEL_CONFIG_ITEMS[0]),
    };
    return group;
}
