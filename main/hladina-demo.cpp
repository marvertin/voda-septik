#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/gpio.h>

#ifdef __cplusplus
}
#endif

#include "trimmed_mean.hpp"
#include "config_webapp.h"
#include "sensor_events.h"
#include "debug_mqtt.h"

#define TAG "LEVEL_DEMO"

// ADC konfigurace pro senzor hladiny
// Tlakový pouzdový senzor do 2m s 150Ω šunt odporem
// Rozsah ADC: 0-3.3V (0-4095 na 12-bit ADC)

// ADC kanál pro senzor hladiny - GPIO3, ADC1_CHANNEL_2
// Upravte podle vaší skutečné konfigurace
static const adc_channel_t LEVEL_ADC_CHANNEL = ADC_CHANNEL_6;
static const adc_unit_t LEVEL_ADC_UNIT = ADC_UNIT_1;

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
};

typedef struct {
    int32_t adc_raw_min;
    int32_t adc_raw_max;
    float height_min;
    float height_max;
} level_calibration_config_t;

static level_calibration_config_t g_level_config = {
    .adc_raw_min = 540,
    .adc_raw_max = 950,
    .height_min = 0.0f,
    .height_max = 0.290f,
};

static adc_oneshot_unit_handle_t adc_handle = NULL;

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

    ESP_LOGI(TAG,
             "Nactena kalibrace hladiny: raw_min=%ld raw_max=%ld h_min=%.3f m h_max=%.3f m",
             (long)g_level_config.adc_raw_min,
             (long)g_level_config.adc_raw_max,
             g_level_config.height_min,
             g_level_config.height_max);
}

/**
 * Inicializuje ADC pro čtení senzoru hladiny
 */
static esp_err_t adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config;
    memset(&init_config, 0, sizeof(init_config));
    init_config.unit_id = LEVEL_ADC_UNIT;
    
    if (adc_oneshot_new_unit(&init_config, &adc_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Chyba: Nelze inicializovat ADC jednotku");
        return ESP_FAIL;
    }
    
    adc_oneshot_chan_cfg_t config;
    memset(&config, 0, sizeof(config));
    config.bitwidth = ADC_BITWIDTH_12;
    config.atten = ADC_ATTEN_DB_12;
    
    if (adc_oneshot_config_channel(adc_handle, LEVEL_ADC_CHANNEL, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Chyba: Nelze nakonfigurovat ADC kanál");
        return ESP_FAIL;
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
    if (adc_oneshot_read(adc_handle, LEVEL_ADC_CHANNEL, &raw_value) != ESP_OK) {
        ESP_LOGE(TAG, "Chyba při čtení ADC");
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

static void level_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Spouštění demá čtení hladiny...");
    
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
    float height;
    
    while (1)
    {
        // Čtení průměru z ADC
        raw_value = adc_read_average();
        
        // Převod na výšku
        height = adc_raw_to_height(raw_value);
        
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
                            .raw_value = raw_value,
                            .height_m = height,
                        },
                    },
                },
            },
        };

        bool queued = sensor_events_publish(&event, pdMS_TO_TICKS(20));
        if (!queued) {
            ESP_LOGW(TAG, "Fronta sensor eventu je plna, hladina zahozena");
        }

        DEBUG_PUBLISH("level",
                      "queued=%d ts=%lld raw=%lu height=%.6f raw_min=%ld raw_max=%ld h_min=%.3f h_max=%.3f",
                      queued ? 1 : 0,
                      (long long)event.timestamp_us,
                      (unsigned long)raw_value,
                      (double)height,
                      (long)g_level_config.adc_raw_min,
                      (long)g_level_config.adc_raw_max,
                      (double)g_level_config.height_min,
                      (double)g_level_config.height_max);
        
        // Čtení každou sekundu
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void hladina_demo_init(void)
{
    load_level_calibration_config();

    xTaskCreate(level_task, TAG, configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}

config_group_t hladina_demo_get_config_group(void)
{
    config_group_t group = {
        .items = LEVEL_CONFIG_ITEMS,
        .item_count = sizeof(LEVEL_CONFIG_ITEMS) / sizeof(LEVEL_CONFIG_ITEMS[0]),
    };
    return group;
}
