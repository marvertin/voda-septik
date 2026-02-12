#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/gpio.h>

#ifdef __cplusplus
}
#endif

#include "lcd.h"
#include "trimmed_mean.hpp"

#define TAG "LEVEL_DEMO"

// ADC konfigurace pro senzor hladiny
// Tlakový pouzdový senzor do 2m s 150Ω šunt odporem
// Rozsah ADC: 0-3.3V (0-4095 na 12-bit ADC)

// ADC kanál pro senzor hladiny - GPIO3, ADC1_CHANNEL_2
// Upravte podle vaší skutečné konfigurace
static const adc_channel_t LEVEL_ADC_CHANNEL = ADC_CHANNEL_6;
static const adc_unit_t LEVEL_ADC_UNIT = ADC_UNIT_1;

// Kalibrace senzoru
// Tlakový senzor: 0-2m vody = 0-0.2 bar
// Výstupní napětí: obvykle 0.5-4.5V (některé mají 0-3.3V)
// Kalibrace: Ajustujte tyto hodnoty podle vašeho konkrétního senzoru
#define ADC_RAW_MIN 520        // RAW hodnota pro 0m
#define ADC_RAW_MAX 4095     // RAW hodnota pro 2m
#define HEIGHT_MIN 0.0f      // Výška v metrech pro minimální napětí
#define HEIGHT_MAX 2.0f      // Výška v metrech pro maximální napětí

static adc_oneshot_unit_handle_t adc_handle = NULL;

// Vytvoříme instanci filtrů pro měření hladiny (31 prvků, 5 oříznutých z obou stran)
static TrimmedMean<31, 5> level_filter;

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
    float height = HEIGHT_MIN + (float)(raw_value - ADC_RAW_MIN) * 
                   (HEIGHT_MAX - HEIGHT_MIN) / (float)(ADC_RAW_MAX - ADC_RAW_MIN);
    
    // Omezení na rozsah
    if (height < HEIGHT_MIN) height = HEIGHT_MIN;
    if (height > HEIGHT_MAX) height = HEIGHT_MAX;
    
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
    
    uint32_t raw_value;
    float height;
    
    while (1)
    {
        // Čtení průměru z ADC
        raw_value = adc_read_average();
        
        // Převod na výšku
        height = adc_raw_to_height(raw_value);
        
        // Výstup do logu
        ESP_LOGI(TAG, "Surová hodnota: %lu | Výška hladiny: %.3f m", raw_value, height);
        
        // Zobrazení na LCD
        char buf[12];
        snprintf(buf, sizeof(buf), "H:%.2fm", height);
        lcd_print(0, 1, buf, true, 0); // Zobraz na druhý řádek, první sloupec
        
        // Čtení každou sekundu
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void hladina_demo_init(void)
{
   
    xTaskCreate(level_task, TAG, configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}
