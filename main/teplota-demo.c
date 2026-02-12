#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <onewire.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include "pins.h"
#include "lcd.h"

#define TAG "TEMP_DEMO"

// GPIO pin pro 1Wire senzor (DS18B20)
// Upravte podle vaší konfigurace
static const gpio_num_t SENSOR_GPIO = GPIO_NUM_16;

// DS18B20 Commands
#define DS18B20_CMD_CONVERT_TEMP  0x44       // Start temperature conversion
#define DS18B20_CMD_READ_SCRATCH  0xBE       // Read scratchpad (9 bytes)
#define DS18B20_CMD_SKIP_ROM      0xCC       // Skip ROM (for single device)
#define DS18B20_CMD_SEARCH_ROM    0xF0       // Search ROM

// DS18B20 struktura - teplotní data
typedef struct {
    uint8_t temp_lsb;      // LSB teploty
    uint8_t temp_msb;      // MSB teploty
    uint8_t reserved[7];   // ostatní bajty
} ds18b20_scratchpad_t;

/**
 * Přečte teplotu z DS18B20 sensoru
 * @param gpio GPIO pin s 1-Wire senzorem
 * @param temp ukazatel na float kde se uloží výsledek
 * @return true pokud se podařilo, false pokud chyba
 */
static bool ds18b20_read_temperature(gpio_num_t gpio, float *temp)
{
    if (!temp) return false;
    
    // Reset bus
    if (!onewire_reset(gpio)) {
        ESP_LOGE(TAG, "Chyba: senzor neodpověděl na reset");
        return false;
    }
    
    // Skip ROM (pokud je jen jeden senzor na bus)
    if (!onewire_skip_rom(gpio)) {
        ESP_LOGE(TAG, "Chyba: Skip ROM selhal");
        return false;
    }
    
    // Příkaz pro konverzi teploty
    if (!onewire_write(gpio, DS18B20_CMD_CONVERT_TEMP)) {
        ESP_LOGE(TAG, "Chyba: Nebylo možno poslat Convert T příkaz");
        return false;
    }
    
    // Čekáme na konverzi (max 750ms pro 12-bit rozlišení)
    vTaskDelay(pdMS_TO_TICKS(800));
    
    // Reset bus znovu
    if (!onewire_reset(gpio)) {
        ESP_LOGE(TAG, "Chyba: senzor neodpověděl na druhý reset");
        return false;
    }
    
    // Skip ROM znovu
    if (!onewire_skip_rom(gpio)) {
        ESP_LOGE(TAG, "Chyba: Druhý Skip ROM selhal");
        return false;
    }
    
    // Příkaz pro čtení scratchpad registru
    if (!onewire_write(gpio, DS18B20_CMD_READ_SCRATCH)) {
        ESP_LOGE(TAG, "Chyba: Nebylo možno poslat Read Scratchpad příkaz");
        return false;
    }
    
    // Čteme 9 bajtů (stačily by nám 2, ale čteme všechny pro bezpečnost)
    ds18b20_scratchpad_t scratch;
    if (!onewire_read_bytes(gpio, (uint8_t *)&scratch, sizeof(scratch))) {
        ESP_LOGE(TAG, "Chyba: Nebylo možno přečíst scratchpad");
        return false;
    }
    
    // Převod 16-bit teploty na float
    // DS18B20 formát: MSB je integer část, LSB je frakční část
    // Frakční část je v horních 4 bitech LSB
    int16_t raw_temp = ((int16_t)scratch.temp_msb << 8) | scratch.temp_lsb;
    *temp = (float)(raw_temp >> 4) + ((float)(raw_temp & 0x0F)) / 16.0f;
    
    return true;
}

static void temperature_task(void *pvParameters)
{
    // Nastavení pull-up rezistoru na GPIO pinu
    gpio_set_pull_mode(SENSOR_GPIO, GPIO_PULLUP_ONLY);
    
    float temperature;
    
    while (1)
    {
        if (ds18b20_read_temperature(SENSOR_GPIO, &temperature)) {
            ESP_LOGI(TAG, "Teplota: %.2f°C", temperature);
            char buf[8];
            snprintf(buf, sizeof(buf), "T:%.1f", temperature);
            lcd_print(8, 0, buf, true, 0); // Zobraz na první řádek, sloupec 8
        } else {
            ESP_LOGE(TAG, "Nebylo možno přečíst teplotu");
        }
        
        // Čtení každou sekundu
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void teplota_demo_init(void)
{
    xTaskCreate(temperature_task, TAG, configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
}
