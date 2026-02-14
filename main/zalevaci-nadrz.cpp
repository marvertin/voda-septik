#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_rom_sys.h"
#include <stdio.h>
#include "tm1637.h"
#include "i2cdev.h"
#include "pcf8574.h"

#include "pins.h"
#include "blikaniled.h"
#include "lcd-demo.h"
#include "prutokomer-demo.h"
#include "teplota-demo.h"
#include "hladina-demo.h"

#include "lcd.h"
#include "wifi_init.h"
#include "mqtt_init.h"
#include "config_webapp.h"

static const config_item_t APP_CONFIG_ITEMS[] = {
    {
        .key = "interval_s",
        .label = "Interval měření [s]",
        .description = "Perioda měření a publikace hodnot.",
        .type = CONFIG_VALUE_INT32,
        .default_string = nullptr,
        .default_int = 30,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 5,
        .max_int = 3600,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "tepl_max",
        .label = "Max. teplota [°C]",
        .description = "Prahová teplota pro alarm nebo ochranu.",
        .type = CONFIG_VALUE_FLOAT,
        .default_string = nullptr,
        .default_int = 0,
        .default_float = 35.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 0,
        .min_float = -20.0f,
        .max_float = 90.0f,
    },
    {
        .key = "auto_mode",
        .label = "Automatický režim",
        .description = "Zapíná automatické vyhodnocení závlahy.",
        .type = CONFIG_VALUE_BOOL,
        .default_string = nullptr,
        .default_int = 0,
        .default_float = 0.0f,
        .default_bool = true,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "mqtt_topic",
        .label = "MQTT topic",
        .description = "Kořenový topic pro publikaci dat zařízení.",
        .type = CONFIG_VALUE_STRING,
        .default_string = "zalevaci-nadrz",
        .default_int = 0,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 63,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
};

extern "C" {
    void cpp_app_main(void);
}

void cpp_app_main(void)
{
    // Inicializace WiFi
    ESP_ERROR_CHECK(wifi_init_sta());
    
    // Čekáme na připojení (timeout 10 sekund)
    wifi_wait_connected(10000);

    esp_err_t config_result = config_webapp_start(
        "app_cfg",
        APP_CONFIG_ITEMS,
        sizeof(APP_CONFIG_ITEMS) / sizeof(APP_CONFIG_ITEMS[0]),
        80);
    if (config_result != ESP_OK) {
        ESP_LOGW("main", "Config web app se nepodarilo spustit: %s", esp_err_to_name(config_result));
    }
    
    // Inicializace MQTT - upravte URI podle vaší Home Assistant instance
    ESP_ERROR_CHECK(mqtt_init("mqtt://192.168.2.108:1883"));
    
    lcd_init(); // Inicializace LCD před spuštěním ostatních demo úloh, aby mohly ihned zobrazovat informace
    
    // initialize flowmeter + display tasks
    prutokomer_demo_init();

    // vytvoření paralelních tasků
    blikaniled_init();
    // lcd_demo_init();
    teplota_demo_init();
    hladina_demo_init();
}
