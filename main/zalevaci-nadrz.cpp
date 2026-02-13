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

extern "C" {
    void cpp_app_main(void);
}

void cpp_app_main(void)
{
    // Inicializace WiFi
    ESP_ERROR_CHECK(wifi_init_sta());
    
    // Čekáme na připojení (timeout 10 sekund)
    wifi_wait_connected(10000);
    
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
