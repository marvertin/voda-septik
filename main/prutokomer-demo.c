#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "tm1637.h"
#include "pins.h"
#include "lcd.h"

#define TAG "FLOW_DEMO"

// sdílený counter z ISR
static volatile uint32_t pulse_count = 0;

// Configure pins
static tm1637_config_t config = {
    .clk_pin = GPIO_NUM_18,
    .dio_pin = GPIO_NUM_19,
    .bit_delay_us = 100
};

static tm1637_handle_t display;

// ISR handler
static void IRAM_ATTR flow_isr_handler(void *arg) {
    pulse_count++;
}

static void pocitani_pulsu(void *pvParameters)
{
    char buf[8];
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(200));   // měřím 1× za sekundu

        // zobraz pulzy/seknu (zatím žádný přepočet na L/min)
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)pulse_count);
        lcd_print(0, 0, buf, true, 0); // Zobraz na první řádek, první sloupec

        // zobraz pulzy/seknu (zatím žádný přepočet na L/min)
        tm1637_show_number(display, pulse_count, false, 4, 0);
    }
}

void prutokomer_demo_init(void)
{
    tm1637_init(&config, &display);

    // --- Nastavení GPIO pro flow senzor ---
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << FLOW_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(FLOW_GPIO, flow_isr_handler, NULL);

    ESP_LOGI(TAG, "Startuji měření pulzů...");

    xTaskCreate(pocitani_pulsu, "pocitani_pulsu", 2048, NULL, 1, NULL);
}
