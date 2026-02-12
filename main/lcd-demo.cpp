#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>
#include <hd44780.h>
#include <pcf8574.h>

#ifdef __cplusplus
}
#endif

#include "pins.h"
#include "lcd.h"

static i2c_dev_t pcf8574;

#define CONFIG_EXAMPLE_I2C_ADDR 0x27

static uint32_t get_time_sec()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec;
}

void lcd_demo_task(void *pvParameters)
{
    ESP_ERROR_CHECK(i2cdev_init());
    lcd_init();

    char time[16];
    while (1)
    {
        snprintf(time, sizeof(time), "%lu", (unsigned long)get_time_sec());
        lcd_print(10, 1, time, true, portMAX_DELAY); // Zobraz čas na druhý řádek, sloupec 10
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void lcd_demo_init()
{
    xTaskCreate(lcd_demo_task, "lcd_demo_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}
