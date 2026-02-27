#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>

#ifdef __cplusplus
}
#endif

#include "lcd.h"

static uint32_t get_time_sec()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec;
}

void lcd_demo_task(void *pvParameters)
{

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
