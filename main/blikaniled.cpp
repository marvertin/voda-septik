#include "blikaniled.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#ifdef __cplusplus
}
#endif

static void led_task_2(void *pvParameters)
{
    while (1) {
        gpio_set_level(LED2_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1370));
        gpio_set_level(LED2_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1370));
    }
}

// vytvoření dvou paralelních tasků
  
void blikaniled_init(void)
{
    gpio_reset_pin(LED2_PIN);
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);

    xTaskCreate(led_task_2, "led_task_2", 2048, NULL, 5, NULL);
}
