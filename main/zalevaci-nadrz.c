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


void app_main(void)
{
    // initialize flowmeter + display tasks
    prutokomer_demo_init();

    // vytvoření paralelních tasků
    blikaniled_init();
    lcd_demo_init();
    teplota_demo_init();
    hladina_demo_init();
}
