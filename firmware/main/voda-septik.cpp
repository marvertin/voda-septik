#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_rom_sys.h"
#include <stdio.h>
#include "i2cdev.h"
#include "pcf8574.h"

#include "pins.h"
#include "adc_shared.h"
#include "prutokomer.h"
#include "teplota.h"
#include "zasoba.h"
#include "tlak.h"
#include "network_config.h"
#include "system_config.h"
#include "config_store.h"
#include "config_webapp.h"
#include "boot_button.h"
#include "sensor_events.h"
#include "state_manager.h"
#include "network_event_bridge.h"
#include "mqtt_publisher_task.h"
#include "mqtt_commands.h"
#include "mqtt_topics.h"
#include "ads1115_logger.h"

#include "lcd.h"
#include "network_init.h"
#include "app_error_check.h"
#include "status_display.h"
#include "webapp_startup.h"

#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"

extern "C" {
    void cpp_app_main(void);
}

static const char *TAG = "voda_septik";
static constexpr uint32_t TASK_WDT_TIMEOUT_MS = 5000;

static esp_err_t task_wdt_init_or_reconfigure(const esp_task_wdt_config_t *cfg)
{
    if (cfg == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_err_t init_result = esp_task_wdt_init(cfg);
    if (init_result == ESP_ERR_INVALID_STATE) {
        return esp_task_wdt_reconfigure(cfg);
    }

    return init_result;
}

static bool is_error_reset_reason(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_PANIC:
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
        case ESP_RST_BROWNOUT:
        case ESP_RST_PWR_GLITCH:
        case ESP_RST_CPU_LOCKUP:
            return true;
        default:
            return false;
    }
}

static void indicate_error_reset_if_needed(void)
{
    esp_reset_reason_t reason = esp_reset_reason();
    if (!is_error_reset_reason(reason)) {
        return;
    }

    ESP_LOGW(TAG, "Detekovan chybovy reset (reason=%d), spoustim chybovou LED sekvenci", static_cast<int>(reason));

    gpio_reset_pin(STATUS_LED_GPIO);
    gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);

    const TickType_t fast_delay = pdMS_TO_TICKS(50);
    const TickType_t fast_total = pdMS_TO_TICKS(10000);
    TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < fast_total) {
        gpio_set_level(STATUS_LED_GPIO, 1);
        vTaskDelay(fast_delay);
        gpio_set_level(STATUS_LED_GPIO, 0);
        vTaskDelay(fast_delay);
    }

    gpio_set_level(STATUS_LED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(STATUS_LED_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
}

static void log_config_webapp_url(void)
{
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif != NULL) {
        esp_netif_ip_info_t ip_info = {};
        if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            ESP_LOGI(TAG, "Konfiguracni aplikace bezi na: http://" IPSTR "/", IP2STR(&ip_info.ip));
            return;
        }
    }

    ESP_LOGI(TAG, "Konfiguracni aplikace bezi na: http://192.168.4.1/");
}

static bool s_ap_switch_done = false;

static void on_boot_button_pressed(void *ctx)
{
    (void)ctx;

    if (s_ap_switch_done) {
        return;
    }

    ESP_LOGW(TAG, "BOOT tlacitko stisknuto, prepinam do konfiguracniho AP rezimu");
    esp_err_t ap_result = network_init_ap("voda-septik-config", "");
    if (ap_result == ESP_OK) {
        s_ap_switch_done = true;
        ESP_LOGI(TAG, "Konfiguracni AP rezim aktivni");
        esp_err_t webapp_result = webapp_startup_start();
        if (webapp_result != ESP_OK) {
            ESP_LOGW(TAG, "Automaticky start konfiguracni webapp po prepnuti do AP selhal: %s", esp_err_to_name(webapp_result));
        }
        vTaskDelay(pdMS_TO_TICKS(300));
        log_config_webapp_url();
    } else {
        ESP_LOGE(TAG, "Prepnuti do AP rezimu selhalo: %s", esp_err_to_name(ap_result));
    }
}

void print_partitions(void)
{
    esp_partition_iterator_t it =
        esp_partition_find(ESP_PARTITION_TYPE_ANY,
                           ESP_PARTITION_SUBTYPE_ANY,
                           NULL);

    while (it != NULL) {
        const esp_partition_t *part =
            esp_partition_get(it);

        printf("Label: %s, Type: %d, Subtype: %d, Addr: 0x%lx, Size: 0x%lx\n",
               part->label,
               part->type,
               part->subtype,
             static_cast<unsigned long>(part->address),
             static_cast<unsigned long>(part->size));

        it = esp_partition_next(it);
    }

    const esp_partition_t *running =  esp_ota_get_running_partition();

    printf("Running from: %s at 0x%lx\n",
       running->label,
         static_cast<unsigned long>(running->address));
}

void cpp_app_main(void)
{

    ads1115_logger_init();


}
