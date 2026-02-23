#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs_flash.h"
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
#include "prutokomer.h"
#include "teplota-demo.h"
#include "hladina-demo.h"
#include "app-config.h"
#include "boot_button.h"
#include "sensor_events.h"
#include "state_manager.h"
#include "network_event_bridge.h"

#include "lcd.h"
#include "network_init.h"
#include "app_error_check.h"

#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_netif.h"

extern "C" {
    void cpp_app_main(void);
}

static void app_error_code_log_handler(const char *error_code)
{
    ESP_LOGE("main", "Error code: %s", error_code);
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

    ESP_LOGW("main", "Detekovan chybovy reset (reason=%d), spoustim chybovou LED sekvenci", static_cast<int>(reason));

    gpio_reset_pin(ERRORLED_PIN);
    gpio_set_direction(ERRORLED_PIN, GPIO_MODE_OUTPUT);

    const TickType_t fast_delay = pdMS_TO_TICKS(50);
    const TickType_t fast_total = pdMS_TO_TICKS(10000);
    TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < fast_total) {
        gpio_set_level(ERRORLED_PIN, 1);
        vTaskDelay(fast_delay);
        gpio_set_level(ERRORLED_PIN, 0);
        vTaskDelay(fast_delay);
    }

    gpio_set_level(ERRORLED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(ERRORLED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
}

static void log_config_webapp_url(void)
{
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif != NULL) {
        esp_netif_ip_info_t ip_info = {};
        if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            ESP_LOGI("main", "Konfiguracni aplikace bezi na: http://" IPSTR "/", IP2STR(&ip_info.ip));
            return;
        }
    }

    ESP_LOGI("main", "Konfiguracni aplikace bezi na: http://192.168.4.1/");
}

static bool s_ap_switch_done = false;

static void on_boot_button_pressed(void *ctx)
{
    (void)ctx;

    if (s_ap_switch_done) {
        return;
    }

    ESP_LOGW("main", "BOOT tlacitko stisknuto, prepinam do konfiguracniho AP rezimu");
    esp_err_t ap_result = network_init_ap("zalevaci-config", "");
    if (ap_result == ESP_OK) {
        s_ap_switch_done = true;
        ESP_LOGI("main", "Konfiguracni AP rezim aktivni");
        vTaskDelay(pdMS_TO_TICKS(300));
        log_config_webapp_url();
    } else {
        ESP_LOGE("main", "Prepnuti do AP rezimu selhalo: %s", esp_err_to_name(ap_result));
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
    app_error_check_set_handler(app_error_code_log_handler);
    indicate_error_reset_if_needed();
    print_partitions();
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES || nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        APP_ERROR_CHECK("E100", nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    APP_ERROR_CHECK("E101", nvs_result);

    APP_ERROR_CHECK("E102", app_config_ensure_defaults());
    APP_ERROR_CHECK("E103", app_config_load_runtime_flags());

    if (app_config_is_service_mode()) {
        ESP_LOGW("main", "System bezi v SERVISNIM rezimu");
    } else {
        ESP_LOGI("main", "System bezi v normalnim rezimu");
    }

    sensor_events_init(32);
    network_event_bridge_init();

    char wifi_ssid[32] = {0};
    char wifi_password[64] = {0};
    char mqtt_uri[128] = {0};
    char mqtt_topic[64] = {0};
    char mqtt_username[64] = {0};
    char mqtt_password[128] = {0};
    APP_ERROR_CHECK("E104", app_config_load_wifi_credentials(wifi_ssid, sizeof(wifi_ssid), wifi_password, sizeof(wifi_password)));
    APP_ERROR_CHECK("E105", app_config_load_mqtt_uri(mqtt_uri, sizeof(mqtt_uri)));
    APP_ERROR_CHECK("E106", app_config_load_mqtt_topic(mqtt_topic, sizeof(mqtt_topic)));
    APP_ERROR_CHECK("E107", app_config_load_mqtt_credentials(mqtt_username, sizeof(mqtt_username), mqtt_password, sizeof(mqtt_password)));

    const config_group_t config_groups[] = {
        app_config_get_config_group(),
        hladina_demo_get_config_group(),
    };
    APP_ERROR_CHECK("E108", config_webapp_prepare("app_cfg",
                                                    config_groups,
                                                    sizeof(config_groups) / sizeof(config_groups[0])));
    
    char status_topic[96] = {0};
    snprintf(status_topic, sizeof(status_topic), "%s/status", mqtt_topic);

    network_mqtt_lwt_config_t lwt_cfg = {
        .enabled = true,
        .status_topic = status_topic,
        .qos = 1,
        .retain = true,
    };

    ESP_LOGI("main",
             "MQTT cfg pred pripojenim: uri=%s, user=%s, password_set=%s, status_topic=%s",
             mqtt_uri,
             (mqtt_username[0] != '\0') ? mqtt_username : "(none)",
             (mqtt_password[0] != '\0') ? "yes" : "no",
             status_topic);
    APP_ERROR_CHECK("E109", network_init_with_mqtt_ex(wifi_ssid,
                                                        wifi_password,
                                                        mqtt_uri,
                                                        mqtt_username,
                                                        mqtt_password,
                                                        &lwt_cfg));

    APP_ERROR_CHECK("E110", boot_button_start(BOOT_BUTTON_GPIO, on_boot_button_pressed, nullptr));
    
    lcd_init(); // Inicializace LCD před spuštěním ostatních demo úloh, aby mohly ihned zobrazovat informace

    state_manager_start();
    
    // initialize sensor producer tasks
    prutokomer_init();

    // vytvoření paralelních tasků
    blikaniled_init();
    // lcd_demo_init();
    teplota_demo_init();
    hladina_demo_init();

}
