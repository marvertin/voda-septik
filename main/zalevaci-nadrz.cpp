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
#include "restart_info.h"
#include "sensor_events.h"
#include "state_manager.h"
#include "network_event_bridge.h"

#include "lcd.h"
#include "network_init.h"
#include "config_webapp.h"

#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_netif.h"

extern "C" {
    void cpp_app_main(void);
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

static void boot_button_ap_switch_task(void *pvParameters)
{
    (void)pvParameters;

    gpio_config_t boot_btn_cfg = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&boot_btn_cfg));

    bool ap_switch_done = false;
    bool last_pressed = false;

    while (true) {
        bool pressed = (gpio_get_level(BOOT_BUTTON_GPIO) == 0);

        if (pressed && !last_pressed) {
            vTaskDelay(pdMS_TO_TICKS(40));
            pressed = (gpio_get_level(BOOT_BUTTON_GPIO) == 0);
        }

        if (pressed && !last_pressed && !ap_switch_done) {
            ESP_LOGW("main", "BOOT tlacitko stisknuto, prepinam do konfiguracniho AP rezimu");
            esp_err_t ap_result = network_init_ap("zalevaci-config", "");
            if (ap_result == ESP_OK) {
                ap_switch_done = true;
                ESP_LOGI("main", "Konfiguracni AP rezim aktivni");
                vTaskDelay(pdMS_TO_TICKS(300));
                log_config_webapp_url();
            } else {
                ESP_LOGE("main", "Prepnuti do AP rezimu selhalo: %s", esp_err_to_name(ap_result));
            }
        }

        last_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(50));
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
    print_partitions();
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES || nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);

    ESP_ERROR_CHECK(app_config_ensure_defaults());
    ESP_ERROR_CHECK(app_config_load_runtime_flags());

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
    ESP_ERROR_CHECK(app_config_load_wifi_credentials(wifi_ssid, sizeof(wifi_ssid), wifi_password, sizeof(wifi_password)));
    ESP_ERROR_CHECK(app_config_load_mqtt_uri(mqtt_uri, sizeof(mqtt_uri)));
    ESP_ERROR_CHECK(app_config_load_mqtt_topic(mqtt_topic, sizeof(mqtt_topic)));
    ESP_ERROR_CHECK(app_config_load_mqtt_credentials(mqtt_username, sizeof(mqtt_username), mqtt_password, sizeof(mqtt_password)));

    ESP_ERROR_CHECK(network_init_sta(wifi_ssid, wifi_password));
    network_wait_connected(10000);

    const config_group_t config_groups[] = {
        app_config_get_config_group(),
        hladina_demo_get_config_group(),
    };

    app_restart_info_t restart_info = {};
    ESP_ERROR_CHECK(app_restart_info_update_and_load(&restart_info));

    config_webapp_restart_info_t webapp_restart_info = {
        .boot_count = restart_info.boot_count,
        .last_reason = static_cast<int32_t>(restart_info.last_reason),
        .last_restart_unix = restart_info.last_restart_unix,
    };

    config_webapp_network_info_t webapp_network_info = {
        .is_ap_mode = false,
        .active_ssid = wifi_ssid,
    };

    esp_err_t config_result = config_webapp_start(
        "app_cfg",
        config_groups,
        sizeof(config_groups) / sizeof(config_groups[0]),
        80,
        &webapp_restart_info,
        &webapp_network_info);
    if (config_result != ESP_OK) {
        ESP_LOGW("main", "Config web app se nepodarilo spustit: %s", esp_err_to_name(config_result));
    }
    
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
    ESP_ERROR_CHECK(network_mqtt_start_ex(mqtt_uri, mqtt_username, mqtt_password, &lwt_cfg));

    xTaskCreate(boot_button_ap_switch_task,
                "boot_btn_ap",
                configMINIMAL_STACK_SIZE * 3,
                NULL,
                5,
                NULL);
    
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
