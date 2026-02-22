#include "wifi_init.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <string.h>

#include "sensor_events.h"
#include "mqtt_init.h"
#include "esp_timer.h"

#define WIFI_MAX_RETRY 5

static const char *TAG = "wifi";

// Event bity
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool s_wifi_base_inited = false;
static bool s_sta_handlers_registered = false;
static int8_t s_last_rssi = INT8_MIN;
static esp_timer_handle_t s_network_publish_retry_timer = nullptr;

static void publish_network_event(bool wifi_up_hint);

static system_network_level_t get_network_level(bool wifi_up, bool ip_ready, bool mqtt_ready)
{
    if (mqtt_ready) {
        return SYS_NET_MQTT_READY;
    }
    if (ip_ready) {
        return SYS_NET_IP_ONLY;
    }
    if (wifi_up) {
        return SYS_NET_WIFI_ONLY;
    }
    return SYS_NET_DOWN;
}

static void delayed_network_publish_cb(void *arg)
{
    (void)arg;
    publish_network_event(true);
}

static void publish_network_event(bool wifi_up_hint)
{
    wifi_ap_record_t ap_info = {};
    bool ap_info_ok = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
    bool wifi_up = wifi_up_hint || ap_info_ok;

    int8_t last_rssi = INT8_MIN;
    if (ap_info_ok) {
        last_rssi = ap_info.rssi;
        s_last_rssi = ap_info.rssi;
    } else if (wifi_up && s_last_rssi != INT8_MIN) {
        last_rssi = s_last_rssi;
    }

    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip_info = {};
    bool ip_ready = (sta_netif != NULL)
                 && (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK)
                 && (ip_info.ip.addr != 0);
    uint32_t ip_addr = ip_ready ? ip_info.ip.addr : 0;

    bool mqtt_ready = mqtt_is_connected();

    app_event_t event = {
        .event_type = EVT_NETWORK,
        .timestamp_us = esp_timer_get_time(),
        .data = {
            .network = {
                .level = get_network_level(wifi_up, ip_ready, mqtt_ready),
                .last_rssi = last_rssi,
                .ip_addr = ip_addr,
            },
        },
    };

    if (!sensor_events_publish(&event, 0)) {
        ESP_LOGD(TAG, "Network event nebylo mozne publikovat");
    }
}

static esp_err_t wifi_platform_init(void)
{
    if (s_wifi_base_inited) {
        return ESP_OK;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if (s_network_publish_retry_timer == nullptr) {
        esp_timer_create_args_t timer_args = {
            .callback = &delayed_network_publish_cb,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "net_pub_retry",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_network_publish_retry_timer));
    }

    s_wifi_base_inited = true;
    return ESP_OK;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    auto schedule_retry_publish = []() {
        if (s_network_publish_retry_timer == nullptr) {
            return;
        }
        esp_timer_stop(s_network_publish_retry_timer);
        esp_timer_start_once(s_network_publish_retry_timer, 300 * 1000);
    };

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi spuštěno, probíhá připojení...");
        publish_network_event(false);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi pripojeno na AP, cekam na IP");
        publish_network_event(true);
        schedule_retry_publish();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Opakování připojení k AP, pokus %d/%d", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Selhalo připojení k WiFi");
        }
        publish_network_event(false);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Získána IP adresa:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        publish_network_event(true);
        schedule_retry_publish();
    }
}

esp_err_t wifi_init_sta(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL || strlen(ssid) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(wifi_platform_init());

    // Vytvořit event group pro synchronizaci
    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
    }
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    esp_netif_create_default_wifi_sta();

    if (!s_sta_handlers_registered) {
        esp_event_handler_instance_t instance_any_id;
        esp_event_handler_instance_t instance_got_ip;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            &instance_any_id));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            &instance_got_ip));
        s_sta_handlers_registered = true;
    }

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi inicializace dokončena. Připojuji se k SSID:%s", ssid);

    return ESP_OK;
}

esp_err_t wifi_init_ap(const char *ap_ssid, const char *ap_password)
{
    if (ap_ssid == NULL || strlen(ap_ssid) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(wifi_platform_init());
    esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.ap.ssid, ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(ap_ssid);
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;

    if (ap_password != NULL && strlen(ap_password) > 0) {
        strncpy((char *)wifi_config.ap.password, ap_password, sizeof(wifi_config.ap.password) - 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif != NULL) {
        esp_netif_ip_info_t ip_info = {};
        if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) {
            ESP_LOGI(TAG, "AP mode: SSID=%s, IP=" IPSTR, ap_ssid, IP2STR(&ip_info.ip));
        }
    }

    return ESP_OK;
}

bool wifi_wait_connected(uint32_t timeout_ms)
{
    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            ticks);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Připojeno k WiFi síti");
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Selhalo připojení k WiFi");
        return false;
    } else {
        ESP_LOGW(TAG, "Timeout při čekání na WiFi připojení");
        return false;
    }
}
