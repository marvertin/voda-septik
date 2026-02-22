#include "mqtt_init.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "sensor_events.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include <cstring>

static const char *TAG = "mqtt";

#define MQTT_CONNECTED_BIT BIT0
#define MQTT_URI_MAX_LEN 128
#define MQTT_USER_MAX_LEN 64
#define MQTT_PASS_MAX_LEN 128

static esp_mqtt_client_handle_t mqtt_client = NULL;
static EventGroupHandle_t mqtt_event_group = NULL;
static bool mqtt_connected = false;
static char s_mqtt_uri[MQTT_URI_MAX_LEN] = {0};
static char s_mqtt_username[MQTT_USER_MAX_LEN] = {0};
static char s_mqtt_password[MQTT_PASS_MAX_LEN] = {0};

static bool is_valid_mqtt_uri(const char *broker_uri)
{
    if (broker_uri == NULL || broker_uri[0] == '\0') {
        return false;
    }

    const bool scheme_ok = (strncmp(broker_uri, "mqtt://", 7) == 0)
                        || (strncmp(broker_uri, "mqtts://", 8) == 0);
    if (!scheme_ok) {
        return false;
    }

    const char *host = strstr(broker_uri, "://");
    if (host == NULL) {
        return false;
    }
    host += 3;

    if (host[0] == '\0' || host[0] == ':' || host[0] == '/') {
        return false;
    }

    return true;
}

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

static void publish_network_event_from_mqtt(void)
{
    wifi_ap_record_t ap_info = {};
    bool wifi_up = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
    int8_t last_rssi = wifi_up ? ap_info.rssi : INT8_MIN;

    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip_info = {};
    bool ip_ready = (sta_netif != NULL)
                 && (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK)
                 && (ip_info.ip.addr != 0);
    uint32_t ip_addr = ip_ready ? ip_info.ip.addr : 0;

    app_event_t event = {
        .event_type = EVT_NETWORK,
        .timestamp_us = esp_timer_get_time(),
        .data = {
            .network = {
                .level = get_network_level(wifi_up, ip_ready, mqtt_connected),
                .last_rssi = last_rssi,
                .ip_addr = ip_addr,
            },
        },
    };

    if (!sensor_events_publish(&event, 0)) {
        ESP_LOGD(TAG, "Network event z MQTT nebylo mozne publikovat");
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT připojeno");
            mqtt_connected = true;
            xEventGroupSetBits(mqtt_event_group, MQTT_CONNECTED_BIT);
            publish_network_event_from_mqtt();
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT odpojeno");
            mqtt_connected = false;
            xEventGroupClearBits(mqtt_event_group, MQTT_CONNECTED_BIT);
            publish_network_event_from_mqtt();
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "Subscribe úspěšný, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "Unsubscribe úspěšný, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "Publikováno, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGD(TAG, "Data přijata %s=%.*s", event->topic, event->data_len, event->data);
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT chyba");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Poslední chyba stacku: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "Číslo chyby TLS: 0x%x", event->error_handle->esp_tls_stack_err);
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGE(TAG, "Broker odmítl připojení");
            }
            break;
            
        default:
            ESP_LOGD(TAG, "Ostatní event id: %d", event->event_id);
            break;
    }
}

esp_err_t mqtt_init(const char *broker_uri, const char *username, const char *password)
{
    if (mqtt_client != NULL) {
        ESP_LOGW(TAG, "MQTT již inicializován");
        return ESP_OK;
    }

    if (!is_valid_mqtt_uri(broker_uri)) {
        ESP_LOGE(TAG, "Neplatne MQTT URI: '%s'", broker_uri != NULL ? broker_uri : "(null)");
        ESP_LOGE(TAG, "Ocekavam format mqtt://host:port nebo mqtts://host:port");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG,
             "MQTT connect cfg: uri='%s', user='%s', password_set=%s",
             broker_uri,
             (username != NULL && username[0] != '\0') ? username : "(none)",
             (password != NULL && password[0] != '\0') ? "yes" : "no");

    strncpy(s_mqtt_uri, broker_uri, sizeof(s_mqtt_uri) - 1);
    s_mqtt_uri[sizeof(s_mqtt_uri) - 1] = '\0';

    s_mqtt_username[0] = '\0';
    if (username != NULL) {
        strncpy(s_mqtt_username, username, sizeof(s_mqtt_username) - 1);
        s_mqtt_username[sizeof(s_mqtt_username) - 1] = '\0';
    }

    s_mqtt_password[0] = '\0';
    if (password != NULL) {
        strncpy(s_mqtt_password, password, sizeof(s_mqtt_password) - 1);
        s_mqtt_password[sizeof(s_mqtt_password) - 1] = '\0';
    }

    mqtt_event_group = xEventGroupCreate();
    if (mqtt_event_group == NULL) {
        ESP_LOGE(TAG, "Nelze vytvořit event group");
        return ESP_ERR_NO_MEM;
    }

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = s_mqtt_uri;
    mqtt_cfg.network.disable_auto_reconnect = false;
    mqtt_cfg.credentials.username = (s_mqtt_username[0] != '\0') ? s_mqtt_username : NULL;
    mqtt_cfg.credentials.authentication.password = (s_mqtt_password[0] != '\0') ? s_mqtt_password : NULL;

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Nelze inicializovat MQTT klienta");
        return ESP_FAIL;
    }

    esp_err_t ret = esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nelze registrovat event handler");
        return ret;
    }

    ret = esp_mqtt_client_start(mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nelze spustit MQTT klienta");
        return ret;
    }

    ESP_LOGI(TAG, "MQTT klient inicializován: %s", broker_uri);
    return ESP_OK;
}

esp_err_t mqtt_publish(const char *topic, const char *data, bool retain)
{
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT klient není inicializován");
        return ESP_ERR_INVALID_STATE;
    }

    if (!mqtt_connected) {
        ESP_LOGW(TAG, "MQTT není připojeno, publikování selhalo");
        return ESP_ERR_INVALID_STATE;
    }

    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, data, 0, 1, retain);
    if (msg_id == -1) {
        ESP_LOGE(TAG, "Publikování selhalo: %s = %s", topic, data);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Publikováno: %s = %s (msg_id: %d)", topic, data, msg_id);
    return ESP_OK;
}

bool mqtt_wait_connected(uint32_t timeout_ms)
{
    if (mqtt_event_group == NULL) {
        ESP_LOGE(TAG, "Event group není inicializován");
        return false;
    }

    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    EventBits_t bits = xEventGroupWaitBits(mqtt_event_group, MQTT_CONNECTED_BIT, pdFALSE, pdFALSE, ticks);

    return (bits & MQTT_CONNECTED_BIT) != 0;
}

bool mqtt_is_connected(void)
{
    return mqtt_connected;
}
