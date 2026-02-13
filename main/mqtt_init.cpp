#include "mqtt_init.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char *TAG = "mqtt";

#define MQTT_CONNECTED_BIT BIT0

static esp_mqtt_client_handle_t mqtt_client = NULL;
static EventGroupHandle_t mqtt_event_group = NULL;
static bool mqtt_connected = false;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT připojeno");
            mqtt_connected = true;
            xEventGroupSetBits(mqtt_event_group, MQTT_CONNECTED_BIT);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT odpojeno");
            mqtt_connected = false;
            xEventGroupClearBits(mqtt_event_group, MQTT_CONNECTED_BIT);
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

esp_err_t mqtt_init(const char *broker_uri)
{
    if (mqtt_client != NULL) {
        ESP_LOGW(TAG, "MQTT již inicializován");
        return ESP_OK;
    }

    mqtt_event_group = xEventGroupCreate();
    if (mqtt_event_group == NULL) {
        ESP_LOGE(TAG, "Nelze vytvořit event group");
        return ESP_ERR_NO_MEM;
    }

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = broker_uri;
    mqtt_cfg.network.disable_auto_reconnect = false;

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
