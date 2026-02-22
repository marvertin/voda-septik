#include "mqtt_publish.h"

#include "network_init.h"

#include "esp_log.h"

#include <stdio.h>

static const char *TAG = "network";

esp_err_t mqtt_publish(const char *topic, const char *data, bool retain)
{
    esp_mqtt_client_handle_t mqtt_client = network_mqtt_client();
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT klient neni inicializovan");
        return ESP_ERR_INVALID_STATE;
    }

    if (!network_mqtt_is_connected()) {
        ESP_LOGW(TAG, "MQTT neni pripojeno, publikovani selhalo");
        return ESP_ERR_INVALID_STATE;
    }

    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, data, 0, 1, retain);
    if (msg_id == -1) {
        ESP_LOGE(TAG, "Publikovani selhalo: %s = %s", topic, data);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Publikovano: %s = %s (msg_id: %d)", topic, data, msg_id);
    return ESP_OK;
}

esp_err_t mqtt_publish_online_status(const char *base_topic)
{
    if (base_topic == NULL || base_topic[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    char status_topic[96] = {0};
    int written = snprintf(status_topic, sizeof(status_topic), "%s/status", base_topic);
    if (written <= 0 || written >= (int)sizeof(status_topic)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return mqtt_publish(status_topic, "online", true);
}

bool mqtt_is_connected(void)
{
    return network_mqtt_is_connected();
}
