#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "mqtt_client.h"

#ifdef __cplusplus
}
#endif

#include "network_init.h"

#include "debug_mqtt.h"

static const char *TAG = "debug_mqtt";

bool g_debug_enabled = false;

void debug_mqtt_publish(const char *topic, const char *text)
{
    if (!g_debug_enabled || topic == nullptr || text == nullptr) {
        return;
    }

    if (!network_mqtt_is_connected()) {
        return;
    }

    esp_mqtt_client_handle_t client = network_mqtt_client();
    if (client == nullptr) {
        return;
    }

    int msg_id = esp_mqtt_client_publish(client, topic, text, 0, 0, 0);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "Debug publish selhal: topic=%s", topic);
    }
}
