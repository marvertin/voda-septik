#include "network_state_machine.h"

#include "esp_log.h"

static const char *TAG = "network";

system_network_level_t network_state_machine_level(bool wifi_up, bool ip_ready, bool mqtt_ready)
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

void network_state_machine_publish(bool wifi_up,
                                   bool ip_ready,
                                   bool mqtt_ready,
                                   int8_t last_rssi,
                                   uint32_t ip_addr,
                                   int64_t timestamp_us)
{
    app_event_t event = {
        .event_type = EVT_NETWORK,
        .timestamp_us = timestamp_us,
        .data = {
            .network = {
                .level = network_state_machine_level(wifi_up, ip_ready, mqtt_ready),
                .last_rssi = last_rssi,
                .ip_addr = ip_addr,
            },
        },
    };

    if (!sensor_events_publish(&event, 0)) {
        ESP_LOGD(TAG, "Network event nebylo mozne publikovat");
    }
}
