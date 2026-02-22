#include "network_event.h"

system_network_level_t network_event_level(bool wifi_up, bool ip_ready, bool mqtt_ready)
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

network_event_t network_event_make(bool wifi_up,
                                   bool ip_ready,
                                   bool mqtt_ready,
                                   int8_t last_rssi,
                                   uint32_t ip_addr)
{
    network_event_t event = {
        .level = network_event_level(wifi_up, ip_ready, mqtt_ready),
        .last_rssi = last_rssi,
        .ip_addr = ip_addr,
    };
    return event;
}
