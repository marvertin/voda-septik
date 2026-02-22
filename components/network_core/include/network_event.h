#ifndef NETWORK_EVENT_H
#define NETWORK_EVENT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYS_NET_DOWN,
    SYS_NET_WIFI_ONLY,
    SYS_NET_IP_ONLY,
    SYS_NET_MQTT_READY,
    SYS_NET_AP_CONFIG
} system_network_level_t;

typedef struct {
    system_network_level_t level;
    int8_t last_rssi;
    uint32_t ip_addr;
} network_event_t;

system_network_level_t network_event_level(bool ap_mode, bool wifi_up, bool ip_ready, bool mqtt_ready);
network_event_t network_event_make(bool ap_mode,
                                   bool wifi_up,
                                   bool ip_ready,
                                   bool mqtt_ready,
                                   int8_t last_rssi,
                                   uint32_t ip_addr);

#ifdef __cplusplus
}
#endif

#endif // NETWORK_EVENT_H
