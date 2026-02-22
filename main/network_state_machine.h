#ifndef NETWORK_STATE_MACHINE_H
#define NETWORK_STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

#include "sensor_events.h"

#ifdef __cplusplus
extern "C" {
#endif

system_network_level_t network_state_machine_level(bool wifi_up, bool ip_ready, bool mqtt_ready);
void network_state_machine_publish(bool wifi_up,
                                   bool ip_ready,
                                   bool mqtt_ready,
                                   int8_t last_rssi,
                                   uint32_t ip_addr,
                                   int64_t timestamp_us);

#ifdef __cplusplus
}
#endif

#endif // NETWORK_STATE_MACHINE_H
