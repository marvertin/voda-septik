#include "network_event_bridge.h"

#include "network_init.h"
#include "network_state_machine.h"

static void on_network_state_changed(const network_state_t *state, void *ctx)
{
    (void)ctx;

    if (state == nullptr) {
        return;
    }

    network_state_machine_publish(state->wifi_up,
                                  state->ip_ready,
                                  state->mqtt_ready,
                                  state->last_rssi,
                                  state->ip_addr,
                                  state->timestamp_us);
}

void network_event_bridge_init(void)
{
    network_register_state_callback(on_network_state_changed, nullptr);
}
