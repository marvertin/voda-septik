#include "network_event_bridge.h"

#include "network_init.h"
#include "sensor_events.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "network";

static void on_network_event(const network_event_t *network_event, void *ctx)
{
    (void)ctx;

    if (network_event == nullptr) {
        return;
    }

    app_event_t event = {
        .event_type = EVT_NETWORK,
        .timestamp_us = esp_timer_get_time(),
        .data = {
            .network = *network_event,
        },
    };

    if (!sensor_events_publish(&event, 0)) {
        ESP_LOGD(TAG, "Network event nebylo mozne publikovat");
    }
}

void network_event_bridge_init(void)
{
    network_register_event_callback(on_network_event, nullptr);
}
