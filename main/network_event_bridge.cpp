#include "network_event_bridge.h"

#include "network_init.h"
#include "sensor_events.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "network";
static const int64_t NETWORK_TELEMETRY_PERIOD_US = 10LL * 1000LL * 1000LL;

static bool s_has_last_level = false;
static system_network_level_t s_last_level = SYS_NET_DOWN;
static int64_t s_last_telemetry_publish_us = 0;

static void publish_state_change_event(system_network_level_t from_level,
                                       system_network_level_t to_level,
                                       const network_event_t *network_event,
                                       int64_t timestamp_us)
{
    app_event_t event = {
        .event_type = EVT_NETWORK_STATE_CHANGE,
        .timestamp_us = timestamp_us,
        .data = {
            .network_state_change = {
                .from_level = from_level,
                .to_level = to_level,
                .snapshot = *network_event,
            },
        },
    };

    if (!sensor_events_publish(&event, 0)) {
        ESP_LOGD(TAG, "Network state-change event nebylo mozne publikovat");
    }
}

static void publish_telemetry_event(const network_event_t *network_event, int64_t timestamp_us)
{
    app_event_t event = {
        .event_type = EVT_NETWORK_TELEMETRY,
        .timestamp_us = timestamp_us,
        .data = {
            .network_telemetry = {
                .snapshot = *network_event,
            },
        },
    };

    if (!sensor_events_publish(&event, 0)) {
        ESP_LOGD(TAG, "Network telemetry event nebylo mozne publikovat");
    }
}

static void on_network_event(const network_event_t *network_event, void *ctx)
{
    (void)ctx;

    if (network_event == nullptr) {
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    const system_network_level_t current_level = network_event->level;

    bool state_changed = false;
    system_network_level_t from_level = SYS_NET_DOWN;
    if (!s_has_last_level) {
        s_has_last_level = true;
        s_last_level = current_level;
        if (current_level != SYS_NET_DOWN) {
            state_changed = true;
            from_level = SYS_NET_DOWN;
        }
    } else if (current_level != s_last_level) {
        state_changed = true;
        from_level = s_last_level;
        s_last_level = current_level;
    }

    if (state_changed) {
        publish_state_change_event(from_level, current_level, network_event, now_us);
    }

    const bool telemetry_due = (s_last_telemetry_publish_us == 0)
                           || ((now_us - s_last_telemetry_publish_us) >= NETWORK_TELEMETRY_PERIOD_US)
                           || state_changed;
    if (telemetry_due) {
        s_last_telemetry_publish_us = now_us;
        publish_telemetry_event(network_event, now_us);
    }
}

void network_event_bridge_init(void)
{
    network_register_event_callback(on_network_event, nullptr);
}
