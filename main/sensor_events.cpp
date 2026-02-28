#include "sensor_events.h"

#include <stdio.h>

#include "esp_log.h"
#include <freertos/queue.h>

static const char *TAG = "sensor_events";
static QueueHandle_t s_sensor_events_queue = nullptr;

static const char *event_type_to_string(event_type_t event_type)
{
    switch (event_type) {
        case EVT_SENSOR:
            return "sensor";
        case EVT_NETWORK_STATE_CHANGE:
            return "network_state_change";
        case EVT_NETWORK_TELEMETRY:
            return "network_telemetry";
        case EVT_TICK:
            return "tick";
        default:
            return "unknown";
    }
}

static const char *temperature_probe_to_string(sensor_temperature_probe_t probe)
{
    switch (probe) {
        case SENSOR_TEMPERATURE_PROBE_WATER:
            return "water";
        case SENSOR_TEMPERATURE_PROBE_AIR:
            return "air";
        default:
            return "unknown";
    }
}

void sensor_events_init(size_t queue_length)
{
    if (s_sensor_events_queue != nullptr) {
        return;
    }

    s_sensor_events_queue = xQueueCreate(queue_length, sizeof(app_event_t));
    if (s_sensor_events_queue == nullptr) {
        ESP_LOGE(TAG, "Nelze vytvorit frontu sensor eventu");
        abort();
    }
}

bool sensor_events_publish(const app_event_t *event, TickType_t timeout)
{
    if (s_sensor_events_queue == nullptr || event == nullptr) {
        return false;
    }

    return xQueueSend(s_sensor_events_queue, event, timeout) == pdTRUE;
}

bool sensor_events_receive(app_event_t *event, TickType_t timeout)
{
    if (s_sensor_events_queue == nullptr || event == nullptr) {
        return false;
    }

    return xQueueReceive(s_sensor_events_queue, event, timeout) == pdTRUE;
}

void sensor_event_to_string(const app_event_t *event, char *buffer, size_t buffer_len)
{
    if (buffer == nullptr || buffer_len == 0) {
        return;
    }

    if (event == nullptr) {
        snprintf(buffer, buffer_len, "event=null");
        return;
    }

    switch (event->event_type) {
        case EVT_SENSOR:
            switch (event->data.sensor.sensor_type) {
                case SENSOR_EVENT_TEMPERATURE:
                    snprintf(buffer,
                             buffer_len,
                             "event=%s type=temperature probe=%s ts=%lld temp=%.2fC",
                             event_type_to_string(event->event_type),
                             temperature_probe_to_string(event->data.sensor.data.temperature.probe),
                             (long long)event->timestamp_us,
                             event->data.sensor.data.temperature.temperature_c);
                    break;

                case SENSOR_EVENT_ZASOBA:
                    snprintf(buffer,
                             buffer_len,
                             "event=%s type=zasoba ts=%lld objem=%.3fl hladina=%.3fm",
                             event_type_to_string(event->event_type),
                             (long long)event->timestamp_us,
                             event->data.sensor.data.zasoba.objem,
                             event->data.sensor.data.zasoba.hladina);
                    break;

                case SENSOR_EVENT_FLOW:
                    snprintf(buffer,
                             buffer_len,
                             "event=%s type=flow ts=%lld flow=%.2f l/min total=%.2f l",
                             event_type_to_string(event->event_type),
                             (long long)event->timestamp_us,
                             event->data.sensor.data.flow.prutok,
                             event->data.sensor.data.flow.cerpano_celkem);
                    break;

                case SENSOR_EVENT_PRESSURE:
                    snprintf(buffer,
                             buffer_len,
                             "event=%s type=pressure ts=%lld p_before=%.3fbar p_after=%.3fbar dp=%.3fbar clog=%.1f%%",
                             event_type_to_string(event->event_type),
                             (long long)event->timestamp_us,
                             event->data.sensor.data.pressure.pred_filtrem,
                             event->data.sensor.data.pressure.za_filtrem,
                             event->data.sensor.data.pressure.rozdil_filtru,
                             event->data.sensor.data.pressure.zanesenost_filtru);
                    break;

                default:
                    snprintf(buffer,
                             buffer_len,
                             "event=%s type=sensor_unknown(%d) ts=%lld",
                             event_type_to_string(event->event_type),
                             (int)event->data.sensor.sensor_type,
                             (long long)event->timestamp_us);
                    break;
            }
            break;

        case EVT_NETWORK_STATE_CHANGE:
            snprintf(buffer,
                     buffer_len,
                     "event=%s ts=%lld from=%d to=%d rssi=%d ip=0x%08lx reconn_attempts=%lu reconn_success=%lu",
                     event_type_to_string(event->event_type),
                     (long long)event->timestamp_us,
                     (int)event->data.network_state_change.from_level,
                     (int)event->data.network_state_change.to_level,
                     (int)event->data.network_state_change.snapshot.last_rssi,
                     (unsigned long)event->data.network_state_change.snapshot.ip_addr,
                     (unsigned long)event->data.network_state_change.snapshot.reconnect_attempts,
                     (unsigned long)event->data.network_state_change.snapshot.reconnect_successes);
            break;

        case EVT_NETWORK_TELEMETRY:
            snprintf(buffer,
                     buffer_len,
                     "event=%s ts=%lld level=%d rssi=%d ip=0x%08lx reconn_attempts=%lu reconn_success=%lu",
                     event_type_to_string(event->event_type),
                     (long long)event->timestamp_us,
                     (int)event->data.network_telemetry.snapshot.level,
                     (int)event->data.network_telemetry.snapshot.last_rssi,
                     (unsigned long)event->data.network_telemetry.snapshot.ip_addr,
                     (unsigned long)event->data.network_telemetry.snapshot.reconnect_attempts,
                     (unsigned long)event->data.network_telemetry.snapshot.reconnect_successes);
            break;

        case EVT_TICK:
            snprintf(buffer,
                     buffer_len,
                     "event=%s ts=%lld",
                     event_type_to_string(event->event_type),
                     (long long)event->timestamp_us);
            break;

        default:
            snprintf(buffer,
                     buffer_len,
                     "event=%s ts=%lld",
                     event_type_to_string(event->event_type),
                     (long long)event->timestamp_us);
            break;
    }
}
