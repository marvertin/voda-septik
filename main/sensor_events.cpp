#include "sensor_events.h"

#include <stdio.h>

#include "esp_log.h"
#include <freertos/queue.h>

static const char *TAG = "sensor";
static QueueHandle_t s_sensor_events_queue = nullptr;

static const char *event_type_to_string(event_type_t event_type)
{
    switch (event_type) {
        case EVT_SENSOR:
            return "sensor";
        case EVT_NETWORK:
            return "network";
        case EVT_TICK:
            return "tick";
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
                             "event=%s type=temperature ts=%lld temp=%.2fC",
                             event_type_to_string(event->event_type),
                             (long long)event->timestamp_us,
                             event->data.sensor.data.temperature.temperature_c);
                    break;

                case SENSOR_EVENT_LEVEL:
                    snprintf(buffer,
                             buffer_len,
                             "event=%s type=level ts=%lld raw=%lu height=%.3fm",
                             event_type_to_string(event->event_type),
                             (long long)event->timestamp_us,
                             (unsigned long)event->data.sensor.data.level.raw_value,
                             event->data.sensor.data.level.height_m);
                    break;

                case SENSOR_EVENT_FLOW:
                    snprintf(buffer,
                             buffer_len,
                             "event=%s type=flow ts=%lld flow=%.2f l/min total=%.2f l",
                             event_type_to_string(event->event_type),
                             (long long)event->timestamp_us,
                             event->data.sensor.data.flow.flow_l_min,
                             event->data.sensor.data.flow.total_volume_l);
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

        case EVT_NETWORK:
            snprintf(buffer,
                     buffer_len,
                     "event=%s ts=%lld level=%d rssi=%d ip=0x%08lx reconn_attempts=%lu reconn_success=%lu",
                     event_type_to_string(event->event_type),
                     (long long)event->timestamp_us,
                     (int)event->data.network.level,
                     (int)event->data.network.last_rssi,
                     (unsigned long)event->data.network.ip_addr,
                     (unsigned long)event->data.network.reconnect_attempts,
                     (unsigned long)event->data.network.reconnect_successes);
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
