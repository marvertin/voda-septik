#include "sensor_events.h"

#include <stdio.h>

#include "esp_log.h"
#include <freertos/queue.h>

static const char *TAG = "SENSOR_EVENTS";
static QueueHandle_t s_sensor_events_queue = nullptr;

void sensor_events_init(size_t queue_length)
{
    if (s_sensor_events_queue != nullptr) {
        return;
    }

    s_sensor_events_queue = xQueueCreate(queue_length, sizeof(sensor_event_t));
    if (s_sensor_events_queue == nullptr) {
        ESP_LOGE(TAG, "Nelze vytvorit frontu sensor eventu");
        abort();
    }
}

bool sensor_events_publish(const sensor_event_t *event, TickType_t timeout)
{
    if (s_sensor_events_queue == nullptr || event == nullptr) {
        return false;
    }

    return xQueueSend(s_sensor_events_queue, event, timeout) == pdTRUE;
}

bool sensor_events_receive(sensor_event_t *event, TickType_t timeout)
{
    if (s_sensor_events_queue == nullptr || event == nullptr) {
        return false;
    }

    return xQueueReceive(s_sensor_events_queue, event, timeout) == pdTRUE;
}

void sensor_event_to_string(const sensor_event_t *event, char *buffer, size_t buffer_len)
{
    if (buffer == nullptr || buffer_len == 0) {
        return;
    }

    if (event == nullptr) {
        snprintf(buffer, buffer_len, "event=null");
        return;
    }

    switch (event->type) {
        case SENSOR_EVENT_TEMPERATURE:
            snprintf(buffer,
                     buffer_len,
                     "type=temperature ts=%lld temp=%.2fC",
                     (long long)event->timestamp_us,
                     event->data.temperature.temperature_c);
            break;

        case SENSOR_EVENT_LEVEL:
            snprintf(buffer,
                     buffer_len,
                     "type=level ts=%lld raw=%lu height=%.3fm",
                     (long long)event->timestamp_us,
                     (unsigned long)event->data.level.raw_value,
                     event->data.level.height_m);
            break;

        case SENSOR_EVENT_FLOW:
            snprintf(buffer,
                     buffer_len,
                     "type=flow ts=%lld flow=%.2f l/min total=%.2f l",
                     (long long)event->timestamp_us,
                     event->data.flow.flow_l_min,
                     event->data.flow.total_volume_l);
            break;

        default:
            snprintf(buffer,
                     buffer_len,
                     "type=unknown(%d) ts=%lld",
                     (int)event->type,
                     (long long)event->timestamp_us);
            break;
    }
}
