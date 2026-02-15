#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <freertos/FreeRTOS.h>

typedef enum {
    SENSOR_EVENT_TEMPERATURE = 0,
    SENSOR_EVENT_LEVEL,
    SENSOR_EVENT_FLOW,
} sensor_event_type_t;

typedef struct {
    float temperature_c;
} sensor_temperature_data_t;

typedef struct {
    uint32_t raw_value;
    float height_m;
} sensor_level_data_t;

typedef struct {
    float flow_l_min;
    float total_volume_l;
} sensor_flow_data_t;

typedef struct {
    sensor_event_type_t type;
    int64_t timestamp_us;
    union {
        sensor_temperature_data_t temperature;
        sensor_level_data_t level;
        sensor_flow_data_t flow;
    } data;
} sensor_event_t;

void sensor_events_init(size_t queue_length);
bool sensor_events_publish(const sensor_event_t *event, TickType_t timeout);
bool sensor_events_receive(sensor_event_t *event, TickType_t timeout);
void sensor_event_to_string(const sensor_event_t *event, char *buffer, size_t buffer_len);

#ifdef __cplusplus
}
#endif
