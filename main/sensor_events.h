#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <freertos/FreeRTOS.h>
#include "network_event.h"

typedef enum {
    EVT_SENSOR,
    EVT_NETWORK_STATE_CHANGE,
    EVT_NETWORK_TELEMETRY,
    EVT_TICK
} event_type_t;

typedef enum {
    SENSOR_EVENT_TEMPERATURE = 0,
    SENSOR_EVENT_LEVEL,
    SENSOR_EVENT_FLOW,
    SENSOR_EVENT_PRESSURE,
} sensor_event_type_t;

typedef enum {
    SENSOR_TEMPERATURE_PROBE_WATER = 0,
    SENSOR_TEMPERATURE_PROBE_AIR,
} sensor_temperature_probe_t;

typedef struct {
    float temperature_c;
    sensor_temperature_probe_t probe;
} sensor_temperature_data_t;

typedef struct {
    float objem;
    float hladina;
} sensor_level_data_t;

typedef struct {
    float prutok;
    float cerpano_celkem;
} sensor_flow_data_t;

typedef struct {
    float pred_filtrem;
    float za_filtrem;
    float rozdil_filtru;
    float zanesenost_filtru;
} sensor_pressure_data_t;

typedef struct {
    sensor_event_type_t sensor_type;
    union {
        sensor_temperature_data_t temperature;
        sensor_level_data_t level;
        sensor_flow_data_t flow;
        sensor_pressure_data_t pressure;
    } data;
} sensor_event_t;

typedef struct {
    system_network_level_t from_level;
    system_network_level_t to_level;
    network_event_t snapshot;
} network_state_change_event_t;

typedef struct {
    network_event_t snapshot;
} network_telemetry_event_t;

typedef struct {
    event_type_t event_type;
    int64_t timestamp_us;
    union {
        sensor_event_t sensor;
        network_state_change_event_t network_state_change;
        network_telemetry_event_t network_telemetry;
    } data;
} app_event_t;

void sensor_events_init(size_t queue_length);
bool sensor_events_publish(const app_event_t *event, TickType_t timeout);
bool sensor_events_receive(app_event_t *event, TickType_t timeout);
void sensor_event_to_string(const app_event_t *event, char *buffer, size_t buffer_len);

#ifdef __cplusplus
}
#endif
