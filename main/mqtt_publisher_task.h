#pragma once

#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "mqtt_topics.h"

static constexpr size_t MQTT_PUBLISH_TEXT_MAX_LEN = 128;

enum class mqtt_publish_value_type_t : uint8_t {
    BOOL = 0,
    INT64,
    DOUBLE,
    TEXT,
};

union mqtt_publish_value_t {
    bool as_bool;
    int64_t as_int64;
    double as_double;
    char as_text[MQTT_PUBLISH_TEXT_MAX_LEN];
};

struct mqtt_publish_event_t {
    mqtt_topic_id_t topic_id;
    mqtt_publish_value_type_t value_type;
    mqtt_publish_value_t value;
};

esp_err_t mqtt_publisher_task_start(uint32_t queue_length,
                                    UBaseType_t task_priority,
                                    uint32_t stack_size_words);
esp_err_t mqtt_publisher_enqueue(const mqtt_publish_event_t *event, TickType_t timeout_ticks);
esp_err_t mqtt_publisher_enqueue_bool(mqtt_topic_id_t topic_id, bool value);
esp_err_t mqtt_publisher_enqueue_int64(mqtt_topic_id_t topic_id, int64_t value);
esp_err_t mqtt_publisher_enqueue_double(mqtt_topic_id_t topic_id, double value);
esp_err_t mqtt_publisher_enqueue_text(mqtt_topic_id_t topic_id, const char *value);
bool mqtt_publisher_is_running(void);
