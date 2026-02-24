#include "mqtt_publisher_task.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "mqtt_publish.h"

static const char *TAG = "mqtt_pub_task";
static const TickType_t MQTT_PUBLISH_ENQUEUE_TIMEOUT_TICKS = pdMS_TO_TICKS(50);

static QueueHandle_t s_publish_queue = nullptr;
static TaskHandle_t s_publish_task = nullptr;

struct topic_last_state_t {
    bool valid;
    mqtt_publish_event_t event;
};

static topic_last_state_t s_last_state[(size_t)mqtt_topic_id_t::COUNT] = {};

static bool value_type_matches_topic(mqtt_payload_kind_t payload_kind, mqtt_publish_value_type_t value_type)
{
    static constexpr bool COMPAT[(size_t)mqtt_payload_kind_t::JSON + 1][(size_t)mqtt_publish_value_type_t::TEXT + 1] = {
        /* NUMBER  */ {false, true,  true,  false},
        /* BOOLEAN */ {true,  false, false, false},
        /* TEXT    */ {false, false, false, true },
        /* JSON    */ {false, false, false, true },
    };

    const size_t payload_index = (size_t)payload_kind;
    const size_t value_index = (size_t)value_type;
    if (payload_index > (size_t)mqtt_payload_kind_t::JSON || value_index > (size_t)mqtt_publish_value_type_t::TEXT) {
        return false;
    }

    return COMPAT[payload_index][value_index];
}

static bool value_equals(const mqtt_publish_event_t &current, const topic_last_state_t &last)
{
    if (!last.valid) {
        return false;
    }

    return memcmp(&current, &last.event, sizeof(current)) == 0;
}
static esp_err_t build_payload_string(const mqtt_publish_event_t &event, char *payload, size_t payload_len)
{
    if (payload == nullptr || payload_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (event.value_type) {
        case mqtt_publish_value_type_t::BOOL:
            snprintf(payload, payload_len, "%d", event.value.as_bool ? 1 : 0);
            return ESP_OK;
        case mqtt_publish_value_type_t::INT64:
            snprintf(payload, payload_len, "%lld", (long long)event.value.as_int64);
            return ESP_OK;
        case mqtt_publish_value_type_t::DOUBLE:
            snprintf(payload, payload_len, "%.6f", event.value.as_double);
            return ESP_OK;
        case mqtt_publish_value_type_t::TEXT:
            snprintf(payload, payload_len, "%s", event.value.as_text);
            return ESP_OK;
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static void save_last_state(const mqtt_publish_event_t &event)
{
    const size_t topic_index = (size_t)event.topic_id;
    if (topic_index >= (size_t)mqtt_topic_id_t::COUNT) {
        return;
    }

    topic_last_state_t &last = s_last_state[topic_index];
    last.valid = true;
    last.event = event;
}

static esp_err_t publish_if_changed(const mqtt_publish_event_t &event)
{
    const mqtt_topic_descriptor_t *topic = mqtt_topic_descriptor(event.topic_id);
    if (topic == nullptr) {
        ESP_LOGW(TAG, "Neznamy topic id: %u", (unsigned)event.topic_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (topic->direction != mqtt_topic_direction_t::PUBLISH_ONLY) {
        ESP_LOGW(TAG, "Topic %s neni urcen pro publish", topic->full_topic);
        return ESP_ERR_INVALID_ARG;
    }

    if (!value_type_matches_topic(topic->payload_kind, event.value_type)) {
        ESP_LOGW(TAG, "Nekompatibilni typ hodnoty pro topic %s", topic->full_topic);
        return ESP_ERR_INVALID_ARG;
    }

    const size_t topic_index = (size_t)event.topic_id;
    if (topic_index >= (size_t)mqtt_topic_id_t::COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    if (value_equals(event, s_last_state[topic_index])) {
        return ESP_OK;
    }

    char payload[MQTT_PUBLISH_TEXT_MAX_LEN] = {0};
    esp_err_t payload_result = build_payload_string(event, payload, sizeof(payload));
    if (payload_result != ESP_OK) {
        return payload_result;
    }

    esp_err_t publish_result = mqtt_publish(topic->full_topic, payload, topic->retain);
    if (publish_result != ESP_OK) {
        return publish_result;
    }

    save_last_state(event);
    return ESP_OK;
}

static void mqtt_publisher_task(void *param)
{
    (void)param;

    mqtt_publish_event_t event = {};
    while (true) {
        if (xQueueReceive(s_publish_queue, &event, portMAX_DELAY) != pdPASS) {
            continue;
        }

        esp_err_t result = publish_if_changed(event);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "Zpracovani publish eventu selhalo: %s", esp_err_to_name(result));
        }
    }
}

esp_err_t mqtt_publisher_task_start(uint32_t queue_length,
                                    UBaseType_t task_priority,
                                    uint32_t stack_size_words)
{
    if (queue_length == 0 || stack_size_words == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_publish_task != nullptr) {
        return ESP_OK;
    }

    memset(s_last_state, 0, sizeof(s_last_state));

    s_publish_queue = xQueueCreate((UBaseType_t)queue_length, sizeof(mqtt_publish_event_t));
    if (s_publish_queue == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t task_result = xTaskCreate(mqtt_publisher_task,
                                         TAG,
                                         (configSTACK_DEPTH_TYPE)stack_size_words,
                                         nullptr,
                                         task_priority,
                                         &s_publish_task);
    if (task_result != pdPASS) {
        vQueueDelete(s_publish_queue);
        s_publish_queue = nullptr;
        s_publish_task = nullptr;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t mqtt_publisher_enqueue(const mqtt_publish_event_t *event, TickType_t timeout_ticks)
{
    if (event == nullptr || s_publish_queue == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    if ((size_t)event->topic_id >= (size_t)mqtt_topic_id_t::COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    mqtt_publish_event_t copy = {};
    copy.topic_id = event->topic_id;
    copy.value_type = event->value_type;
    memcpy(&copy.value, &event->value, sizeof(copy.value));

    return (xQueueSend(s_publish_queue, &copy, timeout_ticks) == pdPASS) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t mqtt_publisher_enqueue_bool(mqtt_topic_id_t topic_id, bool value)
{
    mqtt_publish_event_t event = {};
    event.topic_id = topic_id;
    event.value_type = mqtt_publish_value_type_t::BOOL;
    event.value.as_bool = value;
    return mqtt_publisher_enqueue(&event, MQTT_PUBLISH_ENQUEUE_TIMEOUT_TICKS);
}

esp_err_t mqtt_publisher_enqueue_int64(mqtt_topic_id_t topic_id, int64_t value)
{
    mqtt_publish_event_t event = {};
    event.topic_id = topic_id;
    event.value_type = mqtt_publish_value_type_t::INT64;
    event.value.as_int64 = value;
    return mqtt_publisher_enqueue(&event, MQTT_PUBLISH_ENQUEUE_TIMEOUT_TICKS);
}

esp_err_t mqtt_publisher_enqueue_double(mqtt_topic_id_t topic_id, double value)
{
    mqtt_publish_event_t event = {};
    event.topic_id = topic_id;
    event.value_type = mqtt_publish_value_type_t::DOUBLE;
    event.value.as_double = value;
    return mqtt_publisher_enqueue(&event, MQTT_PUBLISH_ENQUEUE_TIMEOUT_TICKS);
}

esp_err_t mqtt_publisher_enqueue_text(mqtt_topic_id_t topic_id, const char *value)
{
    if (value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    mqtt_publish_event_t event = {};
    event.topic_id = topic_id;
    event.value_type = mqtt_publish_value_type_t::TEXT;
    strncpy(event.value.as_text, value, MQTT_PUBLISH_TEXT_MAX_LEN - 1);
    event.value.as_text[MQTT_PUBLISH_TEXT_MAX_LEN - 1] = '\0';
    return mqtt_publisher_enqueue(&event, MQTT_PUBLISH_ENQUEUE_TIMEOUT_TICKS);
}

bool mqtt_publisher_is_running(void)
{
    return s_publish_task != nullptr;
}
