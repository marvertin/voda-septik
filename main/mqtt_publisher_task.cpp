#include "mqtt_publisher_task.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "mqtt_publish.h"
#include "status_display.h"

static const char *TAG = "mqtt_pub_task";
static const TickType_t MQTT_PUBLISH_ENQUEUE_TIMEOUT_TICKS = pdMS_TO_TICKS(50);
static const TickType_t MQTT_PUBLISH_REFRESH_INTERVAL_TICKS = pdMS_TO_TICKS(60 * 1000);

static QueueHandle_t s_publish_queue = nullptr;
static TaskHandle_t s_publish_task = nullptr;
static volatile bool s_mqtt_connected = false;

enum class queue_item_type_t : uint8_t {
    PUBLISH_EVENT = 0,
    FLUSH_CACHED,
};

struct mqtt_publish_queue_item_t {
    queue_item_type_t type;
    mqtt_publish_event_t event;
};

struct topic_last_state_t {
    bool valid;
    bool published_once;
    TickType_t last_publish_tick;
    mqtt_publish_event_t event;
};

static topic_last_state_t s_last_state[(size_t)mqtt_topic_id_t::COUNT] = {};

static bool value_type_matches_topic(mqtt_payload_kind_t payload_kind, mqtt_publish_value_type_t value_type);
static esp_err_t build_payload_string(const mqtt_publish_event_t &event, char *payload, size_t payload_len);
static bool refresh_due(const topic_last_state_t &last, TickType_t now_ticks);
static void mark_published(topic_last_state_t &last, TickType_t now_ticks);

static esp_err_t publish_event_now(const mqtt_publish_event_t &event)
{
    const mqtt_topic_descriptor_t *topic = mqtt_topic_descriptor(event.topic_id);
    if (topic == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (topic->direction != mqtt_topic_direction_t::PUBLISH_ONLY) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!value_type_matches_topic(topic->payload_kind, event.value_type)) {
        return ESP_ERR_INVALID_ARG;
    }

    char payload[MQTT_PUBLISH_TEXT_MAX_LEN] = {0};
    esp_err_t payload_result = build_payload_string(event, payload, sizeof(payload));
    if (payload_result != ESP_OK) {
        return payload_result;
    }

    return mqtt_publish(topic->full_topic, payload, topic->retain);
}

static void flush_cached_values(void)
{
    if (!s_mqtt_connected) {
        return;
    }

    for (size_t index = 0; index < (size_t)mqtt_topic_id_t::COUNT; ++index) {
        topic_last_state_t &last = s_last_state[index];
        if (!last.valid) {
            continue;
        }

        esp_err_t publish_result = publish_event_now(last.event);
        if (publish_result != ESP_OK) {
            ESP_LOGW(TAG,
                     "Flush cached topicu %u selhal: %s",
                     (unsigned)last.event.topic_id,
                     esp_err_to_name(publish_result));
            continue;
        }

        mark_published(last, xTaskGetTickCount());
    }
}

static bool value_type_matches_topic(mqtt_payload_kind_t payload_kind, mqtt_publish_value_type_t value_type)
{
    static constexpr bool COMPAT[(size_t)mqtt_payload_kind_t::JSON + 1][(size_t)mqtt_publish_value_type_t::EMPTY + 1] = {
        /* NUMBER  */ {false, true,  true,  false, true },
        /* BOOLEAN */ {true,  false, false, false, true },
        /* TEXT    */ {false, false, false, true,  true },
        /* JSON    */ {false, false, false, true,  true },
    };

    const size_t payload_index = (size_t)payload_kind;
    const size_t value_index = (size_t)value_type;
    if (payload_index > (size_t)mqtt_payload_kind_t::JSON || value_index > (size_t)mqtt_publish_value_type_t::EMPTY) {
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

static bool refresh_due(const topic_last_state_t &last, TickType_t now_ticks)
{
    if (!last.published_once) {
        return true;
    }

    return (now_ticks - last.last_publish_tick) >= MQTT_PUBLISH_REFRESH_INTERVAL_TICKS;
}

static void mark_published(topic_last_state_t &last, TickType_t now_ticks)
{
    last.published_once = true;
    last.last_publish_tick = now_ticks;
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
        case mqtt_publish_value_type_t::EMPTY:
            payload[0] = '\0';
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

    const bool changed = !value_equals(event, s_last_state[topic_index]);
    topic_last_state_t &last = s_last_state[topic_index];

    if (changed) {
        save_last_state(event);
    }

    if (!s_mqtt_connected) {
        return ESP_OK;
    }

    const TickType_t now_ticks = xTaskGetTickCount();
    if (!changed && !refresh_due(last, now_ticks)) {
        return ESP_OK;
    }

    const mqtt_publish_event_t &event_to_publish = changed ? event : last.event;
    esp_err_t publish_result = publish_event_now(event_to_publish);
    if (publish_result == ESP_OK) {
        status_display_notify_mqtt_activity();
        mark_published(last, now_ticks);
    }

    return publish_result;
}

static void mqtt_publisher_task(void *param)
{
    (void)param;

    mqtt_publish_queue_item_t item;
    memset(&item, 0, sizeof(item));
    while (true) {
        if (xQueueReceive(s_publish_queue, &item, portMAX_DELAY) != pdPASS) {
            continue;
        }

        if (item.type == queue_item_type_t::FLUSH_CACHED) {
            flush_cached_values();
            continue;
        }

        esp_err_t result = publish_if_changed(item.event);
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

    s_publish_queue = xQueueCreate((UBaseType_t)queue_length, sizeof(mqtt_publish_queue_item_t));
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

    mqtt_publish_queue_item_t item;
    memset(&item, 0, sizeof(item));
    item.type = queue_item_type_t::PUBLISH_EVENT;
    item.event.topic_id = event->topic_id;
    item.event.value_type = event->value_type;

    switch (event->value_type) {
        case mqtt_publish_value_type_t::BOOL:
            item.event.value.as_bool = event->value.as_bool;
            break;
        case mqtt_publish_value_type_t::INT64:
            item.event.value.as_int64 = event->value.as_int64;
            break;
        case mqtt_publish_value_type_t::DOUBLE:
            item.event.value.as_double = event->value.as_double;
            break;
        case mqtt_publish_value_type_t::TEXT:
            strncpy(item.event.value.as_text, event->value.as_text, MQTT_PUBLISH_TEXT_MAX_LEN - 1);
            item.event.value.as_text[MQTT_PUBLISH_TEXT_MAX_LEN - 1] = '\0';
            break;
        case mqtt_publish_value_type_t::EMPTY:
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return (xQueueSend(s_publish_queue, &item, timeout_ticks) == pdPASS) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t mqtt_publisher_enqueue_bool(mqtt_topic_id_t topic_id, bool value)
{
    mqtt_publish_event_t event;
    memset(&event, 0, sizeof(event));
    event.topic_id = topic_id;
    event.value_type = mqtt_publish_value_type_t::BOOL;
    event.value.as_bool = value;
    return mqtt_publisher_enqueue(&event, MQTT_PUBLISH_ENQUEUE_TIMEOUT_TICKS);
}

esp_err_t mqtt_publisher_enqueue_int64(mqtt_topic_id_t topic_id, int64_t value)
{
    mqtt_publish_event_t event;
    memset(&event, 0, sizeof(event));
    event.topic_id = topic_id;
    event.value_type = mqtt_publish_value_type_t::INT64;
    event.value.as_int64 = value;
    return mqtt_publisher_enqueue(&event, MQTT_PUBLISH_ENQUEUE_TIMEOUT_TICKS);
}

esp_err_t mqtt_publisher_enqueue_double(mqtt_topic_id_t topic_id, double value)
{
    mqtt_publish_event_t event;
    memset(&event, 0, sizeof(event));
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

    mqtt_publish_event_t event;
    memset(&event, 0, sizeof(event));
    event.topic_id = topic_id;
    event.value_type = mqtt_publish_value_type_t::TEXT;
    strncpy(event.value.as_text, value, MQTT_PUBLISH_TEXT_MAX_LEN - 1);
    event.value.as_text[MQTT_PUBLISH_TEXT_MAX_LEN - 1] = '\0';
    return mqtt_publisher_enqueue(&event, MQTT_PUBLISH_ENQUEUE_TIMEOUT_TICKS);
}

esp_err_t mqtt_publisher_enqueue_empty(mqtt_topic_id_t topic_id)
{
    mqtt_publish_event_t event;
    memset(&event, 0, sizeof(event));
    event.topic_id = topic_id;
    event.value_type = mqtt_publish_value_type_t::EMPTY;
    return mqtt_publisher_enqueue(&event, MQTT_PUBLISH_ENQUEUE_TIMEOUT_TICKS);
}

esp_err_t mqtt_publisher_set_mqtt_connected(bool connected)
{
    s_mqtt_connected = connected;

    if (!connected) {
        return ESP_OK;
    }

    if (s_publish_queue == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    mqtt_publish_queue_item_t item;
    memset(&item, 0, sizeof(item));
    item.type = queue_item_type_t::FLUSH_CACHED;

    return (xQueueSend(s_publish_queue, &item, 0) == pdPASS) ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool mqtt_publisher_is_running(void)
{
    return s_publish_task != nullptr;
}
