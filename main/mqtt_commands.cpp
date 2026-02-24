#include "mqtt_commands.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

#include "mqtt_client.h"

#include "mqtt_publisher_task.h"
#include "mqtt_topics.h"
#include "network_init.h"
#include "webapp_startup.h"

static const char *TAG = "mqtt_cmd";
static constexpr TickType_t REGISTER_RETRY_DELAY_TICKS = pdMS_TO_TICKS(500);
static constexpr TickType_t DEBUG_IDLE_DELAY_TICKS = pdMS_TO_TICKS(500);
static constexpr uint32_t DEBUG_INTERVAL_MIN_MS = 100;
static constexpr uint32_t DEBUG_INTERVAL_MAX_MS = 600000;

static bool s_handler_registered = false;
static bool s_debug_enabled = false;
static uint32_t s_debug_interval_ms = 5000;
static char s_debug_sensors[MQTT_PUBLISH_TEXT_MAX_LEN] = "all";
static portMUX_TYPE s_debug_mux = portMUX_INITIALIZER_UNLOCKED;

static const mqtt_topic_descriptor_t *find_command_topic(const char *topic, int topic_len)
{
    if (topic == nullptr || topic_len <= 0) {
        return nullptr;
    }

    for (size_t index = 0; index < (size_t)mqtt_topic_id_t::COUNT; ++index) {
        const mqtt_topic_descriptor_t &descriptor = MQTT_TOPIC_TABLE[index];
        if (descriptor.direction != mqtt_topic_direction_t::SUBSCRIBE_ONLY) {
            continue;
        }

        const size_t descriptor_len = strlen(descriptor.full_topic);
        if (descriptor_len != (size_t)topic_len) {
            continue;
        }

        if (memcmp(descriptor.full_topic, topic, descriptor_len) == 0) {
            ESP_LOGI(TAG, "Rozpoznan command topic: %s", descriptor.full_topic);
            return &descriptor;
        }
    }

    return nullptr;
}

static bool payload_is_truthy(const char *payload)
{
    if (payload == nullptr || payload[0] == '\0') {
        return true;
    }

    while (*payload != '\0' && isspace((unsigned char)*payload) != 0) {
        ++payload;
    }

    if (*payload == '\0') {
        return true;
    }

    if (strcasecmp(payload, "1") == 0 ||
        strcasecmp(payload, "true") == 0 ||
        strcasecmp(payload, "on") == 0 ||
        strcasecmp(payload, "yes") == 0 ||
        strcasecmp(payload, "now") == 0) {
        return true;
    }

    return false;
}

static void command_set_debug_enabled(bool enabled)
{
    taskENTER_CRITICAL(&s_debug_mux);
    s_debug_enabled = enabled;
    taskEXIT_CRITICAL(&s_debug_mux);

    ESP_LOGI(TAG, "Debug režim: %s", enabled ? "ON" : "OFF");
}

static void command_set_debug_interval(const char *payload)
{
    if (payload == nullptr || payload[0] == '\0') {
        ESP_LOGW(TAG, "cmd/debug/interval_ms: prazdny payload");
        return;
    }

    char *endptr = nullptr;
    long value = strtol(payload, &endptr, 10);
    if (endptr == payload) {
        ESP_LOGW(TAG, "cmd/debug/interval_ms: neplatna hodnota '%s'", payload);
        return;
    }

    if (value < (long)DEBUG_INTERVAL_MIN_MS) {
        value = (long)DEBUG_INTERVAL_MIN_MS;
    }
    if (value > (long)DEBUG_INTERVAL_MAX_MS) {
        value = (long)DEBUG_INTERVAL_MAX_MS;
    }

    taskENTER_CRITICAL(&s_debug_mux);
    s_debug_interval_ms = (uint32_t)value;
    taskEXIT_CRITICAL(&s_debug_mux);

    ESP_LOGI(TAG, "Debug interval nastaven na %lu ms", (unsigned long)value);
}

static void command_set_debug_sensors(const char *payload)
{
    if (payload == nullptr) {
        return;
    }

    taskENTER_CRITICAL(&s_debug_mux);
    strncpy(s_debug_sensors, payload, sizeof(s_debug_sensors) - 1);
    s_debug_sensors[sizeof(s_debug_sensors) - 1] = '\0';
    taskEXIT_CRITICAL(&s_debug_mux);

    ESP_LOGI(TAG, "Debug sensors filter: %s", s_debug_sensors);
}

static void handle_command(mqtt_topic_id_t command_id, const char *payload)
{
    ESP_LOGI(TAG,
             "Dispatch command id=%u payload='%s'",
             (unsigned)command_id,
             (payload != nullptr) ? payload : "");

    switch (command_id) {
        case mqtt_topic_id_t::TOPIC_CMD_REBOOT:
            if (payload_is_truthy(payload)) {
                ESP_LOGW(TAG, "Prijat cmd/reboot, restartuji...");
                esp_restart();
            } else {
                ESP_LOGI(TAG, "cmd/reboot ignorovan (payload neni truthy)");
            }
            break;

        case mqtt_topic_id_t::TOPIC_CMD_WEBAPP_START: {
            esp_err_t result = webapp_startup_start();
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "cmd/webapp/start selhal: %s", esp_err_to_name(result));
            }
            break;
        }

        case mqtt_topic_id_t::TOPIC_CMD_WEBAPP_STOP: {
            esp_err_t result = webapp_startup_stop();
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "cmd/webapp/stop selhal: %s", esp_err_to_name(result));
            }
            break;
        }

        case mqtt_topic_id_t::TOPIC_CMD_DEBUG_START:
            command_set_debug_enabled(true);
            break;

        case mqtt_topic_id_t::TOPIC_CMD_DEBUG_STOP:
            command_set_debug_enabled(false);
            break;

        case mqtt_topic_id_t::TOPIC_CMD_DEBUG_INTERVAL_MS:
            command_set_debug_interval(payload);
            break;

        case mqtt_topic_id_t::TOPIC_CMD_DEBUG_SENSORS:
            command_set_debug_sensors(payload);
            break;

        default:
            ESP_LOGW(TAG, "Neznamy command topic id: %u", (unsigned)command_id);
            break;
    }
}

static void subscribe_command_topics(esp_mqtt_client_handle_t client)
{
    ESP_LOGI(TAG, "Subscribuji command topicy...");

    size_t subscribed = 0;
    for (size_t index = 0; index < (size_t)mqtt_topic_id_t::COUNT; ++index) {
        const mqtt_topic_descriptor_t &descriptor = MQTT_TOPIC_TABLE[index];
        if (descriptor.direction != mqtt_topic_direction_t::SUBSCRIBE_ONLY) {
            continue;
        }

        int msg_id = esp_mqtt_client_subscribe(client, descriptor.full_topic, descriptor.qos);
        if (msg_id < 0) {
            ESP_LOGW(TAG, "Subscribe topicu %s selhal", descriptor.full_topic);
        } else {
            ESP_LOGI(TAG, "Subscribe topicu %s (msg_id=%d)", descriptor.full_topic, msg_id);
            subscribed++;
        }
    }

    ESP_LOGI(TAG, "Subscribe command topicu hotov: %u", (unsigned)subscribed);
}

static void mqtt_commands_event_handler(void *handler_args,
                                        esp_event_base_t base,
                                        int32_t event_id,
                                        void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    if (event == nullptr) {
        ESP_LOGI(TAG, "MQTT event ignorovan: event_data je nullptr");
        return;
    }

    ESP_LOGI(TAG,
             "MQTT event prijat: id=%ld topic_len=%d data_len=%d",
             (long)event_id,
             event->topic_len,
             event->data_len);

    if (event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT connected event -> subscribe command topicu");
        subscribe_command_topics(event->client);
        return;
    }

    if (event_id != MQTT_EVENT_DATA) {
        ESP_LOGI(TAG, "MQTT event id=%ld neni DATA, preskakuji", (long)event_id);
        return;
    }

    const mqtt_topic_descriptor_t *command = find_command_topic(event->topic, event->topic_len);
    if (command == nullptr) {
        ESP_LOGI(TAG, "MQTT DATA na neregistrovany command topic, ignoruji");
        return;
    }

    char payload[MQTT_PUBLISH_TEXT_MAX_LEN] = {0};
    if (event->data != nullptr && event->data_len > 0) {
        const int copy_len = (event->data_len < (int)(sizeof(payload) - 1)) ? event->data_len : (int)(sizeof(payload) - 1);
        memcpy(payload, event->data, (size_t)copy_len);
        payload[copy_len] = '\0';
    }

    ESP_LOGI(TAG, "MQTT command: %s payload='%s'", command->full_topic, payload);
    handle_command(command->id, payload);
}

static void mqtt_commands_register_task(void *param)
{
    (void)param;

    ESP_LOGI(TAG, "Start mqtt_commands_register_task");

    while (!s_handler_registered) {
        esp_mqtt_client_handle_t client = network_mqtt_client();
        if (client != nullptr) {
            ESP_LOGI(TAG, "MQTT client handle dostupny, registruji event handler");
            esp_err_t result = esp_mqtt_client_register_event(client,
                                                              MQTT_EVENT_ANY,
                                                              mqtt_commands_event_handler,
                                                              nullptr);
            if (result == ESP_OK) {
                s_handler_registered = true;
                ESP_LOGI(TAG, "MQTT command handler registrovan");

                if (network_mqtt_is_connected()) {
                    ESP_LOGI(TAG, "MQTT uz je pripojeno, subscribuji command topicy ihned");
                    subscribe_command_topics(client);
                } else {
                    ESP_LOGI(TAG, "MQTT zatim nepripojeno, subscribe probehne pri MQTT_EVENT_CONNECTED");
                }
                break;
            }
            ESP_LOGW(TAG, "Registrace MQTT command handleru selhala: %s", esp_err_to_name(result));
        } else {
            ESP_LOGI(TAG, "MQTT client handle zatim neni k dispozici, cekam...");
        }

        vTaskDelay(REGISTER_RETRY_DELAY_TICKS);
    }

    vTaskDelete(nullptr);
}

static void mqtt_debug_publish_task(void *param)
{
    (void)param;

    ESP_LOGI(TAG, "Start mqtt_debug_publish_task");

    char sensors_copy[MQTT_PUBLISH_TEXT_MAX_LEN] = {0};
    char debug_line[MQTT_PUBLISH_TEXT_MAX_LEN] = {0};
    bool last_enabled = false;

    while (true) {
        bool enabled = false;
        uint32_t interval_ms = 0;

        taskENTER_CRITICAL(&s_debug_mux);
        enabled = s_debug_enabled;
        interval_ms = s_debug_interval_ms;
        strncpy(sensors_copy, s_debug_sensors, sizeof(sensors_copy) - 1);
        sensors_copy[sizeof(sensors_copy) - 1] = '\0';
        taskEXIT_CRITICAL(&s_debug_mux);

        if (enabled != last_enabled) {
            ESP_LOGI(TAG, "Debug publisher stav: %s", enabled ? "ENABLED" : "DISABLED");
            last_enabled = enabled;
        }

        if (!enabled) {
            vTaskDelay(DEBUG_IDLE_DELAY_TICKS);
            continue;
        }

        int64_t uptime_s = esp_timer_get_time() / 1000000LL;
        snprintf(debug_line,
                 sizeof(debug_line),
                 "{\"uptime_s\":%lld,\"heap\":%u,\"sensors\":\"%.48s\"}",
                 (long long)uptime_s,
                 (unsigned)esp_get_free_heap_size(),
                 sensors_copy);

        (void)mqtt_publisher_enqueue_text(mqtt_topic_id_t::TOPIC_DEBUG_INTERMEDIATE, debug_line);
        (void)mqtt_publisher_enqueue_text(mqtt_topic_id_t::TOPIC_DEBUG_RAW, sensors_copy);

        ESP_LOGI(TAG,
                 "Debug publish odeslan (interval=%lu ms, sensors='%s')",
                 (unsigned long)interval_ms,
                 sensors_copy);

        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
}

esp_err_t mqtt_commands_start(void)
{
    ESP_LOGI(TAG, "mqtt_commands_start() volano");

    if (xTaskCreate(mqtt_commands_register_task,
                    "mqtt_cmd_reg",
                    configMINIMAL_STACK_SIZE * 4,
                    nullptr,
                    4,
                    nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Nelze vytvorit mqtt_cmd_reg task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Task mqtt_cmd_reg vytvoren");

    if (xTaskCreate(mqtt_debug_publish_task,
                    "mqtt_cmd_dbg",
                    configMINIMAL_STACK_SIZE * 4,
                    nullptr,
                    4,
                    nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Nelze vytvorit mqtt_cmd_dbg task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Task mqtt_cmd_dbg vytvoren");

    return ESP_OK;
}
