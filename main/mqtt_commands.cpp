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
#include "freertos/timers.h"
}

#include "mqtt_client.h"

#include "mqtt_publisher_task.h"
#include "mqtt_topics.h"
#include "network_init.h"
#include "ota_manager.h"
#include "status_display.h"
#include "teplota.h"
#include "webapp_startup.h"
#include "debug_mqtt.h"

static const char *TAG = "mqtt_cmd";
static constexpr TickType_t REGISTER_RETRY_DELAY_TICKS = pdMS_TO_TICKS(500);
static constexpr uint32_t DEBUG_AUTO_OFF_MS = 2U * 60U * 60U * 1000U;

static bool s_handler_registered = false;
static portMUX_TYPE s_debug_mux = portMUX_INITIALIZER_UNLOCKED;
static TimerHandle_t s_debug_auto_off_timer = nullptr;

static void debug_auto_off_timer_callback(TimerHandle_t timer)
{
    (void)timer;

    taskENTER_CRITICAL(&s_debug_mux);
    g_debug_enabled = false;
    taskEXIT_CRITICAL(&s_debug_mux);

    ESP_LOGW(TAG, "Debug režim automaticky vypnut po 2 hodinach");
}

static void ensure_debug_auto_off_timer(void)
{
    if (s_debug_auto_off_timer != nullptr) {
        return;
    }

    s_debug_auto_off_timer = xTimerCreate(
        "dbg_auto_off",
        pdMS_TO_TICKS(DEBUG_AUTO_OFF_MS),
        pdFALSE,
        nullptr,
        debug_auto_off_timer_callback);

    if (s_debug_auto_off_timer == nullptr) {
        ESP_LOGE(TAG, "Nelze vytvorit debug auto-off timer");
    }
}

static const char *mqtt_event_name(int32_t event_id)
{
    switch (event_id) {
        case MQTT_EVENT_CONNECTED: return "MQTT_EVENT_CONNECTED";
        case MQTT_EVENT_DISCONNECTED: return "MQTT_EVENT_DISCONNECTED";
        case MQTT_EVENT_SUBSCRIBED: return "MQTT_EVENT_SUBSCRIBED";
        case MQTT_EVENT_UNSUBSCRIBED: return "MQTT_EVENT_UNSUBSCRIBED";
        case MQTT_EVENT_PUBLISHED: return "MQTT_EVENT_PUBLISHED";
        case MQTT_EVENT_DATA: return "MQTT_EVENT_DATA";
        case MQTT_EVENT_ERROR: return "MQTT_EVENT_ERROR";
        case MQTT_EVENT_BEFORE_CONNECT: return "MQTT_EVENT_BEFORE_CONNECT";
        case MQTT_EVENT_DELETED: return "MQTT_EVENT_DELETED";
        default: return "MQTT_EVENT_UNKNOWN";
    }
}

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
    ensure_debug_auto_off_timer();

    taskENTER_CRITICAL(&s_debug_mux);
    g_debug_enabled = enabled;
    taskEXIT_CRITICAL(&s_debug_mux);

    if (s_debug_auto_off_timer != nullptr) {
        if (enabled) {
            xTimerStop(s_debug_auto_off_timer, 0);
            xTimerChangePeriod(s_debug_auto_off_timer, pdMS_TO_TICKS(DEBUG_AUTO_OFF_MS), 0);
            xTimerStart(s_debug_auto_off_timer, 0);
        } else {
            xTimerStop(s_debug_auto_off_timer, 0);
        }
    }

    ESP_LOGI(TAG, "Debug režim: %s", enabled ? "ON" : "OFF");
}

static char *trim_in_place(char *text)
{
    if (text == nullptr) {
        return nullptr;
    }

    while (*text != '\0' && isspace((unsigned char)*text) != 0) {
        ++text;
    }

    char *end = text + strlen(text);
    while (end > text && isspace((unsigned char)*(end - 1)) != 0) {
        --end;
    }
    *end = '\0';

    return text;
}

static bool parse_log_level(const char *text, esp_log_level_t *out_level)
{
    if (text == nullptr || out_level == nullptr) {
        return false;
    }

    if (strcasecmp(text, "none") == 0 || strcmp(text, "0") == 0) {
        *out_level = ESP_LOG_NONE;
        return true;
    }
    if (strcasecmp(text, "error") == 0 || strcasecmp(text, "err") == 0 || strcmp(text, "1") == 0) {
        *out_level = ESP_LOG_ERROR;
        return true;
    }
    if (strcasecmp(text, "warn") == 0 || strcasecmp(text, "warning") == 0 || strcmp(text, "2") == 0) {
        *out_level = ESP_LOG_WARN;
        return true;
    }
    if (strcasecmp(text, "info") == 0 || strcmp(text, "3") == 0) {
        *out_level = ESP_LOG_INFO;
        return true;
    }
    if (strcasecmp(text, "debug") == 0 || strcmp(text, "4") == 0) {
        *out_level = ESP_LOG_DEBUG;
        return true;
    }
    if (strcasecmp(text, "verbose") == 0 || strcasecmp(text, "trace") == 0 || strcmp(text, "5") == 0) {
        *out_level = ESP_LOG_VERBOSE;
        return true;
    }

    return false;
}

static const char *log_level_name(esp_log_level_t level)
{
    switch (level) {
        case ESP_LOG_NONE: return "NONE";
        case ESP_LOG_ERROR: return "ERROR";
        case ESP_LOG_WARN: return "WARN";
        case ESP_LOG_INFO: return "INFO";
        case ESP_LOG_DEBUG: return "DEBUG";
        case ESP_LOG_VERBOSE: return "VERBOSE";
        default: return "UNKNOWN";
    }
}

static void command_set_log_level(const char *payload)
{
    if (payload == nullptr || payload[0] == '\0') {
        ESP_LOGW(TAG, "cmd/log/level: prazdny payload, ocekavam 'tag=level' nebo 'tag level'");
        return;
    }

    char buffer[MQTT_PUBLISH_TEXT_MAX_LEN] = {0};
    strncpy(buffer, payload, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *cursor = trim_in_place(buffer);
    if (cursor == nullptr || cursor[0] == '\0') {
        ESP_LOGW(TAG, "cmd/log/level: prazdny payload po trimu");
        return;
    }

    char *separator = strchr(cursor, '=');
    if (separator == nullptr) {
        separator = strchr(cursor, ':');
    }
    if (separator == nullptr) {
        for (char *scan = cursor; *scan != '\0'; ++scan) {
            if (isspace((unsigned char)*scan) != 0) {
                separator = scan;
                break;
            }
        }
    }

    if (separator == nullptr) {
        ESP_LOGW(TAG, "cmd/log/level: neplatny payload '%s', ocekavam oddeleni tag/level", cursor);
        return;
    }

    *separator = '\0';
    char *tag = trim_in_place(cursor);
    char *level_text = trim_in_place(separator + 1);

    if (tag == nullptr || tag[0] == '\0' || level_text == nullptr || level_text[0] == '\0') {
        ESP_LOGW(TAG, "cmd/log/level: neplatny payload '%s'", payload);
        return;
    }

    esp_log_level_t level = ESP_LOG_INFO;
    if (!parse_log_level(level_text, &level)) {
        ESP_LOGW(TAG, "cmd/log/level: neznama uroven '%s'", level_text);
        return;
    }

    esp_log_level_set(tag, level);
    ESP_LOGW(TAG, "Log level nastaven: tag='%s' level=%s", tag, log_level_name(level));
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

        case mqtt_topic_id_t::TOPIC_CMD_LOG_LEVEL:
            command_set_log_level(payload);
            break;

        case mqtt_topic_id_t::TOPIC_CMD_OTA_START: {
            esp_err_t result = ota_manager_start_from_url(payload);
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "cmd/ota/start selhal: %s", esp_err_to_name(result));
            } else {
                ESP_LOGW(TAG, "cmd/ota/start prijat, OTA task spusten");
            }
            break;
        }

        case mqtt_topic_id_t::TOPIC_CMD_OTA_CONFIRM: {
            esp_err_t result = ota_manager_confirm_running_firmware();
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "cmd/ota/confirm selhal: %s", esp_err_to_name(result));
            } else {
                ESP_LOGW(TAG, "cmd/ota/confirm prijat, firmware potvrzen");
            }
            break;
        }

        case mqtt_topic_id_t::TOPIC_CMD_TEPLOTA_SCAN: {
            const bool enabled = payload_is_truthy(payload);
            esp_err_t result = teplota_set_scan_enabled(enabled);
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "cmd/teplota/scan selhal: %s", esp_err_to_name(result));
            } else {
                ESP_LOGW(TAG, "cmd/teplota/scan: %s", enabled ? "ON" : "OFF");
            }
            break;
        }

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

    if (event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT connected event -> subscribe command topicu");
        subscribe_command_topics(event->client);
        return;
    }

    if (event_id != MQTT_EVENT_DATA) {
        ESP_LOGV(TAG, "MQTT event id=%ld (%s) neni DATA, preskakuji", (long)event_id, mqtt_event_name(event_id));
        return;
    }

    char topic_preview[96] = {0};
    if (event->topic != nullptr && event->topic_len > 0) {
        const int topic_copy_len = (event->topic_len < (int)(sizeof(topic_preview) - 1))
                                       ? event->topic_len
                                       : (int)(sizeof(topic_preview) - 1);
        memcpy(topic_preview, event->topic, (size_t)topic_copy_len);
        topic_preview[topic_copy_len] = '\0';
    }

    char data_preview[96] = {0};
    if (event->data != nullptr && event->data_len > 0) {
        const int data_copy_len = (event->data_len < (int)(sizeof(data_preview) - 1))
                                      ? event->data_len
                                      : (int)(sizeof(data_preview) - 1);
        memcpy(data_preview, event->data, (size_t)data_copy_len);
        data_preview[data_copy_len] = '\0';
    }

    ESP_LOGI(TAG,
             "MQTT DATA event: msg_id=%d topic=%s payload=%s len=%d retained=%d",
             event->msg_id,
             (topic_preview[0] != '\0') ? topic_preview : "(empty)",
             (data_preview[0] != '\0') ? data_preview : "(empty)",
             event->data_len,
             event->retain);

    status_display_notify_mqtt_activity();

    if (event->retain != 0) {
        ESP_LOGW(TAG, "Retained command zprava ignorovana: topic=%s", (topic_preview[0] != '\0') ? topic_preview : "(empty)");
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

    return ESP_OK;
}

bool mqtt_commands_debug_enabled(void)
{
    bool enabled = false;
    taskENTER_CRITICAL(&s_debug_mux);
    enabled = g_debug_enabled;
    taskEXIT_CRITICAL(&s_debug_mux);
    return enabled;
}
