#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#ifdef __cplusplus
}
#endif

#include <cmath>
#include <stdio.h>

#include "state_manager.h"
#include "sensor_events.h"
#include "lcd.h"
#include "mqtt_publisher_task.h"
#include "webapp_startup.h"
#include "status_display.h"
#include <tm1637.h>

static const char *TAG = "state";

static bool s_temp_probe_fault_water = false;
static bool s_temp_probe_fault_air = false;



static void set_sensor_fault_indicator(sensor_event_type_t sensor_type, bool is_fault)
{
    status_display_set_sensor_fault(sensor_type, is_fault);


}

static void publish_boot_diagnostics_once(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    const esp_err_t state_result = (running != nullptr)
                                     ? esp_ota_get_state_partition(running, &state)
                                     : ESP_ERR_INVALID_STATE;

    const char *boot_mode = "normal";
    if (state_result == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
        boot_mode = "ota";
    }

    esp_err_t boot_mode_result = mqtt_publisher_enqueue_text(mqtt_topic_id_t::TOPIC_SYSTEM_BOOT_MODE, boot_mode);
    if (boot_mode_result != ESP_OK) {
        ESP_LOGW(TAG, "Publikace boot mode selhala: %s", esp_err_to_name(boot_mode_result));
    }

    const esp_app_desc_t *app_desc = esp_app_get_description();
    if (app_desc != nullptr) {
        esp_err_t fw_result = mqtt_publisher_enqueue_text(mqtt_topic_id_t::TOPIC_DIAG_FW_VERSION, app_desc->version);
        if (fw_result != ESP_OK) {
            ESP_LOGW(TAG, "Publikace fw_version selhala: %s", esp_err_to_name(fw_result));
        }
    }
}

static void publish_temperature_to_outputs(const sensor_event_t &event)
{
    const sensor_temperature_probe_t probe = event.data.temperature.probe;
    char text[16];
    esp_err_t enqueue_result;
    const bool sensor_fault = std::isnan(event.data.temperature.temperature_c);

    if (probe == SENSOR_TEMPERATURE_PROBE_AIR) {
        s_temp_probe_fault_air = sensor_fault;
    } else {
        s_temp_probe_fault_water = sensor_fault;
    }
    set_sensor_fault_indicator(SENSOR_EVENT_TEMPERATURE, s_temp_probe_fault_water || s_temp_probe_fault_air);

    const mqtt_topic_id_t topic_id =
        (probe == SENSOR_TEMPERATURE_PROBE_AIR)
            ? mqtt_topic_id_t::TOPIC_STAV_TEPLOTA_VZDUCH
            : mqtt_topic_id_t::TOPIC_STAV_TEPLOTA_VODA;

    if (sensor_fault) {
        if (probe == SENSOR_TEMPERATURE_PROBE_WATER) {
            snprintf(text, sizeof(text), "T: --.- ");
            lcd_print(8, 0, text, false, 0);
        }

        enqueue_result = mqtt_publisher_enqueue_empty(topic_id);
    } else {
        if (probe == SENSOR_TEMPERATURE_PROBE_WATER) {
            snprintf(text, sizeof(text), "T:%4.1f ", event.data.temperature.temperature_c);
            lcd_print(8, 0, text, false, 0);
        }

        enqueue_result = mqtt_publisher_enqueue_double(
            topic_id,
            (double)event.data.temperature.temperature_c);
    }

    if (enqueue_result != ESP_OK) {
        const char *probe_name = (probe == SENSOR_TEMPERATURE_PROBE_AIR) ? "vzduchu" : "vody";
        ESP_LOGW(TAG, "Enqueue teploty %s selhalo: %s", probe_name, esp_err_to_name(enqueue_result));
    }
}

static void publish_level_to_outputs(const sensor_event_t &event)
{
    char text[16];
    esp_err_t enqueue_result;
    const bool sensor_fault = (event.data.level.raw_value == 0);
    set_sensor_fault_indicator(SENSOR_EVENT_LEVEL, sensor_fault);

    if (sensor_fault) {
        snprintf(text, sizeof(text), "H: ---  ");
        lcd_print(8, 1, text, false, 0);

        enqueue_result = mqtt_publisher_enqueue_empty(mqtt_topic_id_t::TOPIC_STAV_OBJEM);
    } else {
        snprintf(text, sizeof(text), "H:%3.0fcm ", event.data.level.height_m * 100.0f);
        lcd_print(8, 1, text, false, 0);

        enqueue_result = mqtt_publisher_enqueue_double(
            mqtt_topic_id_t::TOPIC_STAV_OBJEM,
            (double)event.data.level.height_m);
    }

    if (enqueue_result != ESP_OK) {
        ESP_LOGW(TAG, "Enqueue hladiny/objemu selhalo: %s", esp_err_to_name(enqueue_result));
    }
}

static void publish_flow_to_outputs(const sensor_event_t &event)
{
    const bool sensor_fault = !std::isfinite(event.data.flow.flow_l_min) || !std::isfinite(event.data.flow.total_volume_l);
    set_sensor_fault_indicator(SENSOR_EVENT_FLOW, sensor_fault);
    status_display_set_flow_rate(event.data.flow.flow_l_min);

    char liters_text[16];
    snprintf(liters_text, sizeof(liters_text), "L:%5.1f ", event.data.flow.total_volume_l);
    lcd_print(0, 0, liters_text, false, 0);

    char flow_text[16];
    snprintf(flow_text, sizeof(flow_text), "Q:%4.1f ", event.data.flow.flow_l_min);
    lcd_print(0, 1, flow_text, false, 0);

    esp_err_t flow_result = mqtt_publisher_enqueue_double(
        mqtt_topic_id_t::TOPIC_STAV_PRUTOK,
        (double)event.data.flow.flow_l_min);
    if (flow_result != ESP_OK) {
        ESP_LOGW(TAG, "Enqueue prutoku selhalo: %s", esp_err_to_name(flow_result));
    }

    esp_err_t total_result = mqtt_publisher_enqueue_double(
        mqtt_topic_id_t::TOPIC_STAV_CERPANO_CELKEM,
        (double)event.data.flow.total_volume_l);
    if (total_result != ESP_OK) {
        ESP_LOGW(TAG, "Enqueue cerpano_celkem selhalo: %s", esp_err_to_name(total_result));
    }

}

static void state_manager_task(void *pvParameters)
{
    (void)pvParameters;

    app_event_t event = {};
    char debug_line[128];
    bool mqtt_ready_published = false;
    bool boot_diagnostics_published = false;

    while (true) {
        if (!sensor_events_receive(&event, portMAX_DELAY)) {
            continue;
        }

        sensor_event_to_string(&event, debug_line, sizeof(debug_line));
        ESP_LOGD(TAG, "%s", debug_line);

        switch (event.event_type) {
            case EVT_SENSOR:
                switch (event.data.sensor.sensor_type) {
                    case SENSOR_EVENT_TEMPERATURE:
                        publish_temperature_to_outputs(event.data.sensor);
                        break;
                    case SENSOR_EVENT_LEVEL:
                        publish_level_to_outputs(event.data.sensor);
                        break;
                    case SENSOR_EVENT_FLOW: {
                        publish_flow_to_outputs(event.data.sensor);
                        break;
                    }
                    default:
                        ESP_LOGW(TAG, "Neznamy sensor event: %d", (int)event.data.sensor.sensor_type);
                        break;
                }
                break;
            case EVT_NETWORK_STATE_CHANGE: {
                const network_event_t *network_snapshot = &event.data.network_state_change.snapshot;

                ESP_LOGW(TAG,
                         "Network state change: %d -> %d (rssi=%d ip=0x%08lx reconn_attempts=%lu reconn_success=%lu)",
                         (int)event.data.network_state_change.from_level,
                         (int)event.data.network_state_change.to_level,
                         (int)network_snapshot->last_rssi,
                         (unsigned long)network_snapshot->ip_addr,
                         (unsigned long)network_snapshot->reconnect_attempts,
                         (unsigned long)network_snapshot->reconnect_successes);

                if (event.data.network_state_change.to_level == SYS_NET_AP_CONFIG) {
                    esp_err_t mqtt_state_result = mqtt_publisher_set_mqtt_connected(false);
                    if (mqtt_state_result != ESP_OK) {
                        ESP_LOGW(TAG, "Vypnuti MQTT publisheru v AP rezimu selhalo: %s", esp_err_to_name(mqtt_state_result));
                    }

                    status_display_ap_mode();
                    ESP_LOGW(TAG, "AP rezim aktivni: state manager se ukoncuje (z AP vede jen reset)");
                    vTaskDelete(NULL);
                }

                status_display_set_network_state(network_snapshot);
                webapp_startup_on_network_event(network_snapshot);

                const bool mqtt_ready = (event.data.network_state_change.to_level == SYS_NET_MQTT_READY);
                esp_err_t mqtt_state_result = mqtt_publisher_set_mqtt_connected(mqtt_ready);
                if (mqtt_state_result != ESP_OK) {
                    ESP_LOGW(TAG, "Nastaveni MQTT stavu publisheru selhalo: %s", esp_err_to_name(mqtt_state_result));
                }

                if (mqtt_ready) {

                    if (!boot_diagnostics_published) {
                        publish_boot_diagnostics_once();
                        boot_diagnostics_published = true;
                    }

                    if (!mqtt_ready_published) {
                        esp_err_t enqueue_result = mqtt_publisher_enqueue_text(
                            mqtt_topic_id_t::TOPIC_SYSTEM_STATUS,
                            "online");
                        if (enqueue_result == ESP_OK) {
                            mqtt_ready_published = true;
                            ESP_LOGI(TAG, "MQTT online status publikovan");
                        } else {
                            ESP_LOGW(TAG, "Publikace online statusu selhala: %s", esp_err_to_name(enqueue_result));
                        }
                    }
                } else {
                    mqtt_ready_published = false;
                    boot_diagnostics_published = false;
                }
                break;
            }
            case EVT_NETWORK_TELEMETRY: {
                const network_event_t *network_snapshot = &event.data.network_telemetry.snapshot;
                ESP_LOGD(TAG,
                         "Network telemetry: level=%d rssi=%d ip=0x%08lx reconn_attempts=%lu reconn_success=%lu",
                         (int)network_snapshot->level,
                         (int)network_snapshot->last_rssi,
                         (unsigned long)network_snapshot->ip_addr,
                         (unsigned long)network_snapshot->reconnect_attempts,
                         (unsigned long)network_snapshot->reconnect_successes);
                break;
            }
            case EVT_TICK:
                ESP_LOGD(TAG, "Tick event zatim neni implementovany");
                break;
            default:
                ESP_LOGW(TAG, "Neznamy event_type: %d", (int)event.event_type);
                break;
        }
    }
}

void state_manager_start(void)
{
    xTaskCreate(state_manager_task, TAG, configMINIMAL_STACK_SIZE * 5, NULL, 4, NULL);
}
