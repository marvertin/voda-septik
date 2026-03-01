#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_system.h"

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
#include "restart_info.h"
#include "app_error_check.h"
#include <tm1637.h>

static const char *TAG = "state_manager";
static constexpr TickType_t STATE_MANAGER_EVENT_WAIT_TICKS = pdMS_TO_TICKS(1000);

static bool s_temp_probe_fault_water = false;
static bool s_temp_probe_fault_air = false;
static uint32_t s_nvs_errors = 0;
static uint32_t s_mqtt_reconnects = 0;
static int32_t s_last_mqtt_rc = 0;
static bool s_mqtt_ready_seen_once = false;
static bool s_disconnect_timer_active = false;
static int64_t s_disconnect_started_us = 0;

static void publish_runtime_diagnostics(const network_event_t *network_snapshot)
{
    const int64_t uptime_s = esp_timer_get_time() / 1000000LL;
    (void)mqtt_publisher_enqueue_int64(mqtt_topic_id_t::TOPIC_DIAG_UPTIME_S, uptime_s);

    if (network_snapshot != nullptr) {
        if (network_snapshot->last_rssi != INT8_MIN) {
            (void)mqtt_publisher_enqueue_int64(mqtt_topic_id_t::TOPIC_DIAG_WIFI_RSSI_DBM,
                                               (int64_t)network_snapshot->last_rssi);
        }
        (void)mqtt_publisher_enqueue_int64(mqtt_topic_id_t::TOPIC_DIAG_WIFI_RECONNECT_TRY,
                                           (int64_t)network_snapshot->reconnect_attempts);
        (void)mqtt_publisher_enqueue_int64(mqtt_topic_id_t::TOPIC_DIAG_WIFI_RECONNECT_SUCCESS,
                                           (int64_t)network_snapshot->reconnect_successes);
    }

    (void)mqtt_publisher_enqueue_int64(mqtt_topic_id_t::TOPIC_DIAG_MQTT_RECONNECTS,
                                       (int64_t)s_mqtt_reconnects);
    (void)mqtt_publisher_enqueue_int64(mqtt_topic_id_t::TOPIC_DIAG_LAST_MQTT_RC,
                                       (int64_t)s_last_mqtt_rc);

    (void)mqtt_publisher_enqueue_int64(mqtt_topic_id_t::TOPIC_DIAG_HEAP_FREE_B,
                                       (int64_t)esp_get_free_heap_size());
    (void)mqtt_publisher_enqueue_int64(mqtt_topic_id_t::TOPIC_DIAG_HEAP_MIN_FREE_B,
                                       (int64_t)esp_get_minimum_free_heap_size());
    (void)mqtt_publisher_enqueue_int64(mqtt_topic_id_t::TOPIC_DIAG_NVS_ERRORS,
                                       (int64_t)s_nvs_errors);
}



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

        char build_timestamp[48] = {0};
        snprintf(build_timestamp,
                 sizeof(build_timestamp),
                 "%s %s",
                 app_desc->date,
                 app_desc->time);
        (void)mqtt_publisher_enqueue_text(mqtt_topic_id_t::TOPIC_DIAG_BUILD_TIMESTAMP, build_timestamp);

        char git_hash[48] = {0};
        strncpy(git_hash, app_desc->version, sizeof(git_hash) - 1);
        git_hash[sizeof(git_hash) - 1] = '\0';
        char *dash = strchr(git_hash, '-');
        if (dash != nullptr) {
            *dash = '\0';
        }
        (void)mqtt_publisher_enqueue_text(mqtt_topic_id_t::TOPIC_DIAG_GIT_HASH, git_hash);
    }

    app_restart_info_t restart_info = {};
    esp_err_t restart_result = app_restart_info_update_and_load(&restart_info);
    if (restart_result != ESP_OK) {
        s_nvs_errors++;
        ESP_LOGW(TAG, "Publikace restart info selhala: %s", esp_err_to_name(restart_result));
    } else {
        char reason_text[16] = {0};
        snprintf(reason_text, sizeof(reason_text), "%d", (int)restart_info.last_reason);
        (void)mqtt_publisher_enqueue_text(mqtt_topic_id_t::TOPIC_SYSTEM_REBOOT_REASON, reason_text);
        (void)mqtt_publisher_enqueue_int64(mqtt_topic_id_t::TOPIC_SYSTEM_REBOOT_COUNTER,
                                           (int64_t)restart_info.boot_count);
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

static void publish_zasoba_to_outputs(const sensor_event_t &event)
{
    char text[16];
    esp_err_t enqueue_result;
    const bool sensor_fault = !std::isfinite(event.data.zasoba.objem) || !std::isfinite(event.data.zasoba.hladina);
    set_sensor_fault_indicator(SENSOR_EVENT_ZASOBA, sensor_fault);

    if (sensor_fault) {
        snprintf(text, sizeof(text), "O: ---  ");
        lcd_print(8, 1, text, false, 0);

        enqueue_result = mqtt_publisher_enqueue_empty(mqtt_topic_id_t::TOPIC_STAV_ZASOBA_OBJEM);
        (void)mqtt_publisher_enqueue_empty(mqtt_topic_id_t::TOPIC_STAV_ZASOBA_HLADINA);
    } else {
        snprintf(text, sizeof(text), "O:%4.0fL", event.data.zasoba.objem);
        lcd_print(8, 1, text, false, 0);

        enqueue_result = mqtt_publisher_enqueue_double(
            mqtt_topic_id_t::TOPIC_STAV_ZASOBA_OBJEM,
            (double)event.data.zasoba.objem);
        esp_err_t hladina_result = mqtt_publisher_enqueue_double(
            mqtt_topic_id_t::TOPIC_STAV_ZASOBA_HLADINA,
            (double)event.data.zasoba.hladina);
        if (hladina_result != ESP_OK) {
            ESP_LOGW(TAG, "Enqueue hladiny selhalo: %s", esp_err_to_name(hladina_result));
        }
    }

    if (enqueue_result != ESP_OK) {
        ESP_LOGW(TAG, "Enqueue hladiny/objemu selhalo: %s", esp_err_to_name(enqueue_result));
    }
}

static void publish_flow_to_outputs(const sensor_event_t &event)
{
    const bool sensor_fault = !std::isfinite(event.data.flow.prutok) || !std::isfinite(event.data.flow.cerpano_celkem);
    set_sensor_fault_indicator(SENSOR_EVENT_FLOW, sensor_fault);
    status_display_set_prutok(event.data.flow.prutok);

    char liters_text[16];
    snprintf(liters_text, sizeof(liters_text), "L:%5.1f ", event.data.flow.cerpano_celkem);
    lcd_print(0, 0, liters_text, false, 0);

    char flow_text[16];
    snprintf(flow_text, sizeof(flow_text), "Q:%4.1f ", event.data.flow.prutok);
    lcd_print(0, 1, flow_text, false, 0);

    esp_err_t flow_result = mqtt_publisher_enqueue_double(
        mqtt_topic_id_t::TOPIC_STAV_CERPANI_PRUTOK,
        (double)event.data.flow.prutok);
    if (flow_result != ESP_OK) {
        ESP_LOGW(TAG, "Enqueue prutoku selhalo: %s", esp_err_to_name(flow_result));
    }

    esp_err_t total_result = mqtt_publisher_enqueue_double(
        mqtt_topic_id_t::TOPIC_STAV_CERPANI_CERPANO_CELKEM,
        (double)event.data.flow.cerpano_celkem);
    if (total_result != ESP_OK) {
        ESP_LOGW(TAG, "Enqueue cerpano_celkem selhalo: %s", esp_err_to_name(total_result));
    }

}

static void publish_pressure_to_outputs(const sensor_event_t &event)
{
    const float pred_filtrem = event.data.pressure.pred_filtrem;
    const float za_filtrem = event.data.pressure.za_filtrem;
    const float rozdil_filtru = event.data.pressure.rozdil_filtru;
    const float zanesenost_filtru = event.data.pressure.zanesenost_filtru;

    const bool sensor_fault =
        !std::isfinite(pred_filtrem) ||
        !std::isfinite(za_filtrem) ||
        !std::isfinite(rozdil_filtru);
    set_sensor_fault_indicator(SENSOR_EVENT_PRESSURE, sensor_fault);

    esp_err_t before_result = mqtt_publisher_enqueue_double(
        mqtt_topic_id_t::TOPIC_STAV_TLAK_PRED_FILTREM,
        (double)pred_filtrem);
    if (before_result != ESP_OK) {
        ESP_LOGW(TAG, "Enqueue tlaku pred filtrem selhalo: %s", esp_err_to_name(before_result));
    }

    esp_err_t after_result = mqtt_publisher_enqueue_double(
        mqtt_topic_id_t::TOPIC_STAV_TLAK_ZA_FILTREM,
        (double)za_filtrem);
    if (after_result != ESP_OK) {
        ESP_LOGW(TAG, "Enqueue tlaku za filtrem selhalo: %s", esp_err_to_name(after_result));
    }

    esp_err_t diff_result = mqtt_publisher_enqueue_double(
        mqtt_topic_id_t::TOPIC_STAV_ROZDIL_TLAKU_FILTRU,
        (double)rozdil_filtru);
    if (diff_result != ESP_OK) {
        ESP_LOGW(TAG, "Enqueue rozdilu tlaku filtru selhalo: %s", esp_err_to_name(diff_result));
    }

    esp_err_t clog_result = mqtt_publisher_enqueue_double(
        mqtt_topic_id_t::TOPIC_STAV_ZANESENOST_FILTRU_PERCENT,
        (double)zanesenost_filtru);
    if (clog_result != ESP_OK) {
        ESP_LOGW(TAG, "Enqueue zanesenosti filtru selhalo: %s", esp_err_to_name(clog_result));
    }
}

static void state_manager_task(void *pvParameters)
{
    (void)pvParameters;
    APP_ERROR_CHECK("E531", esp_task_wdt_add(nullptr));

    app_event_t event = {};
    char debug_line[128];
    bool mqtt_ready_published = false;
    bool boot_diagnostics_published = false;

    while (true) {
        if (!sensor_events_receive(&event, STATE_MANAGER_EVENT_WAIT_TICKS)) {
            APP_ERROR_CHECK("E532", esp_task_wdt_reset());
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
                    case SENSOR_EVENT_ZASOBA:
                        publish_zasoba_to_outputs(event.data.sensor);
                        break;
                    case SENSOR_EVENT_FLOW: {
                        publish_flow_to_outputs(event.data.sensor);
                        break;
                    }
                    case SENSOR_EVENT_PRESSURE:
                        publish_pressure_to_outputs(event.data.sensor);
                        break;
                    default:
                        ESP_LOGW(TAG, "Neznamy sensor event: %d", (int)event.data.sensor.sensor_type);
                        break;
                }
                break;
            case EVT_NETWORK_STATE_CHANGE: {
                const network_event_t *network_snapshot = &event.data.network_state_change.snapshot;

                ESP_LOGI(TAG,
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
                    APP_ERROR_CHECK("E533", esp_task_wdt_delete(nullptr));
                    vTaskDelete(NULL);
                }

                status_display_set_network_state(network_snapshot);
                webapp_startup_on_network_event(network_snapshot);

                const bool mqtt_ready = (event.data.network_state_change.to_level == SYS_NET_MQTT_READY);

                if (!mqtt_ready && event.data.network_state_change.from_level == SYS_NET_MQTT_READY) {
                    s_last_mqtt_rc = -1;
                    s_disconnect_started_us = event.timestamp_us;
                    s_disconnect_timer_active = true;
                }

                if (mqtt_ready && event.data.network_state_change.from_level != SYS_NET_MQTT_READY) {
                    if (s_mqtt_ready_seen_once) {
                        s_mqtt_reconnects++;
                    }
                    s_mqtt_ready_seen_once = true;
                    s_last_mqtt_rc = 0;

                    if (s_disconnect_timer_active) {
                        int64_t disconnect_us = event.timestamp_us - s_disconnect_started_us;
                        if (disconnect_us < 0) {
                            disconnect_us = 0;
                        }
                        (void)mqtt_publisher_enqueue_int64(
                            mqtt_topic_id_t::TOPIC_SYSTEM_LAST_DISCONNECT_DURATION_S,
                            disconnect_us / 1000000LL);
                        s_disconnect_timer_active = false;
                    }
                }

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

                publish_runtime_diagnostics(network_snapshot);
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
                publish_runtime_diagnostics(network_snapshot);
                break;
            }
            case EVT_TICK:
                ESP_LOGD(TAG, "Tick event zatim neni implementovany");
                break;
            default:
                ESP_LOGW(TAG, "Neznamy event_type: %d", (int)event.event_type);
                break;
        }

        APP_ERROR_CHECK("E534", esp_task_wdt_reset());
    }
}

void state_manager_start(void)
{
    xTaskCreate(state_manager_task, TAG, configMINIMAL_STACK_SIZE * 5, NULL, 4, NULL);
}
