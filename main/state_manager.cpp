#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "esp_log.h"

#ifdef __cplusplus
}
#endif

#include <stdio.h>

#include "state_manager.h"
#include "sensor_events.h"
#include "lcd.h"
#include "mqtt_publisher_task.h"
#include "webapp_startup.h"
#include "status_display.h"
#include <tm1637.h>

static const char *TAG = "state";

static void publish_temperature_to_outputs(const sensor_event_t &event)
{
    char text[16];
    snprintf(text, sizeof(text), "T:%4.1f ", event.data.temperature.temperature_c);
    lcd_print(8, 0, text, false, 0);

    esp_err_t enqueue_result = mqtt_publisher_enqueue_double(
        mqtt_topic_id_t::TOPIC_STAV_TEPLOTA_VODA,
        (double)event.data.temperature.temperature_c);
    if (enqueue_result != ESP_OK) {
        ESP_LOGW(TAG, "Enqueue teploty vody selhalo: %s", esp_err_to_name(enqueue_result));
    }
}

static void publish_level_to_outputs(const sensor_event_t &event)
{
    char text[16];
    snprintf(text, sizeof(text), "H:%3.0fcm ", event.data.level.height_m * 100.0f);
    lcd_print(8, 1, text, false, 0);
}

static void publish_flow_to_outputs(const sensor_event_t &event)
{
    char liters_text[16];
    snprintf(liters_text, sizeof(liters_text), "L:%5.1f ", event.data.flow.total_volume_l);
    lcd_print(0, 0, liters_text, false, 0);

    char flow_text[16];
    snprintf(flow_text, sizeof(flow_text), "Q:%4.1f ", event.data.flow.flow_l_min);
    lcd_print(0, 1, flow_text, false, 0);

}

static void state_manager_task(void *pvParameters)
{
    (void)pvParameters;

    app_event_t event = {};
    char debug_line[128];
    bool mqtt_ready_published = false;

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
                        float total = event.data.sensor.data.flow.total_volume_l * 50.0f;
                        int ktery = static_cast<int>(total) % 3;
                        if ((int)total % 2) {
                            set_segments(TM1637_SEG_A, ktery, true);
                        } else {
                            set_segments(TM1637_SEG_A, ktery, false);
                        }
                        

                        break;
                    }
                    default:
                        ESP_LOGW(TAG, "Neznamy sensor event: %d", (int)event.data.sensor.sensor_type);
                        break;
                }
                break;
            case EVT_NETWORK:
                ESP_LOGW(TAG,
                         "Network level=%d rssi=%d ip=0x%08lx reconn_attempts=%lu reconn_success=%lu",
                         (int)event.data.network.level,
                         (int)event.data.network.last_rssi,
                         (unsigned long)event.data.network.ip_addr,
                         (unsigned long)event.data.network.reconnect_attempts,
                         (unsigned long)event.data.network.reconnect_successes);

                status_display_set_network_state(&event.data.network);
                webapp_startup_on_network_event(&event.data.network);

                if (event.data.network.level == SYS_NET_MQTT_READY) {
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
                }
                break;
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
