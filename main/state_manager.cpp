#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "tm1637.h"
#include "esp_log.h"

#ifdef __cplusplus
}
#endif

#include <stdio.h>

#include "state_manager.h"
#include "sensor_events.h"
#include "lcd.h"
#include "mqtt_publish.h"
#include "pins.h"

static const char *TAG = "STATE_MANAGER";

static tm1637_config_t s_tm1637_config = {
    .clk_pin = GPIO_NUM_18,
    .dio_pin = GPIO_NUM_19,
    .bit_delay_us = 100,
};

static tm1637_handle_t s_tm1637_display = nullptr;

static void publish_temperature_to_outputs(const sensor_event_t &event)
{
    char text[16];
    snprintf(text, sizeof(text), "T:%4.1f ", event.data.temperature.temperature_c);
    lcd_print(8, 0, text, false, 0);

    if (mqtt_is_connected()) {
        char payload[32];
        snprintf(payload, sizeof(payload), "%.2f", event.data.temperature.temperature_c);
        mqtt_publish("homeassistant/sensor/zalevaci_nadrz/temperature/state", payload, true);
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

    if (s_tm1637_display != nullptr) {
        tm1637_show_number(s_tm1637_display, (int)event.data.flow.total_volume_l, false, 4, 0);
    }
}

static void state_manager_task(void *pvParameters)
{
    app_event_t event = {};
    char debug_line[128];

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
                    case SENSOR_EVENT_FLOW:
                        publish_flow_to_outputs(event.data.sensor);
                        break;
                    default:
                        ESP_LOGW(TAG, "Neznamy sensor event: %d", (int)event.data.sensor.sensor_type);
                        break;
                }
                break;
            case EVT_NETWORK:
                ESP_LOGW(TAG,
                         "Network level=%d rssi=%d ip=0x%08lx",
                         (int)event.data.network.level,
                         (int)event.data.network.last_rssi,
                         (unsigned long)event.data.network.ip_addr);
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
    tm1637_init(&s_tm1637_config, &s_tm1637_display);
    xTaskCreate(state_manager_task, TAG, configMINIMAL_STACK_SIZE * 5, NULL, 4, NULL);
}
