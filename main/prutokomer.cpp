#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#ifdef __cplusplus
}
#endif

#include "pins.h"
#include "sensor_events.h"
#include "flash_monotonic_counter.h"
#include "app_error_check.h"
#include "debug_mqtt.h"

#define TAG "prutokomer"

static constexpr uint32_t FLOW_PULSES_PER_LITER = 270; // F = 4.5 * Q, Q v l/min
static constexpr uint32_t COUNTER_INCREMENT_LITERS = 1; // Kolik litrů odpovídá jednomu kroku v monotonic counteru
static constexpr uint32_t PULSES_PER_COUNTER_INCREMENT = FLOW_PULSES_PER_LITER * COUNTER_INCREMENT_LITERS;
static constexpr TickType_t FLOW_SAMPLE_PERIOD = pdMS_TO_TICKS(200);
static constexpr float FLOW_EMA_ALPHA = 0.25f;
static constexpr UBaseType_t FLOW_TASK_STACK_SIZE = 4096;
static constexpr uint8_t FLOW_LOG_EVERY_N_SAMPLES = 5;
static const char *FLOW_COUNTER_PARTITION_LABEL = "flow_data0";

// sdílený counter z ISR
static volatile uint32_t pulse_count = 0;
static FlashMonotonicCounter s_flow_counter;
static uint64_t s_total_pulses = 0;
static uint64_t s_persisted_counter_steps = 0;
static float s_prutok_ema = 0.0f;
static bool s_prutok_inicializovan = false;

// ISR handler
static void IRAM_ATTR flow_isr_handler(void *arg) {
    pulse_count += 1;
}

static void pocitani_pulsu(void *pvParameters)
{
    uint32_t previous_pulse_count = pulse_count;
    int64_t previous_sample_us = esp_timer_get_time();
    uint8_t sample_counter = 0;

    while(1) {
        vTaskDelay(FLOW_SAMPLE_PERIOD);

        const int64_t now_us = esp_timer_get_time();
        const int64_t elapsed_us = now_us - previous_sample_us;
        previous_sample_us = now_us;

        const uint32_t current_pulse_count = pulse_count;
        const uint32_t new_pulses = current_pulse_count - previous_pulse_count;
        previous_pulse_count = current_pulse_count;

        s_total_pulses += new_pulses;
//        ESP_LOGI(TAG, "Nové pulzy: %lu, Celkem pulzů: %llu, Elapsed: %lld us",
  //               new_pulses,
    //             (unsigned long long)s_total_pulses,
      //           (long long)elapsed_us);

        const uint64_t target_persisted_steps = s_total_pulses / PULSES_PER_COUNTER_INCREMENT;
        while (s_persisted_counter_steps < target_persisted_steps) {
            const esp_err_t increment_result = s_flow_counter.increment(1);
            if (increment_result != ESP_OK) {
                ESP_LOGE(TAG, "Nelze zapsat flow counter: %s", esp_err_to_name(increment_result));
                break;
            }
            s_persisted_counter_steps += 1;
        }

        float surovy_prutok = 0.0f;
        if (elapsed_us > 0) {
            surovy_prutok = (static_cast<float>(new_pulses) * 60000000.0f)
                          / (static_cast<float>(elapsed_us) * static_cast<float>(FLOW_PULSES_PER_LITER));
        }

        if (!s_prutok_inicializovan) {
            s_prutok_ema = surovy_prutok;
            s_prutok_inicializovan = true;
        } else {
            s_prutok_ema = FLOW_EMA_ALPHA * surovy_prutok
                         + (1.0f - FLOW_EMA_ALPHA) * s_prutok_ema;
        }

        const float cerpano_celkem =
            static_cast<float>(s_total_pulses) / static_cast<float>(FLOW_PULSES_PER_LITER);

        sample_counter += 1;
        if (sample_counter >= FLOW_LOG_EVERY_N_SAMPLES) {
            sample_counter = 0;
            ESP_LOGI(TAG,
                     "Prutok raw=%.2f l/min, ema=%.2f l/min, celkem=%.2f l",
                     surovy_prutok,
                     s_prutok_ema,
                     cerpano_celkem);
        }

        app_event_t event = {
            .event_type = EVT_SENSOR,
            .timestamp_us = esp_timer_get_time(),
            .data = {
                .sensor = {
                    .sensor_type = SENSOR_EVENT_FLOW,
                    .data = {
                        .flow = {
                            .prutok = s_prutok_ema,
                            .cerpano_celkem = cerpano_celkem,
                        },
                    },
                },
            },
        };

        bool queued = sensor_events_publish(&event, pdMS_TO_TICKS(20));
        if (!queued) {
            ESP_LOGW(TAG, "Fronta sensor eventu je plna, prutok zahozen");
        }

        DEBUG_PUBLISH("prutok",
                      "queued=%d ts=%lld new_pulses=%lu elapsed_us=%lld raw_l_min=%.4f ema_l_min=%.4f total_l=%.4f persisted_steps=%llu",
                      queued ? 1 : 0,
                      (long long)now_us,
                      (unsigned long)new_pulses,
                      (long long)elapsed_us,
                      (double)surovy_prutok,
                      (double)s_prutok_ema,
                      (double)cerpano_celkem,
                      (unsigned long long)s_persisted_counter_steps);
    }
}

void prutokomer_init(void)
{


    APP_ERROR_CHECK("E200", s_flow_counter.init(FLOW_COUNTER_PARTITION_LABEL));

    // APP_ERROR_CHECK("E201", s_flow_counter.reset());

    s_persisted_counter_steps = s_flow_counter.value();
    s_total_pulses = s_persisted_counter_steps * static_cast<uint64_t>(PULSES_PER_COUNTER_INCREMENT);
    
    ESP_LOGW(TAG,
             "Flow counter inicializovan, kroky=%llu, start_pulsy=%llu, objem=%llu l",
             (unsigned long long)s_persisted_counter_steps,
             (unsigned long long)s_total_pulses,
             (unsigned long long)(s_persisted_counter_steps * COUNTER_INCREMENT_LITERS));
    //TODO Někam to nastavit ..

    // --- Nastavení GPIO pro flow senzor ---
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << FLOW_SENSOR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    APP_ERROR_CHECK("E525", gpio_config(&io_conf));

    APP_ERROR_CHECK("E526", gpio_install_isr_service(0));
    APP_ERROR_CHECK("E527", gpio_isr_handler_add(FLOW_SENSOR_GPIO, flow_isr_handler, NULL));

    ESP_LOGI(TAG, "Startuji měření pulzů...");

    APP_ERROR_CHECK("E528",
                    xTaskCreate(pocitani_pulsu, "pocitani_pulsu", FLOW_TASK_STACK_SIZE, NULL, 1, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_FAIL);
}
