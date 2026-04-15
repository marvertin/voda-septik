#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"

#ifdef __cplusplus
}
#endif

#include "pins.h"
#include "sensor_events.h"
#include "flash_monotonic_counter.h"
#include "config_store.h"
#include "app_error_check.h"
#include "debug_mqtt.h"

#define TAG "prutokomer"

static constexpr int32_t FLOW_DEFAULT_PULSES_PER_LITER = 38; // F = 4.5 * Q, Q v l/min
static constexpr int32_t FLOW_MIN_PULSES_PER_LITER = 1;
static constexpr int32_t FLOW_MAX_PULSES_PER_LITER = 200;
static constexpr uint32_t COUNTER_INCREMENT_LITERS = 1; // Kolik litrů odpovídá jednomu kroku v monotonic counteru
static constexpr TickType_t FLOW_SAMPLE_PERIOD = pdMS_TO_TICKS(200);
static constexpr UBaseType_t FLOW_TASK_STACK_SIZE = 4096;
static constexpr uint8_t FLOW_LOG_EVERY_N_SAMPLES = 5;
static constexpr uint32_t FLOW_PULSE_TIMESTAMPS_BUFFER_SIZE = 128;
static constexpr int64_t FLOW_TARGET_WINDOW_US = 3000000; // cílové okno historie pro odhad průtoku
static constexpr uint32_t FLOW_MIN_INTERVALS_FOR_ESTIMATE = 4;
static constexpr uint32_t FLOW_ZERO_TIMEOUT_US = 30000000; // pokud bez impulsu, průtok = 0

static constexpr bool FLOW_SIMULATOR_ENABLED = false;
static constexpr float FLOW_SIMULATED_LITERS_PER_MIN = 60.0f;
static constexpr TickType_t FLOW_SIMULATOR_PERIOD = pdMS_TO_TICKS(100);
static constexpr UBaseType_t FLOW_SIMULATOR_TASK_STACK_SIZE = 2048;

static const config_item_t FLOW_PULSES_PER_LITER_ITEM = {
    .key = "flow_pulses_l", .label = "Prutokomer pulsy na litr", .description = "Kalibracni konstanta prutokomeru: kolik impulsu odpovida jednomu litru.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = FLOW_DEFAULT_PULSES_PER_LITER, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = FLOW_MIN_PULSES_PER_LITER, .max_int = FLOW_MAX_PULSES_PER_LITER, .min_float = 0.0f, .max_float = 0.0f,
};

static const char *FLOW_COUNTER_PARTITION_LABEL = "flow_data0";

// Kruhový buffer posledních N časů impulsů pro výpočet průtoku
static int64_t s_pulse_timestamps[FLOW_PULSE_TIMESTAMPS_BUFFER_SIZE];
static volatile uint32_t s_pulse_write_idx = 0;

// sdílený counter z ISR
static volatile uint32_t pulse_count = 0;
static portMUX_TYPE s_pulse_count_mux = portMUX_INITIALIZER_UNLOCKED;
static FlashMonotonicCounter s_flow_counter;
static uint64_t s_persisted_counter_steps = 0;
static uint64_t s_unpersisted_pulses = 0;
static uint32_t s_flow_pulses_per_liter = static_cast<uint32_t>(FLOW_DEFAULT_PULSES_PER_LITER);
static uint32_t s_pulses_per_counter_increment =
    static_cast<uint32_t>(FLOW_DEFAULT_PULSES_PER_LITER) * COUNTER_INCREMENT_LITERS;

void prutokomer_register_config_items(void)
{
    APP_ERROR_CHECK("E706", config_store_register_item(&FLOW_PULSES_PER_LITER_ITEM));
}

static void load_flow_config(void)
{
    const int32_t configured = config_store_get_i32_item(&FLOW_PULSES_PER_LITER_ITEM);
    const int32_t clamped =
        (configured < FLOW_MIN_PULSES_PER_LITER)
            ? FLOW_MIN_PULSES_PER_LITER
            : ((configured > FLOW_MAX_PULSES_PER_LITER)
                   ? FLOW_MAX_PULSES_PER_LITER
                   : configured);

    s_flow_pulses_per_liter = static_cast<uint32_t>(clamped);
    s_pulses_per_counter_increment = s_flow_pulses_per_liter * COUNTER_INCREMENT_LITERS;
}

// ISR handler - počítá pulsy a ukládá časy do bufferu
static void IRAM_ATTR flow_isr_handler(void *arg) {
    (void)arg;
    int64_t timestamp = esp_timer_get_time();
    
    // Počítat pulsy pro celkový objem
    portENTER_CRITICAL_ISR(&s_pulse_count_mux);
    pulse_count += 1;
    portEXIT_CRITICAL_ISR(&s_pulse_count_mux);

    s_pulse_timestamps[s_pulse_write_idx] = timestamp;
    s_pulse_write_idx = (s_pulse_write_idx + 1) % FLOW_PULSE_TIMESTAMPS_BUFFER_SIZE;
}

static uint32_t get_and_clear_pulse_count(void)
{
    uint32_t count = 0;
    portENTER_CRITICAL(&s_pulse_count_mux);
    count = pulse_count;
    pulse_count = 0;
    portEXIT_CRITICAL(&s_pulse_count_mux);
    return count;
}

static void flow_pulse_simulator_task(void *pvParameters)
{
    (void)pvParameters;

    const float pulses_per_period =
        (FLOW_SIMULATED_LITERS_PER_MIN * static_cast<float>(s_flow_pulses_per_liter)
         * static_cast<float>(pdTICKS_TO_MS(FLOW_SIMULATOR_PERIOD)))
        / 60000.0f;
    float pulse_accumulator = 0.0f;

    while (1) {
        vTaskDelay(FLOW_SIMULATOR_PERIOD);

        pulse_accumulator += pulses_per_period;
        const uint32_t pulses_to_add = static_cast<uint32_t>(pulse_accumulator);
        if (pulses_to_add == 0) {
            continue;
        }
        pulse_accumulator -= static_cast<float>(pulses_to_add);

         int64_t timestamp = esp_timer_get_time();
         
         // Přidat do počítadla pulsů
         portENTER_CRITICAL(&s_pulse_count_mux);
         pulse_count = pulse_count + pulses_to_add;
         portEXIT_CRITICAL(&s_pulse_count_mux);

         for (uint32_t i = 0; i < pulses_to_add; i++) {
             s_pulse_timestamps[s_pulse_write_idx] = timestamp;
             s_pulse_write_idx = (s_pulse_write_idx + 1) % FLOW_PULSE_TIMESTAMPS_BUFFER_SIZE;
             timestamp = timestamp + 1000; // Rozestup mezi simulovanými pulsy
         }
    }
}

static float vypocitej_surovy_prutok(int64_t now_us)
{
    const uint32_t write_idx = s_pulse_write_idx;
    int64_t last_pulse_time = 0;
    int64_t first_pulse_time = 0;
    uint32_t intervals = 0;

    const uint32_t last_idx =
        (write_idx + FLOW_PULSE_TIMESTAMPS_BUFFER_SIZE - 1U) % FLOW_PULSE_TIMESTAMPS_BUFFER_SIZE;
    last_pulse_time = s_pulse_timestamps[last_idx];

    if (last_pulse_time <= 0) {
        return 0.0f;
    }

    const int64_t time_since_last_pulse = now_us - last_pulse_time;
    if (time_since_last_pulse > FLOW_ZERO_TIMEOUT_US) {
        return 0.0f;
    }

    first_pulse_time = last_pulse_time;

    for (uint32_t step = 1; step < FLOW_PULSE_TIMESTAMPS_BUFFER_SIZE; ++step) {
        const uint32_t idx =
            (last_idx + FLOW_PULSE_TIMESTAMPS_BUFFER_SIZE - step) % FLOW_PULSE_TIMESTAMPS_BUFFER_SIZE;
        const int64_t candidate_time = s_pulse_timestamps[idx];

        if (candidate_time <= 0 || candidate_time > first_pulse_time) {
            break;
        }

        first_pulse_time = candidate_time;
        intervals = step;

        const int64_t time_span_us = last_pulse_time - first_pulse_time;
        if (time_span_us >= FLOW_TARGET_WINDOW_US && intervals >= FLOW_MIN_INTERVALS_FOR_ESTIMATE) {
            break;
        }
    }

    const int64_t time_span_us = last_pulse_time - first_pulse_time;
    if (time_span_us <= 0 || intervals == 0) {
        return 0.0f;
    }

    return (static_cast<float>(intervals) * 60000000.0f)
         / (static_cast<float>(time_span_us) * static_cast<float>(s_flow_pulses_per_liter));
}

static float zpracuj_cerpano_celkem(uint32_t sampled_pulses)
{
    if (sampled_pulses > 0) {
        s_unpersisted_pulses += static_cast<uint64_t>(sampled_pulses);

        while (s_unpersisted_pulses >= static_cast<uint64_t>(s_pulses_per_counter_increment)) {
            ESP_LOGD(TAG,
                     "Persistuji krok flow counteru: %llu -> %llu (batch_pulses=%llu)",
                     (unsigned long long)s_persisted_counter_steps,
                     (unsigned long long)(s_persisted_counter_steps + 1),
                     (unsigned long long)s_unpersisted_pulses);
            const esp_err_t increment_result = s_flow_counter.increment(1);
            if (increment_result != ESP_OK) {
                ESP_LOGE(TAG, "Nelze zapsat flow counter: %s", esp_err_to_name(increment_result));
                break;
            }
            s_persisted_counter_steps += 1;
            s_unpersisted_pulses -= static_cast<uint64_t>(s_pulses_per_counter_increment);
        }
    }

    return static_cast<float>(s_persisted_counter_steps * COUNTER_INCREMENT_LITERS)
         + (static_cast<float>(s_unpersisted_pulses) / static_cast<float>(s_flow_pulses_per_liter));
}

static void pocitani_pulsu(void *pvParameters)
{
    (void)pvParameters;
    APP_ERROR_CHECK("E707", esp_task_wdt_add(nullptr));

    uint8_t sample_counter = 0;

    while(1) {
        vTaskDelay(FLOW_SAMPLE_PERIOD);

        const int64_t now_us = esp_timer_get_time();

        const float surovy_prutok = vypocitej_surovy_prutok(now_us);

        // Počet pulsů od poslední vzorky
        const uint32_t sampled_pulses = get_and_clear_pulse_count();
        const float cerpano_celkem = zpracuj_cerpano_celkem(sampled_pulses);

        sample_counter += 1;
        if (sample_counter >= FLOW_LOG_EVERY_N_SAMPLES) {
            sample_counter = 0;
            ESP_LOGD(TAG,
                     "Prutok=%.2f l/min, celkem=%.2f l",
                     surovy_prutok,
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
                            .prutok = surovy_prutok,
                            .cerpano_celkem = cerpano_celkem,
                        },
                    },
                },
            },
        };

        bool queued = sensor_events_publish(&event, pdMS_TO_TICKS(20));

        DEBUG_PUBLISH("prutok",
                      "queued=%d ts=%lld sampled_pulses=%lu flow_l_min=%.4f total_l=%.4f persisted_steps=%llu",
                      queued ? 1 : 0,
                      (long long)now_us,
                      (unsigned long)sampled_pulses,
                      (double)surovy_prutok,
                      (double)cerpano_celkem,
                      (unsigned long long)s_persisted_counter_steps);

        APP_ERROR_CHECK("E708", esp_task_wdt_reset());
    }
}

void prutokomer_init(void)
{
    load_flow_config();

    ESP_LOGI(TAG,
             "Init flow: gpio=%d pulses_per_l=%lu sample_period_ms=%lu target_window_ms=%lld",
             (int)FLOW_SENSOR_GPIO,
             (unsigned long)s_flow_pulses_per_liter,
             (unsigned long)pdTICKS_TO_MS(FLOW_SAMPLE_PERIOD),
             (long long)(FLOW_TARGET_WINDOW_US / 1000));

    APP_ERROR_CHECK("E709", s_flow_counter.init(FLOW_COUNTER_PARTITION_LABEL));
    // APP_ERROR_CHECK("E710", s_flow_counter.reset());

    s_persisted_counter_steps = s_flow_counter.value();
    s_unpersisted_pulses = 0;
    
    ESP_LOGI(TAG,
             "Flow counter inicializovan, kroky=%llu, objem=%llu l, pulses_per_l=%lu",
             (unsigned long long)s_persisted_counter_steps,
             (unsigned long long)(s_persisted_counter_steps * COUNTER_INCREMENT_LITERS),
             (unsigned long)s_flow_pulses_per_liter);
    //TODO Někam to nastavit ..

    // --- Nastavení GPIO pro flow senzor ---
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << FLOW_SENSOR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    APP_ERROR_CHECK("E711", gpio_config(&io_conf));

    APP_ERROR_CHECK("E712", gpio_install_isr_service(0));
    APP_ERROR_CHECK("E713", gpio_isr_handler_add(FLOW_SENSOR_GPIO, flow_isr_handler, NULL));

    ESP_LOGI(TAG,
             "GPIO flow nastaven: pullup=1 pulldown=0 intr=posedge pin=%d",
             (int)FLOW_SENSOR_GPIO);

    if (FLOW_SIMULATOR_ENABLED) {
        ESP_LOGW(TAG,
                 "Flow simulator ENABLED: %.2f l/min, period=%lu ms",
                 (double)FLOW_SIMULATED_LITERS_PER_MIN,
                 (unsigned long)pdTICKS_TO_MS(FLOW_SIMULATOR_PERIOD));
        APP_ERROR_CHECK("E715",
                        xTaskCreate(flow_pulse_simulator_task,
                                    "flow_sim",
                                    FLOW_SIMULATOR_TASK_STACK_SIZE,
                                    NULL,
                                    1,
                                    NULL) == pdPASS
                            ? ESP_OK
                            : ESP_FAIL);
    }

    ESP_LOGI(TAG, "Startuji mereni pulzu...");

    APP_ERROR_CHECK("E714",
                    xTaskCreate(pocitani_pulsu, "pocitani_pulsu", FLOW_TASK_STACK_SIZE, NULL, 1, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_FAIL);
}
