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
#include <math.h>

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
static constexpr uint32_t FLOW_ZERO_TIMEOUT_US = 3000000; // pokud bez impulsu, průtok = 0

static constexpr bool FLOW_SIMULATOR_ENABLED = true;
static constexpr uint32_t FLOW_SIMULATED_PULSE_WIDTH_US = 1000;
static constexpr const char *FLOW_SIMULATOR_PERIODIC_TIMER_NAME = "flow_sim_period";
static constexpr const char *FLOW_SIMULATOR_PULSE_OFF_TIMER_NAME = "flow_sim_off";
static constexpr TickType_t FLOW_SIMULATOR_PROFILE_UPDATE_PERIOD = pdMS_TO_TICKS(200);
static constexpr UBaseType_t FLOW_SIMULATOR_TASK_STACK_SIZE = 3072;

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
static esp_timer_handle_t s_flow_simulator_periodic_timer = nullptr;
static esp_timer_handle_t s_flow_simulator_pulse_off_timer = nullptr;
static uint32_t s_flow_simulator_pulse_width_us = FLOW_SIMULATED_PULSE_WIDTH_US;
static float s_flow_simulator_current_l_min = 0.0f;
static bool s_prutok_zaokrouhleny_valid = false;
static float s_prutok_zaokrouhleny = 0.0f;

typedef struct {
    const char *name;
    float start_l_min;
    float end_l_min;
    TickType_t duration_ticks;
} flow_simulator_profile_step_t;

static const flow_simulator_profile_step_t FLOW_SIMULATOR_PROFILE[] = {
    {.name = "stopped",        .start_l_min = 0.0f,  .end_l_min = 0.0f,  .duration_ticks = pdMS_TO_TICKS(3000)},
    {.name = "slow_rise",      .start_l_min = 0.0f,  .end_l_min = 8.0f,  .duration_ticks = pdMS_TO_TICKS(12000)},
    {.name = "fast_rise",      .start_l_min = 8.0f,  .end_l_min = 55.0f, .duration_ticks = pdMS_TO_TICKS(5000)},
    {.name = "hold_high",      .start_l_min = 55.0f, .end_l_min = 55.0f, .duration_ticks = pdMS_TO_TICKS(5000)},
    {.name = "slow_fall",      .start_l_min = 55.0f, .end_l_min = 18.0f, .duration_ticks = pdMS_TO_TICKS(10000)},
    {.name = "fast_stop",      .start_l_min = 18.0f, .end_l_min = 0.0f,  .duration_ticks = pdMS_TO_TICKS(4000)},
    {.name = "stopped_again",  .start_l_min = 0.0f,  .end_l_min = 0.0f,  .duration_ticks = pdMS_TO_TICKS(4000)},
    {.name = "quick_restart",  .start_l_min = 0.0f,  .end_l_min = 42.0f, .duration_ticks = pdMS_TO_TICKS(3000)},
    {.name = "hold_medium",    .start_l_min = 42.0f, .end_l_min = 42.0f, .duration_ticks = pdMS_TO_TICKS(5000)},
    {.name = "slow_taper",     .start_l_min = 42.0f, .end_l_min = 6.0f,  .duration_ticks = pdMS_TO_TICKS(8000)},
    {.name = "hold_low",       .start_l_min = 6.0f,  .end_l_min = 6.0f,  .duration_ticks = pdMS_TO_TICKS(5000)},
};

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

static void flow_pulse_simulator_off_callback(void *arg)
{
    (void)arg;
    gpio_set_level(FLOW_SIMULATOR_GPIO, 0);
}

static void flow_pulse_simulator_periodic_callback(void *arg)
{
    (void)arg;

    gpio_set_level(FLOW_SIMULATOR_GPIO, 1);
    esp_err_t pulse_off_result = esp_timer_start_once(s_flow_simulator_pulse_off_timer,
                                                      s_flow_simulator_pulse_width_us);
    if (pulse_off_result != ESP_OK) {
        gpio_set_level(FLOW_SIMULATOR_GPIO, 0);
    }
}

static esp_err_t stop_flow_simulator_timer(esp_timer_handle_t timer)
{
    if (timer == nullptr) {
        return ESP_OK;
    }

    const esp_err_t stop_result = esp_timer_stop(timer);
    if (stop_result == ESP_OK || stop_result == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }
    return stop_result;
}

static esp_err_t nastav_flow_simulator_prutok(float flow_l_min)
{
    APP_ERROR_CHECK("E718", stop_flow_simulator_timer(s_flow_simulator_periodic_timer));
    APP_ERROR_CHECK("E719", stop_flow_simulator_timer(s_flow_simulator_pulse_off_timer));
    APP_ERROR_CHECK("E720", gpio_set_level(FLOW_SIMULATOR_GPIO, 0));

    s_flow_simulator_current_l_min = flow_l_min;

    if (flow_l_min <= 0.0f || s_flow_pulses_per_liter == 0) {
        return ESP_OK;
    }

    const float pulses_per_second =
        (flow_l_min * static_cast<float>(s_flow_pulses_per_liter)) / 60.0f;
    if (pulses_per_second <= 0.0f) {
        return ESP_OK;
    }

    const int64_t period_us = static_cast<int64_t>(1000000.0f / pulses_per_second);
    const int64_t pulse_width_us =
        (period_us > static_cast<int64_t>(FLOW_SIMULATED_PULSE_WIDTH_US))
            ? static_cast<int64_t>(FLOW_SIMULATED_PULSE_WIDTH_US)
            : (period_us / 2);
    if (period_us <= 0 || pulse_width_us <= 0 || pulse_width_us >= period_us) {
        return ESP_ERR_INVALID_ARG;
    }
    s_flow_simulator_pulse_width_us = static_cast<uint32_t>(pulse_width_us);
    return esp_timer_start_periodic(s_flow_simulator_periodic_timer, period_us);
}

static esp_err_t start_flow_pulse_simulator(void)
{
    if (s_flow_pulses_per_liter == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_timer_create_args_t pulse_off_timer_args = {
        .callback = &flow_pulse_simulator_off_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = FLOW_SIMULATOR_PULSE_OFF_TIMER_NAME,
        .skip_unhandled_events = true,
    };
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &flow_pulse_simulator_periodic_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = FLOW_SIMULATOR_PERIODIC_TIMER_NAME,
        .skip_unhandled_events = true,
    };

    esp_err_t create_result = esp_timer_create(&pulse_off_timer_args, &s_flow_simulator_pulse_off_timer);
    if (create_result != ESP_OK) {
        return create_result;
    }

    create_result = esp_timer_create(&periodic_timer_args, &s_flow_simulator_periodic_timer);
    if (create_result != ESP_OK) {
        return create_result;
    }

    gpio_set_level(FLOW_SIMULATOR_GPIO, 0);
    return nastav_flow_simulator_prutok(0.0f);
}

static void flow_pulse_simulator_task(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        for (size_t step_index = 0; step_index < (sizeof(FLOW_SIMULATOR_PROFILE) / sizeof(FLOW_SIMULATOR_PROFILE[0])); ++step_index) {
            const flow_simulator_profile_step_t &step = FLOW_SIMULATOR_PROFILE[step_index];
            const TickType_t update_ticks =
                (FLOW_SIMULATOR_PROFILE_UPDATE_PERIOD > 0) ? FLOW_SIMULATOR_PROFILE_UPDATE_PERIOD : 1;
            const TickType_t duration_ticks = (step.duration_ticks > 0) ? step.duration_ticks : update_ticks;
            const uint32_t updates = (uint32_t)((duration_ticks + update_ticks - 1) / update_ticks);

            for (uint32_t update_index = 0; update_index < updates; ++update_index) {
                const float progress =
                    (updates <= 1) ? 1.0f : (float)update_index / (float)(updates - 1U);
                const float flow_l_min =
                    step.start_l_min + ((step.end_l_min - step.start_l_min) * progress);

                APP_ERROR_CHECK("E721", nastav_flow_simulator_prutok(flow_l_min));
                ESP_LOGI(TAG,
                         "Flow simulator step=%s setpoint=%.2f l/min",
                         step.name,
                         (double)flow_l_min);
                vTaskDelay(update_ticks);
            }
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

static float zaokrouhli_prutok_s_hysterezi(float prutok)
{
    if (!(prutok >= 0.0f)) {
        s_prutok_zaokrouhleny_valid = false;
        s_prutok_zaokrouhleny = 0.0f;
        return 0.0f;
    }

    const float krok = (prutok >= 10.0f) ? 1.0f : 0.1f;
    const float scaled = prutok / krok;
    const float lower = floorf(scaled);
    const float upper = lower + 1.0f;
    const float discarded_digit = (scaled - lower) * 10.0f;

    float rounded_scaled = lower;
    if (discarded_digit >= 7.0f) {
        rounded_scaled = upper;
    } else if (discarded_digit < 3.0f) {
        rounded_scaled = lower;
    } else if (s_prutok_zaokrouhleny_valid) {
        const float previous_scaled = s_prutok_zaokrouhleny / krok;
        if (fabsf(previous_scaled - upper) < 0.25f) {
            rounded_scaled = upper;
        } else if (fabsf(previous_scaled - lower) < 0.25f) {
            rounded_scaled = lower;
        } else {
            rounded_scaled = (scaled >= (lower + 0.5f)) ? upper : lower;
        }
    } else {
        rounded_scaled = (scaled >= (lower + 0.5f)) ? upper : lower;
    }

    s_prutok_zaokrouhleny = rounded_scaled * krok;
    s_prutok_zaokrouhleny_valid = true;
    return s_prutok_zaokrouhleny;
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
        const float publikovany_prutok = zaokrouhli_prutok_s_hysterezi(surovy_prutok);

        // Počet pulsů od poslední vzorky
        const uint32_t sampled_pulses = get_and_clear_pulse_count();
        const float cerpano_celkem = zpracuj_cerpano_celkem(sampled_pulses);

        sample_counter += 1;
        if (sample_counter >= FLOW_LOG_EVERY_N_SAMPLES) {
            sample_counter = 0;
            ESP_LOGD(TAG,
                     "Prutok raw=%.3f rounded=%.3f l/min, celkem=%.2f l",
                     surovy_prutok,
                     publikovany_prutok,
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
                            .prutok = publikovany_prutok,
                            .cerpano_celkem = cerpano_celkem,
                        },
                    },
                },
            },
        };

        bool queued = sensor_events_publish(&event, pdMS_TO_TICKS(20));

        DEBUG_PUBLISH("prutok",
                      "queued=%d ts=%lld sampled_pulses=%lu raw_l_min=%.4f rounded_l_min=%.4f total_l=%.4f persisted_steps=%llu",
                      queued ? 1 : 0,
                      (long long)now_us,
                      (unsigned long)sampled_pulses,
                      (double)surovy_prutok,
                      (double)publikovany_prutok,
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
        gpio_config_t sim_io_conf = {
            .pin_bit_mask = 1ULL << FLOW_SIMULATOR_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        APP_ERROR_CHECK("E716", gpio_config(&sim_io_conf));
        APP_ERROR_CHECK("E717", gpio_set_level(FLOW_SIMULATOR_GPIO, 0));

        ESP_LOGW(TAG,
                 "Flow simulator ENABLED: profile_mode=1, out_pin=%d, pulse_width_us=%lu, update_period_ms=%lu",
                 (int)FLOW_SIMULATOR_GPIO,
                 (unsigned long)FLOW_SIMULATED_PULSE_WIDTH_US,
                 (unsigned long)pdTICKS_TO_MS(FLOW_SIMULATOR_PROFILE_UPDATE_PERIOD));
        APP_ERROR_CHECK("E715", start_flow_pulse_simulator());
        APP_ERROR_CHECK("E722",
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
