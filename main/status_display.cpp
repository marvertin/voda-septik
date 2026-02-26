#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "tm1637.h"
#include "esp_log.h"

#ifdef __cplusplus
}
#endif

#include "status_display.h"
#include "tm1637_startup_animation.h"

#include "pins.h"
#include "app_error_check.h"

static const char *TAG = "status_display";
static constexpr TickType_t STATUS_DISPLAY_TASK_PERIOD = pdMS_TO_TICKS(100);
static constexpr uint32_t MQTT_ACTIVITY_COLON_ON_MS = 200;
static constexpr uint32_t MQTT_ACTIVITY_COLON_OFF_MS = 30;
static constexpr uint32_t BRIGHTNESS_ALERT_HOLD_MS = 2000;
static constexpr uint8_t BRIGHTNESS_LEVEL_LOW = 1;
static constexpr uint8_t BRIGHTNESS_LEVEL_HIGH = 7;

static tm1637_config_t s_tm1637_config = {
    .clk_pin = TM_CLK,
    .dio_pin = TM_DIO,
    .bit_delay_us = 100,
};

static tm1637_handle_t s_tm1637_display = nullptr;
static bool s_display_latched = false;

static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static system_network_level_t s_network_level = SYS_NET_DOWN;

static bool s_mqtt_activity_timer_running = false;
static float s_flow_l_min = 0.0f;
static uint8_t s_current_brightness_level = BRIGHTNESS_LEVEL_HIGH;

static constexpr uint8_t SENSOR_FAULT_TEMP_POS = 2;
static constexpr uint8_t SENSOR_FAULT_LEVEL_POS = 2;
static constexpr uint8_t SENSOR_FAULT_FLOW_POS = 3;

static constexpr uint8_t SENSOR_FAULT_TEMP_SEGMENTS = static_cast<uint8_t>(TM1637_SEG_B | TM1637_SEG_C);
static constexpr uint8_t SENSOR_FAULT_LEVEL_SEGMENTS = static_cast<uint8_t>(TM1637_SEG_E | TM1637_SEG_F);
static constexpr uint8_t SENSOR_FAULT_FLOW_SEGMENTS = static_cast<uint8_t>(TM1637_SEG_D | TM1637_SEG_G);

StaticTimer_t s_mqtt_activity_colon_on_timer_buffer;
StaticTimer_t s_mqtt_activity_colon_off_timer_buffer;
TimerHandle_t s_mqtt_activity_colon_on_timer = nullptr; 
TimerHandle_t s_mqtt_activity_colon_off_timer = nullptr;
StaticTimer_t s_brightness_alert_hold_timer_buffer;
TimerHandle_t s_brightness_alert_hold_timer = nullptr;

//
static constexpr struct {
    uint32_t on_ms;
    uint32_t off_ms;
    uint32_t blink_count;
    uint32_t gap_ms;
} s_network_level_display_config[] = {
    [SYS_NET_DOWN]        = { 80,  80,  3, 200},
    [SYS_NET_WIFI_ONLY]   = {200,  80,  2, 300},
    [SYS_NET_IP_ONLY]     = {300, 100,  1, 500},
    [SYS_NET_MQTT_READY]  = {200,  0,  0, 0},
    [SYS_NET_AP_CONFIG]   = {400,  200, 20, 500},
};

static uint8_t svitici_segmenty[] = { 0, 0, 0, 0 };

static constexpr float FLOW_SPINNER_START_THRESHOLD_L_MIN = 0.05f;
static constexpr float FLOW_SPINNER_SPEED_MIN_FLOW_L_MIN = 0.20f;
static constexpr float FLOW_SPINNER_SPEED_MAX_FLOW_L_MIN = 30.0f;
static constexpr uint32_t FLOW_SPINNER_MIN_PERIOD_MS = 80;
static constexpr uint32_t FLOW_SPINNER_MAX_PERIOD_MS = 450;
static constexpr TickType_t FLOW_SPINNER_IDLE_POLL_DELAY = pdMS_TO_TICKS(200);

static constexpr uint8_t FLOW_SPINNER_SEGMENT_MASK_POS0 = static_cast<uint8_t>(
    TM1637_SEG_A | TM1637_SEG_D | TM1637_SEG_E | TM1637_SEG_F);
static constexpr uint8_t FLOW_SPINNER_SEGMENT_MASK_POS1 = static_cast<uint8_t>(
    TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_D);

struct flow_spinner_frame_t {
    uint8_t seg_pos0;
    uint8_t seg_pos1;
};

static constexpr flow_spinner_frame_t FLOW_SPINNER_FRAMES[] = {
    {TM1637_SEG_A, 0},
    {0, TM1637_SEG_A},
    {0, TM1637_SEG_B},
    {0, TM1637_SEG_C},
    {0, TM1637_SEG_D},
    {TM1637_SEG_D, 0},
    {TM1637_SEG_E, 0},
    {TM1637_SEG_F, 0},
};

static uint8_t s_flow_spinner_frame_index = 0;
static TickType_t s_flow_spinner_next_tick = 0;
static bool s_flow_spinner_active = false;

static void set_max_briteness_for_some_time(void);

static void set_error_led(bool on) {
    gpio_set_level(ERRORLED_PIN, on ? 1 : 0);
}


static void apply_brightness(uint8_t level)
{
    if (s_current_brightness_level == level) {
        return;
    }
    tm1637_set_brightness(s_tm1637_display, level, true);
    s_current_brightness_level = level;
}

static void restart_brightness_alert_hold_timer(void)
{
    xTimerReset(s_brightness_alert_hold_timer, 0);
}

static void refresh_brightness(bool trigger_alert_hold)
{
    if (trigger_alert_hold) {
        apply_brightness(BRIGHTNESS_LEVEL_HIGH);
    } else {
        apply_brightness(BRIGHTNESS_LEVEL_LOW);
    }
}

/**
 * Timer vypíná. po určité době zvýšené jasnosti, která je spuštěna při detekci chybového stavu, aby nedošlo k trvalému vypálení displeje.
 */
static void brightness_alert_hold_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    if (s_network_level != SYS_NET_MQTT_READY) {
        // Pokud není síť v pořádku, držíme jas na maximu, aby byla chyba dobře vidět a znovu pustíme timer
        refresh_brightness(true); // Pro jistotu znovu nastavíme jas na maximum, i když už tam je, aby se obnovil případný timer pro držení této úrovně jasu
        restart_brightness_alert_hold_timer();
    } else 
    {
        // Pokud je síť v pořádku, můžeme snížit jas, protože už není potřeba upozorňovat na chybu
        refresh_brightness(false);
    }
}

void status_display_set_flow_rate(float flow_l_min)
{
    taskENTER_CRITICAL(&s_status_mux);
    s_flow_l_min = flow_l_min;
    taskEXIT_CRITICAL(&s_status_mux);
}

void status_display_set_sensor_fault(sensor_event_type_t sensor_type, bool is_fault)
{
    switch (sensor_type) {
        case SENSOR_EVENT_TEMPERATURE:
            set_segments(SENSOR_FAULT_TEMP_SEGMENTS, SENSOR_FAULT_TEMP_POS, is_fault);
            break;
        case SENSOR_EVENT_LEVEL:
            set_segments(SENSOR_FAULT_LEVEL_SEGMENTS, SENSOR_FAULT_LEVEL_POS, is_fault);
            break;
        case SENSOR_EVENT_FLOW:
            set_segments(SENSOR_FAULT_FLOW_SEGMENTS, SENSOR_FAULT_FLOW_POS, is_fault);
            break;
        default:
            break;
    }
    if (is_fault) {
        set_max_briteness_for_some_time();
    }
    
}
static void set_max_briteness_for_some_time(void)
{
    refresh_brightness(true);
    restart_brightness_alert_hold_timer();
}

static float status_display_get_flow_rate(void)
{
    float flow_snapshot = 0.0f;
    taskENTER_CRITICAL(&s_status_mux);
    flow_snapshot = s_flow_l_min;
    taskEXIT_CRITICAL(&s_status_mux);
    return flow_snapshot;
}

static bool flow_spinner_compute_period_ms(float flow_l_min, uint32_t *out_period_ms)
{
    if (out_period_ms == nullptr) {
        return false;
    }

    if (!isfinite(flow_l_min) || flow_l_min <= FLOW_SPINNER_START_THRESHOLD_L_MIN) {
        return false;
    }

    float clamped_flow = flow_l_min;
    if (clamped_flow < FLOW_SPINNER_SPEED_MIN_FLOW_L_MIN) {
        clamped_flow = FLOW_SPINNER_SPEED_MIN_FLOW_L_MIN;
    }
    if (clamped_flow > FLOW_SPINNER_SPEED_MAX_FLOW_L_MIN) {
        clamped_flow = FLOW_SPINNER_SPEED_MAX_FLOW_L_MIN;
    }

    const float normalized =
        (clamped_flow - FLOW_SPINNER_SPEED_MIN_FLOW_L_MIN) /
        (FLOW_SPINNER_SPEED_MAX_FLOW_L_MIN - FLOW_SPINNER_SPEED_MIN_FLOW_L_MIN);

    const float period = (float)FLOW_SPINNER_MAX_PERIOD_MS -
                         normalized * (float)(FLOW_SPINNER_MAX_PERIOD_MS - FLOW_SPINNER_MIN_PERIOD_MS);

    *out_period_ms = (uint32_t)period;
    return true;
}

static void flow_spinner_clear(void)
{
    set_segments(FLOW_SPINNER_SEGMENT_MASK_POS0, 0, false);
    set_segments(FLOW_SPINNER_SEGMENT_MASK_POS1, 1, false);
}

static void flow_spinner_show_frame(uint8_t frame_index)
{
    flow_spinner_clear();

    const flow_spinner_frame_t &frame = FLOW_SPINNER_FRAMES[frame_index % (sizeof(FLOW_SPINNER_FRAMES) / sizeof(FLOW_SPINNER_FRAMES[0]))];
    if (frame.seg_pos0 != 0) {
        set_segments(frame.seg_pos0, 0, true);
    }
    if (frame.seg_pos1 != 0) {
        set_segments(frame.seg_pos1, 1, true);
    }
}

void set_segments(const uint8_t segments, uint8_t position, bool on) 
{
    if (s_display_latched) { // Display se nachází v latched stavu, což znamená, že zobrazuje důležitou informaci (například chybu), kterou nechceme přepsat běžnými aktualizacemi segmentů, dokud nebude zařízení restartováno nebo dokud se znovu nezobrazí jiná informace, která tento latched stav přepíše. V tomto stavu ignorujeme všechny pokusy o změnu segmentů, aby nedošlo k nechtěné změně zobrazení důležité informace.
        return;
    }
    if (position < 4) {
        const uint8_t updated_segments = on ? (svitici_segmenty[position] | segments)
                                            : (svitici_segmenty[position] & ~segments);
        if (updated_segments == svitici_segmenty[position]) {
            return;
        }

        svitici_segmenty[position] = updated_segments;
        tm1637_set_segments(s_tm1637_display, svitici_segmenty, 4, 0);
        set_error_led(svitici_segmenty[1] & TM1637_SEG_DP); // pokud se mění segment DP, aktualizuj i error LED, která je s ním propojená
    }
}

static void set_colon(bool colon_on)
{
    set_segments(TM1637_SEG_DP, 1, colon_on);
}

static void blink_pattern_blocking(system_network_level_t level)
{
    const auto& config = s_network_level_display_config[level];
    
    if (config.on_ms == 0) {
        return;
    }
    
    for (uint32_t i = 0; i < config.blink_count; ++i) {
        set_colon(true);
        vTaskDelay(pdMS_TO_TICKS(config.on_ms));
        if (config.off_ms > 0) {
            set_colon(false);
            vTaskDelay(pdMS_TO_TICKS(config.off_ms));
        }
    }
    vTaskDelay(pdMS_TO_TICKS(config.gap_ms));
}


/**
 * Zobrazí na stavovém displeji indikaci aktuálního stavu sítě. 
 * Pokud zobrazení není dostupné, bude místo toho použita chybová LED pro indikaci stavu.
 */
static void network_colon_status_display_task(void *pvParameters)
{
    (void)pvParameters;

    while (true) {
        system_network_level_t level_snapshot;

        level_snapshot = s_network_level;

        if (level_snapshot == SYS_NET_MQTT_READY) {
            set_colon(true);
            vTaskDelay(STATUS_DISPLAY_TASK_PERIOD);
            continue;
        }

        for (int i = 0; i < 2; ++i) {
            blink_pattern_blocking(level_snapshot);
        }
    }
}

/**
 * Zobrazí na stavovém displeji indikaci aktuálního stavu sítě. 
 * Pokud zobrazení není dostupné, bude místo toho použita chybová LED pro indikaci stavu.
 */
static void flow_spinner_status_display_task(void *pvParameters)
{
    (void)pvParameters;
    while (true) {
        uint32_t spinner_period_ms = 0;
        const float flow_snapshot = status_display_get_flow_rate();
        const bool spinner_enabled = flow_spinner_compute_period_ms(flow_snapshot, &spinner_period_ms);
        if (spinner_enabled) {
            ESP_LOGE(TAG, "Flow spinner - flow=%.3f l/min, enabled=%d, period=%lu ms", flow_snapshot, spinner_enabled ? 1 : 0, (unsigned long)spinner_period_ms);
            s_flow_spinner_frame_index =
                (uint8_t)((s_flow_spinner_frame_index + 1) % (sizeof(FLOW_SPINNER_FRAMES) / sizeof(FLOW_SPINNER_FRAMES[0])));
            flow_spinner_show_frame(s_flow_spinner_frame_index);
            vTaskDelay(pdMS_TO_TICKS(spinner_period_ms));

        } else {
            flow_spinner_clear();
            vTaskDelay(FLOW_SPINNER_IDLE_POLL_DELAY);
        }
    }
}



/**
 * Zobrazí na stavovém displeji kód chyby pomocí čtyřmístného zobrazení. Pokud zobrazení není dostupné, bude místo toho použita chybová LED pro indikaci stavu.
 * Kód chyby bude zobrazen trvale, dokud nebude zařízení restartováno, nebo dokud nebude zobrazení znovu použito pro zobrazení jiné informace (v takovém případě bude kód chyby přepsán).
 */
static void show_error_code_on_tm1637(const char *error_code)
{
    s_display_latched = true;
    if (error_code == nullptr || s_tm1637_display == nullptr) {
        return;
    }

    char display_text[5] = {' ', ' ', ' ', ' ', '\0'};
    for (size_t index = 0; index < 4 && error_code[index] != '\0'; ++index) {
        display_text[index] = error_code[index];
    }

    tm1637_handle_t display = s_tm1637_display;
    s_tm1637_display = nullptr; // zabrani dalsim pokusum o pouziti displeje, kdyz už na nem bude číslo chyby, bude stejně abort a chyba na displeji musí zůstat vidět
    tm1637_write_string(display, display_text);
    tm1637_set_brightness(display, BRIGHTNESS_LEVEL_HIGH, true);
}

static void app_error_code_log_handler(const char *error_code)
{
    ESP_LOGE(TAG, "Error code: %s", (error_code != nullptr) ? error_code : "(null)");
    show_error_code_on_tm1637(error_code);
}


/**
 * Nastaví stav sítě pro zobrazení na displeji.
 */
void status_display_set_network_state(const network_event_t *event)
{
    if (event == nullptr) {
        return;
    }

    taskENTER_CRITICAL(&s_status_mux);
    s_network_level = event->level;
    taskEXIT_CRITICAL(&s_status_mux);

    const bool network_problem = (event->level != SYS_NET_MQTT_READY);
    if (network_problem) {  
       set_max_briteness_for_some_time();
    }   
}

/// @brief //////// Poblikáván LED při MQTT aktivitě///////////////////////////////////////////////////////////////////////////////  
/// @param xTimer 

void mqtt_activiti_led_off_finished ( TimerHandle_t xTimer ) 
{
    set_colon(false);
    xTimerStart(s_mqtt_activity_colon_off_timer, 0);
}

void mqtt_activiti_led_on_finished ( TimerHandle_t xTimer ) 
{
    set_colon(true);
    s_mqtt_activity_timer_running = false;
}


void status_display_notify_mqtt_activity(void)
{
    if (s_mqtt_activity_timer_running || s_network_level != SYS_NET_MQTT_READY) {
        return;
    }

    s_mqtt_activity_timer_running = true;
    set_colon(false);
    xTimerStart(s_mqtt_activity_colon_on_timer, 0);

}

static void mqtt_activity_init_timers(void)
{
     s_mqtt_activity_colon_on_timer = xTimerCreateStatic (
        "mqtt_colon",
        pdMS_TO_TICKS(MQTT_ACTIVITY_COLON_ON_MS),
        pdFALSE,
        nullptr,
        mqtt_activiti_led_off_finished,
        &s_mqtt_activity_colon_on_timer_buffer
    );
    s_mqtt_activity_colon_off_timer = xTimerCreateStatic (
        "mqtt_colon_off",
        pdMS_TO_TICKS(MQTT_ACTIVITY_COLON_OFF_MS),
        pdFALSE,
        nullptr,
        mqtt_activiti_led_on_finished,
        &s_mqtt_activity_colon_off_timer_buffer
    );
    s_brightness_alert_hold_timer = xTimerCreateStatic(
        "disp_bright_hold",
        pdMS_TO_TICKS(BRIGHTNESS_ALERT_HOLD_MS),
        pdFALSE,
        nullptr,
        brightness_alert_hold_timer_cb,
        &s_brightness_alert_hold_timer_buffer);
}

//////////////////////////////////////////////////////////////////////////////////

/**
 * Zobrazí na displeji indikaci, že zařízení je v režimu přístupového bodu pro konfiguraci Wi-Fi. 
 * Indikace zůstane zobrazena trvale, dokud nebude zařízení restartováno do normálního režimu.
 */
void status_display_ap_mode()
{

    s_display_latched = true;
    apply_brightness(BRIGHTNESS_LEVEL_HIGH);

    tm1637_write_string(s_tm1637_display, "-AP-");
    set_error_led(true);
    apply_brightness(BRIGHTNESS_LEVEL_HIGH);
}


/**
 * Inicializuje stavový displej a spustí úlohu pro aktualizaci zobrazení.
 * Pokud inicializace displeje selže, bude místo toho použita chybová LED pro indikaci stavu.
 */
void status_display_init(void)
{

    gpio_reset_pin(ERRORLED_PIN);
    gpio_set_direction(ERRORLED_PIN, GPIO_MODE_OUTPUT);

    mqtt_activity_init_timers();

    esp_err_t init_result = tm1637_init(&s_tm1637_config, &s_tm1637_display);
    if (init_result != ESP_OK) {
        s_tm1637_display = nullptr;
        ESP_LOGE(TAG, "TM1637 init selhal, displej nebude pouzit: %s", esp_err_to_name(init_result));
    } else {
        s_current_brightness_level = BRIGHTNESS_LEVEL_HIGH;
        esp_err_t brightness_result = tm1637_set_brightness(s_tm1637_display, BRIGHTNESS_LEVEL_HIGH, true);
        esp_err_t startup_result = tm1637_startup_animation_play_preset(s_tm1637_display, TM1637_STARTUP_ANIMATION_FAST);
        (void)brightness_result;
        (void)startup_result;
    }

    xTaskCreate(network_colon_status_display_task, "network_colon_status_display", 3072, NULL, 3, NULL);
    xTaskCreate(flow_spinner_status_display_task, "flow_spinner_status_display", 3072, NULL, 3, NULL);



    app_error_check_set_handler(app_error_code_log_handler);
}


















