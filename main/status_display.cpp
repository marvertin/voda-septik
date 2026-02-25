#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>

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

static tm1637_config_t s_tm1637_config = {
    .clk_pin = TM_CLK,
    .dio_pin = TM_DIO,
    .bit_delay_us = 100,
};

static tm1637_handle_t s_tm1637_display = nullptr;
static bool s_error_latched = false;
static bool s_text_latched = false;

static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static system_network_level_t s_network_level = SYS_NET_DOWN;

static bool s_mqtt_activity_timer_running = false;


StaticTimer_t s_mqtt_activity_colon_on_timer_buffer;
StaticTimer_t s_mqtt_activity_colon_off_timer_buffer;
TimerHandle_t s_mqtt_activity_colon_on_timer = nullptr; 
TimerHandle_t s_mqtt_activity_colon_off_timer = nullptr;

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

static void set_error_led(bool on) {
    gpio_set_level(ERRORLED_PIN, on ? 1 : 0);
}

void set_segments(const uint8_t segments, uint8_t position, bool on) 
{
    if (position < 4) {
        svitici_segmenty[position] = on ? (svitici_segmenty[position] | segments) : (svitici_segmenty[position] & ~segments);
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
static void status_display_task(void *pvParameters)
{
    (void)pvParameters;


    /*
    set_segments(TM1637_SEG_A, 0, true);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 5; ++j) {
            blink_pattern_blocking(static_cast<system_network_level_t>(i));
        }
    }
        */
    while (true) {
        if ( !s_error_latched && !s_text_latched) {
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
        } else {
            vTaskDelay(STATUS_DISPLAY_TASK_PERIOD);
        }
    }
}


/**
 * Zobrazí na stavovém displeji kód chyby pomocí čtyřmístného zobrazení. Pokud zobrazení není dostupné, bude místo toho použita chybová LED pro indikaci stavu.
 * Kód chyby bude zobrazen trvale, dokud nebude zařízení restartováno, nebo dokud nebude zobrazení znovu použito pro zobrazení jiné informace (v takovém případě bude kód chyby přepsán).
 */
static void show_error_code_on_tm1637(const char *error_code)
{
    if (error_code == nullptr || s_tm1637_display == nullptr) {
        return;
    }

    char display_text[5] = {' ', ' ', ' ', ' ', '\0'};
    for (size_t index = 0; index < 4 && error_code[index] != '\0'; ++index) {
        display_text[index] = error_code[index];
    }

    tm1637_handle_t display = s_tm1637_display;
    s_tm1637_display = nullptr; // zabrani dalsim pokusum o pouziti displeje, kdyz už na nem bude číslo chyby, bude stejně abort a chyba na displeji musí zůstat vidět
    esp_err_t write_result = tm1637_write_string(display, display_text);
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

    s_network_level = event->level;
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
    if (s_error_latched || s_text_latched || s_mqtt_activity_timer_running || s_network_level != SYS_NET_MQTT_READY) {
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
}

//////////////////////////////////////////////////////////////////////////////////

/**
 * Zobrazí na displeji indikaci, že zařízení je v režimu přístupového bodu pro konfiguraci Wi-Fi. 
 * Indikace zůstane zobrazena trvale, dokud nebude zařízení restartováno do normálního režimu.
 */
void status_display_ap_mode()
{

    s_text_latched = true;

    static constexpr uint8_t ALL_SEGMENTS = static_cast<uint8_t>(
        TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_D |
        TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G | TM1637_SEG_DP);
    static constexpr uint8_t DASH_SEGMENTS = TM1637_SEG_G;
    static constexpr uint8_t A_SEGMENTS = static_cast<uint8_t>(TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G);
    static constexpr uint8_t P_SEGMENTS = static_cast<uint8_t>(TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G);

    for (uint8_t position = 0; position < 4; ++position) {
        set_segments(ALL_SEGMENTS, position, false);
    }

    set_segments(DASH_SEGMENTS, 0, true);
    set_segments(A_SEGMENTS, 1, true);
    set_segments(P_SEGMENTS, 2, true);
    set_segments(DASH_SEGMENTS, 3, true);
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
        s_tm1637_available = false;
        ESP_LOGE(TAG, "TM1637 init selhal, displej nebude pouzit: %s", esp_err_to_name(init_result));
    } 
    esp_err_t brightness_result = tm1637_set_brightness(s_tm1637_display, 7, true);
    esp_err_t startup_result = tm1637_startup_animation_play_preset(s_tm1637_display, TM1637_STARTUP_ANIMATION_FAST);

    xTaskCreate(status_display_task, "status_display", 3072, NULL, 3, NULL);

    app_error_check_set_handler(app_error_code_log_handler);
}


















