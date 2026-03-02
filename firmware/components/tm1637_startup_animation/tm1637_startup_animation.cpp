#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "tm1637.h"

#ifdef __cplusplus
}
#endif

#include "tm1637_startup_animation.h"

// Startup sekvence (TM1637, 4 pozice):
// 1) krátká tma,
// 2) "běžící" segment napříč displejem,
// 3) dvojitý pulz vodorovných čar,
// 4) krátký plný záblesk,
// 5) zhasnutí a předání řízení dalšímu initu.

typedef struct {
    uint32_t all_off_initial_ms;
    uint32_t ring_step_ms;
    uint8_t ring_steps;
    uint32_t pulse_step_ms;
    uint8_t pulse_passes;
    uint32_t flash_ms;
    uint32_t all_off_final_ms;
} animation_timing_t;

static animation_timing_t get_timing(tm1637_startup_animation_preset_t preset)
{
    switch (preset) {
        case TM1637_STARTUP_ANIMATION_FAST:
            return {
                .all_off_initial_ms = 35,
                .ring_step_ms = 28,
                .ring_steps = 10,
                .pulse_step_ms = 45,
                .pulse_passes = 1,
                .flash_ms = 75,
                .all_off_final_ms = 50,
            };

        case TM1637_STARTUP_ANIMATION_CALM:
        default:
            return {
                .all_off_initial_ms = 60,
                .ring_step_ms = 45,
                .ring_steps = 14,
                .pulse_step_ms = 70,
                .pulse_passes = 2,
                .flash_ms = 110,
                .all_off_final_ms = 80,
            };
    }
}

static esp_err_t show_frame(tm1637_handle_t display, const uint8_t frame[4], uint32_t delay_ms)
{
    esp_err_t write_result = tm1637_set_segments(display, frame, 4, 0);
    if (write_result != ESP_OK) {
        return write_result;
    }

    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    return ESP_OK;
}

esp_err_t tm1637_startup_animation_play_preset(tm1637_handle_t display, tm1637_startup_animation_preset_t preset)
{
    if (display == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const animation_timing_t timing = get_timing(preset);

    static constexpr uint8_t kAllOff[4] = {0, 0, 0, 0};
    static constexpr uint8_t kDigitFill[4] = {
        TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_D | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G,
        TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_D | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G,
        TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_D | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G,
        TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_D | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G,
    };

    // Blok 1: inicializační tma (vše vypnuto), aby animace vždy začínala z čistého stavu.
    // Vizuál: [    ][    ][    ][    ]
    esp_err_t result = show_frame(display, kAllOff, timing.all_off_initial_ms);
    if (result != ESP_OK) {
        return result;
    }

    const uint8_t segment_order[] = {
        TM1637_SEG_A,
        TM1637_SEG_B,
        TM1637_SEG_C,
        TM1637_SEG_D,
        TM1637_SEG_E,
        TM1637_SEG_F,
        TM1637_SEG_G,
    };

    // Blok 2: "ring/scan" efekt — vždy svítí jediný segment na jedné pozici,
    // který se posouvá přes 4 číslice a střídá A..G.
    // Vizuál: [A   ][    ][    ][    ] -> [    ][B   ][    ][    ] -> ...
    uint8_t ring_frame[4] = {0, 0, 0, 0};
    for (uint8_t step = 0; step < timing.ring_steps; ++step) {
        uint8_t digit = step % 4;
        uint8_t seg = segment_order[step % 7];
        ring_frame[0] = 0;
        ring_frame[1] = 0;
        ring_frame[2] = 0;
        ring_frame[3] = 0;
        ring_frame[digit] = seg;

        result = show_frame(display, ring_frame, timing.ring_step_ms);
        if (result != ESP_OK) {
            return result;
        }
    }

    // Blok 3: pulz vodorovných čar (A + G + D) — nejdřív se postupně zaplňují
    // zleva doprava, pak se stejným tempem vypínají také zleva doprava; celé 2 průchody.
    // Vizuál plnění: [≡   ][    ][    ][    ] -> [≡   ][≡   ][    ][    ] -> ...
    uint8_t pulse_frame[4] = {0, 0, 0, 0};
    for (uint8_t pass = 0; pass < timing.pulse_passes; ++pass) {
        for (uint8_t digit = 0; digit < 4; ++digit) {
            pulse_frame[digit] = TM1637_SEG_A | TM1637_SEG_D | TM1637_SEG_G;
            result = show_frame(display, pulse_frame, timing.pulse_step_ms);
            if (result != ESP_OK) {
                return result;
            }
        }
        for (uint8_t digit = 0; digit < 4; ++digit) {
            pulse_frame[digit] = 0;
            result = show_frame(display, pulse_frame, timing.pulse_step_ms);
            if (result != ESP_OK) {
                return result;
            }
        }
    }

    // Blok 4: krátký "flash" — všechny segmenty všech 4 pozic současně.
    // Vizuál: [8][8][8][8]
    result = show_frame(display, kDigitFill, timing.flash_ms);
    if (result != ESP_OK) {
        return result;
    }

    // Blok 5: finální zhasnutí před předáním displeje běžné logice aplikace.
    // Vizuál: [    ][    ][    ][    ]
    result = show_frame(display, kAllOff, timing.all_off_final_ms);
    if (result != ESP_OK) {
        return result;
    }

    return ESP_OK;
}

esp_err_t tm1637_startup_animation_play(tm1637_handle_t display)
{
    return tm1637_startup_animation_play_preset(display, TM1637_STARTUP_ANIMATION_CALM);
}
