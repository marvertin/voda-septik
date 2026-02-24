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

#include "status_display_startup_animation.h"

// Startup sekvence (TM1637, 4 pozice):
// 1) krátká tma,
// 2) "běžící" segment napříč displejem,
// 3) dvojitý pulz vodorovných čar,
// 4) krátký plný záblesk,
// 5) zhasnutí a předání řízení dalšímu initu.

static esp_err_t show_frame(tm1637_handle_t display, const uint8_t frame[4], uint32_t delay_ms)
{
    esp_err_t write_result = tm1637_set_segments(display, frame, 4, 0);
    if (write_result != ESP_OK) {
        return write_result;
    }

    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    return ESP_OK;
}

esp_err_t status_display_play_startup_sequence(tm1637_handle_t display)
{
    if (display == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    static constexpr uint8_t kAllOff[4] = {0, 0, 0, 0};
    static constexpr uint8_t kDigitFill[4] = {
        TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_D | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G,
        TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_D | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G,
        TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_D | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G,
        TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C | TM1637_SEG_D | TM1637_SEG_E | TM1637_SEG_F | TM1637_SEG_G,
    };

    // Blok 1: inicializační tma (vše vypnuto), aby animace vždy začínala z čistého stavu.
    // Vizuál: [    ][    ][    ][    ]
    esp_err_t result = show_frame(display, kAllOff, 60);
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
    for (uint8_t step = 0; step < 14; ++step) {
        uint8_t digit = step % 4;
        uint8_t seg = segment_order[step % 7];
        ring_frame[0] = 0;
        ring_frame[1] = 0;
        ring_frame[2] = 0;
        ring_frame[3] = 0;
        ring_frame[digit] = seg;

        result = show_frame(display, ring_frame, 45);
        if (result != ESP_OK) {
            return result;
        }
    }

    // Blok 3: pulz vodorovných čar (A + G + D) — nejdřív se postupně zaplňují
    // zleva doprava, pak se stejným tempem vypínají zprava doleva; celé 2 průchody.
    // Vizuál plnění: [≡   ][    ][    ][    ] -> [≡   ][≡   ][    ][    ] -> ...
    uint8_t pulse_frame[4] = {0, 0, 0, 0};
    for (uint8_t pass = 0; pass < 2; ++pass) {
        for (uint8_t digit = 0; digit < 4; ++digit) {
            pulse_frame[digit] = TM1637_SEG_A | TM1637_SEG_D | TM1637_SEG_G;
            result = show_frame(display, pulse_frame, 70);
            if (result != ESP_OK) {
                return result;
            }
        }
        for (int8_t digit = 3; digit >= 0; --digit) {
            pulse_frame[digit] = 0;
            result = show_frame(display, pulse_frame, 70);
            if (result != ESP_OK) {
                return result;
            }
        }
    }

    // Blok 4: krátký "flash" — všechny segmenty všech 4 pozic současně.
    // Vizuál: [8][8][8][8]
    result = show_frame(display, kDigitFill, 110);
    if (result != ESP_OK) {
        return result;
    }

    // Blok 5: finální zhasnutí před předáním displeje běžné logice aplikace.
    // Vizuál: [    ][    ][    ][    ]
    result = show_frame(display, kAllOff, 80);
    if (result != ESP_OK) {
        return result;
    }

    return ESP_OK;
}
