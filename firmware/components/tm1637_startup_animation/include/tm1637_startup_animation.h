#pragma once

#include "esp_err.h"
#include "tm1637.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	TM1637_STARTUP_ANIMATION_FAST = 0,
	TM1637_STARTUP_ANIMATION_CALM = 1,
} tm1637_startup_animation_preset_t;

esp_err_t tm1637_startup_animation_play(tm1637_handle_t display);
esp_err_t tm1637_startup_animation_play_preset(tm1637_handle_t display, tm1637_startup_animation_preset_t preset);

#ifdef __cplusplus
}
#endif
