#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "tm1637.h"

esp_err_t status_display_play_startup_sequence(tm1637_handle_t display);

#ifdef __cplusplus
}
#endif
