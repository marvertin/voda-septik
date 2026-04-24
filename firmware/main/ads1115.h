#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADS1115_INVALID_RAW_VALUE INT16_MIN

typedef struct {
    int16_t pressure_after_filter_raw;
    int16_t pressure_before_filter_raw;
} ads1115_pressure_sample_t;

typedef struct {
    int16_t level_raw;
} ads1115_level_sample_t;

void ads1115_start(void);

QueueHandle_t ads1115_pressure_queue(void);
QueueHandle_t ads1115_level_queue(void);

#ifdef __cplusplus
}
#endif
