#pragma once

#include <stdint.h>

void ads1115_logger_init(void);

/**
 * @brief Get raw ADC value from specified ADS1115 channel (0-3).
 * Returns zero if channel is invalid or not yet read.
 */
int16_t ads1115_get_channel_value(uint8_t channel);

/**
 * @brief Check if latest ADC read was successful.
 */
bool ads1115_last_read_ok(void);
