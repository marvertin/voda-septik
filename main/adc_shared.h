#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_err.h>
#include <esp_adc/adc_oneshot.h>

esp_err_t adc_shared_init(adc_unit_t unit);
esp_err_t adc_shared_config_channel(adc_channel_t channel,
                                    adc_bitwidth_t bitwidth,
                                    adc_atten_t atten);
esp_err_t adc_shared_read(adc_channel_t channel, int *raw_value);

#ifdef __cplusplus
}
#endif
