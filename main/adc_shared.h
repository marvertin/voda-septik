#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_adc/adc_oneshot.h>

void adc_shared_init(void);
void adc_channel_init(adc_channel_t channel);
int adc_read(adc_channel_t channel);

#ifdef __cplusplus
}
#endif
