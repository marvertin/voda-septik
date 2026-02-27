#pragma once

#include "esp_err.h"

void teplota_demo_init(void);
esp_err_t teplota_demo_set_scan_enabled(bool enabled);
bool teplota_demo_scan_enabled(void);
