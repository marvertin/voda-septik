#pragma once

#include "esp_err.h"

void teplota_init(void);
esp_err_t teplota_set_scan_enabled(bool enabled);
bool teplota_scan_enabled(void);
