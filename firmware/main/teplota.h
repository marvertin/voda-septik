#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool ok;
    uint32_t bus_errors;
    uint32_t crc_errors;
    uint32_t read_errors;
    int64_t last_ok_age_s;
} teplota_diag_t;

void teplota_register_config_items(void);
void teplota_init(void);
esp_err_t teplota_set_scan_enabled(bool enabled);
bool teplota_scan_enabled(void);
teplota_diag_t teplota_diag_snapshot(void);
