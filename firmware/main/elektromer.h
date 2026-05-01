#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool ok;
    uint32_t timeouts;
    uint32_t crc_errors;
    uint32_t read_errors;
    int64_t last_ok_age_s;
} elektromer_diag_t;

void elektromer_register_config_items(void);
void elektromer_init(void);
elektromer_diag_t elektromer_diag_snapshot(void);
