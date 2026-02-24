#pragma once

#include "network_event.h"
#include "esp_err.h"

void webapp_startup_on_network_event(const network_event_t *event);
esp_err_t webapp_startup_start(void);
esp_err_t webapp_startup_stop(void);
