#pragma once

#include "esp_err.h"

esp_err_t ota_manager_start_from_url(const char *url);
esp_err_t ota_manager_confirm_running_firmware(void);
