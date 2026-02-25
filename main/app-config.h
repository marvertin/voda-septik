#pragma once

#include "config_webapp.h"
#include <stddef.h>
#include "esp_err.h"

config_group_t app_config_get_config_group(void);
esp_err_t app_config_ensure_defaults(void);
esp_err_t app_config_load_wifi_credentials(char *ssid, size_t ssid_len, char *password, size_t password_len);
esp_err_t app_config_load_mqtt_uri(char *uri, size_t uri_len);
esp_err_t app_config_load_mqtt_credentials(char *username, size_t username_len, char *password, size_t password_len);
