#pragma once

#include <stddef.h>
#include "esp_err.h"

void network_config_register_config_items(void);
esp_err_t network_config_load_wifi_credentials(char *ssid, size_t ssid_len, char *password, size_t password_len);
esp_err_t network_config_load_mqtt_uri(char *uri, size_t uri_len);
esp_err_t network_config_load_mqtt_credentials(char *username, size_t username_len, char *password, size_t password_len);
