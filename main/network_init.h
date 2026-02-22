#ifndef NETWORK_INIT_H
#define NETWORK_INIT_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t network_init_sta(const char *ssid, const char *password);
esp_err_t network_init_ap(const char *ap_ssid, const char *ap_password);
bool network_wait_connected(uint32_t timeout_ms);

esp_err_t network_mqtt_start(const char *broker_uri, const char *username, const char *password);
bool network_mqtt_wait_connected(uint32_t timeout_ms);
bool network_mqtt_is_connected(void);
esp_mqtt_client_handle_t network_mqtt_client(void);

#ifdef __cplusplus
}
#endif

#endif // NETWORK_INIT_H
