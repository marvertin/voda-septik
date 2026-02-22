#ifndef NETWORK_INIT_H
#define NETWORK_INIT_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool wifi_up;
    bool ip_ready;
    bool mqtt_ready;
    int8_t last_rssi;
    uint32_t ip_addr;
    int64_t timestamp_us;
} network_state_t;

typedef void (*network_state_callback_t)(const network_state_t *state, void *ctx);

typedef struct {
    bool enabled;
    const char *topic;
    const char *message;
    int qos;
    bool retain;
} network_mqtt_lwt_config_t;

esp_err_t network_register_state_callback(network_state_callback_t callback, void *ctx);

esp_err_t network_init_sta(const char *ssid, const char *password);
esp_err_t network_init_ap(const char *ap_ssid, const char *ap_password);
bool network_wait_connected(uint32_t timeout_ms);

esp_err_t network_mqtt_start(const char *broker_uri, const char *username, const char *password);
esp_err_t network_mqtt_start_ex(const char *broker_uri,
                                const char *username,
                                const char *password,
                                const network_mqtt_lwt_config_t *lwt_config);
bool network_mqtt_wait_connected(uint32_t timeout_ms);
bool network_mqtt_is_connected(void);
esp_mqtt_client_handle_t network_mqtt_client(void);

#ifdef __cplusplus
}
#endif

#endif // NETWORK_INIT_H
