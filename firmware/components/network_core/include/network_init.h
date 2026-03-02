#ifndef NETWORK_INIT_H
#define NETWORK_INIT_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "mqtt_client.h"
#include "network_event.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*network_event_callback_t)(const network_event_t *event, void *ctx);

typedef struct {
    bool enabled;
    const char *status_topic;
    int qos;
    bool retain;
} network_mqtt_lwt_config_t;

esp_err_t network_register_event_callback(network_event_callback_t callback, void *ctx);

esp_err_t network_init_sta(const char *ssid, const char *password);
esp_err_t network_init_ap(const char *ap_ssid, const char *ap_password);

esp_err_t network_mqtt_start(const char *broker_uri, const char *username, const char *password);
esp_err_t network_mqtt_start_ex(const char *broker_uri,
                                const char *username,
                                const char *password,
                                const network_mqtt_lwt_config_t *lwt_config);
esp_err_t network_init_with_mqtt_ex(const char *wifi_ssid,
                                    const char *wifi_password,
                                    const char *broker_uri,
                                    const char *mqtt_username,
                                    const char *mqtt_password,
                                    const network_mqtt_lwt_config_t *lwt_config);
bool network_mqtt_is_connected(void);
esp_mqtt_client_handle_t network_mqtt_client(void);
const char *network_mqtt_status_topic(void);

#ifdef __cplusplus
}
#endif

#endif // NETWORK_INIT_H
