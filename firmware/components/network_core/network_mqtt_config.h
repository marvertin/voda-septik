#ifndef NETWORK_MQTT_CONFIG_H
#define NETWORK_MQTT_CONFIG_H

#include <stdbool.h>

bool network_mqtt_config_prepare(const char *broker_uri, const char *username, const char *password);
const char *network_mqtt_config_uri(void);
const char *network_mqtt_config_username_or_null(void);
const char *network_mqtt_config_password_or_null(void);

#endif // NETWORK_MQTT_CONFIG_H
