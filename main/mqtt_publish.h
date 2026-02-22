#ifndef MQTT_PUBLISH_H
#define MQTT_PUBLISH_H

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mqtt_publish(const char *topic, const char *data, bool retain);
bool mqtt_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif // MQTT_PUBLISH_H
