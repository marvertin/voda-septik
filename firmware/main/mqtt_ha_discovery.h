#pragma once

#include "esp_err.h"
#include "mqtt_topics.h"

esp_err_t mqtt_ha_discovery_publish_all(void);
esp_err_t mqtt_ha_discovery_set_human_name(mqtt_topic_id_t topic_id, const char *human_name);
esp_err_t mqtt_ha_discovery_clear_human_name(mqtt_topic_id_t topic_id);
