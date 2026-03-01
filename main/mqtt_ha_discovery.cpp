#include "mqtt_ha_discovery.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "mqtt_publish.h"
#include "mqtt_topics.h"

static const char *TAG = "mqtt_ha_discovery";

static constexpr const char *HA_DISCOVERY_ROOT = "homeassistant";
static constexpr const char *DEVICE_ID = "voda_septik_esp32";
static constexpr const char *DEVICE_NAME = "Voda Septik";
static constexpr const char *DEVICE_MODEL = "ESP32 voda-septik";
static constexpr const char *DEVICE_MANUFACTURER = "voda-septik";

struct ha_entity_meta_t {
    const char *component;
    const char *device_class;
    const char *unit;
    const char *state_class;
    const char *entity_category;
    const char *icon;
    const char *value_template;
    const char *json_attributes_topic;
    const char *payload_on;
    const char *payload_off;
};

static void sanitize_to_id(const char *input, char *output, size_t output_len)
{
    if (output == nullptr || output_len == 0) {
        return;
    }

    size_t out = 0;
    for (size_t i = 0; input != nullptr && input[i] != '\0' && out + 1 < output_len; ++i) {
        const unsigned char ch = (unsigned char)input[i];
        if (isalnum(ch) != 0) {
            output[out++] = (char)tolower(ch);
            continue;
        }
        output[out++] = '_';
    }
    output[out] = '\0';

    if (out == 0) {
        strncpy(output, "topic", output_len - 1);
        output[output_len - 1] = '\0';
    }
}

static void make_human_name(const char *topic, char *name, size_t name_len)
{
    if (name == nullptr || name_len == 0) {
        return;
    }

    const char *start = topic;
    char topic_prefix[96] = {0};
    snprintf(topic_prefix, sizeof(topic_prefix), "%s/", MQTT_TOPIC_ROOT);
    const size_t topic_prefix_len = strlen(topic_prefix);

    if (start != nullptr && strncmp(start, topic_prefix, topic_prefix_len) == 0) {
        start += topic_prefix_len;
    }

    size_t out = 0;
    bool upper_next = true;
    for (size_t i = 0; start != nullptr && start[i] != '\0' && out + 1 < name_len; ++i) {
        char ch = start[i];
        if (ch == '/' || ch == '_') {
            if (out > 0 && name[out - 1] != ' ') {
                name[out++] = ' ';
            }
            upper_next = true;
            continue;
        }

        if (upper_next && isalpha((unsigned char)ch) != 0) {
            name[out++] = (char)toupper((unsigned char)ch);
        } else {
            name[out++] = ch;
        }
        upper_next = false;
    }

    while (out > 0 && name[out - 1] == ' ') {
        --out;
    }
    name[out] = '\0';

    if (out == 0) {
        strncpy(name, "Voda Septik", name_len - 1);
        name[name_len - 1] = '\0';
    }
}

static bool topic_ends_with(const char *topic, const char *suffix)
{
    if (topic == nullptr || suffix == nullptr) {
        return false;
    }

    const size_t topic_len = strlen(topic);
    const size_t suffix_len = strlen(suffix);
    if (suffix_len > topic_len) {
        return false;
    }

    return strcmp(topic + (topic_len - suffix_len), suffix) == 0;
}

static ha_entity_meta_t infer_meta(const mqtt_topic_descriptor_t &topic)
{
    ha_entity_meta_t meta = {
        .component = "sensor",
        .device_class = nullptr,
        .unit = nullptr,
        .state_class = nullptr,
        .entity_category = nullptr,
        .icon = nullptr,
        .value_template = nullptr,
        .json_attributes_topic = nullptr,
        .payload_on = nullptr,
        .payload_off = nullptr,
    };

    const char *full = topic.full_topic;

    if (topic.id == mqtt_topic_id_t::TOPIC_SYSTEM_STATUS) {
        meta.component = "binary_sensor";
        meta.device_class = "connectivity";
        meta.payload_on = "online";
        meta.payload_off = "offline";
        return meta;
    }

    if (topic.payload_kind == mqtt_payload_kind_t::BOOLEAN) {
        meta.component = "binary_sensor";
        meta.payload_on = "1";
        meta.payload_off = "0";
        if (topic.id == mqtt_topic_id_t::TOPIC_STAV_CERPANI_PUMPA_BEZI) {
            meta.device_class = "running";
        }
        return meta;
    }

    if (topic.payload_kind == mqtt_payload_kind_t::TEXT) {
        meta.component = "sensor";
    }

    if (topic.payload_kind == mqtt_payload_kind_t::JSON) {
        meta.component = "sensor";
        meta.icon = "mdi:code-json";
        if (topic.id == mqtt_topic_id_t::TOPIC_DIAG_TEPLOTA_SCAN) {
            meta.value_template = "{{ value_json.found | count }}";
            meta.unit = "count";
            meta.json_attributes_topic = topic.full_topic;
        }
    }

    if (strstr(full, "/diag/") != nullptr || strstr(full, "/system/") != nullptr) {
        meta.entity_category = "diagnostic";
    }

    if (topic_ends_with(full, "/voda") || topic_ends_with(full, "/vzduch")) {
        meta.device_class = "temperature";
        meta.unit = "°C";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "_m3")) {
        meta.device_class = "volume";
        meta.unit = "m³";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "_m")) {
        meta.device_class = "distance";
        meta.unit = "m";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "_l_min")) {
        meta.device_class = "volume_flow_rate";
        meta.unit = "L/min";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "cerpano_celkem_l")) {
        meta.device_class = "volume";
        meta.unit = "L";
        meta.state_class = "total_increasing";
    } else if (topic_ends_with(full, "_bar")) {
        meta.device_class = "pressure";
        meta.unit = "bar";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "_percent") || topic_ends_with(full, "/progress")) {
        meta.unit = "%";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "_w")) {
        meta.device_class = "power";
        meta.unit = "W";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "_var")) {
        meta.unit = "var";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "_a")) {
        meta.device_class = "current";
        meta.unit = "A";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "_v")) {
        meta.device_class = "voltage";
        meta.unit = "V";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "_kwh")) {
        meta.device_class = "energy";
        meta.unit = "kWh";
        meta.state_class = "total_increasing";
    } else if (topic_ends_with(full, "_kvarh")) {
        meta.unit = "kvarh";
        meta.state_class = "total_increasing";
    } else if (topic_ends_with(full, "_s")) {
        meta.device_class = "duration";
        meta.unit = "s";
        if (strstr(full, "uptime") != nullptr || strstr(full, "reconnect") != nullptr || strstr(full, "counter") != nullptr) {
            meta.state_class = "total_increasing";
        } else {
            meta.state_class = "measurement";
        }
    } else if (topic_ends_with(full, "_dbm")) {
        meta.device_class = "signal_strength";
        meta.unit = "dBm";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "_b")) {
        meta.device_class = "data_size";
        meta.unit = "B";
        meta.state_class = "measurement";
    } else if (topic_ends_with(full, "_mv")) {
        meta.device_class = "voltage";
        meta.unit = "mV";
        meta.state_class = "measurement";
    }

    if (topic.id == mqtt_topic_id_t::TOPIC_SYSTEM_REBOOT_COUNTER ||
        topic.id == mqtt_topic_id_t::TOPIC_DIAG_WIFI_RECONNECT_TRY ||
        topic.id == mqtt_topic_id_t::TOPIC_DIAG_WIFI_RECONNECT_SUCCESS ||
        topic.id == mqtt_topic_id_t::TOPIC_DIAG_MQTT_RECONNECTS ||
        topic.id == mqtt_topic_id_t::TOPIC_DIAG_NVS_ERRORS) {
        meta.state_class = "total_increasing";
    }

    return meta;
}

static bool append_json_field(char *payload,
                              size_t payload_len,
                              size_t *offset,
                              bool *first,
                              const char *key,
                              const char *value,
                              bool quoted)
{
    if (payload == nullptr || offset == nullptr || first == nullptr || key == nullptr || value == nullptr) {
        return false;
    }

    int written = 0;
    if (*first) {
        written = snprintf(payload + *offset,
                           payload_len - *offset,
                           quoted ? "\"%s\":\"%s\"" : "\"%s\":%s",
                           key,
                           value);
        *first = false;
    } else {
        written = snprintf(payload + *offset,
                           payload_len - *offset,
                           quoted ? ",\"%s\":\"%s\"" : ",\"%s\":%s",
                           key,
                           value);
    }

    if (written <= 0 || (size_t)written >= (payload_len - *offset)) {
        return false;
    }

    *offset += (size_t)written;
    return true;
}

static esp_err_t publish_discovery_for_topic(const mqtt_topic_descriptor_t &topic)
{
    char slug[160] = {0};
    sanitize_to_id(topic.full_topic, slug, sizeof(slug));

    char unique_id[192] = {0};
    snprintf(unique_id, sizeof(unique_id), "voda_septik_%s", slug);

    const ha_entity_meta_t meta = infer_meta(topic);

    char object_id[224] = {0};
    snprintf(object_id, sizeof(object_id), "%s", unique_id);

    char discovery_topic[320] = {0};
    snprintf(discovery_topic,
             sizeof(discovery_topic),
             "%s/%s/%s/config",
             HA_DISCOVERY_ROOT,
             meta.component,
             object_id);

    char name[160] = {0};
    make_human_name(topic.full_topic, name, sizeof(name));

    char payload[1400] = {0};
    size_t offset = 0;
    bool first = true;

    if (snprintf(payload + offset, sizeof(payload) - offset, "{") <= 0) {
        return ESP_FAIL;
    }
    offset += 1;

    if (!append_json_field(payload, sizeof(payload), &offset, &first, "name", name, true) ||
        !append_json_field(payload, sizeof(payload), &offset, &first, "unique_id", unique_id, true) ||
        !append_json_field(payload, sizeof(payload), &offset, &first, "state_topic", topic.full_topic, true)) {
        return ESP_ERR_NO_MEM;
    }

    if (topic.id != mqtt_topic_id_t::TOPIC_SYSTEM_STATUS) {
        const mqtt_topic_descriptor_t *status_topic = mqtt_topic_descriptor(mqtt_topic_id_t::TOPIC_SYSTEM_STATUS);
        if (status_topic != nullptr && status_topic->full_topic != nullptr) {
            if (!append_json_field(payload,
                                   sizeof(payload),
                                   &offset,
                                   &first,
                                   "availability_topic",
                                   status_topic->full_topic,
                                   true) ||
                !append_json_field(payload, sizeof(payload), &offset, &first, "payload_available", "online", true) ||
                !append_json_field(payload, sizeof(payload), &offset, &first, "payload_not_available", "offline", true)) {
                return ESP_ERR_NO_MEM;
            }
        }
    }

    if (meta.payload_on != nullptr && !append_json_field(payload, sizeof(payload), &offset, &first, "payload_on", meta.payload_on, true)) {
        return ESP_ERR_NO_MEM;
    }
    if (meta.payload_off != nullptr && !append_json_field(payload, sizeof(payload), &offset, &first, "payload_off", meta.payload_off, true)) {
        return ESP_ERR_NO_MEM;
    }
    if (meta.device_class != nullptr && !append_json_field(payload, sizeof(payload), &offset, &first, "device_class", meta.device_class, true)) {
        return ESP_ERR_NO_MEM;
    }
    if (meta.unit != nullptr && !append_json_field(payload, sizeof(payload), &offset, &first, "unit_of_measurement", meta.unit, true)) {
        return ESP_ERR_NO_MEM;
    }
    if (meta.state_class != nullptr && !append_json_field(payload, sizeof(payload), &offset, &first, "state_class", meta.state_class, true)) {
        return ESP_ERR_NO_MEM;
    }
    if (meta.entity_category != nullptr && !append_json_field(payload, sizeof(payload), &offset, &first, "entity_category", meta.entity_category, true)) {
        return ESP_ERR_NO_MEM;
    }
    if (meta.icon != nullptr && !append_json_field(payload, sizeof(payload), &offset, &first, "icon", meta.icon, true)) {
        return ESP_ERR_NO_MEM;
    }
    if (meta.value_template != nullptr && !append_json_field(payload, sizeof(payload), &offset, &first, "value_template", meta.value_template, true)) {
        return ESP_ERR_NO_MEM;
    }
    if (meta.json_attributes_topic != nullptr && !append_json_field(payload,
                                                                     sizeof(payload),
                                                                     &offset,
                                                                     &first,
                                                                     "json_attributes_topic",
                                                                     meta.json_attributes_topic,
                                                                     true)) {
        return ESP_ERR_NO_MEM;
    }

    char device_json[256] = {0};
    snprintf(device_json,
             sizeof(device_json),
             "{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"model\":\"%s\",\"manufacturer\":\"%s\"}",
             DEVICE_ID,
             DEVICE_NAME,
             DEVICE_MODEL,
             DEVICE_MANUFACTURER);

    if (!append_json_field(payload,
                           sizeof(payload),
                           &offset,
                           &first,
                           "device",
                           device_json,
                           false)) {
        return ESP_ERR_NO_MEM;
    }

    if (offset + 2 >= sizeof(payload)) {
        return ESP_ERR_NO_MEM;
    }
    payload[offset++] = '}';
    payload[offset] = '\0';

    esp_err_t publish_result = mqtt_publish(discovery_topic, payload, true);
    if (publish_result != ESP_OK) {
        ESP_LOGW(TAG,
                 "HA discovery publish selhal: topic=%s err=%s",
                 discovery_topic,
                 esp_err_to_name(publish_result));
    }

    return publish_result;
}

esp_err_t mqtt_ha_discovery_publish_all(void)
{
    if (!mqtt_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t published = 0;
    esp_err_t first_error = ESP_OK;

    for (size_t index = 0; index < (size_t)mqtt_topic_id_t::COUNT; ++index) {
        const mqtt_topic_descriptor_t &topic = MQTT_TOPIC_TABLE[index];
        if (topic.direction != mqtt_topic_direction_t::PUBLISH_ONLY) {
            continue;
        }

        esp_err_t result = publish_discovery_for_topic(topic);
        if (result != ESP_OK && first_error == ESP_OK) {
            first_error = result;
        }

        if (result == ESP_OK) {
            ++published;
        }
    }

    ESP_LOGI(TAG,
             "HA discovery publikovano: %u topicu (error=%s)",
             (unsigned)published,
             esp_err_to_name(first_error));

    return first_error;
}
