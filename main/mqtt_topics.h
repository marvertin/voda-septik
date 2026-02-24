#pragma once

#include <stddef.h>
#include <stdint.h>

static constexpr const char *MQTT_TOPIC_ROOT = "zalivka/nadrz";

enum class mqtt_topic_id_t : uint16_t {
    TOPIC_STAV_OBJEM = 0,
    TOPIC_STAV_PRUTOK,
    TOPIC_STAV_CERPANO_CELKEM,
    TOPIC_STAV_TEPLOTA_VODA,
    TOPIC_STAV_TEPLOTA_NADRZ,
    TOPIC_STAV_TLAK_PRED_FILTREM,
    TOPIC_STAV_TLAK_ZA_FILTREM,
    TOPIC_STAV_ROZDIL_TLAKU_FILTRU,
    TOPIC_STAV_ZANESENOST_FILTRU_PERCENT,
    TOPIC_STAV_PUMPA_BEZI,
    TOPIC_STAV_PUMPA_VYKON_CINNY_W,
    TOPIC_STAV_PUMPA_JALOVY_VYKON_VAR,
    TOPIC_STAV_PUMPA_COSFI,
    TOPIC_STAV_PUMPA_PROUD_A,
    TOPIC_STAV_PUMPA_NAPETI_V,
    TOPIC_STAV_PUMPA_ENERGIE_CINNA_KWH,
    TOPIC_STAV_PUMPA_ENERGIE_JALOVA_KVARH,

    TOPIC_SYSTEM_STATUS,
    TOPIC_SYSTEM_REBOOT_REASON,
    TOPIC_SYSTEM_REBOOT_COUNTER,
    TOPIC_SYSTEM_LAST_DISCONNECT_DURATION_S,

    TOPIC_DIAG_FW_VERSION,
    TOPIC_DIAG_BUILD_TIMESTAMP,
    TOPIC_DIAG_GIT_HASH,
    TOPIC_DIAG_UPTIME_S,
    TOPIC_DIAG_WIFI_RSSI_DBM,
    TOPIC_DIAG_WIFI_RECONNECT_TRY,
    TOPIC_DIAG_WIFI_RECONNECT_SUCCESS,
    TOPIC_DIAG_MQTT_RECONNECTS,
    TOPIC_DIAG_LAST_MQTT_RC,
    TOPIC_DIAG_HEAP_FREE_B,
    TOPIC_DIAG_HEAP_MIN_FREE_B,
    TOPIC_DIAG_ESP_VCC_MV,
    TOPIC_DIAG_NVS_ERRORS,

    TOPIC_CMD_REBOOT,
    TOPIC_CMD_WEBAPP_START,
    TOPIC_CMD_WEBAPP_STOP,
    TOPIC_CMD_DEBUG_START,
    TOPIC_CMD_DEBUG_STOP,
    TOPIC_CMD_DEBUG_INTERVAL_MS,
    TOPIC_CMD_DEBUG_SENSORS,

    TOPIC_DEBUG_RAW,
    TOPIC_DEBUG_INTERMEDIATE,

    COUNT,
};

enum class mqtt_topic_direction_t : uint8_t {
    PUBLISH_ONLY = 0,
    SUBSCRIBE_ONLY,
};

enum class mqtt_payload_kind_t : uint8_t {
    NUMBER = 0,
    BOOLEAN,
    TEXT,
    JSON,
};

struct mqtt_topic_descriptor_t {
    mqtt_topic_id_t id;
    const char *full_topic;

    mqtt_topic_direction_t direction;
    mqtt_payload_kind_t payload_kind;
    uint8_t qos;
    bool retain;
};

extern const mqtt_topic_descriptor_t MQTT_TOPIC_TABLE[(size_t)mqtt_topic_id_t::COUNT];

const mqtt_topic_descriptor_t *mqtt_topic_descriptor(mqtt_topic_id_t id);
