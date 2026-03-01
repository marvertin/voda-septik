#include "mqtt_topics.h"

#define TOPIC_ENTRY(ID, PATH, DIR, KIND, QOS_VALUE, RETAIN_VALUE) \
    { \
        mqtt_topic_id_t::ID, \
        "voda/septik/" PATH, \
        mqtt_topic_direction_t::DIR, \
        mqtt_payload_kind_t::KIND, \
        QOS_VALUE, \
        RETAIN_VALUE, \
    }

const mqtt_topic_descriptor_t MQTT_TOPIC_TABLE[(size_t)mqtt_topic_id_t::COUNT] = {
    TOPIC_ENTRY(TOPIC_STAV_ZASOBA_OBJEM,                 "stav/zasoba/objem_m3",             PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_ZASOBA_HLADINA,               "stav/zasoba/hladina_m",            PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_CERPANI_PRUTOK,               "stav/cerpani/prutok_l_min",        PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_CERPANI_CERPANO_CELKEM,       "stav/cerpani/cerpano_celkem_l",    PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_TEPLOTA_VODA,                 "stav/teplota/voda",                PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_TEPLOTA_VZDUCH,               "stav/teplota/vzduch",              PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_TLAK_PRED_FILTREM,            "stav/tlak/pred_filtrem_bar",       PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_TLAK_ZA_FILTREM,              "stav/tlak/za_filtrem_bar",         PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_ROZDIL_TLAKU_FILTRU,          "stav/tlak/rozdil_filtru_bar",      PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_ZANESENOST_FILTRU_PERCENT,    "stav/zanesenost_filtru_percent",   PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_CERPANI_PUMPA_BEZI,           "stav/cerpani/pumpa/bezi",          PUBLISH_ONLY,   BOOLEAN, 1, true),
    TOPIC_ENTRY(TOPIC_STAV_CERPANI_PUMPA_VYKON_CINNY_W,  "stav/cerpani/pumpa/vykon_cinny_w", PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_CERPANI_PUMPA_JALOVY_VYKON_VAR, "stav/cerpani/pumpa/jalovy_vykon_var", PUBLISH_ONLY, NUMBER, 1, true),
    TOPIC_ENTRY(TOPIC_STAV_CERPANI_PUMPA_COSFI,          "stav/cerpani/pumpa/cosfi",         PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_CERPANI_PUMPA_PROUD_A,        "stav/cerpani/pumpa/proud_a",       PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_CERPANI_PUMPA_NAPETI_V,       "stav/cerpani/pumpa/napeti_v",      PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_STAV_CERPANI_PUMPA_ENERGIE_CINNA_KWH, "stav/cerpani/pumpa/energie_cinna_kwh", PUBLISH_ONLY, NUMBER, 1, true),
    TOPIC_ENTRY(TOPIC_STAV_CERPANI_PUMPA_ENERGIE_JALOVA_KVARH, "stav/cerpani/pumpa/energie_jalova_kvarh", PUBLISH_ONLY, NUMBER, 1, true),

    TOPIC_ENTRY(TOPIC_SYSTEM_STATUS,                     "system/status",                    PUBLISH_ONLY,   TEXT,    1, true),
    TOPIC_ENTRY(TOPIC_SYSTEM_BOOT_MODE,                  "system/boot_mode",                 PUBLISH_ONLY,   TEXT,    1, true),
    TOPIC_ENTRY(TOPIC_SYSTEM_OTA_EVENT,                  "system/ota/event",                 PUBLISH_ONLY,   TEXT,    1, false),
    TOPIC_ENTRY(TOPIC_SYSTEM_OTA_PROGRESS,               "system/ota/progress",              PUBLISH_ONLY,   NUMBER,  1, false),
    TOPIC_ENTRY(TOPIC_SYSTEM_REBOOT_REASON,              "system/reboot_reason",             PUBLISH_ONLY,   TEXT,    1, true),
    TOPIC_ENTRY(TOPIC_SYSTEM_REBOOT_COUNTER,             "system/reboot_counter",            PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_SYSTEM_LAST_DISCONNECT_DURATION_S, "system/last_disconnect_duration_s", PUBLISH_ONLY,  NUMBER,  1, true),

    TOPIC_ENTRY(TOPIC_DIAG_FW_VERSION,                   "diag/fw_version",                  PUBLISH_ONLY,   TEXT,    1, true),
    TOPIC_ENTRY(TOPIC_DIAG_BUILD_TIMESTAMP,              "diag/build_timestamp",             PUBLISH_ONLY,   TEXT,    1, true),
    TOPIC_ENTRY(TOPIC_DIAG_GIT_HASH,                     "diag/git_hash",                    PUBLISH_ONLY,   TEXT,    1, true),
    TOPIC_ENTRY(TOPIC_DIAG_UPTIME_S,                     "diag/uptime_s",                    PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_DIAG_WIFI_RSSI_DBM,                "diag/wifi_rssi_dbm",               PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_DIAG_WIFI_RECONNECT_TRY,           "diag/wifi_reconnect_try",          PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_DIAG_WIFI_RECONNECT_SUCCESS,       "diag/wifi_reconnect_success",      PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_DIAG_MQTT_RECONNECTS,              "diag/mqtt_reconnects",             PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_DIAG_LAST_MQTT_RC,                 "diag/last_mqtt_rc",                PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_DIAG_HEAP_FREE_B,                  "diag/heap_free_b",                 PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_DIAG_HEAP_MIN_FREE_B,              "diag/heap_min_free_b",             PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_DIAG_ESP_VCC_MV,                   "diag/esp_vcc_mv",                  PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_DIAG_NVS_ERRORS,                   "diag/nvs_errors",                  PUBLISH_ONLY,   NUMBER,  1, true),
    TOPIC_ENTRY(TOPIC_DIAG_TEPLOTA_SCAN,                 "diag/teplota_scan",                PUBLISH_ONLY,   JSON,    1, false),

    TOPIC_ENTRY(TOPIC_CMD_REBOOT,                        "cmd/reboot",                       SUBSCRIBE_ONLY, TEXT,    1, false),
    TOPIC_ENTRY(TOPIC_CMD_WEBAPP,                        "cmd/webapp",                       SUBSCRIBE_ONLY, TEXT,    1, false),
    TOPIC_ENTRY(TOPIC_CMD_DEBUG,                         "cmd/debug",                        SUBSCRIBE_ONLY, TEXT,    1, false),
    TOPIC_ENTRY(TOPIC_CMD_LOG_LEVEL,                     "cmd/log/level",                    SUBSCRIBE_ONLY, TEXT,    1, false),
    TOPIC_ENTRY(TOPIC_CMD_OTA_START,                     "cmd/ota/start",                    SUBSCRIBE_ONLY, TEXT,    1, false),
    TOPIC_ENTRY(TOPIC_CMD_OTA_CONFIRM,                   "cmd/ota/confirm",                  SUBSCRIBE_ONLY, TEXT,    1, false),
    TOPIC_ENTRY(TOPIC_CMD_TEPLOTA_SCAN,                  "cmd/teplota/scan",                 SUBSCRIBE_ONLY, TEXT,    1, false),
};

#undef TOPIC_ENTRY

static_assert(sizeof(MQTT_TOPIC_TABLE) / sizeof(MQTT_TOPIC_TABLE[0]) ==
                  (size_t)mqtt_topic_id_t::COUNT,
              "MQTT topic table must match mqtt_topic_id_t::COUNT");

const mqtt_topic_descriptor_t *mqtt_topic_descriptor(mqtt_topic_id_t id)
{
    const size_t index = (size_t)id;
    if (index >= (size_t)mqtt_topic_id_t::COUNT) {
        return nullptr;
    }
    return &MQTT_TOPIC_TABLE[index];
}
