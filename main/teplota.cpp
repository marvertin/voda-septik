#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <onewire.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <driver/gpio.h>

#ifdef __cplusplus
}
#endif

#include <algorithm>
#include <array>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app-config.h"
#include "pins.h"
#include "sensor_events.h"
#include "mqtt_topics.h"
#include "mqtt_publish.h"
#include <app_error_check.h>
#include "debug_mqtt.h"

#define TAG "TEMP"

// DS18B20 Commands
#define DS18B20_CMD_CONVERT_TEMP  0x44       // Start temperature conversion
#define DS18B20_CMD_READ_SCRATCH  0xBE       // Read scratchpad (9 bytes)
#define DS18B20_CMD_SKIP_ROM      0xCC       // Broadcast command for all devices

#define DS18B20_FAMILY_CODE       0x28

static constexpr int TEMPERATURE_CONVERSION_MS = 800;
static constexpr int READ_PERIOD_MS = 1000;
static constexpr int SENSOR_DISCOVERY_PERIOD_S = 30;
static constexpr int ADDRESS_SCAN_PUBLISH_PERIOD_S = 5;

typedef struct {
    std::array<uint8_t, 9> bytes;
} ds18b20_scratchpad_t;

typedef struct {
    sensor_temperature_probe_t probe;
    const char *name;
    onewire_addr_t configured_address;
    onewire_addr_t resolved_address;
    bool available;
} ds18b20_probe_t;

typedef struct {
    onewire_addr_t water;
    onewire_addr_t air;
} ds18b20_config_addresses_t;

static portMUX_TYPE s_scan_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_scan_enabled = false;

static bool ds18b20_addr_is_valid(onewire_addr_t addr);

static const char *probe_name(sensor_temperature_probe_t probe)
{
    return (probe == SENSOR_TEMPERATURE_PROBE_AIR) ? "air" : "water";
}

static void format_onewire_addr(onewire_addr_t addr, char *out, size_t out_len)
{
    if (out == nullptr || out_len == 0) {
        return;
    }
    if (addr == ONEWIRE_NONE) {
        out[0] = '\0';
        return;
    }
    snprintf(out, out_len, "0x%016" PRIx64, (uint64_t)addr);
}

static bool parse_onewire_addr(const char *text, onewire_addr_t *out)
{
    if (out == nullptr) {
        return false;
    }
    *out = ONEWIRE_NONE;

    if (text == nullptr) {
        return false;
    }

    while (*text != '\0' && isspace((unsigned char)*text) != 0) {
        ++text;
    }

    if (*text == '\0') {
        return true;
    }

    if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
        text += 2;
    }

    char normalized[17] = {0};
    size_t idx = 0;
    while (*text != '\0' && idx < 16) {
        if (isspace((unsigned char)*text) != 0) {
            ++text;
            continue;
        }
        if (!isxdigit((unsigned char)*text)) {
            return false;
        }
        normalized[idx++] = *text;
        ++text;
    }

    while (*text != '\0') {
        if (!isspace((unsigned char)*text)) {
            return false;
        }
        ++text;
    }

    if (idx != 16) {
        return false;
    }

    char *end = nullptr;
    const uint64_t parsed = strtoull(normalized, &end, 16);
    if (end == nullptr || *end != '\0') {
        return false;
    }

    *out = (onewire_addr_t)parsed;
    return true;
}

static ds18b20_config_addresses_t load_configured_addresses(void)
{
    ds18b20_config_addresses_t cfg = {
        .water = ONEWIRE_NONE,
        .air = ONEWIRE_NONE,
    };

    char water_text[24] = {0};
    char air_text[24] = {0};
    const esp_err_t load_result = app_config_load_temperature_addresses(water_text,
                                                                        sizeof(water_text),
                                                                        air_text,
                                                                        sizeof(air_text));
    if (load_result != ESP_OK) {
        ESP_LOGW(TAG, "Adresy teplotnich cidel nebyly nacteny z konfigurace: %s", esp_err_to_name(load_result));
        return cfg;
    }

    onewire_addr_t parsed = ONEWIRE_NONE;
    if (parse_onewire_addr(water_text, &parsed) && (parsed == ONEWIRE_NONE || ds18b20_addr_is_valid(parsed))) {
        cfg.water = parsed;
    } else {
        ESP_LOGW(TAG, "Neplatna adresa temp_addr_water='%s'", water_text);
    }

    parsed = ONEWIRE_NONE;
    if (parse_onewire_addr(air_text, &parsed) && (parsed == ONEWIRE_NONE || ds18b20_addr_is_valid(parsed))) {
        cfg.air = parsed;
    } else {
        ESP_LOGW(TAG, "Neplatna adresa temp_addr_air='%s'", air_text);
    }

    char water_fmt[24] = {0};
    char air_fmt[24] = {0};
    format_onewire_addr(cfg.water, water_fmt, sizeof(water_fmt));
    format_onewire_addr(cfg.air, air_fmt, sizeof(air_fmt));
    ESP_LOGI(TAG, "Konfigurace DS18B20 adres: water=%s air=%s",
             water_fmt[0] != '\0' ? water_fmt : "(unset)",
             air_fmt[0] != '\0' ? air_fmt : "(unset)");

    return cfg;
}

static bool ds18b20_addr_is_valid(onewire_addr_t addr)
{
    if (addr == ONEWIRE_NONE) {
        return false;
    }

    uint8_t rom[8];
    for (size_t index = 0; index < 8; ++index) {
        rom[index] = (uint8_t)((addr >> (index * 8U)) & 0xFFU);
    }

    if (rom[0] != DS18B20_FAMILY_CODE) {
        return false;
    }

    return onewire_crc8(rom, 7) == rom[7];
}

static bool ds18b20_read_temperature_by_address(gpio_num_t gpio,
                                                onewire_addr_t address,
                                                float *temp,
                                                int16_t *raw_temp_out)
{
    if (temp == nullptr || !ds18b20_addr_is_valid(address)) {
        return false;
    }

    if (!onewire_reset(gpio)) {
        ESP_LOGE(TAG, "Chyba: reset busu pred ctenim senzoru 0x%016" PRIx64 " selhal", (uint64_t)address);
        return false;
    }

    if (!onewire_select(gpio, address)) {
        ESP_LOGE(TAG, "Chyba: select ROM senzoru 0x%016" PRIx64 " selhal", (uint64_t)address);
        return false;
    }

    if (!onewire_write(gpio, DS18B20_CMD_READ_SCRATCH)) {
        ESP_LOGE(TAG, "Chyba: Read Scratchpad pro 0x%016" PRIx64 " selhal", (uint64_t)address);
        return false;
    }

    ds18b20_scratchpad_t scratch = {};
    if (!onewire_read_bytes(gpio, scratch.bytes.data(), scratch.bytes.size())) {
        ESP_LOGE(TAG, "Chyba: cteni scratchpadu pro 0x%016" PRIx64 " selhalo", (uint64_t)address);
        return false;
    }

    if (onewire_crc8(scratch.bytes.data(), 8) != scratch.bytes[8]) {
        ESP_LOGE(TAG, "Chyba: CRC scratchpadu pro 0x%016" PRIx64 " nesouhlasi", (uint64_t)address);
        return false;
    }

    const int16_t raw_temp = (int16_t)(((uint16_t)scratch.bytes[1] << 8) | scratch.bytes[0]);
    if (raw_temp_out != nullptr) {
        *raw_temp_out = raw_temp;
    }

    *temp = (float)raw_temp / 16.0f;
    return true;
}

static bool ds18b20_start_conversion_all(gpio_num_t gpio)
{
    if (!onewire_reset(gpio)) {
        ESP_LOGE(TAG, "Chyba: senzor neodpovedel na reset pred konverzi");
        return false;
    }

    if (!onewire_skip_rom(gpio)) {
        ESP_LOGE(TAG, "Chyba: Skip ROM pred konverzi selhal");
        return false;
    }

    if (!onewire_write(gpio, DS18B20_CMD_CONVERT_TEMP)) {
        ESP_LOGE(TAG, "Chyba: poslat Convert T prikaz selhalo");
        return false;
    }

    return true;
}

static size_t detect_ds18b20_addresses(gpio_num_t gpio,
                                       std::array<onewire_addr_t, 8> *detected)
{
    if (detected == nullptr) {
        return 0;
    }

    detected->fill(ONEWIRE_NONE);
    size_t detected_count = 0;

    onewire_search_t search = {};
    onewire_search_start(&search);
    onewire_search_prefix(&search, DS18B20_FAMILY_CODE);

    while (detected_count < detected->size()) {
        const onewire_addr_t addr = onewire_search_next(&search, gpio);
        if (addr == ONEWIRE_NONE) {
            break;
        }

        if (!ds18b20_addr_is_valid(addr)) {
            ESP_LOGW(TAG, "Preskakuji neplatnou adresu senzoru 0x%016" PRIx64, (uint64_t)addr);
            continue;
        }

        bool duplicate = false;
        for (size_t i = 0; i < detected_count; ++i) {
            if ((*detected)[i] == addr) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }

        (*detected)[detected_count++] = addr;
    }

    std::sort(detected->begin(), detected->begin() + detected_count);
    return detected_count;
}

static void publish_address_scan_report(gpio_num_t gpio,
                                        const ds18b20_probe_t *probes,
                                        size_t probe_count,
                                        const std::array<onewire_addr_t, 8> &detected,
                                        size_t detected_count)
{
    const mqtt_topic_descriptor_t *topic = mqtt_topic_descriptor(mqtt_topic_id_t::TOPIC_DIAG_TEPLOTA_SCAN);
    if (topic == nullptr || topic->full_topic == nullptr || !mqtt_is_connected()) {
        return;
    }

    char payload[1024] = {0};
    size_t offset = 0;

    offset += (size_t)snprintf(payload + offset,
                               sizeof(payload) - offset,
                               "{\"scan_enabled\":1,\"found\":[");

    for (size_t i = 0; i < detected_count && offset < sizeof(payload); ++i) {
        if (i > 0) {
            offset += (size_t)snprintf(payload + offset, sizeof(payload) - offset, ",");
        }

        const onewire_addr_t found_addr = detected[i];
        char addr_text[24] = {0};
        format_onewire_addr(found_addr, addr_text, sizeof(addr_text));

        float temp_c = NAN;
        int16_t raw_temp = 0;
        const bool read_ok = ds18b20_read_temperature_by_address(gpio, found_addr, &temp_c, &raw_temp);

        const char *matches_water = "false";
        const char *matches_air = "false";
        for (size_t probe_idx = 0; probe_idx < probe_count; ++probe_idx) {
            const ds18b20_probe_t &probe = probes[probe_idx];
            if (probe.configured_address == ONEWIRE_NONE) {
                continue;
            }
            if (probe.configured_address == found_addr) {
                if (probe.probe == SENSOR_TEMPERATURE_PROBE_WATER) {
                    matches_water = "true";
                }
                if (probe.probe == SENSOR_TEMPERATURE_PROBE_AIR) {
                    matches_air = "true";
                }
            }
        }

        if (read_ok) {
            offset += (size_t)snprintf(payload + offset,
                                       sizeof(payload) - offset,
                                       "{\"addr\":\"%s\",\"temp_c\":%.4f,\"read_ok\":true,\"matches\":{\"water\":%s,\"air\":%s}}",
                                       addr_text,
                                       (double)temp_c,
                                       matches_water,
                                       matches_air);
        } else {
            offset += (size_t)snprintf(payload + offset,
                                       sizeof(payload) - offset,
                                       "{\"addr\":\"%s\",\"temp_c\":null,\"read_ok\":false,\"matches\":{\"water\":%s,\"air\":%s}}",
                                       addr_text,
                                       matches_water,
                                       matches_air);
        }
    }

    char water_cfg[24] = {0};
    char air_cfg[24] = {0};
    for (size_t probe_idx = 0; probe_idx < probe_count; ++probe_idx) {
        if (probes[probe_idx].probe == SENSOR_TEMPERATURE_PROBE_WATER) {
            format_onewire_addr(probes[probe_idx].configured_address, water_cfg, sizeof(water_cfg));
        } else if (probes[probe_idx].probe == SENSOR_TEMPERATURE_PROBE_AIR) {
            format_onewire_addr(probes[probe_idx].configured_address, air_cfg, sizeof(air_cfg));
        }
    }

    snprintf(payload + offset,
             sizeof(payload) - offset,
             "],\"configured\":{\"water\":\"%s\",\"air\":\"%s\"}}",
             water_cfg,
             air_cfg);

    mqtt_publish(topic->full_topic, payload, topic->retain);
}

static bool discover_ds18b20_sensors(gpio_num_t gpio, ds18b20_probe_t *probes, size_t probe_count)
{
    if (probes == nullptr || probe_count == 0) {
        return false;
    }

    for (size_t i = 0; i < probe_count; ++i) {
        probes[i].resolved_address = ONEWIRE_NONE;
        probes[i].available = false;
    }

    std::array<onewire_addr_t, 8> detected = {};
    const size_t detected_count = detect_ds18b20_addresses(gpio, &detected);

    if (detected_count == 0) {
        ESP_LOGW(TAG, "Na 1-Wire sbernici nebyl nalezen zadny DS18B20");
        return false;
    }

    for (size_t probe_index = 0; probe_index < probe_count; ++probe_index) {
        const onewire_addr_t configured = probes[probe_index].configured_address;
        if (configured == ONEWIRE_NONE) {
            ESP_LOGW(TAG, "Senzor %s nema nastavenou adresu ve flash konfiguraci", probes[probe_index].name);
            continue;
        }

        for (size_t found_index = 0; found_index < detected_count; ++found_index) {
            if (detected[found_index] == configured) {
                probes[probe_index].resolved_address = configured;
                probes[probe_index].available = true;
                break;
            }
        }

        if (!probes[probe_index].available) {
            ESP_LOGW(TAG,
                     "Konfigurovany senzor %s 0x%016" PRIx64 " nebyl nalezen",
                     probes[probe_index].name,
                     (uint64_t)configured);
        }
    }

    for (size_t probe_index = 0; probe_index < probe_count; ++probe_index) {
        if (probes[probe_index].available) {
            ESP_LOGI(TAG,
                     "Senzor %s mapovan na ROM 0x%016" PRIx64,
                     probes[probe_index].name,
                     (uint64_t)probes[probe_index].resolved_address);
        } else {
            ESP_LOGW(TAG, "Senzor %s neni dostupny", probes[probe_index].name);
        }
    }

    return true;
}

static void publish_temperature_event(sensor_temperature_probe_t probe, bool read_ok, float temperature, int16_t raw_temp)
{
    app_event_t event = {
        .event_type = EVT_SENSOR,
        .timestamp_us = esp_timer_get_time(),
        .data = {
            .sensor = {
                .sensor_type = SENSOR_EVENT_TEMPERATURE,
                .data = {
                    .temperature = {
                        .temperature_c = read_ok ? temperature : NAN,
                        .probe = probe,
                    },
                },
            },
        },
    };

    const bool queued = sensor_events_publish(&event, pdMS_TO_TICKS(50));
    if (!queued) {
        ESP_LOGW(TAG, "Fronta sensor eventu je plna, teplota zahozena (probe=%d)", (int)probe);
    }

    const char *name = probe_name(probe);
    if (read_ok) {
        DEBUG_PUBLISH("temperature",
                      "queued=%d ts=%lld probe=%s temp_c=%.4f raw_temp=%d gpio=%d",
                      queued ? 1 : 0,
                      (long long)event.timestamp_us,
                      name,
                      (double)temperature,
                      (int)raw_temp,
                      (int)TEMPERATURE_SENSOR_GPIO);
    } else {
        DEBUG_PUBLISH("temperature",
                      "queued=%d ts=%lld probe=%s read_failed=1 gpio=%d",
                      queued ? 1 : 0,
                      (long long)event.timestamp_us,
                      name,
                      (int)TEMPERATURE_SENSOR_GPIO);
    }
}

/**
 * Čte dvě DS18B20 čidla (voda + vzduch) na jedné 1-Wire sběrnici.
 * Pokud nejsou zadané konkrétní ROM adresy, mapování se udělá automaticky
 * podle pořadí adres (nižší ROM = voda, vyšší ROM = vzduch).
 */
static void temperature_task(void *pvParameters)
{
    (void)pvParameters;

    // Nastavení pull-up rezistoru na GPIO pinu
    gpio_set_pull_mode(TEMPERATURE_SENSOR_GPIO, GPIO_PULLUP_ONLY);

    const ds18b20_config_addresses_t cfg_addresses = load_configured_addresses();

    ds18b20_probe_t probes[] = {
        {
            .probe = SENSOR_TEMPERATURE_PROBE_WATER,
            .name = "voda",
            .configured_address = cfg_addresses.water,
            .resolved_address = ONEWIRE_NONE,
            .available = false,
        },
        {
            .probe = SENSOR_TEMPERATURE_PROBE_AIR,
            .name = "vzduch",
            .configured_address = cfg_addresses.air,
            .resolved_address = ONEWIRE_NONE,
            .available = false,
        },
    };

    int64_t next_discovery_us = 0;
    int64_t next_scan_publish_us = 0;
    
    while (1)
    {
        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_discovery_us || !probes[0].available || !probes[1].available) {
            discover_ds18b20_sensors(TEMPERATURE_SENSOR_GPIO, probes, sizeof(probes) / sizeof(probes[0]));
            next_discovery_us = now_us + (int64_t)SENSOR_DISCOVERY_PERIOD_S * 1000000LL;
        }

        bool scan_enabled = false;
        taskENTER_CRITICAL(&s_scan_mux);
        scan_enabled = s_scan_enabled;
        taskEXIT_CRITICAL(&s_scan_mux);

        if (scan_enabled && now_us >= next_scan_publish_us) {
            std::array<onewire_addr_t, 8> detected = {};
            const size_t detected_count = detect_ds18b20_addresses(TEMPERATURE_SENSOR_GPIO, &detected);
            publish_address_scan_report(TEMPERATURE_SENSOR_GPIO,
                                        probes,
                                        sizeof(probes) / sizeof(probes[0]),
                                        detected,
                                        detected_count);
            next_scan_publish_us = now_us + (int64_t)ADDRESS_SCAN_PUBLISH_PERIOD_S * 1000000LL;
        }

        const bool conversion_started = ds18b20_start_conversion_all(TEMPERATURE_SENSOR_GPIO);
        if (conversion_started) {
            vTaskDelay(pdMS_TO_TICKS(TEMPERATURE_CONVERSION_MS));
        } else {
            ESP_LOGE(TAG, "Nebylo mozne spustit hromadnou konverzi teplot");
        }

        for (size_t index = 0; index < sizeof(probes) / sizeof(probes[0]); ++index) {
            ds18b20_probe_t &probe = probes[index];

            if (!probe.available || !conversion_started) {
                publish_temperature_event(probe.probe, false, 0.0f, 0);
                continue;
            }

            float temperature = NAN;
            int16_t raw_temp = 0;
            const bool read_ok = ds18b20_read_temperature_by_address(
                TEMPERATURE_SENSOR_GPIO,
                probe.resolved_address,
                &temperature,
                &raw_temp);

            if (read_ok) {
                ESP_LOGI(TAG, "Teplota (%s): %.2f °C", probe.name, temperature);
            } else {
                ESP_LOGE(TAG, "Nebylo mozne precist teplotu (%s)", probe.name);
            }

            publish_temperature_event(probe.probe, read_ok, temperature, raw_temp);
        }
        
        // Čtení každou sekundu
        vTaskDelay(pdMS_TO_TICKS(READ_PERIOD_MS));
    }
}

void teplota_init(void)
{
    xTaskCreate(temperature_task, TAG, configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    // APP_ERROR_CHECK("E977", ESP_FAIL);
}

esp_err_t teplota_set_scan_enabled(bool enabled)
{
    taskENTER_CRITICAL(&s_scan_mux);
    s_scan_enabled = enabled;
    taskEXIT_CRITICAL(&s_scan_mux);

    ESP_LOGW(TAG, "Address scan mode: %s", enabled ? "ON" : "OFF");
    return ESP_OK;
}

bool teplota_scan_enabled(void)
{
    bool enabled = false;
    taskENTER_CRITICAL(&s_scan_mux);
    enabled = s_scan_enabled;
    taskEXIT_CRITICAL(&s_scan_mux);
    return enabled;
}
