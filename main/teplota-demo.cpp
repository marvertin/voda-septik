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

#include "pins.h"
#include "sensor_events.h"
#include <app_error_check.h>
#include "debug_mqtt.h"

#define TAG "TEMP_DEMO"

// GPIO pin pro 1Wire sběrnici (DS18B20)
static const gpio_num_t SENSOR_GPIO = GPIO_NUM_16;

// DS18B20 Commands
#define DS18B20_CMD_CONVERT_TEMP  0x44       // Start temperature conversion
#define DS18B20_CMD_READ_SCRATCH  0xBE       // Read scratchpad (9 bytes)
#define DS18B20_CMD_SKIP_ROM      0xCC       // Broadcast command for all devices

#define DS18B20_FAMILY_CODE       0x28

static constexpr onewire_addr_t WATER_SENSOR_ADDRESS = ONEWIRE_NONE;
static constexpr onewire_addr_t AIR_SENSOR_ADDRESS = ONEWIRE_NONE;

static constexpr int TEMPERATURE_CONVERSION_MS = 800;
static constexpr int READ_PERIOD_MS = 1000;
static constexpr int SENSOR_DISCOVERY_PERIOD_S = 30;

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
    size_t detected_count = 0;

    onewire_search_t search = {};
    onewire_search_start(&search);
    onewire_search_prefix(&search, DS18B20_FAMILY_CODE);

    while (detected_count < detected.size()) {
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
            if (detected[i] == addr) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }

        detected[detected_count++] = addr;
    }

    if (detected_count == 0) {
        ESP_LOGW(TAG, "Na 1-Wire sbernici nebyl nalezen zadny DS18B20");
        return false;
    }

    std::sort(detected.begin(), detected.begin() + detected_count);

    std::array<bool, 8> used = {};
    for (size_t probe_index = 0; probe_index < probe_count; ++probe_index) {
        const onewire_addr_t configured = probes[probe_index].configured_address;
        if (configured == ONEWIRE_NONE) {
            continue;
        }

        for (size_t found_index = 0; found_index < detected_count; ++found_index) {
            if (detected[found_index] == configured) {
                probes[probe_index].resolved_address = configured;
                probes[probe_index].available = true;
                used[found_index] = true;
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
            continue;
        }

        for (size_t found_index = 0; found_index < detected_count; ++found_index) {
            if (used[found_index]) {
                continue;
            }
            probes[probe_index].resolved_address = detected[found_index];
            probes[probe_index].available = true;
            used[found_index] = true;
            break;
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

    const char *probe_name = (probe == SENSOR_TEMPERATURE_PROBE_AIR) ? "air" : "water";
    if (read_ok) {
        DEBUG_PUBLISH("temperature",
                      "queued=%d ts=%lld probe=%s temp_c=%.4f raw_temp=%d gpio=%d",
                      queued ? 1 : 0,
                      (long long)event.timestamp_us,
                      probe_name,
                      (double)temperature,
                      (int)raw_temp,
                      (int)SENSOR_GPIO);
    } else {
        DEBUG_PUBLISH("temperature",
                      "queued=%d ts=%lld probe=%s read_failed=1 gpio=%d",
                      queued ? 1 : 0,
                      (long long)event.timestamp_us,
                      probe_name,
                      (int)SENSOR_GPIO);
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
    gpio_set_pull_mode(SENSOR_GPIO, GPIO_PULLUP_ONLY);

    ds18b20_probe_t probes[] = {
        {
            .probe = SENSOR_TEMPERATURE_PROBE_WATER,
            .name = "voda",
            .configured_address = WATER_SENSOR_ADDRESS,
            .resolved_address = ONEWIRE_NONE,
            .available = false,
        },
        {
            .probe = SENSOR_TEMPERATURE_PROBE_AIR,
            .name = "vzduch",
            .configured_address = AIR_SENSOR_ADDRESS,
            .resolved_address = ONEWIRE_NONE,
            .available = false,
        },
    };

    int64_t next_discovery_us = 0;
    
    while (1)
    {
        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_discovery_us || !probes[0].available || !probes[1].available) {
            discover_ds18b20_sensors(SENSOR_GPIO, probes, sizeof(probes) / sizeof(probes[0]));
            next_discovery_us = now_us + (int64_t)SENSOR_DISCOVERY_PERIOD_S * 1000000LL;
        }

        const bool conversion_started = ds18b20_start_conversion_all(SENSOR_GPIO);
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
                SENSOR_GPIO,
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

void teplota_demo_init(void)
{
    xTaskCreate(temperature_task, TAG, configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    // APP_ERROR_CHECK("E977", ESP_FAIL);
}
