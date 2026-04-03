#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/uart.h>

#ifdef __cplusplus
}
#endif

#include <cstdint>
#include <cstring>
#include <cmath>
#include <limits>

#include "kws_303l.h"
#include "config_store.h"
#include "app_error_check.h"
#include "pins.h"

#define TAG "kws_303l"

namespace {

static constexpr int32_t KWS_DEFAULT_SLAVE_ADDR = 5;
static constexpr int32_t KWS_DEFAULT_SAMPLE_MS = 1000;
static constexpr uart_port_t KWS_UART_PORT = UART_NUM_2;
static constexpr int32_t KWS_UART_BAUD = 9600;
static constexpr TickType_t KWS_MODBUS_RX_TIMEOUT_TICKS = pdMS_TO_TICKS(200);
static constexpr TickType_t KWS_RS485_TURNAROUND_TICKS = pdMS_TO_TICKS(2);
static constexpr uint8_t KWS_MODBUS_FUNC_READ_HOLDING = 0x03;
static constexpr uint8_t KWS_MODBUS_FUNC_READ_INPUT = 0x04;
static constexpr int32_t KWS_HARDCODED_SLAVE_ADDR_A = 165;
static constexpr int32_t KWS_HARDCODED_SLAVE_ADDR_B = 1;

static constexpr uint16_t KWS_REG_VOLTAGE = 14;
static constexpr uint16_t KWS_REG_CURRENT = 18;
static constexpr uint16_t KWS_REG_POWER = 26;
static constexpr uint16_t KWS_REG_ENERGY_TOTAL = 55;
static constexpr uint16_t KWS_REG_ENERGY_AUX = 61;
static constexpr uint16_t KWS_REG_APPARENT_POWER = 42;
static constexpr uint16_t KWS_REG_REACTIVE_POWER = 34;
static constexpr uint16_t KWS_REG_FREQUENCY = 72;

static constexpr uint16_t TAC_REG_VOLTAGE = 0x0000;
static constexpr uint16_t TAC_REG_CURRENT = 0x0006;
static constexpr uint16_t TAC_REG_POWER = 0x000C;
static constexpr uint16_t TAC_REG_REACTIVE_POWER = 0x0012;
static constexpr uint16_t TAC_REG_APPARENT_POWER = 0x0018;
static constexpr uint16_t TAC_REG_FREQUENCY = 0x0030;
static constexpr uint16_t TAC_REG_TOTAL_ACTIVE_ENERGY_KWH = 0x0504;

static constexpr float KWS_VOLTAGE_DIV = 100.0f;
static constexpr float KWS_CURRENT_DIV = 1000.0f;
static constexpr float KWS_POWER_DIV = 10.0f;
static constexpr float KWS_APPARENT_POWER_DIV = 10.0f;
static constexpr float KWS_FREQUENCY_DIV = 1.0f;

static constexpr uint16_t KWS_ENERGY_SCAN_START = 32;
static constexpr uint16_t KWS_ENERGY_SCAN_END = 80;
static constexpr uint32_t KWS_ENERGY_SCAN_EVERY_N_CYCLES = 10;
static constexpr bool KWS_ENABLE_DIAG_SCANS = false;

static const config_item_t KWS_SLAVE_ADDR_ITEM = {
    .key = "kws_addr", .label = "KWS-303L Modbus adresa", .description = "Adresa Modbus slave (1-247).",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = KWS_DEFAULT_SLAVE_ADDR, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 1, .max_int = 247, .min_float = 0.0f, .max_float = 0.0f,
};

static const config_item_t KWS_SAMPLE_MS_ITEM = {
    .key = "kws_sample_ms", .label = "KWS-303L perioda [ms]", .description = "Perioda debug dotazu KWS-303L.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = KWS_DEFAULT_SAMPLE_MS, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 100, .max_int = 5000, .min_float = 0.0f, .max_float = 0.0f,
};

typedef struct {
    int32_t slave_addr;
    int32_t sample_ms;
} kws_config_t;

static kws_config_t s_cfg = {
    .slave_addr = KWS_DEFAULT_SLAVE_ADDR,
    .sample_ms = KWS_DEFAULT_SAMPLE_MS,
};

static int32_t s_active_slave_addr = KWS_HARDCODED_SLAVE_ADDR_A;

enum meter_type_t {
    METER_TYPE_KWS,
    METER_TYPE_TAC1100,
};

enum class reg_read_kind_t : uint8_t {
    NONE = 0,
    HOLDING_U16,
    INPUT_FLOAT,
};

typedef struct {
    reg_read_kind_t kind;
    uint16_t reg;
    float multiplier;
} reg_binding_t;

typedef struct {
    float voltage_v;
    float current_a;
    float power_w;
    float reactive_power_var;
    float apparent_power_va;
    float frequency_hz;
    float energy_total_wh;
    float energy_aux_wh;
} meter_values_t;

typedef struct {
    const char *name;
    reg_binding_t voltage;
    reg_binding_t current;
    reg_binding_t power;
    reg_binding_t reactive_power;
    reg_binding_t apparent_power;
    reg_binding_t frequency;
    reg_binding_t energy_total;
    reg_binding_t energy_aux;
} meter_register_map_t;

static constexpr uint32_t METER_READ_RETRIES = 3;

static const meter_register_map_t KWS_METER_MAP = {
    .name = "KWS",
    .voltage = {reg_read_kind_t::HOLDING_U16, KWS_REG_VOLTAGE, 0.01f},
    .current = {reg_read_kind_t::HOLDING_U16, KWS_REG_CURRENT, 0.001f},
    .power = {reg_read_kind_t::HOLDING_U16, KWS_REG_POWER, 0.1f},
    .reactive_power = {reg_read_kind_t::HOLDING_U16, KWS_REG_REACTIVE_POWER, 0.1f},
    .apparent_power = {reg_read_kind_t::HOLDING_U16, KWS_REG_APPARENT_POWER, 0.1f},
    .frequency = {reg_read_kind_t::HOLDING_U16, KWS_REG_FREQUENCY, 1.0f},
    .energy_total = {reg_read_kind_t::HOLDING_U16, KWS_REG_ENERGY_TOTAL, 1.0f},
    .energy_aux = {reg_read_kind_t::HOLDING_U16, KWS_REG_ENERGY_AUX, 1.0f},
};

static const meter_register_map_t TAC_METER_MAP = {
    .name = "TAC1100",
    .voltage = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_VOLTAGE, 1.0f},
    .current = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_CURRENT, 1.0f},
    .power = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_POWER, 1.0f},
    .reactive_power = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_REACTIVE_POWER, 1.0f},
    .apparent_power = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_APPARENT_POWER, 1.0f},
    .frequency = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_FREQUENCY, 1.0f},
    .energy_total = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_TOTAL_ACTIVE_ENERGY_KWH, 1000.0f},
    .energy_aux = {reg_read_kind_t::NONE, 0, 0.0f},
};

static meter_type_t meter_type_for_addr(int32_t addr)
{
    return (addr == KWS_HARDCODED_SLAVE_ADDR_B) ? METER_TYPE_TAC1100 : METER_TYPE_KWS;
}

static inline float float_nan()
{
    return std::numeric_limits<float>::quiet_NaN();
}

static inline bool binding_enabled(const reg_binding_t &binding)
{
    return binding.kind != reg_read_kind_t::NONE;
}

static bool modbus_read_u16(uint16_t reg, uint16_t *value);
static bool modbus_read_input_float(uint16_t reg, float *value);

static bool read_binding_value(const reg_binding_t &binding, float *value)
{
    if (value == nullptr) {
        return false;
    }

    if (binding.kind == reg_read_kind_t::HOLDING_U16) {
        uint16_t raw = 0;
        if (!modbus_read_u16(binding.reg, &raw)) {
            return false;
        }
        *value = (float)raw * binding.multiplier;
        return true;
    }

    if (binding.kind == reg_read_kind_t::INPUT_FLOAT) {
        float raw = 0.0f;
        if (!modbus_read_input_float(binding.reg, &raw)) {
            return false;
        }
        *value = raw * binding.multiplier;
        return true;
    }

    return false;
}

static bool values_complete(const meter_values_t &values, const meter_register_map_t &map)
{
    if (binding_enabled(map.voltage) && std::isnan(values.voltage_v)) {
        return false;
    }
    if (binding_enabled(map.current) && std::isnan(values.current_a)) {
        return false;
    }
    if (binding_enabled(map.power) && std::isnan(values.power_w)) {
        return false;
    }
    if (binding_enabled(map.reactive_power) && std::isnan(values.reactive_power_var)) {
        return false;
    }
    if (binding_enabled(map.apparent_power) && std::isnan(values.apparent_power_va)) {
        return false;
    }
    if (binding_enabled(map.frequency) && std::isnan(values.frequency_hz)) {
        return false;
    }
    if (binding_enabled(map.energy_total) && std::isnan(values.energy_total_wh)) {
        return false;
    }
    if (binding_enabled(map.energy_aux) && std::isnan(values.energy_aux_wh)) {
        return false;
    }
    return true;
}

static void read_values_once(const meter_register_map_t &map, meter_values_t *values)
{
    if (values == nullptr) {
        return;
    }

    float v = 0.0f;
    if (std::isnan(values->voltage_v) && binding_enabled(map.voltage) && read_binding_value(map.voltage, &v)) {
        values->voltage_v = v;
    }
    if (std::isnan(values->current_a) && binding_enabled(map.current) && read_binding_value(map.current, &v)) {
        values->current_a = v;
    }
    if (std::isnan(values->power_w) && binding_enabled(map.power) && read_binding_value(map.power, &v)) {
        values->power_w = v;
    }
    if (std::isnan(values->reactive_power_var) && binding_enabled(map.reactive_power) && read_binding_value(map.reactive_power, &v)) {
        values->reactive_power_var = v;
    }
    if (std::isnan(values->apparent_power_va) && binding_enabled(map.apparent_power) && read_binding_value(map.apparent_power, &v)) {
        values->apparent_power_va = v;
    }
    if (std::isnan(values->frequency_hz) && binding_enabled(map.frequency) && read_binding_value(map.frequency, &v)) {
        values->frequency_hz = v;
    }
    if (std::isnan(values->energy_total_wh) && binding_enabled(map.energy_total) && read_binding_value(map.energy_total, &v)) {
        values->energy_total_wh = v;
    }
    if (std::isnan(values->energy_aux_wh) && binding_enabled(map.energy_aux) && read_binding_value(map.energy_aux, &v)) {
        values->energy_aux_wh = v;
    }
}

static meter_values_t read_meter_values_with_retry(int32_t slave_addr, const meter_register_map_t &map)
{
    meter_values_t values = {
        .voltage_v = float_nan(),
        .current_a = float_nan(),
        .power_w = float_nan(),
        .reactive_power_var = float_nan(),
        .apparent_power_va = float_nan(),
        .frequency_hz = float_nan(),
        .energy_total_wh = float_nan(),
        .energy_aux_wh = float_nan(),
    };

    if (!binding_enabled(map.energy_aux)) {
        values.energy_aux_wh = float_nan();
    }

    const int32_t previous_addr = s_active_slave_addr;
    s_active_slave_addr = slave_addr;

    for (uint32_t attempt = 0; attempt < METER_READ_RETRIES; ++attempt) {
        read_values_once(map, &values);
        if (values_complete(values, map)) {
            break;
        }
    }

    s_active_slave_addr = previous_addr;
    return values;
}

static uint16_t reg_or_na(const reg_binding_t &binding)
{
    return binding_enabled(binding) ? binding.reg : 0xFFFF;
}

static void log_meter_values_fixed(int32_t slave_addr,
                                   const meter_register_map_t &map,
                                   const meter_values_t &values,
                                   float ui_va,
                                   float q_from_ps,
                                   float pf_from_ui)
{
    ESP_LOGI(TAG,
             "meter addr=%3ld type=%-7s | U=%7.2fV I=%6.3fA P=%7.2fW Q=%7.2fvar S=%7.2fVA PF=%5.3f f=%6.2fHz | E=%8.1fWh E2=%8.1fWh UI=%7.2fVA Qps=%7.2fvar | regs U=0x%04X I=0x%04X P=0x%04X Q=0x%04X S=0x%04X f=0x%04X Et=0x%04X Ea=0x%04X",
             (long)slave_addr,
             map.name,
             (double)values.voltage_v,
             (double)values.current_a,
             (double)values.power_w,
             (double)values.reactive_power_var,
             (double)values.apparent_power_va,
             (double)pf_from_ui,
             (double)values.frequency_hz,
             (double)values.energy_total_wh,
             (double)values.energy_aux_wh,
             (double)ui_va,
             (double)q_from_ps,
             (unsigned)reg_or_na(map.voltage),
             (unsigned)reg_or_na(map.current),
             (unsigned)reg_or_na(map.power),
             (unsigned)reg_or_na(map.reactive_power),
             (unsigned)reg_or_na(map.apparent_power),
             (unsigned)reg_or_na(map.frequency),
             (unsigned)reg_or_na(map.energy_total),
             (unsigned)reg_or_na(map.energy_aux));
}

static inline float u32_to_float(uint32_t raw)
{
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

static inline void rs485_set_tx_mode()
{
    APP_ERROR_CHECK("E866", gpio_set_level(KWS_RS485_EN_GPIO, 1));
}

static inline void rs485_set_rx_mode()
{
    APP_ERROR_CHECK("E866", gpio_set_level(KWS_RS485_EN_GPIO, 0));
}

static inline void modbus_rx_resync()
{
    (void)uart_flush_input(KWS_UART_PORT);
}

static void log_modbus_frame_bytes(const char *label, uint16_t reg, const uint8_t *data, int len)
{
    ESP_LOGI(TAG, "%s (reg=%u len=%d)", label, (unsigned)reg, len);
    if (data != nullptr && len > 0) {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_INFO);
    }
}


static int uart_read_exact(uint8_t *buffer, int required_len, TickType_t total_timeout_ticks)
{
    if (buffer == nullptr || required_len <= 0) {
        return 0;
    }

    const TickType_t start = xTaskGetTickCount();
    int total_read = 0;

    while (total_read < required_len) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= total_timeout_ticks) {
            break;
        }

        TickType_t remaining = total_timeout_ticks - elapsed;
        int chunk = uart_read_bytes(KWS_UART_PORT,
                                    buffer + total_read,
                                    (uint32_t)(required_len - total_read),
                                    remaining);
        if (chunk <= 0) {
            continue;
        }
        total_read += chunk;
    }

    return total_read;
}

static uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc >>= 1;
                crc ^= 0xA001U;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static bool modbus_read_u16(uint16_t reg, uint16_t *value)
{
    if (value == nullptr) {
        return false;
    }

    uint8_t request[8] = {
        (uint8_t)s_active_slave_addr,
        0x03,
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF),
        0x00,
        0x01,
        0,
        0,
    };

    const uint16_t req_crc = modbus_crc16(request, 6);
    request[6] = (uint8_t)(req_crc & 0xFF);
    request[7] = (uint8_t)(req_crc >> 8);

    rs485_set_tx_mode();
    vTaskDelay(pdMS_TO_TICKS(10)); // Kratka prodleva pro jistotu, aby se RS485 modul stihl prepnout do TX modu pred odesilanim dat. Bez toho se obcas stava, ze prvni bajt odesilaneho requestu je ztracen.

    //ESP_LOG_BUFFER_HEX(TAG, request, sizeof(request));
    const int tx = uart_write_bytes(KWS_UART_PORT, request, (uint32_t)sizeof(request));
    if (tx != (int)sizeof(request)) {
        ESP_LOGI(TAG, "UART TX selhal (reg=%u tx=%d)", (unsigned)reg, tx);
        rs485_set_rx_mode();
        modbus_rx_resync();
        return false;
    }

    (void)uart_wait_tx_done(KWS_UART_PORT, pdMS_TO_TICKS(10000));
    modbus_rx_resync();
    rs485_set_rx_mode();
    vTaskDelay(pdMS_TO_TICKS(10)); // Kratka prodleva pro jistotu, protože jinak se stává, že je vložen extra byte před zprávu. 8 je málo, 20 moc, 10 se zdá být akorát. Bez toho se občas stává, že první bajt přijímané odpovědi je ztracen nebo je vložen extra byte před odpověď.
    modbus_rx_resync();
    //vTaskDelay(KWS_RS485_TURNAROUND_TICKS);
    //ESP_LOGI(TAG, "Request odeslan, cekam na odpoved (reg=%u)...", (unsigned)reg);

    uint8_t response[7] = {0};
    const int header_rx = uart_read_exact(response, 3, KWS_MODBUS_RX_TIMEOUT_TICKS);
    if (header_rx == 0) {
        ESP_LOGI(TAG, "Slave neodpovedel (timeout) (reg=%u)", (unsigned)reg);
        modbus_rx_resync();
        return false;
    }
    if (header_rx != 3) {
        ESP_LOGI(TAG,
                 "Slave odpovedel necitelnym/neuplnym hlavickovym ramcem (reg=%u rx=%d)",
                 (unsigned)reg,
                 header_rx);
        log_modbus_frame_bytes("Neuplna hlavicka odpovedi", reg, response, header_rx);
        modbus_rx_resync();
        return false;
    }

    if (response[0] != (uint8_t)s_active_slave_addr) {
        ESP_LOGI(TAG,
                 "Neplatna Modbus odpoved 1 (reg=%u addr=%u func=%u len=%u)",
                 (unsigned)reg,
                 (unsigned)response[0],
                 (unsigned)response[1],
                 (unsigned)response[2]);
        log_modbus_frame_bytes("Neplatna Modbus odpoved 1 - bajty", reg, response, 3);
        modbus_rx_resync();
        return false;
    }

    if (response[1] == (uint8_t)(KWS_MODBUS_FUNC_READ_HOLDING | 0x80U)) {
        const int tail_rx = uart_read_exact(response + 3, 2, KWS_MODBUS_RX_TIMEOUT_TICKS);
        if (tail_rx != 2) {
            ESP_LOGI(TAG,
                     "Slave vratil neuplnou exception odpoved (reg=%u code=%u rx=%d)",
                     (unsigned)reg,
                     (unsigned)response[2],
                     tail_rx + 3);
            log_modbus_frame_bytes("Neuplna exception odpoved - bajty", reg, response, tail_rx + 3);
            modbus_rx_resync();
            return false;
        }

        const uint16_t rx_crc = (uint16_t)response[3] | ((uint16_t)response[4] << 8);
        const uint16_t calc_crc = modbus_crc16(response, 3);
        if (rx_crc != calc_crc) {
            ESP_LOGI(TAG,
                     "CRC mismatch v exception odpovedi (reg=%u rx=0x%04x calc=0x%04x)",
                     (unsigned)reg,
                     (unsigned)rx_crc,
                     (unsigned)calc_crc);
            log_modbus_frame_bytes("Exception odpoved s chybnym CRC - bajty", reg, response, 5);
            modbus_rx_resync();
            return false;
        }

        ESP_LOGI(TAG,
                 "Slave vratil Modbus exception (reg=%u code=%u)",
                 (unsigned)reg,
                 (unsigned)response[2]);
        return false;
    }

    if (response[1] != KWS_MODBUS_FUNC_READ_HOLDING || response[2] != 0x02) {
        ESP_LOGI(TAG,
                 "Neplatna Modbus odpoved 2 (reg=%u addr=%u func=%u len=%u)",
                 (unsigned)reg,
                 (unsigned)response[0],
                 (unsigned)response[1],
                 (unsigned)response[2]);
        log_modbus_frame_bytes("Neplatna Modbus odpoved 2 - bajty", reg, response, 3);
        modbus_rx_resync();
        return false;
    }

    const int tail_rx = uart_read_exact(response + 3, 4, KWS_MODBUS_RX_TIMEOUT_TICKS);
    if (tail_rx != 4) {
        ESP_LOGI(TAG,
                 "Slave odpovedel neuplnym datovym ramcem (reg=%u rx=%d)",
                 (unsigned)reg,
                 tail_rx + 3);
        log_modbus_frame_bytes("Neuplny datovy ramec - bajty", reg, response, tail_rx + 3);
        modbus_rx_resync();
        return false;
    }

    const uint16_t rx_crc = (uint16_t)response[5] | ((uint16_t)response[6] << 8);
    const uint16_t calc_crc = modbus_crc16(response, 5);
    if (rx_crc != calc_crc) {
        ESP_LOGI(TAG,
                 "CRC mismatch (reg=%u rx=0x%04x calc=0x%04x)",
                 (unsigned)reg,
                 (unsigned)rx_crc,
                 (unsigned)calc_crc);
        log_modbus_frame_bytes("Datova odpoved s chybnym CRC - bajty", reg, response, 7);
        modbus_rx_resync();
        return false;
    }

    *value = (uint16_t)(((uint16_t)response[3] << 8) | response[4]);
    return true;
}

static bool modbus_read_input_float(uint16_t reg, float *value)
{
    if (value == nullptr) {
        return false;
    }

    uint8_t request[8] = {
        (uint8_t)s_active_slave_addr,
        KWS_MODBUS_FUNC_READ_INPUT,
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF),
        0x00,
        0x02,
        0,
        0,
    };

    const uint16_t req_crc = modbus_crc16(request, 6);
    request[6] = (uint8_t)(req_crc & 0xFF);
    request[7] = (uint8_t)(req_crc >> 8);

    // log_modbus_frame_bytes("Modbus TX f04", reg, request, sizeof(request));

    rs485_set_tx_mode();
    vTaskDelay(pdMS_TO_TICKS(10));

    const int tx = uart_write_bytes(KWS_UART_PORT, request, (uint32_t)sizeof(request));
    if (tx != (int)sizeof(request)) {
        ESP_LOGI(TAG, "UART TX selhal (f04 reg=%u tx=%d)", (unsigned)reg, tx);
        rs485_set_rx_mode();
        modbus_rx_resync();
        return false;
    }

    (void)uart_wait_tx_done(KWS_UART_PORT, pdMS_TO_TICKS(10000));
    modbus_rx_resync();
    rs485_set_rx_mode();
    vTaskDelay(pdMS_TO_TICKS(10));
    modbus_rx_resync();

    uint8_t response[9] = {0};
    const int header_rx = uart_read_exact(response, 3, KWS_MODBUS_RX_TIMEOUT_TICKS);
    if (header_rx == 0) {
        ESP_LOGI(TAG, "Slave neodpovedel (timeout) (f04 reg=%u)", (unsigned)reg);
        modbus_rx_resync();
        return false;
    }
    if (header_rx != 3) {
        ESP_LOGI(TAG, "Slave odpovedel neuplnym ramcem (f04 reg=%u rx=%d)", (unsigned)reg, header_rx);
        log_modbus_frame_bytes("f04 neuplna hlavicka", reg, response, header_rx);
        modbus_rx_resync();
        return false;
    }

    if (response[0] != (uint8_t)s_active_slave_addr || response[1] != KWS_MODBUS_FUNC_READ_INPUT || response[2] != 0x04) {
        ESP_LOGI(TAG,
                 "Neplatna Modbus odpoved f04 (reg=%u addr=%u func=%u len=%u)",
                 (unsigned)reg,
                 (unsigned)response[0],
                 (unsigned)response[1],
                 (unsigned)response[2]);
        log_modbus_frame_bytes("Neplatna Modbus odpoved f04 - bajty", reg, response, 3);
        modbus_rx_resync();
        return false;
    }

    const int tail_rx = uart_read_exact(response + 3, 6, KWS_MODBUS_RX_TIMEOUT_TICKS);
    if (tail_rx != 6) {
        ESP_LOGI(TAG, "Slave odpovedel neuplnym datovym ramcem (f04 reg=%u rx=%d)", (unsigned)reg, tail_rx + 3);
        log_modbus_frame_bytes("f04 neuplny datovy ramec", reg, response, tail_rx + 3);
        modbus_rx_resync();
        return false;
    }

    // log_modbus_frame_bytes("Modbus RX f04", reg, response, sizeof(response));

    const uint16_t rx_crc = (uint16_t)response[7] | ((uint16_t)response[8] << 8);
    const uint16_t calc_crc = modbus_crc16(response, 7);
    if (rx_crc != calc_crc) {
        ESP_LOGI(TAG,
                 "CRC mismatch f04 (reg=%u rx=0x%04x calc=0x%04x)",
                 (unsigned)reg,
                 (unsigned)rx_crc,
                 (unsigned)calc_crc);
        log_modbus_frame_bytes("f04 odpoved s chybnym CRC", reg, response, 9);
        modbus_rx_resync();
        return false;
    }

    const uint32_t raw = ((uint32_t)response[3] << 24) | ((uint32_t)response[4] << 16) | ((uint32_t)response[5] << 8) | (uint32_t)response[6];
    *value = u32_to_float(raw);
    return true;
}

static void load_config(void)
{
    s_cfg.sample_ms = config_store_get_i32_item(&KWS_SAMPLE_MS_ITEM);

    s_cfg.slave_addr = KWS_HARDCODED_SLAVE_ADDR_A;
    s_active_slave_addr = KWS_HARDCODED_SLAVE_ADDR_A;

    ESP_LOGI(TAG,
             "cfg addrA=%ld addrB=%ld sm=%ld uart=%d baud=%ld rx=%d tx=%d",
             (long)KWS_HARDCODED_SLAVE_ADDR_A,
             (long)KWS_HARDCODED_SLAVE_ADDR_B,
             (long)s_cfg.sample_ms,
             (int)KWS_UART_PORT,
             (long)KWS_UART_BAUD,
             (int)KWS_RS485_RX_GPIO,
             (int)KWS_RS485_TX_GPIO);
}

static esp_err_t init_uart(void)
{
    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate = KWS_UART_BAUD;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    APP_ERROR_CHECK("E841", uart_driver_install(KWS_UART_PORT, 256, 0, 0, nullptr, 0));
    APP_ERROR_CHECK("E842", uart_param_config(KWS_UART_PORT, &uart_cfg));

    gpio_reset_pin(KWS_RS485_EN_GPIO);
    APP_ERROR_CHECK("E868", gpio_set_direction(KWS_RS485_EN_GPIO, GPIO_MODE_OUTPUT));
    rs485_set_rx_mode();

    APP_ERROR_CHECK("E843",
                    uart_set_pin(KWS_UART_PORT,
                                 (int)KWS_RS485_TX_GPIO,
                                 (int)KWS_RS485_RX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    APP_ERROR_CHECK("E845", uart_set_mode(KWS_UART_PORT, UART_MODE_UART));

    return ESP_OK;
}

static void kws_task(void *pvParameters)
{
    (void)pvParameters;
    uint32_t cycle = 0;
    size_t meter_index = 0;
    static constexpr int32_t kHardcodedMeterAddrs[2] = {
        KWS_HARDCODED_SLAVE_ADDR_A,
        KWS_HARDCODED_SLAVE_ADDR_B,
    };

    while (true) {
        const int32_t slave_addr = kHardcodedMeterAddrs[meter_index];
        const meter_type_t meter_type = meter_type_for_addr(slave_addr);
        const meter_register_map_t &map = (meter_type == METER_TYPE_KWS) ? KWS_METER_MAP : TAC_METER_MAP;
        const meter_values_t values = read_meter_values_with_retry(slave_addr, map);

        const float ui_va = (std::isnan(values.voltage_v) || std::isnan(values.current_a))
                                ? float_nan()
                                : (values.voltage_v * values.current_a);
        const float pf_from_ui = (std::isnan(ui_va) || std::isnan(values.power_w) || ui_va <= 0.01f)
                                     ? float_nan()
                                     : (values.power_w / ui_va);
        const float q_from_ps = (std::isnan(values.apparent_power_va) || std::isnan(values.power_w))
                                    ? float_nan()
                                    : std::sqrt(std::fmax(0.0f, (values.apparent_power_va * values.apparent_power_va) - (values.power_w * values.power_w)));

        if (!std::isnan(values.voltage_v) && !std::isnan(values.current_a) && !std::isnan(values.power_w)) {
            log_meter_values_fixed(slave_addr, map, values, ui_va, q_from_ps, pf_from_ui);

            if (!values_complete(values, map)) {
                ESP_LOGI(TAG,
                         "addr=%ld meter=%s doplnkove cteni nedokonceno: E_total=%d E_aux=%d S=%d Q=%d f=%d",
                         (long)slave_addr,
                         map.name,
                         std::isnan(values.energy_total_wh) ? 0 : 1,
                         binding_enabled(map.energy_aux) ? (std::isnan(values.energy_aux_wh) ? 0 : 1) : 1,
                         std::isnan(values.apparent_power_va) ? 0 : 1,
                         std::isnan(values.reactive_power_var) ? 0 : 1,
                         std::isnan(values.frequency_hz) ? 0 : 1);
            }
        } else {
            ESP_LOGI(TAG,
                     "addr=%ld meter=%s cilene cteni selhalo: U=%d I=%d P=%d",
                     (long)slave_addr,
                     map.name,
                     std::isnan(values.voltage_v) ? 0 : 1,
                     std::isnan(values.current_a) ? 0 : 1,
                     std::isnan(values.power_w) ? 0 : 1);
        }

        if ((cycle % KWS_ENERGY_SCAN_EVERY_N_CYCLES) == 0U) {
            if (!KWS_ENABLE_DIAG_SCANS) {
                ++cycle;
                vTaskDelay(pdMS_TO_TICKS(s_cfg.sample_ms));
                continue;
            }

            ESP_LOGI(TAG,
                     "Energy scan registru %u..%u (nenulove + 32bit dvojice)",
                     (unsigned)KWS_ENERGY_SCAN_START,
                     (unsigned)KWS_ENERGY_SCAN_END);

            for (uint16_t reg = KWS_ENERGY_SCAN_START; reg <= KWS_ENERGY_SCAN_END; ++reg) {
                uint16_t value = 0;
                if (modbus_read_u16(reg, &value) && value != 0U) {
                    ESP_LOGI(TAG,
                             "E-cand reg[%u] = 0x%04X (%u)",
                             (unsigned)reg,
                             (unsigned)value,
                             (unsigned)value);
                }
            }

            for (uint16_t reg = KWS_ENERGY_SCAN_START; reg < KWS_ENERGY_SCAN_END; ++reg) {
                uint16_t hi = 0;
                uint16_t lo = 0;
                if (modbus_read_u16(reg, &hi) && modbus_read_u16((uint16_t)(reg + 1), &lo)) {
                    const uint32_t be32 = ((uint32_t)hi << 16) | lo;
                    if (be32 != 0U) {
                        ESP_LOGI(TAG,
                                 "E32-cand reg[%u:%u] = 0x%08lX (%lu)",
                                 (unsigned)reg,
                                 (unsigned)(reg + 1),
                                 (unsigned long)be32,
                                 (unsigned long)be32);
                    }
                }
            }
        }

        ++cycle;
        meter_index = (meter_index + 1U) % 2U;
        //vTaskDelay(pdMS_TO_TICKS(s_cfg.sample_ms));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

} // namespace

void kws_303l_register_config_items(void)
{
    APP_ERROR_CHECK("E849", config_store_register_item(&KWS_SLAVE_ADDR_ITEM));
    APP_ERROR_CHECK("E850", config_store_register_item(&KWS_SAMPLE_MS_ITEM));
}

bool kws_303l_is_enabled(void)
{
    return true;
}

void kws_303l_init(void)
{
    load_config();
    APP_ERROR_CHECK("E864", init_uart());
    APP_ERROR_CHECK("E865",
                    xTaskCreate(kws_task, TAG, configMINIMAL_STACK_SIZE * 6, nullptr, 5, nullptr) == pdPASS
                        ? ESP_OK
                        : ESP_FAIL);

    ESP_LOGI(TAG, "KWS-303L task spusten");
}
