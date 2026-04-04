#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
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

#define TAG "elektromer"

namespace {

static constexpr int32_t KWS_DEFAULT_SLAVE_ADDR = 5;
static constexpr uart_port_t MODBUS_UART_PORT = UART_NUM_2;
static constexpr int32_t MODBUS_UART_BAUD = 9600;
static constexpr TickType_t MODBUS_RX_TIMEOUT_TICKS = pdMS_TO_TICKS(200);
static constexpr TickType_t RS485_TURNAROUND_TICKS = pdMS_TO_TICKS(2);
static constexpr uint8_t MODBUS_FUNC_READ_HOLDING = 0x03;
static constexpr uint8_t MODBUS_FUNC_READ_INPUT = 0x04;
static constexpr int32_t KWS_HARDCODED_SLAVE_ADDR = 165;
static constexpr int32_t TAC_HARDCODED_SLAVE_ADDR = 1;

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
static constexpr uint16_t TAC_REG_TOTAL_REACTIVE_ENERGY_KVARH = 0x0508;
static constexpr uint16_t KWS_REG_POWER_FACTOR = 48;
static constexpr uint16_t TAC_REG_POWER_FACTOR = 0x001E;
static constexpr uint16_t TAC_REG_PHASE_ANGLE = 0x0024;

static constexpr float KWS_VOLTAGE_DIV = 100.0f;
static constexpr float KWS_CURRENT_DIV = 1000.0f;
static constexpr float KWS_POWER_DIV = 10.0f;
static constexpr float KWS_APPARENT_POWER_DIV = 10.0f;
static constexpr float KWS_FREQUENCY_DIV = 1.0f;

static const config_item_t KWS_SLAVE_ADDR_ITEM = {
    .key = "kws_addr", .label = "KWS-303L Modbus adresa", .description = "Adresa Modbus slave (1-247).",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = KWS_DEFAULT_SLAVE_ADDR, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 1, .max_int = 247, .min_float = 0.0f, .max_float = 0.0f,
};

typedef struct {
    int32_t slave_addr;
} kws_config_t;

static kws_config_t s_cfg = {
    .slave_addr = KWS_DEFAULT_SLAVE_ADDR,
};

static int32_t s_active_slave_addr = KWS_HARDCODED_SLAVE_ADDR;

enum class reg_read_kind_t : uint8_t {
    NONE = 0,
    HOLDING_U16,
    HOLDING_U32_BE,
    HOLDING_U32_LE,
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
    float power_factor;
    float phase_angle_deg;
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
    reg_binding_t power_factor;
    reg_binding_t phase_angle;
    reg_binding_t energy_total;
    reg_binding_t energy_aux;
} meter_register_map_t;

static constexpr uint32_t METER_READ_RETRIES = 3;
static constexpr bool KWS_ENERGY_HIGH_WORD_FIRST = false;

static constexpr float PUMP_RUNNING_POWER_THRESHOLD_W = 5.0f;

enum class pump_state_t : uint8_t {
    STOPPED = 0,     // elektroměr odpověděl, výkon je nulový
    RUNNING = 1,     // elektroměr odpověděl, výkon je nenulový
    METER_ERROR = 2, // elektroměr neodpověděl
};

static const meter_register_map_t KWS_METER_MAP = {
    .name = "KWS",
    .voltage = {reg_read_kind_t::HOLDING_U16, KWS_REG_VOLTAGE, 0.01f},
    .current = {reg_read_kind_t::HOLDING_U16, KWS_REG_CURRENT, 0.001f},
    .power = {reg_read_kind_t::HOLDING_U16, KWS_REG_POWER, 0.1f},
    .reactive_power = {reg_read_kind_t::HOLDING_U16, KWS_REG_REACTIVE_POWER, 0.1f},
    .apparent_power = {reg_read_kind_t::HOLDING_U16, KWS_REG_APPARENT_POWER, 0.1f},
    .frequency = {reg_read_kind_t::HOLDING_U16, KWS_REG_FREQUENCY, 1.0f},
    .power_factor = {reg_read_kind_t::HOLDING_U16, KWS_REG_POWER_FACTOR, 0.001f},
    .phase_angle = {reg_read_kind_t::NONE, 0, 0.0f},
    .energy_total = {KWS_ENERGY_HIGH_WORD_FIRST ? reg_read_kind_t::HOLDING_U32_BE : reg_read_kind_t::HOLDING_U32_LE,
                     KWS_REG_ENERGY_TOTAL,
                     1.0f},
    .energy_aux = {KWS_ENERGY_HIGH_WORD_FIRST ? reg_read_kind_t::HOLDING_U32_BE : reg_read_kind_t::HOLDING_U32_LE,
                   KWS_REG_ENERGY_AUX,
                   1.0f},
};

static const meter_register_map_t TAC_METER_MAP = {
    .name = "TAC1100",
    .voltage = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_VOLTAGE, 1.0f},
    .current = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_CURRENT, 1.0f},
    .power = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_POWER, 1.0f},
    .reactive_power = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_REACTIVE_POWER, 1.0f},
    .apparent_power = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_APPARENT_POWER, 1.0f},
    .frequency = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_FREQUENCY, 1.0f},
    .power_factor = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_POWER_FACTOR, 1.0f},
    .phase_angle = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_PHASE_ANGLE, 1.0f},
    .energy_total = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_TOTAL_ACTIVE_ENERGY_KWH, 1000.0f},
    .energy_aux = {reg_read_kind_t::INPUT_FLOAT, TAC_REG_TOTAL_REACTIVE_ENERGY_KVARH, 1000.0f},
};

static inline float float_nan()
{
    return std::numeric_limits<float>::quiet_NaN();
}

static inline bool binding_enabled(const reg_binding_t &binding)
{
    return binding.kind != reg_read_kind_t::NONE;
}

static bool modbus_read_u16(uint16_t reg, uint16_t *value);
static bool modbus_read_u32(uint16_t reg, bool high_word_first, uint32_t *value);
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

    if (binding.kind == reg_read_kind_t::HOLDING_U32_BE || binding.kind == reg_read_kind_t::HOLDING_U32_LE) {
        uint32_t raw = 0;
        const bool high_word_first = (binding.kind == reg_read_kind_t::HOLDING_U32_BE);
        if (!modbus_read_u32(binding.reg, high_word_first, &raw)) {
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
    if (binding_enabled(map.power_factor) && std::isnan(values.power_factor)) {
        return false;
    }
    if (binding_enabled(map.phase_angle) && std::isnan(values.phase_angle_deg)) {
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
    if (std::isnan(values->power_factor) && binding_enabled(map.power_factor) && read_binding_value(map.power_factor, &v)) {
        values->power_factor = v;
    }
    if (std::isnan(values->phase_angle_deg) && binding_enabled(map.phase_angle) && read_binding_value(map.phase_angle, &v)) {
        values->phase_angle_deg = v;
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
        .power_factor = float_nan(),
        .phase_angle_deg = float_nan(),
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

static pump_state_t check_pump_state(int32_t slave_addr, const meter_register_map_t &map)
{
    if (!binding_enabled(map.power)) {
        return pump_state_t::METER_ERROR;
    }

    const int32_t previous_addr = s_active_slave_addr;
    s_active_slave_addr = slave_addr;

    float power_w = float_nan();
    const bool ok = read_binding_value(map.power, &power_w);

    s_active_slave_addr = previous_addr;

    if (!ok || std::isnan(power_w)) {
        return pump_state_t::METER_ERROR;
    }
    return (power_w >= PUMP_RUNNING_POWER_THRESHOLD_W) ? pump_state_t::RUNNING : pump_state_t::STOPPED;
}

static uint16_t reg_or_na(const reg_binding_t &binding)
{
    return binding_enabled(binding) ? binding.reg : 0xFFFF;
}

static const char *fmt_float_or_nan(char *dst, size_t dst_len, float value, const char *fmt)
{
    if (dst == nullptr || dst_len == 0) {
        return "";
    }

    if (std::isnan(value)) {
        int width = 0;
        if (fmt != nullptr && fmt[0] == '%') {
            size_t i = 1;
            while (fmt[i] == '-' || fmt[i] == '+' || fmt[i] == ' ' || fmt[i] == '#' || fmt[i] == '0') {
                ++i;
            }
            while (fmt[i] >= '0' && fmt[i] <= '9') {
                width = (width * 10) + (fmt[i] - '0');
                ++i;
            }
        }
        if (width > 0) {
            (void)snprintf(dst, dst_len, "%*s", width, "NaN");
        } else {
            (void)snprintf(dst, dst_len, "NaN");
        }
    } else {
        (void)snprintf(dst, dst_len, fmt, (double)value);
    }
    return dst;
}

static void log_meter_values_fixed(int32_t slave_addr,
                                   const meter_register_map_t &map,
                                   const meter_values_t &values)
{
    const uint64_t uptime_s = (uint64_t)(esp_timer_get_time() / 1000000LL);

    char u_text[16];
    char i_text[16];
    char p_text[16];
    char q_text[16];
    char s_text[16];
    char f_text[16];
    char pf_text[16];
    char phi_text[16];
    char e_text[16];
    char e2_text[16];

    ESP_LOGI(TAG,
             "t=%8llus meter addr=%3ld type=%-7s | inst{U=%sV I=%sA P=%sW Q=%svar S=%sVA f=%sHz PF=%s phi=%sdeg} | cum{E=%sWh E2=%svarh} | regs U=0x%04X I=0x%04X P=0x%04X Q=0x%04X S=0x%04X f=0x%04X PF=0x%04X phi=0x%04X Et=0x%04X E2=0x%04X",
             (unsigned long long)uptime_s,
             (long)slave_addr,
             map.name,
             fmt_float_or_nan(u_text, sizeof(u_text), values.voltage_v, "%7.2f"),
             fmt_float_or_nan(i_text, sizeof(i_text), values.current_a, "%6.3f"),
             fmt_float_or_nan(p_text, sizeof(p_text), values.power_w, "%7.2f"),
             fmt_float_or_nan(q_text, sizeof(q_text), values.reactive_power_var, "%7.2f"),
             fmt_float_or_nan(s_text, sizeof(s_text), values.apparent_power_va, "%7.2f"),
             fmt_float_or_nan(f_text, sizeof(f_text), values.frequency_hz, "%6.2f"),
             fmt_float_or_nan(pf_text, sizeof(pf_text), values.power_factor, "%5.3f"),
             fmt_float_or_nan(phi_text, sizeof(phi_text), values.phase_angle_deg, "%6.2f"),
             fmt_float_or_nan(e_text, sizeof(e_text), values.energy_total_wh, "%8.1f"),
             fmt_float_or_nan(e2_text, sizeof(e2_text), values.energy_aux_wh, "%8.1f"),
             (unsigned)reg_or_na(map.voltage),
             (unsigned)reg_or_na(map.current),
             (unsigned)reg_or_na(map.power),
             (unsigned)reg_or_na(map.reactive_power),
             (unsigned)reg_or_na(map.apparent_power),
             (unsigned)reg_or_na(map.frequency),
             (unsigned)reg_or_na(map.power_factor),
             (unsigned)reg_or_na(map.phase_angle),
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
    APP_ERROR_CHECK("E866", gpio_set_level(RS485_EN_GPIO, 1));
}

static inline void rs485_set_rx_mode()
{
    APP_ERROR_CHECK("E866", gpio_set_level(RS485_EN_GPIO, 0));
}

static inline void modbus_rx_resync()
{
    (void)uart_flush_input(MODBUS_UART_PORT);
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
        int chunk = uart_read_bytes(MODBUS_UART_PORT,
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
    const int tx = uart_write_bytes(MODBUS_UART_PORT, request, (uint32_t)sizeof(request));
    if (tx != (int)sizeof(request)) {
        ESP_LOGI(TAG, "UART TX selhal (reg=%u tx=%d)", (unsigned)reg, tx);
        rs485_set_rx_mode();
        modbus_rx_resync();
        return false;
    }

    (void)uart_wait_tx_done(MODBUS_UART_PORT, pdMS_TO_TICKS(10000));
    modbus_rx_resync();
    rs485_set_rx_mode();
    vTaskDelay(pdMS_TO_TICKS(10)); // Kratka prodleva pro jistotu, protože jinak se stává, že je vložen extra byte před zprávu. 8 je málo, 20 moc, 10 se zdá být akorát. Bez toho se občas stává, že první bajt přijímané odpovědi je ztracen nebo je vložen extra byte před odpověď.
    modbus_rx_resync();
    //vTaskDelay(RS485_TURNAROUND_TICKS);
    //ESP_LOGI(TAG, "Request odeslan, cekam na odpoved (reg=%u)...", (unsigned)reg);

    uint8_t response[7] = {0};
    const int header_rx = uart_read_exact(response, 3, MODBUS_RX_TIMEOUT_TICKS);
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

    if (response[1] == (uint8_t)(MODBUS_FUNC_READ_HOLDING | 0x80U)) {
        const int tail_rx = uart_read_exact(response + 3, 2, MODBUS_RX_TIMEOUT_TICKS);
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

    if (response[1] != MODBUS_FUNC_READ_HOLDING || response[2] != 0x02) {
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

    const int tail_rx = uart_read_exact(response + 3, 4, MODBUS_RX_TIMEOUT_TICKS);
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

static bool modbus_read_u32(uint16_t reg, bool high_word_first, uint32_t *value)
{
    if (value == nullptr) {
        return false;
    }

    uint8_t request[8] = {
        (uint8_t)s_active_slave_addr,
        MODBUS_FUNC_READ_HOLDING,
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

    rs485_set_tx_mode();
    vTaskDelay(pdMS_TO_TICKS(10));

    const int tx = uart_write_bytes(MODBUS_UART_PORT, request, (uint32_t)sizeof(request));
    if (tx != (int)sizeof(request)) {
        ESP_LOGI(TAG, "UART TX selhal (u32 reg=%u tx=%d)", (unsigned)reg, tx);
        rs485_set_rx_mode();
        modbus_rx_resync();
        return false;
    }

    (void)uart_wait_tx_done(MODBUS_UART_PORT, pdMS_TO_TICKS(10000));
    modbus_rx_resync();
    rs485_set_rx_mode();
    vTaskDelay(pdMS_TO_TICKS(10));
    modbus_rx_resync();

    uint8_t response[9] = {0};
    const int header_rx = uart_read_exact(response, 3, MODBUS_RX_TIMEOUT_TICKS);
    if (header_rx == 0) {
        ESP_LOGI(TAG, "Slave neodpovedel (timeout) (u32 reg=%u)", (unsigned)reg);
        modbus_rx_resync();
        return false;
    }
    if (header_rx != 3) {
        ESP_LOGI(TAG,
                 "Slave odpovedel necitelnym/neuplnym hlavickovym ramcem (u32 reg=%u rx=%d)",
                 (unsigned)reg,
                 header_rx);
        log_modbus_frame_bytes("u32 neuplna hlavicka", reg, response, header_rx);
        modbus_rx_resync();
        return false;
    }

    if (response[0] != (uint8_t)s_active_slave_addr) {
        ESP_LOGI(TAG,
                 "Neplatna Modbus odpoved u32-1 (reg=%u addr=%u func=%u len=%u)",
                 (unsigned)reg,
                 (unsigned)response[0],
                 (unsigned)response[1],
                 (unsigned)response[2]);
        log_modbus_frame_bytes("Neplatna Modbus odpoved u32-1 - bajty", reg, response, 3);
        modbus_rx_resync();
        return false;
    }

    if (response[1] == (uint8_t)(MODBUS_FUNC_READ_HOLDING | 0x80U)) {
        const int tail_rx = uart_read_exact(response + 3, 2, MODBUS_RX_TIMEOUT_TICKS);
        if (tail_rx != 2) {
            ESP_LOGI(TAG,
                     "Slave vratil neuplnou exception odpoved (u32 reg=%u code=%u rx=%d)",
                     (unsigned)reg,
                     (unsigned)response[2],
                     tail_rx + 3);
            log_modbus_frame_bytes("u32 neuplna exception", reg, response, tail_rx + 3);
            modbus_rx_resync();
            return false;
        }

        const uint16_t rx_crc = (uint16_t)response[3] | ((uint16_t)response[4] << 8);
        const uint16_t calc_crc = modbus_crc16(response, 3);
        if (rx_crc != calc_crc) {
            ESP_LOGI(TAG,
                     "CRC mismatch v exception odpovedi (u32 reg=%u rx=0x%04x calc=0x%04x)",
                     (unsigned)reg,
                     (unsigned)rx_crc,
                     (unsigned)calc_crc);
            log_modbus_frame_bytes("u32 exception odpoved s chybnym CRC", reg, response, 5);
            modbus_rx_resync();
            return false;
        }

        ESP_LOGI(TAG,
                 "Slave vratil Modbus exception (u32 reg=%u code=%u)",
                 (unsigned)reg,
                 (unsigned)response[2]);
        return false;
    }

    if (response[1] != MODBUS_FUNC_READ_HOLDING || response[2] != 0x04) {
        ESP_LOGI(TAG,
                 "Neplatna Modbus odpoved u32-2 (reg=%u addr=%u func=%u len=%u)",
                 (unsigned)reg,
                 (unsigned)response[0],
                 (unsigned)response[1],
                 (unsigned)response[2]);
        log_modbus_frame_bytes("Neplatna Modbus odpoved u32-2 - bajty", reg, response, 3);
        modbus_rx_resync();
        return false;
    }

    const int tail_rx = uart_read_exact(response + 3, 6, MODBUS_RX_TIMEOUT_TICKS);
    if (tail_rx != 6) {
        ESP_LOGI(TAG,
                 "Slave odpovedel neuplnym datovym ramcem (u32 reg=%u rx=%d)",
                 (unsigned)reg,
                 tail_rx + 3);
        log_modbus_frame_bytes("u32 neuplny datovy ramec", reg, response, tail_rx + 3);
        modbus_rx_resync();
        return false;
    }

    const uint16_t rx_crc = (uint16_t)response[7] | ((uint16_t)response[8] << 8);
    const uint16_t calc_crc = modbus_crc16(response, 7);
    if (rx_crc != calc_crc) {
        ESP_LOGI(TAG,
                 "CRC mismatch (u32 reg=%u rx=0x%04x calc=0x%04x)",
                 (unsigned)reg,
                 (unsigned)rx_crc,
                 (unsigned)calc_crc);
        log_modbus_frame_bytes("u32 odpoved s chybnym CRC", reg, response, 9);
        modbus_rx_resync();
        return false;
    }

    const uint16_t first = (uint16_t)(((uint16_t)response[3] << 8) | response[4]);
    const uint16_t second = (uint16_t)(((uint16_t)response[5] << 8) | response[6]);

    if (high_word_first) {
        *value = ((uint32_t)first << 16) | (uint32_t)second;
    } else {
        *value = ((uint32_t)second << 16) | (uint32_t)first;
    }
    return true;
}

static bool modbus_read_input_float(uint16_t reg, float *value)
{
    if (value == nullptr) {
        return false;
    }

    uint8_t request[8] = {
        (uint8_t)s_active_slave_addr,
        MODBUS_FUNC_READ_INPUT,
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

    const int tx = uart_write_bytes(MODBUS_UART_PORT, request, (uint32_t)sizeof(request));
    if (tx != (int)sizeof(request)) {
        ESP_LOGI(TAG, "UART TX selhal (f04 reg=%u tx=%d)", (unsigned)reg, tx);
        rs485_set_rx_mode();
        modbus_rx_resync();
        return false;
    }

    (void)uart_wait_tx_done(MODBUS_UART_PORT, pdMS_TO_TICKS(10000));
    modbus_rx_resync();
    rs485_set_rx_mode();
    vTaskDelay(pdMS_TO_TICKS(10));
    modbus_rx_resync();

    uint8_t response[9] = {0};
    const int header_rx = uart_read_exact(response, 3, MODBUS_RX_TIMEOUT_TICKS);
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

    if (response[0] != (uint8_t)s_active_slave_addr || response[1] != MODBUS_FUNC_READ_INPUT || response[2] != 0x04) {
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

    const int tail_rx = uart_read_exact(response + 3, 6, MODBUS_RX_TIMEOUT_TICKS);
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
    s_cfg.slave_addr = KWS_HARDCODED_SLAVE_ADDR;
    s_active_slave_addr = KWS_HARDCODED_SLAVE_ADDR;

    ESP_LOGI(TAG,
             "cfg addrA=%ld addrB=%ld uart=%d baud=%ld rx=%d tx=%d",
             (long)KWS_HARDCODED_SLAVE_ADDR,
             (long)TAC_HARDCODED_SLAVE_ADDR,
             (int)MODBUS_UART_PORT,
             (long)MODBUS_UART_BAUD,
             (int)RS485_RX_GPIO,
             (int)RS485_TX_GPIO);
}

static esp_err_t init_uart(void)
{
    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate = MODBUS_UART_BAUD;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    APP_ERROR_CHECK("E841", uart_driver_install(MODBUS_UART_PORT, 256, 0, 0, nullptr, 0));
    APP_ERROR_CHECK("E842", uart_param_config(MODBUS_UART_PORT, &uart_cfg));

    gpio_reset_pin(RS485_EN_GPIO);
    APP_ERROR_CHECK("E868", gpio_set_direction(RS485_EN_GPIO, GPIO_MODE_OUTPUT));
    rs485_set_rx_mode();

    APP_ERROR_CHECK("E843",
                    uart_set_pin(MODBUS_UART_PORT,
                                 (int)RS485_TX_GPIO,
                                 (int)RS485_RX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    APP_ERROR_CHECK("E845", uart_set_mode(MODBUS_UART_PORT, UART_MODE_UART));

    return ESP_OK;
}

static void kws_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_full_read_ticks = 0;

    while (true) {
        const TickType_t loop_start = xTaskGetTickCount();


        const pump_state_t pump = check_pump_state(KWS_HARDCODED_SLAVE_ADDR, KWS_METER_MAP);
        ESP_LOGI(TAG, "Pump state: %s", (pump == pump_state_t::RUNNING) ? "RUNNING" : ((pump == pump_state_t::STOPPED) ? "STOPPED" : "METER_ERROR"));
        vTaskDelay(pdMS_TO_TICKS(100));

        if (pump == pump_state_t::RUNNING) {
            {
                meter_values_t values = read_meter_values_with_retry(KWS_HARDCODED_SLAVE_ADDR, KWS_METER_MAP);
                log_meter_values_fixed(KWS_HARDCODED_SLAVE_ADDR, KWS_METER_MAP, values);
            }
            // {
            //     meter_values_t values = read_meter_values_with_retry(TAC_HARDCODED_SLAVE_ADDR, TAC_METER_MAP);
            //     log_meter_values_fixed(TAC_HARDCODED_SLAVE_ADDR, TAC_METER_MAP, values);
            // }
        }

    }
}

} // namespace

void kws_303l_register_config_items(void)
{
    APP_ERROR_CHECK("E849", config_store_register_item(&KWS_SLAVE_ADDR_ITEM));
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
