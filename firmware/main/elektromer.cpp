#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>

#ifdef __cplusplus
}
#endif

#include <cmath>
#include <cstdint>
#include <limits>

#include "elektromer.h"
#include "sensor_events.h"
#include "config_store.h"
#include "app_error_check.h"
#include "pins.h"

#define TAG "elektromer"

namespace {

static constexpr int32_t KWS_DEFAULT_SLAVE_ADDR = 1;
static constexpr uart_port_t MODBUS_UART_PORT = UART_NUM_2;
static constexpr int32_t MODBUS_UART_BAUD = 9600;
static constexpr TickType_t MODBUS_RX_TIMEOUT_TICKS = pdMS_TO_TICKS(200);
static constexpr TickType_t METER_TASK_PERIOD_TICKS = pdMS_TO_TICKS(5000);
static constexpr uint8_t MODBUS_FUNC_READ_HOLDING = 0x03;
static constexpr uint32_t METER_READ_RETRIES = 3;

static constexpr uint16_t KWS_REG_VOLTAGE = 14;
static constexpr uint16_t KWS_REG_CURRENT = 18;
static constexpr uint16_t KWS_REG_POWER = 26;
static constexpr uint16_t KWS_REG_REACTIVE_POWER = 34;
static constexpr uint16_t KWS_REG_APPARENT_POWER = 42;
static constexpr uint16_t KWS_REG_POWER_FACTOR = 48;
static constexpr uint16_t KWS_REG_ENERGY_TOTAL = 55;
static constexpr uint16_t KWS_REG_ENERGY_AUX = 61;
static constexpr uint16_t KWS_REG_FREQUENCY = 72;

static const config_item_t KWS_SLAVE_ADDR_ITEM = {
    .key = "kws_addr", .label = "KWS-303L Modbus adresa", .description = "Adresa Modbus slave elektromeru KWS-303L (1-247).",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = KWS_DEFAULT_SLAVE_ADDR, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 1, .max_int = 247, .min_float = 0.0f, .max_float = 0.0f,
};

struct meter_values_t {
    float voltage_v;
    float current_a;
    float active_power_w;
    float reactive_power_var;
    float apparent_power_va;
    float frequency_hz;
    float power_factor;
    float active_energy_wh;
    float reactive_energy_varh;
};

static int32_t s_slave_addr = KWS_DEFAULT_SLAVE_ADDR;

static float nanf_value()
{
    return std::numeric_limits<float>::quiet_NaN();
}

static uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t index = 0; index < len; ++index) {
        crc ^= data[index];
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

static void rs485_set_tx_mode()
{
    APP_ERROR_CHECK("E866", gpio_set_level(RS485_EN_GPIO, 1));
}

static void rs485_set_rx_mode()
{
    APP_ERROR_CHECK("E867", gpio_set_level(RS485_EN_GPIO, 0));
}

static void modbus_rx_resync()
{
    (void)uart_flush_input(MODBUS_UART_PORT);
}

static int uart_read_exact(uint8_t *buffer, int required_len, TickType_t total_timeout_ticks)
{
    if (buffer == nullptr || required_len <= 0) {
        return 0;
    }

    const TickType_t start = xTaskGetTickCount();
    int total_read = 0;
    while (total_read < required_len) {
        const TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= total_timeout_ticks) {
            break;
        }

        const TickType_t remaining = total_timeout_ticks - elapsed;
        const int chunk = uart_read_bytes(MODBUS_UART_PORT,
                                          buffer + total_read,
                                          required_len - total_read,
                                          remaining);
        if (chunk > 0) {
            total_read += chunk;
        }
    }

    return total_read;
}

static bool modbus_read_holding(uint16_t reg, uint16_t count, uint8_t *payload, size_t payload_len)
{
    if (payload == nullptr || payload_len < (size_t)count * 2U || count == 0) {
        return false;
    }

    uint8_t request[8] = {
        (uint8_t)s_slave_addr,
        MODBUS_FUNC_READ_HOLDING,
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF),
        (uint8_t)(count >> 8),
        (uint8_t)(count & 0xFF),
        0,
        0,
    };
    const uint16_t req_crc = modbus_crc16(request, 6);
    request[6] = (uint8_t)(req_crc & 0xFF);
    request[7] = (uint8_t)(req_crc >> 8);

    rs485_set_tx_mode();
    vTaskDelay(pdMS_TO_TICKS(10));

    const int tx = uart_write_bytes(MODBUS_UART_PORT, request, sizeof(request));
    if (tx != (int)sizeof(request)) {
        ESP_LOGW(TAG, "UART TX selhal (reg=%u tx=%d)", (unsigned)reg, tx);
        rs485_set_rx_mode();
        modbus_rx_resync();
        return false;
    }

    (void)uart_wait_tx_done(MODBUS_UART_PORT, pdMS_TO_TICKS(1000));
    modbus_rx_resync();
    rs485_set_rx_mode();
    vTaskDelay(pdMS_TO_TICKS(10));
    modbus_rx_resync();

    uint8_t response[9] = {0};
    const int expected_len = 5 + (int)count * 2;
    if (expected_len > (int)sizeof(response)) {
        return false;
    }

    const int rx = uart_read_exact(response, expected_len, MODBUS_RX_TIMEOUT_TICKS);
    if (rx != expected_len) {
        ESP_LOGW(TAG, "Modbus timeout/neuplna odpoved (reg=%u rx=%d exp=%d)", (unsigned)reg, rx, expected_len);
        modbus_rx_resync();
        return false;
    }

    if (response[0] != (uint8_t)s_slave_addr ||
        response[1] != MODBUS_FUNC_READ_HOLDING ||
        response[2] != (uint8_t)(count * 2U)) {
        ESP_LOGW(TAG,
                 "Neplatna Modbus odpoved (reg=%u addr=%u func=%u len=%u)",
                 (unsigned)reg,
                 (unsigned)response[0],
                 (unsigned)response[1],
                 (unsigned)response[2]);
        modbus_rx_resync();
        return false;
    }

    const uint16_t rx_crc = (uint16_t)response[expected_len - 2] | ((uint16_t)response[expected_len - 1] << 8);
    const uint16_t calc_crc = modbus_crc16(response, (size_t)expected_len - 2);
    if (rx_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC mismatch (reg=%u rx=0x%04x calc=0x%04x)", (unsigned)reg, (unsigned)rx_crc, (unsigned)calc_crc);
        modbus_rx_resync();
        return false;
    }

    for (uint16_t index = 0; index < count * 2U; ++index) {
        payload[index] = response[3 + index];
    }
    return true;
}

static bool read_u16_scaled(uint16_t reg, float multiplier, float *value)
{
    if (value == nullptr) {
        return false;
    }

    uint8_t payload[2] = {0};
    if (!modbus_read_holding(reg, 1, payload, sizeof(payload))) {
        return false;
    }

    const uint16_t raw = ((uint16_t)payload[0] << 8) | payload[1];
    *value = (float)raw * multiplier;
    return true;
}

static bool read_u32_le_scaled(uint16_t reg, float multiplier, float *value)
{
    if (value == nullptr) {
        return false;
    }

    uint8_t payload[4] = {0};
    if (!modbus_read_holding(reg, 2, payload, sizeof(payload))) {
        return false;
    }

    const uint16_t low_word = ((uint16_t)payload[0] << 8) | payload[1];
    const uint16_t high_word = ((uint16_t)payload[2] << 8) | payload[3];
    const uint32_t raw = ((uint32_t)high_word << 16) | low_word;
    *value = (float)raw * multiplier;
    return true;
}

static meter_values_t empty_values()
{
    return {
        .voltage_v = nanf_value(),
        .current_a = nanf_value(),
        .active_power_w = nanf_value(),
        .reactive_power_var = nanf_value(),
        .apparent_power_va = nanf_value(),
        .frequency_hz = nanf_value(),
        .power_factor = nanf_value(),
        .active_energy_wh = nanf_value(),
        .reactive_energy_varh = nanf_value(),
    };
}

static bool values_complete(const meter_values_t &values)
{
    return std::isfinite(values.voltage_v)
        && std::isfinite(values.current_a)
        && std::isfinite(values.active_power_w)
        && std::isfinite(values.reactive_power_var)
        && std::isfinite(values.apparent_power_va)
        && std::isfinite(values.frequency_hz)
        && std::isfinite(values.power_factor)
        && std::isfinite(values.active_energy_wh)
        && std::isfinite(values.reactive_energy_varh);
}

static void read_values_once(meter_values_t *values)
{
    if (values == nullptr) {
        return;
    }

    float value = 0.0f;
    if (!std::isfinite(values->voltage_v) && read_u16_scaled(KWS_REG_VOLTAGE, 0.01f, &value)) {
        values->voltage_v = value;
    }
    if (!std::isfinite(values->current_a) && read_u16_scaled(KWS_REG_CURRENT, 0.001f, &value)) {
        values->current_a = value;
    }
    if (!std::isfinite(values->active_power_w) && read_u16_scaled(KWS_REG_POWER, 0.1f, &value)) {
        values->active_power_w = value;
    }
    if (!std::isfinite(values->reactive_power_var) && read_u16_scaled(KWS_REG_REACTIVE_POWER, 0.1f, &value)) {
        values->reactive_power_var = value;
    }
    if (!std::isfinite(values->apparent_power_va) && read_u16_scaled(KWS_REG_APPARENT_POWER, 0.1f, &value)) {
        values->apparent_power_va = value;
    }
    if (!std::isfinite(values->frequency_hz) && read_u16_scaled(KWS_REG_FREQUENCY, 1.0f, &value)) {
        values->frequency_hz = value;
    }
    if (!std::isfinite(values->power_factor) && read_u16_scaled(KWS_REG_POWER_FACTOR, 0.001f, &value)) {
        values->power_factor = value;
    }
    if (!std::isfinite(values->active_energy_wh) && read_u32_le_scaled(KWS_REG_ENERGY_TOTAL, 1.0f, &value)) {
        values->active_energy_wh = value;
    }
    if (!std::isfinite(values->reactive_energy_varh) && read_u32_le_scaled(KWS_REG_ENERGY_AUX, 1.0f, &value)) {
        values->reactive_energy_varh = value;
    }
}

static meter_values_t read_meter_values()
{
    meter_values_t values = empty_values();
    for (uint32_t attempt = 0; attempt < METER_READ_RETRIES; ++attempt) {
        read_values_once(&values);
        if (values_complete(values)) {
            break;
        }
    }
    return values;
}

static void publish_meter_event(const meter_values_t &values)
{
    app_event_t event = {
        .event_type = EVT_SENSOR,
        .timestamp_us = esp_timer_get_time(),
        .data = {
            .sensor = {
                .sensor_type = SENSOR_EVENT_ELECTRIC_METER,
                .data = {
                    .electric_meter = {
                        .voltage_v = values.voltage_v,
                        .current_a = values.current_a,
                        .active_power_w = values.active_power_w,
                        .reactive_power_var = values.reactive_power_var,
                        .apparent_power_va = values.apparent_power_va,
                        .frequency_hz = values.frequency_hz,
                        .power_factor = values.power_factor,
                        .active_energy_wh = values.active_energy_wh,
                        .reactive_energy_varh = values.reactive_energy_varh,
                    },
                },
            },
        },
    };

    if (!sensor_events_publish(&event, pdMS_TO_TICKS(20))) {
        ESP_LOGW(TAG, "Event elektromeru se nepodarilo vlozit do fronty");
    }
}

static void load_config()
{
    s_slave_addr = config_store_get_i32_item(&KWS_SLAVE_ADDR_ITEM);
    ESP_LOGI(TAG,
             "KWS-303L cfg: addr=%ld uart=%d baud=%ld rx=%d tx=%d en=%d",
             (long)s_slave_addr,
             (int)MODBUS_UART_PORT,
             (long)MODBUS_UART_BAUD,
             (int)RS485_RX_GPIO,
             (int)RS485_TX_GPIO,
             (int)RS485_EN_GPIO);
}

static esp_err_t init_uart()
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
    APP_ERROR_CHECK("E843", gpio_set_direction(RS485_EN_GPIO, GPIO_MODE_OUTPUT));
    rs485_set_rx_mode();

    APP_ERROR_CHECK("E844",
                    uart_set_pin(MODBUS_UART_PORT,
                                 (int)RS485_TX_GPIO,
                                 (int)RS485_RX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    APP_ERROR_CHECK("E845", uart_set_mode(MODBUS_UART_PORT, UART_MODE_UART));

    return ESP_OK;
}

static void elektromer_task(void *pv_parameters)
{
    (void)pv_parameters;
    APP_ERROR_CHECK("E846", esp_task_wdt_add(nullptr));

    while (true) {
        const meter_values_t values = read_meter_values();
        ESP_LOGI(TAG,
                 "KWS: U=%.2f V I=%.3f A P=%.2f W Q=%.2f var S=%.2f VA f=%.2f Hz PF=%.3f E=%.1f Wh EQ=%.1f varh",
                 (double)values.voltage_v,
                 (double)values.current_a,
                 (double)values.active_power_w,
                 (double)values.reactive_power_var,
                 (double)values.apparent_power_va,
                 (double)values.frequency_hz,
                 (double)values.power_factor,
                 (double)values.active_energy_wh,
                 (double)values.reactive_energy_varh);
        publish_meter_event(values);

        APP_ERROR_CHECK("E847", esp_task_wdt_reset());
        vTaskDelay(METER_TASK_PERIOD_TICKS);
    }
}

} // namespace

void elektromer_register_config_items(void)
{
    APP_ERROR_CHECK("E849", config_store_register_item(&KWS_SLAVE_ADDR_ITEM));
}

void elektromer_init(void)
{
    load_config();
    APP_ERROR_CHECK("E850", init_uart());
    APP_ERROR_CHECK("E851",
                    xTaskCreate(elektromer_task, TAG, configMINIMAL_STACK_SIZE * 6, nullptr, 5, nullptr) == pdPASS
                        ? ESP_OK
                        : ESP_FAIL);
    ESP_LOGI(TAG, "KWS-303L elektromer task spusten");
}
