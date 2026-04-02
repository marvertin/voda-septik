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

static constexpr uint16_t KWS_DEBUG_REG_0 = 0;
static constexpr uint16_t KWS_DEBUG_REG_1 = 1;
static constexpr uint16_t KWS_DEBUG_REG_2 = 2;

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

static inline void rs485_set_tx_mode()
{
    APP_ERROR_CHECK("E866", gpio_set_level(KWS_RS485_EN_GPIO, 1));
}

static inline void rs485_set_rx_mode()
{
    APP_ERROR_CHECK("E866", gpio_set_level(KWS_RS485_EN_GPIO, 0));
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
        (uint8_t)s_cfg.slave_addr,
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
    
    //vTaskDelay(pdMS_TO_TICKS(500));
    rs485_set_tx_mode();
    vTaskDelay(pdMS_TO_TICKS(200));
    (void)uart_flush_input(KWS_UART_PORT);

    //ESP_LOG_BUFFER_HEX(TAG, request, sizeof(request));
    const int tx = uart_write_bytes(KWS_UART_PORT, request, (uint32_t)sizeof(request));
    if (tx != (int)sizeof(request)) {
        ESP_LOGI(TAG, "UART TX selhal (reg=%u tx=%d)", (unsigned)reg, tx);
        rs485_set_rx_mode();
        return false;
    }

    (void)uart_wait_tx_done(KWS_UART_PORT, pdMS_TO_TICKS(10000));
    rs485_set_rx_mode();
    //ESP_LOGI(TAG, "Request odeslan, cekam na odpoved (reg=%u)...", (unsigned)reg);

    uint8_t response[7] = {0};
    const int header_rx = uart_read_exact(response, 3, KWS_MODBUS_RX_TIMEOUT_TICKS);
    if (header_rx == 0) {
        ESP_LOGI(TAG, "Slave neodpovedel (timeout) (reg=%u)", (unsigned)reg);
        return false;
    }
    if (header_rx != 3) {
        ESP_LOGI(TAG,
                 "Slave odpovedel necitelnym/neuplnym hlavickovym ramcem (reg=%u rx=%d)",
                 (unsigned)reg,
                 header_rx);
        return false;
    }

    if (response[0] != (uint8_t)s_cfg.slave_addr) {
        ESP_LOGI(TAG,
                 "Neplatna Modbus odpoved (reg=%u addr=%u func=%u len=%u)",
                 (unsigned)reg,
                 (unsigned)response[0],
                 (unsigned)response[1],
                 (unsigned)response[2]);
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
                 "Neplatna Modbus odpoved (reg=%u addr=%u func=%u len=%u)",
                 (unsigned)reg,
                 (unsigned)response[0],
                 (unsigned)response[1],
                 (unsigned)response[2]);
        return false;
    }

    const int tail_rx = uart_read_exact(response + 3, 4, KWS_MODBUS_RX_TIMEOUT_TICKS);
    if (tail_rx != 4) {
        ESP_LOGI(TAG,
                 "Slave odpovedel neuplnym datovym ramcem (reg=%u rx=%d)",
                 (unsigned)reg,
                 tail_rx + 3);
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
        return false;
    }

    *value = (uint16_t)(((uint16_t)response[3] << 8) | response[4]);
    return true;
}

static void load_config(void)
{
    s_cfg.slave_addr = config_store_get_i32_item(&KWS_SLAVE_ADDR_ITEM);
    s_cfg.sample_ms = config_store_get_i32_item(&KWS_SAMPLE_MS_ITEM);

    ESP_LOGI(TAG,
             "cfg addr=%ld sm=%ld uart=%d baud=%ld rx=%d tx=%d",
             (long)s_cfg.slave_addr,
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

    while (true) {
        uint16_t reg0 = 0;
        uint16_t reg1 = 0;
        uint16_t reg2 = 0;

        const bool reg0_ok = modbus_read_u16(KWS_DEBUG_REG_0, &reg0);
        const bool reg1_ok = modbus_read_u16(KWS_DEBUG_REG_1, &reg1);
        const bool reg2_ok = modbus_read_u16(KWS_DEBUG_REG_2, &reg2);

        ESP_LOGI(TAG,
                 "poll ok0=%d reg0=%u ok1=%d reg1=%u ok2=%d reg2=%u",
                 reg0_ok ? 1 : 0,
                 (unsigned)reg0,
                 reg1_ok ? 1 : 0,
                 (unsigned)reg1,
                 reg2_ok ? 1 : 0,
                 (unsigned)reg2);

        vTaskDelay(pdMS_TO_TICKS(s_cfg.sample_ms));
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
