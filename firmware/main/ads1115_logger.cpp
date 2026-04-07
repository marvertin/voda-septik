#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>

#include <string.h>

#include <i2cdev.h>

#ifdef __cplusplus
}
#endif

#include "pins.h"
#include "ads1115_logger.h"

#define TAG "ads1115"

namespace {

static constexpr i2c_port_t ADS1115_I2C_PORT = I2C_NUM_0;
static constexpr uint8_t ADS1115_I2C_ADDR = 0x48;
static constexpr uint32_t ADS1115_I2C_FREQ_HZ = 100000;

static constexpr uint8_t ADS1115_REG_CONVERSION = 0x00;
static constexpr uint8_t ADS1115_REG_CONFIG = 0x01;

static constexpr uint16_t ADS1115_CFG_OS_SINGLE = 0x8000;
static constexpr uint16_t ADS1115_CFG_PGA_2_048V = 0x0400;
static constexpr uint16_t ADS1115_CFG_MODE_SINGLE = 0x0100;
static constexpr uint16_t ADS1115_CFG_DR_128SPS = 0x0080;

static constexpr uint16_t ADS1115_CFG_COMP_DISABLE = 0x0003;

static constexpr uint16_t ADS1115_CFG_MUX_AIN0_GND = 0x4000;
static constexpr uint16_t ADS1115_CFG_MUX_AIN1_GND = 0x5000;
static constexpr uint16_t ADS1115_CFG_MUX_AIN2_GND = 0x6000;
static constexpr uint16_t ADS1115_CFG_MUX_AIN3_GND = 0x7000;

static constexpr TickType_t ADS1115_TASK_PERIOD_TICKS = pdMS_TO_TICKS(1000);
static constexpr TickType_t ADS1115_CONVERSION_WAIT_TICKS = pdMS_TO_TICKS(10);

static i2c_dev_t s_ads1115;

static uint16_t ads1115_channel_mux(uint8_t channel)
{
    switch (channel) {
        case 0:
            return ADS1115_CFG_MUX_AIN0_GND;
        case 1:
            return ADS1115_CFG_MUX_AIN1_GND;
        case 2:
            return ADS1115_CFG_MUX_AIN2_GND;
        case 3:
            return ADS1115_CFG_MUX_AIN3_GND;
        default:
            return ADS1115_CFG_MUX_AIN0_GND;
    }
}

static esp_err_t ads1115_init_device(void)
{
    memset(&s_ads1115, 0, sizeof(s_ads1115));
    s_ads1115.port = ADS1115_I2C_PORT;
    s_ads1115.addr = ADS1115_I2C_ADDR;
    s_ads1115.cfg.sda_io_num = I2C_SDA_GPIO;
    s_ads1115.cfg.scl_io_num = I2C_SCL_GPIO;
    s_ads1115.cfg.master.clk_speed = ADS1115_I2C_FREQ_HZ;

    esp_err_t err = i2c_dev_create_mutex(&s_ads1115);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_dev_check_present(&s_ads1115);
    if (err != ESP_OK) {
        (void)i2c_dev_delete_mutex(&s_ads1115);
        return err;
    }

    return ESP_OK;
}

static esp_err_t ads1115_read_single_channel(uint8_t channel, int16_t *raw_value)
{
    if (raw_value == nullptr || channel > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint16_t config_value = ADS1115_CFG_OS_SINGLE
        | ads1115_channel_mux(channel)
        | ADS1115_CFG_PGA_2_048V
        | ADS1115_CFG_MODE_SINGLE
        | ADS1115_CFG_DR_128SPS
        | ADS1115_CFG_COMP_DISABLE;

    const uint8_t config_bytes[2] = {
        static_cast<uint8_t>((config_value >> 8) & 0xFF),
        static_cast<uint8_t>(config_value & 0xFF)
    };

    esp_err_t err = i2c_dev_take_mutex(&s_ads1115);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_dev_write_reg(&s_ads1115, ADS1115_REG_CONFIG, config_bytes, sizeof(config_bytes));
    if (err != ESP_OK) {
        (void)i2c_dev_give_mutex(&s_ads1115);
        return err;
    }

    vTaskDelay(ADS1115_CONVERSION_WAIT_TICKS);

    uint8_t conversion_bytes[2] = {0};
    err = i2c_dev_read_reg(&s_ads1115, ADS1115_REG_CONVERSION, conversion_bytes, sizeof(conversion_bytes));
    (void)i2c_dev_give_mutex(&s_ads1115);
    if (err != ESP_OK) {
        return err;
    }

    const uint16_t raw_u16 = ((uint16_t)conversion_bytes[0] << 8) | conversion_bytes[1];
    *raw_value = static_cast<int16_t>(raw_u16);
    return ESP_OK;
}

static void ads1115_task(void *pv_parameters)
{
    (void)pv_parameters;

    while (true) {
        int16_t values[4] = {0, 0, 0, 0};
        bool read_ok = true;

        for (uint8_t ch = 0; ch < 4; ++ch) {
            esp_err_t err = ads1115_read_single_channel(ch, &values[ch]);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Cteni kanalu CH%u selhalo: %s", (unsigned int)ch, esp_err_to_name(err));
                read_ok = false;
            }
        }

        if (read_ok) {
            ESP_LOGI(TAG,
                     "ADS1115 RAW CH0=%d CH1=%d CH2=%d CH3=%d",
                     (int)values[0],
                     (int)values[1],
                     (int)values[2],
                     (int)values[3]);
        }

        vTaskDelay(ADS1115_TASK_PERIOD_TICKS);
    }
}

} // namespace

void ads1115_logger_init(void)
{
    esp_err_t err = i2cdev_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADS1115 logger disabled: i2cdev_init failed: %s", esp_err_to_name(err));
        return;
    }

    err = ads1115_init_device();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADS1115 logger disabled: init failed on address 0x%02X: %s",
                 ADS1115_I2C_ADDR,
                 esp_err_to_name(err));
        return;
    }

    if (xTaskCreate(ads1115_task, "ads1115_task", 3072, nullptr, 4, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "ADS1115 logger disabled: task creation failed");
        (void)i2c_dev_delete_mutex(&s_ads1115);
        return;
    }

    ESP_LOGI(TAG, "ADS1115 logger task started (addr=0x%02X)", ADS1115_I2C_ADDR);
}
