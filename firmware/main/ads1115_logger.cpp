#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>

#include <string.h>

#include <ads111x.h>

#ifdef __cplusplus
}
#endif

#include "pins.h"
#include "ads1115_logger.h"

#define TAG "ads1115"

namespace {

static constexpr i2c_port_t ADS1115_I2C_PORT = I2C_NUM_0;
static constexpr uint8_t ADS1115_I2C_ADDR = ADS111X_ADDR_GND;
static constexpr uint32_t ADS1115_I2C_FREQ_HZ = 100000;
static constexpr ads111x_gain_t ADS1115_GAIN = ADS111X_GAIN_2V048;
static constexpr ads111x_data_rate_t ADS1115_DATA_RATE = ADS111X_DATA_RATE_128;

static constexpr TickType_t ADS1115_TASK_PERIOD_TICKS = pdMS_TO_TICKS(1000);
static constexpr TickType_t ADS1115_CONVERSION_WAIT_TICKS = pdMS_TO_TICKS(10);

static i2c_dev_t s_ads1115;

static constexpr ads111x_mux_t ADS1115_CHANNEL_MUXES[] = {
    ADS111X_MUX_0_GND,
    ADS111X_MUX_1_GND,
    ADS111X_MUX_2_GND,
    ADS111X_MUX_3_GND,
};

static esp_err_t ads1115_init_device(void)
{
    memset(&s_ads1115, 0, sizeof(s_ads1115));
    esp_err_t err = ads111x_init_desc(&s_ads1115, ADS1115_I2C_ADDR, ADS1115_I2C_PORT, I2C_SDA_GPIO, I2C_SCL_GPIO);
    if (err != ESP_OK) {
        return err;
    }

    // ads111x defaults to 1MHz, but ADS1115 is specified up to 400kHz.
    s_ads1115.cfg.master.clk_speed = ADS1115_I2C_FREQ_HZ;

    err = ads111x_set_mode(&s_ads1115, ADS111X_MODE_SINGLE_SHOT);
    if (err != ESP_OK) {
        (void)ads111x_free_desc(&s_ads1115);
        return err;
    }

    err = ads111x_set_gain(&s_ads1115, ADS1115_GAIN);
    if (err != ESP_OK) {
        (void)ads111x_free_desc(&s_ads1115);
        return err;
    }

    err = ads111x_set_data_rate(&s_ads1115, ADS1115_DATA_RATE);
    if (err != ESP_OK) {
        (void)ads111x_free_desc(&s_ads1115);
        return err;
    }

    return ESP_OK;
}

static esp_err_t ads1115_read_single_channel(uint8_t channel, int16_t *raw_value)
{
    if (raw_value == nullptr || channel > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ads111x_set_input_mux(&s_ads1115, ADS1115_CHANNEL_MUXES[channel]);
    if (err != ESP_OK) {
        return err;
    }

    err = ads111x_start_conversion(&s_ads1115);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(ADS1115_CONVERSION_WAIT_TICKS);
    return ads111x_get_value(&s_ads1115, raw_value);
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
        (void)ads111x_free_desc(&s_ads1115);
        return;
    }

    ESP_LOGI(TAG, "ADS1115 logger task started (addr=0x%02X)", ADS1115_I2C_ADDR);
}
