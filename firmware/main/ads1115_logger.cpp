#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <driver/gpio.h>

#include <stdio.h>
#include <string.h>

#include <ads111x.h>
#include <i2cdev.h>

#ifdef __cplusplus
}
#endif

#include "pins.h"
#include "ads1115_logger.h"

#define TAG "ads1115"

namespace {

static constexpr i2c_port_t ADS1115_I2C_PORT = I2C_NUM_0;

// Global ADC values storage
static int16_t s_adc_values[4] = {0, 0, 0, 0};
static bool s_last_read_ok = false;
static portMUX_TYPE s_adc_mutex = portMUX_INITIALIZER_UNLOCKED;
static constexpr uint8_t ADS1115_I2C_ADDR = ADS111X_ADDR_GND;
static constexpr uint32_t ADS1115_I2C_FREQ_HZ = 100000;
static constexpr ads111x_gain_t ADS1115_GAIN = ADS111X_GAIN_2V048;
static constexpr ads111x_data_rate_t ADS1115_DATA_RATE = ADS111X_DATA_RATE_128;

static constexpr TickType_t ADS1115_TASK_PERIOD_TICKS = pdMS_TO_TICKS(1000);
static constexpr TickType_t ADS1115_CONVERSION_READY_POLL_TICKS = pdMS_TO_TICKS(1);
static constexpr int64_t ADS1115_CONVERSION_READY_TIMEOUT_US = 50000;
// static constexpr TickType_t ADS1115_SCAN_PERIOD_TICKS = pdMS_TO_TICKS(1000);

static i2c_dev_t s_ads1115;

static void ads1115_enable_internal_pullups(void)
{
    gpio_reset_pin(ADS1115_I2C_SDA_GPIO);
    gpio_reset_pin(ADS1115_I2C_SCL_GPIO);
    gpio_set_direction(ADS1115_I2C_SDA_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(ADS1115_I2C_SCL_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_pullup_en(ADS1115_I2C_SDA_GPIO);
    gpio_pullup_en(ADS1115_I2C_SCL_GPIO);
    gpio_set_level(ADS1115_I2C_SDA_GPIO, 1);
    gpio_set_level(ADS1115_I2C_SCL_GPIO, 1);
}

static esp_err_t ads1115_configure_alert_rdy_gpio(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << ADS1115_ALERT_RDY_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io_conf);
}

static esp_err_t ads1115_wait_for_conversion_ready(void)
{
    const int64_t deadline_us = esp_timer_get_time() + ADS1115_CONVERSION_READY_TIMEOUT_US;

    while (esp_timer_get_time() < deadline_us) {
        if (gpio_get_level(ADS1115_ALERT_RDY_GPIO) == 0) {
            return ESP_OK;
        }
        vTaskDelay(ADS1115_CONVERSION_READY_POLL_TICKS);
    }

    ESP_LOGW(TAG,
             "Timeout cekani na ALERT/RDY pin=%d level=%d",
             (int)ADS1115_ALERT_RDY_GPIO,
             gpio_get_level(ADS1115_ALERT_RDY_GPIO));
    return ESP_ERR_TIMEOUT;
}

/*
static void ads1115_scan_task(void *pv_parameters)
{
    (void)pv_parameters;

    static constexpr uint8_t k_scan_addresses[] = {
        ADS111X_ADDR_GND,
        ADS111X_ADDR_VCC,
        ADS111X_ADDR_SDA,
        ADS111X_ADDR_SCL,
    };

    while (true) {
        char scan_result[192] = {0};
        size_t write_offset = 0;

        for (size_t i = 0; i < sizeof(k_scan_addresses) / sizeof(k_scan_addresses[0]); ++i) {
            i2c_dev_t dev = {};
            dev.port = ADS1115_I2C_PORT;
            dev.addr = k_scan_addresses[i];
            dev.cfg.sda_io_num = ADS1115_I2C_SDA_GPIO;
            dev.cfg.scl_io_num = ADS1115_I2C_SCL_GPIO;
            dev.cfg.sda_pullup_en = true;
            dev.cfg.scl_pullup_en = true;
            dev.cfg.master.clk_speed = ADS1115_I2C_FREQ_HZ;

            const esp_err_t probe_result = i2c_dev_check_present(&dev);
            const bool ack = (probe_result == ESP_OK);

            write_offset += snprintf(scan_result + write_offset,
                                     sizeof(scan_result) - write_offset,
                                     "%s0x%02X=%s",
                                     (i == 0) ? "" : " ",
                                     dev.addr,
                                     ack ? "ACK" : esp_err_to_name(probe_result));
            if (write_offset >= sizeof(scan_result)) {
                break;
            }
        }

        ESP_LOGI(TAG,
                 "ADS I2C scan SDA=%d SCL=%d: %s",
                 (int)ADS1115_I2C_SDA_GPIO,
                 (int)ADS1115_I2C_SCL_GPIO,
                 scan_result);
        vTaskDelay(ADS1115_SCAN_PERIOD_TICKS);
    }
}
*/

static constexpr ads111x_mux_t ADS1115_CHANNEL_MUXES[] = {
    ADS111X_MUX_0_GND,
    ADS111X_MUX_1_GND,
    ADS111X_MUX_2_GND,
    ADS111X_MUX_3_GND,
};

static esp_err_t ads1115_init_device(void)
{
    memset(&s_ads1115, 0, sizeof(s_ads1115));
    esp_err_t err = ads111x_init_desc(&s_ads1115,
                                      ADS1115_I2C_ADDR,
                                      ADS1115_I2C_PORT,
                                      ADS1115_I2C_SDA_GPIO,
                                      ADS1115_I2C_SCL_GPIO);
    if (err != ESP_OK) {
        return err;
    }

    // This dedicated bus is wired directly between ESP32 and ADS1115 on the PCB.
    // The current board revision appears to omit external pull-ups, so enable the
    // internal ones to keep the bus operational during bring-up.
    s_ads1115.cfg.sda_pullup_en = true;
    s_ads1115.cfg.scl_pullup_en = true;

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

    err = ads111x_set_comp_mode(&s_ads1115, ADS111X_COMP_MODE_NORMAL);
    if (err != ESP_OK) {
        (void)ads111x_free_desc(&s_ads1115);
        return err;
    }

    err = ads111x_set_comp_polarity(&s_ads1115, ADS111X_COMP_POLARITY_LOW);
    if (err != ESP_OK) {
        (void)ads111x_free_desc(&s_ads1115);
        return err;
    }

    err = ads111x_set_comp_latch(&s_ads1115, ADS111X_COMP_LATCH_DISABLED);
    if (err != ESP_OK) {
        (void)ads111x_free_desc(&s_ads1115);
        return err;
    }

    // Configure ALERT/RDY to pulse low when a conversion is ready.
    err = ads111x_set_comp_low_thresh(&s_ads1115, 0x0000);
    if (err != ESP_OK) {
        (void)ads111x_free_desc(&s_ads1115);
        return err;
    }

    err = ads111x_set_comp_high_thresh(&s_ads1115, static_cast<int16_t>(0x8000));
    if (err != ESP_OK) {
        (void)ads111x_free_desc(&s_ads1115);
        return err;
    }

    err = ads111x_set_comp_queue(&s_ads1115, ADS111X_COMP_QUEUE_1);
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

    // After switching the mux, discard the first conversion so the input RC
    // network and internal sampling capacitor can settle to the new channel.
    err = ads111x_start_conversion(&s_ads1115);
    if (err != ESP_OK) {
        return err;
    }

    err = ads1115_wait_for_conversion_ready();
    if (err != ESP_OK) {
        return err;
    }

    int16_t throwaway_value = 0;
    err = ads111x_get_value(&s_ads1115, &throwaway_value);
    if (err != ESP_OK) {
        return err;
    }

    err = ads111x_start_conversion(&s_ads1115);
    if (err != ESP_OK) {
        return err;
    }

    err = ads1115_wait_for_conversion_ready();
    if (err != ESP_OK) {
        return err;
    }
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
            taskENTER_CRITICAL(&s_adc_mutex);
            for (uint8_t i = 0; i < 4; ++i) {
                s_adc_values[i] = values[i];
            }
            s_last_read_ok = true;
            taskEXIT_CRITICAL(&s_adc_mutex);

            ESP_LOGI(TAG,
                     "ADS1115 RAW CH0=%d CH1=%d CH2=%d CH3=%d",
                     (int)values[0],
                     (int)values[1],
                     (int)values[2],
                     (int)values[3]);
        } else {
            taskENTER_CRITICAL(&s_adc_mutex);
            s_last_read_ok = false;
            taskEXIT_CRITICAL(&s_adc_mutex);
        }

        vTaskDelay(ADS1115_TASK_PERIOD_TICKS);
    }
}

} // namespace

void ads1115_logger_init(void)
{
    ads1115_enable_internal_pullups();
    ESP_LOGW(TAG,
             "ADS I2C interni pull-up zapnuty na SDA=%d SCL=%d",
             (int)ADS1115_I2C_SDA_GPIO,
             (int)ADS1115_I2C_SCL_GPIO);

    esp_err_t err = ads1115_configure_alert_rdy_gpio();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADS1115 logger disabled: ALERT/RDY GPIO init failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG,
             "ADS ALERT/RDY pin inicializovan: gpio=%d level=%d",
             (int)ADS1115_ALERT_RDY_GPIO,
             gpio_get_level(ADS1115_ALERT_RDY_GPIO));

    err = i2cdev_init();
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

int16_t ads1115_get_channel_value(uint8_t channel)
{
    if (channel >= 4) {
        return 0;
    }
    
    taskENTER_CRITICAL(&s_adc_mutex);
    int16_t value = s_adc_values[channel];
    taskEXIT_CRITICAL(&s_adc_mutex);
    
    return value;
}

bool ads1115_last_read_ok(void)
{
    taskENTER_CRITICAL(&s_adc_mutex);
    bool ok = s_last_read_ok;
    taskEXIT_CRITICAL(&s_adc_mutex);
    
    return ok;
}
