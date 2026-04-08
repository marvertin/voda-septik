#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/gpio.h>

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

// Global ADC values storage
static int16_t s_adc_values[4] = {0, 0, 0, 0};
static bool s_last_read_ok = false;
static portMUX_TYPE s_adc_mutex = portMUX_INITIALIZER_UNLOCKED;
static constexpr uint8_t ADS1115_I2C_ADDR = ADS111X_ADDR_GND;
static constexpr uint32_t ADS1115_I2C_FREQ_HZ = 100000;
static constexpr ads111x_gain_t ADS1115_GAIN = ADS111X_GAIN_2V048;
static constexpr ads111x_data_rate_t ADS1115_DATA_RATE = ADS111X_DATA_RATE_128;

static constexpr TickType_t ADS1115_TASK_PERIOD_TICKS = pdMS_TO_TICKS(1000);
static constexpr TickType_t ADS1115_CONVERSION_WAIT_TICKS = pdMS_TO_TICKS(10);

static constexpr float FLOW_MAX_LITERS_PER_MIN = 120.0f;
static constexpr float FLOW_PULSES_PER_LITER = 36.0f;
static constexpr float FLOW_MAX_FREQ_HZ = (FLOW_MAX_LITERS_PER_MIN * FLOW_PULSES_PER_LITER) / 60.0f;
static constexpr TickType_t FLOW_SIM_TASK_PERIOD_TICKS = (pdMS_TO_TICKS(1) > 0) ? pdMS_TO_TICKS(1) : 1;
static constexpr TickType_t FLOW_SIM_SETPOINT_REFRESH_TICKS = (pdMS_TO_TICKS(50) > 0) ? pdMS_TO_TICKS(50) : 1;
static constexpr float FLOW_SIM_TASK_PERIOD_S =
    static_cast<float>(FLOW_SIM_TASK_PERIOD_TICKS) / static_cast<float>(configTICK_RATE_HZ);

static i2c_dev_t s_ads1115;

static float clamp01(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

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

static void flow_pulse_simulator_task(void *pv_parameters)
{
    (void)pv_parameters;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << FLOW_SIMULATOR_OUTPUT_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGE(TAG, "Flow simulator GPIO init selhala (pin=%d)", (int)FLOW_SIMULATOR_OUTPUT_GPIO);
        vTaskDelete(nullptr);
        return;
    }

    gpio_set_level(FLOW_SIMULATOR_OUTPUT_GPIO, 0);

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_setpoint_refresh = 0;
    float freq_hz = 0.0f;
    float pulse_accumulator = 0.0f;
    bool pulse_high = false;
    int16_t last_logged_raw_ch0 = INT16_MIN;
    bool last_logged_read_ok = false;

    while (true) {
        vTaskDelayUntil(&last_wake, FLOW_SIM_TASK_PERIOD_TICKS);

        const TickType_t now_ticks = xTaskGetTickCount();
        if ((now_ticks - last_setpoint_refresh) >= FLOW_SIM_SETPOINT_REFRESH_TICKS) {
            last_setpoint_refresh = now_ticks;

            const int16_t raw_ch0 = ads1115_get_channel_value(0);
            const bool read_ok = ads1115_last_read_ok();

            if (!read_ok || raw_ch0 <= 0) {
                freq_hz = 0.0f;
            } else {
                const float normalized = clamp01(static_cast<float>(raw_ch0)
                    / static_cast<float>(ADS111X_MAX_VALUE));
                freq_hz = normalized * FLOW_MAX_FREQ_HZ;
            }

            if (raw_ch0 != last_logged_raw_ch0 || read_ok != last_logged_read_ok) {
                const float simulated_l_min = (freq_hz * 60.0f) / FLOW_PULSES_PER_LITER;
                ESP_LOGI(TAG,
                         "Flow sim changed: raw_ch0=%d read_ok=%d flow=%.2f l/min freq=%.2f Hz",
                         (int)raw_ch0,
                         read_ok ? 1 : 0,
                         (double)simulated_l_min,
                         (double)freq_hz);
                last_logged_raw_ch0 = raw_ch0;
                last_logged_read_ok = read_ok;
            }
        }

        // Pulse width is one scheduler tick, rising edges encode frequency.
        if (pulse_high) {
            gpio_set_level(FLOW_SIMULATOR_OUTPUT_GPIO, 0);
            pulse_high = false;
        }

        if (freq_hz <= 0.0f) {
            continue;
        }

        pulse_accumulator += freq_hz * FLOW_SIM_TASK_PERIOD_S;
        if (pulse_accumulator >= 1.0f) {
            pulse_accumulator -= 1.0f;
            gpio_set_level(FLOW_SIMULATOR_OUTPUT_GPIO, 1);
            pulse_high = true;
        }
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

    if (xTaskCreate(flow_pulse_simulator_task, "flow_sim_adc", 3072, nullptr, 3, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "Flow pulse simulator task creation failed");
    } else {
        ESP_LOGI(TAG,
                 "Flow pulse simulator started: pin=%d max_flow=%.1f l/min max_freq=%.1f Hz",
                 (int)FLOW_SIMULATOR_OUTPUT_GPIO,
                 (double)FLOW_MAX_LITERS_PER_MIN,
                 (double)FLOW_MAX_FREQ_HZ);
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
