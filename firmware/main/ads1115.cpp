#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>

#include <ads111x.h>
#include <i2cdev.h>

#ifdef __cplusplus
}
#endif

#include <string.h>

#include "ads1115.h"
#include "pins.h"

#define TAG "ads1115"

namespace {

static constexpr i2c_port_t ADS1115_I2C_PORT = I2C_NUM_0;
static constexpr uint8_t ADS1115_I2C_ADDR = ADS111X_ADDR_GND;
static constexpr uint32_t ADS1115_I2C_FREQ_HZ = 100000;
static constexpr ads111x_gain_t ADS1115_GAIN = ADS111X_GAIN_2V048;

static constexpr TickType_t PRESSURE_PERIOD_TICKS = pdMS_TO_TICKS(250);
static constexpr TickType_t LEVEL_PERIOD_TICKS = pdMS_TO_TICKS(2000);
static constexpr TickType_t PRESSURE_READY_TIMEOUT_TICKS = pdMS_TO_TICKS(80);
static constexpr TickType_t LEVEL_READY_TIMEOUT_TICKS = pdMS_TO_TICKS(250);

static_assert(PRESSURE_PERIOD_TICKS > 0, "Pressure period must be at least one tick");
static_assert(LEVEL_PERIOD_TICKS > 0, "Level period must be at least one tick");

static i2c_dev_t s_ads1115;
static bool s_ads1115_ready = false;
static TaskHandle_t s_ads1115_task_handle = nullptr;

static StaticQueue_t s_pressure_queue_storage;
static uint8_t s_pressure_queue_buffer[sizeof(ads1115_pressure_sample_t)];
static QueueHandle_t s_pressure_queue = nullptr;

static StaticQueue_t s_level_queue_storage;
static uint8_t s_level_queue_buffer[sizeof(ads1115_level_sample_t)];
static QueueHandle_t s_level_queue = nullptr;

static bool tick_reached(TickType_t now, TickType_t deadline)
{
    return static_cast<int32_t>(now - deadline) >= 0;
}

static TickType_t earlier_tick(TickType_t a, TickType_t b)
{
    return tick_reached(a, b) ? b : a;
}

static void advance_release_after(TickType_t *release_tick, TickType_t period, TickType_t now)
{
    do {
        *release_tick += period;
    } while (tick_reached(now, *release_tick));
}

static ads1115_pressure_sample_t invalid_pressure_sample(void)
{
    return {
        .pressure_after_filter_raw = ADS1115_INVALID_RAW_VALUE,
        .pressure_before_filter_raw = ADS1115_INVALID_RAW_VALUE,
    };
}

static ads1115_level_sample_t invalid_level_sample(void)
{
    return {
        .level_raw = ADS1115_INVALID_RAW_VALUE,
    };
}

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

static void IRAM_ATTR ads1115_alert_rdy_isr(void *arg)
{
    (void)arg;

    const TaskHandle_t task = s_ads1115_task_handle;
    if (task == nullptr) {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t ads1115_configure_alert_rdy_gpio(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << ADS1115_ALERT_RDY_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = gpio_isr_handler_add(ADS1115_ALERT_RDY_GPIO, ads1115_alert_rdy_isr, nullptr);
    if (err == ESP_ERR_INVALID_STATE) {
        err = gpio_isr_handler_remove(ADS1115_ALERT_RDY_GPIO);
        if (err != ESP_OK) {
            return err;
        }
        err = gpio_isr_handler_add(ADS1115_ALERT_RDY_GPIO, ads1115_alert_rdy_isr, nullptr);
    }

    return err;
}

static esp_err_t ads1115_configure_alert_rdy_mode(void)
{
    esp_err_t err = ads111x_set_comp_mode(&s_ads1115, ADS111X_COMP_MODE_NORMAL);
    if (err != ESP_OK) {
        return err;
    }

    err = ads111x_set_comp_polarity(&s_ads1115, ADS111X_COMP_POLARITY_LOW);
    if (err != ESP_OK) {
        return err;
    }

    err = ads111x_set_comp_latch(&s_ads1115, ADS111X_COMP_LATCH_DISABLED);
    if (err != ESP_OK) {
        return err;
    }

    err = ads111x_set_comp_low_thresh(&s_ads1115, 0x0000);
    if (err != ESP_OK) {
        return err;
    }

    err = ads111x_set_comp_high_thresh(&s_ads1115, static_cast<int16_t>(0x8000));
    if (err != ESP_OK) {
        return err;
    }

    return ads111x_set_comp_queue(&s_ads1115, ADS111X_COMP_QUEUE_1);
}

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

    s_ads1115.cfg.sda_pullup_en = true;
    s_ads1115.cfg.scl_pullup_en = true;
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

    err = ads1115_configure_alert_rdy_mode();
    if (err != ESP_OK) {
        (void)ads111x_free_desc(&s_ads1115);
        return err;
    }

    return ESP_OK;
}

static esp_err_t ads1115_wait_for_conversion_ready(TickType_t timeout_ticks)
{
    if (ulTaskNotifyTake(pdTRUE, timeout_ticks) > 0) {
        return ESP_OK;
    }

    ESP_LOGW(TAG,
             "Timeout cekani na ADS1115 ALERT/RDY pin=%d level=%d",
             (int)ADS1115_ALERT_RDY_GPIO,
             gpio_get_level(ADS1115_ALERT_RDY_GPIO));
    return ESP_ERR_TIMEOUT;
}

static esp_err_t ads1115_read_single_channel(ads111x_mux_t mux,
                                             ads111x_data_rate_t data_rate,
                                             TickType_t ready_timeout_ticks,
                                             int16_t *raw_value)
{
    if (raw_value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ads111x_set_data_rate(&s_ads1115, data_rate);
    if (err != ESP_OK) {
        return err;
    }

    err = ads111x_set_input_mux(&s_ads1115, mux);
    if (err != ESP_OK) {
        return err;
    }

    (void)ulTaskNotifyTake(pdTRUE, 0);
    err = ads111x_start_conversion(&s_ads1115);
    if (err != ESP_OK) {
        return err;
    }

    err = ads1115_wait_for_conversion_ready(ready_timeout_ticks);
    if (err != ESP_OK) {
        return err;
    }

    return ads111x_get_value(&s_ads1115, raw_value);
}

static int16_t ads1115_read_raw_or_invalid(ads111x_mux_t mux,
                                           ads111x_data_rate_t data_rate,
                                           TickType_t ready_timeout_ticks,
                                           const char *name)
{
    if (!s_ads1115_ready) {
        return ADS1115_INVALID_RAW_VALUE;
    }

    int16_t raw_value = ADS1115_INVALID_RAW_VALUE;
    const esp_err_t err = ads1115_read_single_channel(mux, data_rate, ready_timeout_ticks, &raw_value);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Cteni %s selhalo: %s", name, esp_err_to_name(err));
        return ADS1115_INVALID_RAW_VALUE;
    }

    return raw_value;
}

static void publish_pressure_sample(void)
{
    ads1115_pressure_sample_t sample = {
        .pressure_after_filter_raw = ads1115_read_raw_or_invalid(ADS111X_MUX_1_GND,
                                                                 ADS111X_DATA_RATE_128,
                                                                 PRESSURE_READY_TIMEOUT_TICKS,
                                                                 "tlak za filtrem CH1"),
        .pressure_before_filter_raw = ads1115_read_raw_or_invalid(ADS111X_MUX_2_GND,
                                                                  ADS111X_DATA_RATE_128,
                                                                  PRESSURE_READY_TIMEOUT_TICKS,
                                                                  "tlak pred filtrem CH2"),
    };

    (void)xQueueOverwrite(s_pressure_queue, &sample);
}

static void publish_level_sample(void)
{
    ads1115_level_sample_t sample = {
        .level_raw = ads1115_read_raw_or_invalid(ADS111X_MUX_3_GND,
                                                ADS111X_DATA_RATE_16,
                                                LEVEL_READY_TIMEOUT_TICKS,
                                                "hladina CH3"),
    };

    (void)xQueueOverwrite(s_level_queue, &sample);
}

static void ads1115_task(void *pv_parameters)
{
    (void)pv_parameters;

    s_ads1115_task_handle = xTaskGetCurrentTaskHandle();

    ads1115_pressure_sample_t initial_pressure_sample = invalid_pressure_sample();
    ads1115_level_sample_t initial_level_sample = invalid_level_sample();
    xQueueOverwrite(s_pressure_queue, &initial_pressure_sample);
    xQueueOverwrite(s_level_queue, &initial_level_sample);

    TickType_t next_pressure_tick = xTaskGetTickCount();
    TickType_t next_level_tick = next_pressure_tick;

    while (true) {
        const TickType_t now = xTaskGetTickCount();

        if (tick_reached(now, next_pressure_tick)) {
            publish_pressure_sample();
            advance_release_after(&next_pressure_tick, PRESSURE_PERIOD_TICKS, xTaskGetTickCount());
        }

        if (tick_reached(now, next_level_tick)) {
            publish_level_sample();
            advance_release_after(&next_level_tick, LEVEL_PERIOD_TICKS, xTaskGetTickCount());
        }

        const TickType_t wake_tick = earlier_tick(next_pressure_tick, next_level_tick);
        const TickType_t sleep_now = xTaskGetTickCount();
        if (!tick_reached(sleep_now, wake_tick)) {
            vTaskDelay(wake_tick - sleep_now);
        }
    }
}

static void ads1115_pressure_test_task(void *pv_parameters)
{
    (void)pv_parameters;

    ads1115_pressure_sample_t sample = {};
    while (true) {
        if (xQueueReceive(s_pressure_queue, &sample, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        ESP_LOGI(TAG,
                 "RAW tlak: za_filtrem_CH1=%d pred_filtrem_CH2=%d",
                 (int)sample.pressure_after_filter_raw,
                 (int)sample.pressure_before_filter_raw);
    }
}

static void ads1115_level_test_task(void *pv_parameters)
{
    (void)pv_parameters;

    ads1115_level_sample_t sample = {};
    while (true) {
        if (xQueueReceive(s_level_queue, &sample, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        ESP_LOGI(TAG, "RAW hladina_CH3=%d", (int)sample.level_raw);
    }
}

} // namespace

void ads1115_start(void)
{
    if (s_pressure_queue != nullptr || s_level_queue != nullptr) {
        ESP_LOGW(TAG, "ADS1115 uz je spusteny");
        return;
    }

    s_pressure_queue = xQueueCreateStatic(1,
                                          sizeof(ads1115_pressure_sample_t),
                                          s_pressure_queue_buffer,
                                          &s_pressure_queue_storage);
    s_level_queue = xQueueCreateStatic(1,
                                       sizeof(ads1115_level_sample_t),
                                       s_level_queue_buffer,
                                       &s_level_queue_storage);
    if (s_pressure_queue == nullptr || s_level_queue == nullptr) {
        ESP_LOGE(TAG, "Vytvoreni ADS1115 front selhalo");
        return;
    }

    ads1115_enable_internal_pullups();
    ESP_LOGW(TAG,
             "ADS I2C interni pull-up zapnuty na SDA=%d SCL=%d",
             (int)ADS1115_I2C_SDA_GPIO,
             (int)ADS1115_I2C_SCL_GPIO);

    esp_err_t err = i2cdev_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADS1115 i2cdev_init selhal: %s, budu publikovat invalid hodnoty", esp_err_to_name(err));
    } else {
        err = ads1115_init_device();
        if (err != ESP_OK) {
            ESP_LOGW(TAG,
                     "ADS1115 init selhal na adrese 0x%02X: %s, budu publikovat invalid hodnoty",
                     ADS1115_I2C_ADDR,
                     esp_err_to_name(err));
        } else {
            s_ads1115_ready = true;
        }
    }

    if (s_ads1115_ready) {
        err = ads1115_configure_alert_rdy_gpio();
        if (err != ESP_OK) {
            ESP_LOGW(TAG,
                     "ADS1115 ALERT/RDY GPIO init selhal: %s, budu publikovat invalid hodnoty",
                     esp_err_to_name(err));
            s_ads1115_ready = false;
        } else {
            ESP_LOGI(TAG,
                     "ADS1115 ALERT/RDY pin inicializovan: gpio=%d level=%d",
                     (int)ADS1115_ALERT_RDY_GPIO,
                     gpio_get_level(ADS1115_ALERT_RDY_GPIO));
        }
    }

    if (xTaskCreate(ads1115_task, "ads1115", 4096, nullptr, 5, &s_ads1115_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Vytvoreni ADS1115 tasku selhalo");
        s_ads1115_task_handle = nullptr;
        s_ads1115_ready = false;
        return;
    }

    if (xTaskCreate(ads1115_pressure_test_task, "ads1115_tlak_log", 3072, nullptr, 4, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "Vytvoreni ADS1115 test tasku pro tlak selhalo");
    }

    if (xTaskCreate(ads1115_level_test_task, "ads1115_hlad_log", 3072, nullptr, 4, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "Vytvoreni ADS1115 test tasku pro hladinu selhalo");
    }

    ESP_LOGI(TAG,
             "ADS1115 spusten: pressure_period=%lu ms, level_period=%lu ms, adc_ready=%d",
             (unsigned long)pdTICKS_TO_MS(PRESSURE_PERIOD_TICKS),
             (unsigned long)pdTICKS_TO_MS(LEVEL_PERIOD_TICKS),
             s_ads1115_ready ? 1 : 0);
}

QueueHandle_t ads1115_pressure_queue(void)
{
    return s_pressure_queue;
}

QueueHandle_t ads1115_level_queue(void)
{
    return s_level_queue;
}
