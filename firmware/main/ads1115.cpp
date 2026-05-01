#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <ads111x.h>
#include <i2cdev.h>

#ifdef __cplusplus
}
#endif

#include <string.h>

#include "app_error_check.h"
#include "ads1115.h"
#include "pins.h"

#define TAG "ads1115"

/*
 * Modul ADS1115 je producent surovych ADC hodnot pro navazujici logiku tlaku
 * a hladiny. Bezi zde jediny merici task, ktery seriove prepina vstupni MUX
 * externiho ADC, spousti single-shot konverze a po dokonceni publikuje posledni
 * zname hodnoty do jednoprvkovych FreeRTOS front.
 *
 * Zapojeni U3 / ADS1115:
 *   - I2C0, adresa 0x48 (ADDR pin na GND), SDA=GPIO14, SCL=GPIO13.
 *   - ALERT/RDY=GPIO4. Pin je nakonfigurovan jako "conversion ready" vystup a
 *     po dokonceni konverze udela sestupnou hranu, ktera probudi merici task.
 *   - Vstupy jsou pouzite jako single-ended proti GND:
 *       AIN0: zatim nepouzito / rezerva,
 *       AIN1: tlak za filtrem,
 *       AIN2: tlak pred filtrem,
 *       AIN3: hladina zasoby.
 *
 * Architektura schvalne necte ADS1115 z vice mist. I2C, MUX, data-rate i
 * start konverze jsou sdilene registry cipu; centralni task tak drzi poradi
 * mereni deterministicke a konzumenti jen berou hotove vzorky z front.
 */
namespace {

// Staticka konfigurace sbernice a ADC. ADS1115 je sice specifikovany az do
// 400 kHz I2C, ale 100 kHz je zde dostatecne a dava rezervu pri bring-upu desky
// s internimi pull-upy.
static constexpr i2c_port_t ADS1115_I2C_PORT = I2C_NUM_0;
static constexpr uint8_t ADS1115_I2C_ADDR = ADS111X_ADDR_GND;
static constexpr uint32_t ADS1115_I2C_FREQ_HZ = 100000;

static constexpr ads111x_mux_t PRESSURE_AFTER_FILTER_MUX = ADS111X_MUX_1_GND;
static constexpr ads111x_mux_t PRESSURE_BEFORE_FILTER_MUX = ADS111X_MUX_2_GND;
static constexpr ads111x_mux_t LEVEL_MUX = ADS111X_MUX_3_GND;

// PGA rozsah je presne +/-2.048 V. ADS1115 vraci signed 16bit hodnotu, takze
// LSB = 2.048 V / 32768 = 62.5 uV. V single-ended rezimu proti GND se realne
// vyuziva nezaporna pulka rozsahu:
//   raw 0     = 0.0000000 V,
//   raw 32767 = 2.0479375 V.
//
// Proudove smycky 4-20 mA jsou merene pres 100ohm shunt:
//   4 mA  * 100 ohm = 0.4 V -> raw  6400,
//   20 mA * 100 ohm = 2.0 V -> raw 32000.
// Rozsah PGA +/-2.048 V tedy dava nad 20 mA jeste rezervu 48 mV, tj. 768 LSB.
static constexpr ads111x_gain_t ADS1115_GAIN = ADS111X_GAIN_2V048;

// Casove konstanty publikace:
//   - tlaky se ctou po 250 ms, protoze jsou uzitecne pro rychlejsi diagnostiku
//     stavu filtru a cerpadla,
//   - hladina se cte po 2 s, protoze fyzicky meni pomalu a pomalejsi cyklus
//     zmensuje sum i zatez I2C/ADC.
//
// SPS / data-rate:
//   - tlakove kanaly AIN1 a AIN2 se ctou na 128 SPS (~7.8125 ms/konverze),
//   - hladinovy kanal AIN3 se cte na 16 SPS (~62.5 ms/konverze).
//
// Timeouty jsou delsi nez nominalni doba konverze; rezerva pokryva planovani
// FreeRTOS, ISR latenci a I2C provoz.
static constexpr TickType_t PRESSURE_PERIOD_TICKS = pdMS_TO_TICKS(250);
static constexpr TickType_t LEVEL_PERIOD_TICKS = pdMS_TO_TICKS(2000);
static constexpr TickType_t PRESSURE_READY_TIMEOUT_TICKS = pdMS_TO_TICKS(80);
static constexpr TickType_t LEVEL_READY_TIMEOUT_TICKS = pdMS_TO_TICKS(250);
static constexpr int64_t ADS1115_WARN_MIN_INTERVAL_US = 60LL * 1000LL * 1000LL;

static_assert(PRESSURE_PERIOD_TICKS > 0, "Pressure period must be at least one tick");
static_assert(LEVEL_PERIOD_TICKS > 0, "Level period must be at least one tick");

static i2c_dev_t s_ads1115;
static bool s_ads1115_ready = false;
static TaskHandle_t s_ads1115_task_handle = nullptr;
static uint32_t s_read_errors = 0;
static uint32_t s_suppressed_read_warnings = 0;
static int64_t s_last_read_warning_us = 0;
static int64_t s_last_ok_us = 0;

// Jednoprvkove fronty slouzi jako "posledni znama hodnota". Producent pouziva
// xQueueOverwrite(), takze pomaly konzument nikdy nezahlti system starymi vzorky
// a rychly konzument vzdy dostane aktualni snapshot dane skupiny signalu.
static StaticQueue_t s_pressure_queue_storage;
static uint8_t s_pressure_queue_buffer[sizeof(ads1115_pressure_sample_t)];
static QueueHandle_t s_pressure_queue = nullptr;

static StaticQueue_t s_level_queue_storage;
static uint8_t s_level_queue_buffer[sizeof(ads1115_level_sample_t)];
static QueueHandle_t s_level_queue = nullptr;

static bool tick_reached(TickType_t now, TickType_t deadline)
{
    // Porovnani funguje i pres preteceni FreeRTOS tick counteru.
    return static_cast<int32_t>(now - deadline) >= 0;
}

static TickType_t earlier_tick(TickType_t a, TickType_t b)
{
    return tick_reached(a, b) ? b : a;
}

static void advance_release_after(TickType_t *release_tick, TickType_t period, TickType_t now)
{
    // Drzime pevny casovy rastr. Kdyz cteni trvalo moc dlouho, preskocime
    // zmeskane periody misto kumulace zpozdeni.
    do {
        *release_tick += period;
    } while (tick_reached(now, *release_tick));
}

static ads1115_pressure_sample_t invalid_pressure_sample(void)
{
    // Single-ended vstupy z ADS1115 nemaji pro funkcni cidla merit INT16_MIN.
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
    // Na aktualni desce je ADS1115 na samostatne I2C vetvi. Interni pull-upy
    // nejsou idealni pro dlouhe/rychle sbernice, ale pro lokalni 100kHz spoj
    // ESP32 <-> ADS1115 pomahaji, pokud na revizi chybi externi odpory.
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

    // ISR jen probudi merici task. I2C a cteni registru patri az do task kontextu.
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
    // ADS1115 stahuje ALERT/RDY do nuly po dokonceni konverze.
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
    // Specialni kombinace prahu podle datasheetu prepne comparator do
    // "conversion ready" rezimu. ALERT/RDY pak neni alarm od hodnoty signalu,
    // ale hardwarovy priznak dokoncene single-shot konverze.
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

    // Single-shot rezim je dulezity, protoze pred kazdou konverzi menime MUX
    // a pro kazdy kanal muzeme volit jinou rychlost vzorkovani.
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

    return ESP_ERR_TIMEOUT;
}

static void log_read_warning_rate_limited(const char *name, esp_err_t err)
{
    const int64_t now_us = esp_timer_get_time();
    if (s_last_read_warning_us != 0 && (now_us - s_last_read_warning_us) < ADS1115_WARN_MIN_INTERVAL_US) {
        ++s_suppressed_read_warnings;
        return;
    }

    const uint32_t suppressed = s_suppressed_read_warnings;
    s_suppressed_read_warnings = 0;
    s_last_read_warning_us = now_us;

    ESP_LOGW(TAG,
             "Cteni %s selhalo: %s, ALERT/RDY level=%d, potlaceno=%lu",
             name,
             esp_err_to_name(err),
             gpio_get_level(ADS1115_ALERT_RDY_GPIO),
             (unsigned long)suppressed);
}

static esp_err_t ads1115_read_single_channel(ads111x_mux_t mux,
                                             ads111x_data_rate_t data_rate,
                                             TickType_t ready_timeout_ticks,
                                             int16_t *raw_value)
{
    if (raw_value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    // Poradi je zamerne: nejdrive nastavit parametry dalsi konverze, potom
    // smazat starou task notifikaci a az nakonec spustit single-shot mereni.
    esp_err_t err = ads111x_set_data_rate(&s_ads1115, data_rate);
    if (err != ESP_OK) {
        return err;
    }

    err = ads111x_set_input_mux(&s_ads1115, mux);
    if (err != ESP_OK) {
        return err;
    }

    // Zahodime pripadnou starou notifikaci, aby nas probudila az aktualne spustena konverze.
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
        // ADC neni pro dalsi system fatalni; downstream logika dostane hodnotu mimo meritelny rozsah.
        return ADS1115_INVALID_RAW_VALUE;
    }

    int16_t raw_value = ADS1115_INVALID_RAW_VALUE;
    const esp_err_t err = ads1115_read_single_channel(mux, data_rate, ready_timeout_ticks, &raw_value);
    if (err != ESP_OK) {
        ++s_read_errors;
        log_read_warning_rate_limited(name, err);
        return ADS1115_INVALID_RAW_VALUE;
    }

    s_last_ok_us = esp_timer_get_time();
    return raw_value;
}

static void publish_pressure_sample(void)
{
    // CH1 = tlak za filtrem, CH2 = tlak pred filtrem. Oba tlaky cteme hned po
    // sobe a publikujeme jako jeden snapshot, aby navazujici vypocty porovnavaly
    // hodnoty ze stejneho casoveho okna.
    //
    // 128 SPS je kompromis mezi rychlosti a sumem. Jedna konverze trva zhruba
    // 7.8 ms, tedy oba tlakove kanaly se pohodlne vejdou do 250ms periody.
    ads1115_pressure_sample_t sample = {
        .pressure_after_filter_raw = ads1115_read_raw_or_invalid(PRESSURE_AFTER_FILTER_MUX,
                                                                 ADS111X_DATA_RATE_128,
                                                                 PRESSURE_READY_TIMEOUT_TICKS,
                                                                 "tlak za filtrem CH1"),
        .pressure_before_filter_raw = ads1115_read_raw_or_invalid(PRESSURE_BEFORE_FILTER_MUX,
                                                                  ADS111X_DATA_RATE_128,
                                                                  PRESSURE_READY_TIMEOUT_TICKS,
                                                                  "tlak pred filtrem CH2"),
    };

    (void)xQueueOverwrite(s_pressure_queue, &sample);
}

static void publish_level_sample(void)
{
    // CH3 = vyska hladiny. 16 SPS prodlouzi konverzi asi na 62.5 ms, ale hladina
    // je pomaly signal a nizsi data-rate potlacuje sum.
    ads1115_level_sample_t sample = {
        .level_raw = ads1115_read_raw_or_invalid(LEVEL_MUX,
                                                ADS111X_DATA_RATE_16,
                                                LEVEL_READY_TIMEOUT_TICKS,
                                                "hladina CH3"),
    };

    (void)xQueueOverwrite(s_level_queue, &sample);
}

static void ads1115_task(void *pv_parameters)
{
    (void)pv_parameters;

    // Task handle se pouziva jako cil notifikace z ALERT/RDY ISR.
    s_ads1115_task_handle = xTaskGetCurrentTaskHandle();

    // Fronty maji platny obsah okamzite po startu, i kdyz prvni mereni jeste nedobehlo.
    ads1115_pressure_sample_t initial_pressure_sample = invalid_pressure_sample();
    ads1115_level_sample_t initial_level_sample = invalid_level_sample();
    xQueueOverwrite(s_pressure_queue, &initial_pressure_sample);
    xQueueOverwrite(s_level_queue, &initial_level_sample);

    // Obe skupiny startuji okamzite po spusteni tasku. Dale uz se ridi vlastnim
    // periodickym rastrem; hladina tedy nebrzdi rychlejsi tlakovy cyklus mimo
    // okamzik, kdy zrovna probiha jeji delsi single-shot konverze.
    TickType_t next_pressure_tick = xTaskGetTickCount();
    TickType_t next_level_tick = next_pressure_tick;

    while (true) {
        const TickType_t now = xTaskGetTickCount();

        // Casovani publikace se odviji od planovanych ticku, ne od delky posledniho cteni.
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

} // namespace

void ads1115_start(void)
{
    // Konfiguracni a alokacni chyby jsou fatalni. Nepritomnost nebo chyba
    // samotneho ADS1115 fatalni neni: task dal bezi a publikuje INT16_MIN, aby
    // zbytek aplikace mohl signalizovat chybu cidla bez padu firmware.
    APP_ERROR_CHECK("E780", (ADS1115_I2C_SDA_GPIO == ADS1115_I2C_SCL_GPIO) ? ESP_ERR_INVALID_ARG : ESP_OK);
    APP_ERROR_CHECK("E781", (s_pressure_queue == nullptr && s_level_queue == nullptr) ? ESP_OK : ESP_ERR_INVALID_STATE);

    s_pressure_queue = xQueueCreateStatic(1,
                                          sizeof(ads1115_pressure_sample_t),
                                          s_pressure_queue_buffer,
                                          &s_pressure_queue_storage);
    s_level_queue = xQueueCreateStatic(1,
                                       sizeof(ads1115_level_sample_t),
                                       s_level_queue_buffer,
                                       &s_level_queue_storage);
    APP_ERROR_CHECK("E782", s_pressure_queue != nullptr ? ESP_OK : ESP_ERR_NO_MEM);
    APP_ERROR_CHECK("E783", s_level_queue != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

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

    APP_ERROR_CHECK("E784",
                    xTaskCreate(ads1115_task,
                                "ads1115",
                                4096,
                                nullptr,
                                5,
                                &s_ads1115_task_handle) == pdPASS
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);

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

ads1115_diag_t ads1115_diag_snapshot(void)
{
    int64_t last_ok_age_s = -1;
    if (s_last_ok_us > 0) {
        const int64_t age_us = esp_timer_get_time() - s_last_ok_us;
        last_ok_age_s = (age_us > 0) ? (age_us / 1000000LL) : 0;
    }

    return {
        .ready = s_ads1115_ready,
        .read_errors = s_read_errors,
        .last_ok_age_s = last_ok_age_s,
    };
}
