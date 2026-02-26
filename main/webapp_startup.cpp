#include "webapp_startup.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#ifdef __cplusplus
}
#endif

#include "app-config.h"
#include "config_webapp.h"
#include "esp_log.h"
#include "hladina-demo.h"
#include "restart_info.h"

static const char *TAG = "webapp_startup";
static bool s_webapp_started = false;
static TimerHandle_t s_webapp_auto_stop_timer = nullptr;
static constexpr uint32_t WEBAPP_AUTO_STOP_MS = 2U * 60U * 60U * 1000U;

static void webapp_auto_stop_timer_cb(TimerHandle_t timer)
{
    (void)timer;

    if (!s_webapp_started) {
        return;
    }

    esp_err_t stop_result = config_webapp_stop();
    if (stop_result == ESP_OK) {
        s_webapp_started = false;
        ESP_LOGI(TAG, "Config web app automaticky vypnuta po 2 hodinach");
    } else {
        ESP_LOGW(TAG, "Automaticke vypnuti config web app selhalo: %s", esp_err_to_name(stop_result));
    }
}

static void ensure_webapp_auto_stop_timer(void)
{
    if (s_webapp_auto_stop_timer != nullptr) {
        return;
    }

    s_webapp_auto_stop_timer = xTimerCreate(
        "webapp_auto_off",
        pdMS_TO_TICKS(WEBAPP_AUTO_STOP_MS),
        pdFALSE,
        nullptr,
        webapp_auto_stop_timer_cb);

    if (s_webapp_auto_stop_timer == nullptr) {
        ESP_LOGE(TAG, "Nelze vytvorit webapp auto-off timer");
    }
}

static void restart_webapp_auto_stop_timer(void)
{
    ensure_webapp_auto_stop_timer();
    if (s_webapp_auto_stop_timer == nullptr) {
        return;
    }

    xTimerStop(s_webapp_auto_stop_timer, 0);
    xTimerChangePeriod(s_webapp_auto_stop_timer, pdMS_TO_TICKS(WEBAPP_AUTO_STOP_MS), 0);
    xTimerStart(s_webapp_auto_stop_timer, 0);
}

void webapp_startup_on_network_event(const network_event_t *event)
{
    (void)event;
    // Webova aplikace je implicitne vypnuta, startuje se pouze explicitnim commandem.
}

esp_err_t webapp_startup_start(void)
{
    if (s_webapp_started) {
        restart_webapp_auto_stop_timer();
        return ESP_OK;
    }

    char wifi_ssid[32] = {0};
    char wifi_password[64] = {0};
    esp_err_t wifi_cfg_result = app_config_load_wifi_credentials(
        wifi_ssid,
        sizeof(wifi_ssid),
        wifi_password,
        sizeof(wifi_password));
    if (wifi_cfg_result != ESP_OK) {
        ESP_LOGW(TAG, "Nelze nacist WiFi SSID pro webapp: %s", esp_err_to_name(wifi_cfg_result));
        wifi_ssid[0] = '\0';
    }

    const config_group_t config_groups[] = {
        app_config_get_config_group(),
        hladina_demo_get_config_group(),
    };

    app_restart_info_t restart_info = {};
    esp_err_t restart_result = app_restart_info_update_and_load(&restart_info);
    if (restart_result != ESP_OK) {
        ESP_LOGW(TAG, "Nelze nacist restart info pro webapp: %s", esp_err_to_name(restart_result));
    }

    config_webapp_restart_info_t webapp_restart_info = {
        .boot_count = restart_info.boot_count,
        .last_reason = static_cast<int32_t>(restart_info.last_reason),
        .last_restart_unix = restart_info.last_restart_unix,
    };

    config_webapp_network_info_t webapp_network_info = {
        .is_ap_mode = false,
        .active_ssid = wifi_ssid,
    };

    esp_err_t start_result = config_webapp_start(
        "app_cfg",
        config_groups,
        sizeof(config_groups) / sizeof(config_groups[0]),
        80,
        &webapp_restart_info,
        &webapp_network_info);

    if (start_result == ESP_OK) {
        s_webapp_started = true;
        restart_webapp_auto_stop_timer();
        ESP_LOGI(TAG, "Config web app spustena (auto-stop za 2 hodiny)");
    } else {
        ESP_LOGW(TAG, "Config web app se nepodarilo spustit: %s", esp_err_to_name(start_result));
    }

    return start_result;
}

esp_err_t webapp_startup_stop(void)
{
    esp_err_t stop_result = config_webapp_stop();
    if (stop_result == ESP_OK) {
        s_webapp_started = false;
        if (s_webapp_auto_stop_timer != nullptr) {
            xTimerStop(s_webapp_auto_stop_timer, 0);
        }
        ESP_LOGI(TAG, "Config web app zastavena");
    } else {
        ESP_LOGW(TAG, "Zastaveni config web app selhalo: %s", esp_err_to_name(stop_result));
    }

    return stop_result;
}
