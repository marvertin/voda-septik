#include "webapp_startup.h"

#include "app-config.h"
#include "config_webapp.h"
#include "esp_log.h"
#include "hladina-demo.h"
#include "restart_info.h"

static const char *TAG = "webapp_startup";
static bool s_webapp_started = false;

static bool network_event_has_connectivity(const network_event_t *event)
{
    if (event == nullptr) {
        return false;
    }

    return event->level == SYS_NET_IP_ONLY || event->level == SYS_NET_MQTT_READY;
}

void webapp_startup_on_network_event(const network_event_t *event)
{
    if (s_webapp_started || !network_event_has_connectivity(event)) {
        return;
    }

    esp_err_t start_result = webapp_startup_start();
    if (start_result != ESP_OK) {
        ESP_LOGW(TAG, "Config web app se nepodarilo spustit: %s", esp_err_to_name(start_result));
    }
}

esp_err_t webapp_startup_start(void)
{
    if (s_webapp_started) {
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
        ESP_LOGI(TAG, "Config web app spustena po pripojeni site");
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
        ESP_LOGI(TAG, "Config web app zastavena");
    } else {
        ESP_LOGW(TAG, "Zastaveni config web app selhalo: %s", esp_err_to_name(stop_result));
    }

    return stop_result;
}
