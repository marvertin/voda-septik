#include "network_config.h"

#include "config_store.h"
#include "app_error_check.h"

static const config_item_t WIFI_SSID_ITEM = {
    .key = "wifi_ssid", .label = "WiFi SSID", .description = "SSID site, ke ktere se ma zarizeni pripojit.",
    .type = CONFIG_VALUE_STRING, .default_string = "", .default_int = 0, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 31, .min_int = 0, .max_int = 0, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t WIFI_PASS_ITEM = {
    .key = "wifi_pass", .label = "WiFi heslo", .description = "Heslo k WiFi. Kdyz je prazdne, spusti se konfiguracni AP.",
    .type = CONFIG_VALUE_STRING, .default_string = "", .default_int = 0, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 63, .min_int = 0, .max_int = 0, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t MQTT_URI_ITEM = {
    .key = "mqtt_uri", .label = "MQTT URI", .description = "Adresa MQTT brokeru, napr. mqtt://mqtt:1883.",
    .type = CONFIG_VALUE_STRING, .default_string = "mqtt://mqtt:1883", .default_int = 0, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 127, .min_int = 0, .max_int = 0, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t MQTT_USER_ITEM = {
    .key = "mqtt_user", .label = "MQTT uzivatel", .description = "Uzivatelske jmeno pro pripojeni k MQTT brokeru.",
    .type = CONFIG_VALUE_STRING, .default_string = "", .default_int = 0, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 63, .min_int = 0, .max_int = 0, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t MQTT_PASS_ITEM = {
    .key = "mqtt_pass", .label = "MQTT heslo", .description = "Heslo pro pripojeni k MQTT brokeru.",
    .type = CONFIG_VALUE_STRING, .default_string = "", .default_int = 0, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 127, .min_int = 0, .max_int = 0, .min_float = 0.0f, .max_float = 0.0f,
};

void network_config_register_config_items(void)
{
    APP_ERROR_CHECK("E690", config_store_register_item(&WIFI_SSID_ITEM));
    APP_ERROR_CHECK("E691", config_store_register_item(&WIFI_PASS_ITEM));
    APP_ERROR_CHECK("E692", config_store_register_item(&MQTT_URI_ITEM));
    APP_ERROR_CHECK("E693", config_store_register_item(&MQTT_USER_ITEM));
    APP_ERROR_CHECK("E694", config_store_register_item(&MQTT_PASS_ITEM));
}

esp_err_t network_config_load_wifi_credentials(char *ssid, size_t ssid_len, char *password, size_t password_len)
{
    if (ssid == nullptr || password == nullptr || ssid_len == 0 || password_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    config_store_get_string_item(&WIFI_SSID_ITEM, ssid, ssid_len);
    config_store_get_string_item(&WIFI_PASS_ITEM, password, password_len);
    return ESP_OK;
}

esp_err_t network_config_load_mqtt_uri(char *uri, size_t uri_len)
{
    if (uri == nullptr || uri_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    config_store_get_string_item(&MQTT_URI_ITEM, uri, uri_len);
    return ESP_OK;
}

esp_err_t network_config_load_mqtt_credentials(char *username, size_t username_len, char *password, size_t password_len)
{
    if (username == nullptr || password == nullptr || username_len == 0 || password_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    config_store_get_string_item(&MQTT_USER_ITEM, username, username_len);
    config_store_get_string_item(&MQTT_PASS_ITEM, password, password_len);
    return ESP_OK;
}
