#include "app-config.h"

#include "nvs.h"
#include <cstring>

static const char *APP_CFG_NAMESPACE = "app_cfg";
static bool s_service_mode = false;

static const config_item_t APP_CORE_CONFIG_ITEMS[] = {
    {
        .key = "wifi_ssid",
        .label = "WiFi SSID",
        .description = "SSID site, ke ktere se ma zarizeni pripojit.",
        .type = CONFIG_VALUE_STRING,
        .default_string = "",
        .default_int = 0,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 31,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "wifi_pass",
        .label = "WiFi heslo",
        .description = "Heslo k WiFi. Kdyz je prazdne, spusti se konfiguracni AP.",
        .type = CONFIG_VALUE_STRING,
        .default_string = "",
        .default_int = 0,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 63,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "service_mode",
        .label = "Servisni rezim",
        .description = "Zapne servisni rezim systemu po restartu.",
        .type = CONFIG_VALUE_BOOL,
        .default_string = nullptr,
        .default_int = 0,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "interval_s",
        .label = "Interval měření [s]",
        .description = "Perioda měření a publikace hodnot.",
        .type = CONFIG_VALUE_INT32,
        .default_string = nullptr,
        .default_int = 30,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 5,
        .max_int = 3600,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "tepl_max",
        .label = "Max. teplota [°C]",
        .description = "Prahová teplota pro alarm nebo ochranu.",
        .type = CONFIG_VALUE_FLOAT,
        .default_string = nullptr,
        .default_int = 0,
        .default_float = 35.0f,
        .default_bool = false,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 0,
        .min_float = -20.0f,
        .max_float = 90.0f,
    },
    {
        .key = "auto_mode",
        .label = "Automatický režim",
        .description = "Zapíná automatické vyhodnocení závlahy.",
        .type = CONFIG_VALUE_BOOL,
        .default_string = nullptr,
        .default_int = 0,
        .default_float = 0.0f,
        .default_bool = true,
        .max_string_len = 0,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "mqtt_uri",
        .label = "MQTT URI",
        .description = "Adresa MQTT brokeru, napr. mqtt://mqtt:1883.",
        .type = CONFIG_VALUE_STRING,
        .default_string = "mqtt://mqtt:1883",
        .default_int = 0,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 127,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "mqtt_user",
        .label = "MQTT uzivatel",
        .description = "Uzivatelske jmeno pro pripojeni k MQTT brokeru.",
        .type = CONFIG_VALUE_STRING,
        .default_string = "",
        .default_int = 0,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 63,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
    {
        .key = "mqtt_pass",
        .label = "MQTT heslo",
        .description = "Heslo pro pripojeni k MQTT brokeru.",
        .type = CONFIG_VALUE_STRING,
        .default_string = "",
        .default_int = 0,
        .default_float = 0.0f,
        .default_bool = false,
        .max_string_len = 127,
        .min_int = 0,
        .max_int = 0,
        .min_float = 0.0f,
        .max_float = 0.0f,
    },
};

config_group_t app_config_get_config_group(void)
{
    config_group_t group = {
        .items = APP_CORE_CONFIG_ITEMS,
        .item_count = sizeof(APP_CORE_CONFIG_ITEMS) / sizeof(APP_CORE_CONFIG_ITEMS[0]),
    };
    return group;
}

esp_err_t app_config_ensure_defaults(void)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(APP_CFG_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        return result;
    }

    bool changed = false;
    for (size_t index = 0; index < sizeof(APP_CORE_CONFIG_ITEMS) / sizeof(APP_CORE_CONFIG_ITEMS[0]); ++index) {
        const config_item_t &item = APP_CORE_CONFIG_ITEMS[index];
        if (item.type == CONFIG_VALUE_STRING) {
            size_t required_size = 0;
            result = nvs_get_str(handle, item.key, nullptr, &required_size);
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                result = nvs_set_str(handle, item.key, item.default_string != nullptr ? item.default_string : "");
                if (result != ESP_OK) {
                    nvs_close(handle);
                    return result;
                }
                changed = true;
            } else if (result != ESP_OK) {
                nvs_close(handle);
                return result;
            }
        } else if (item.type == CONFIG_VALUE_INT32) {
            int32_t value = 0;
            result = nvs_get_i32(handle, item.key, &value);
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                result = nvs_set_i32(handle, item.key, item.default_int);
                if (result != ESP_OK) {
                    nvs_close(handle);
                    return result;
                }
                changed = true;
            } else if (result != ESP_OK) {
                nvs_close(handle);
                return result;
            }
        } else if (item.type == CONFIG_VALUE_FLOAT) {
            float value = 0.0f;
            size_t size = sizeof(value);
            result = nvs_get_blob(handle, item.key, &value, &size);
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                result = nvs_set_blob(handle, item.key, &item.default_float, sizeof(item.default_float));
                if (result != ESP_OK) {
                    nvs_close(handle);
                    return result;
                }
                changed = true;
            } else if (result != ESP_OK) {
                nvs_close(handle);
                return result;
            }
        } else if (item.type == CONFIG_VALUE_BOOL) {
            uint8_t value = 0;
            result = nvs_get_u8(handle, item.key, &value);
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                result = nvs_set_u8(handle, item.key, item.default_bool ? 1 : 0);
                if (result != ESP_OK) {
                    nvs_close(handle);
                    return result;
                }
                changed = true;
            } else if (result != ESP_OK) {
                nvs_close(handle);
                return result;
            }
        }
    }

    if (changed) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    return result;
}

esp_err_t app_config_load_wifi_credentials(char *ssid, size_t ssid_len, char *password, size_t password_len)
{
    if (ssid == nullptr || password == nullptr || ssid_len == 0 || password_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t result = nvs_open(APP_CFG_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK) {
        return result;
    }

    size_t ssid_required = ssid_len;
    result = nvs_get_str(handle, "wifi_ssid", ssid, &ssid_required);
    if (result != ESP_OK) {
        nvs_close(handle);
        return result;
    }

    size_t pass_required = password_len;
    result = nvs_get_str(handle, "wifi_pass", password, &pass_required);
    nvs_close(handle);
    return result;
}

esp_err_t app_config_load_runtime_flags(void)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(APP_CFG_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK) {
        return result;
    }

    uint8_t service_mode = 0;
    result = nvs_get_u8(handle, "service_mode", &service_mode);
    nvs_close(handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        s_service_mode = false;
        return ESP_OK;
    }
    if (result != ESP_OK) {
        return result;
    }

    s_service_mode = (service_mode != 0);
    return ESP_OK;
}

esp_err_t app_config_load_mqtt_uri(char *uri, size_t uri_len)
{
    if (uri == nullptr || uri_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t result = nvs_open(APP_CFG_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK) {
        return result;
    }

    size_t required = uri_len;
    result = nvs_get_str(handle, "mqtt_uri", uri, &required);
    nvs_close(handle);
    return result;
}

esp_err_t app_config_load_mqtt_credentials(char *username, size_t username_len, char *password, size_t password_len)
{
    if (username == nullptr || password == nullptr || username_len == 0 || password_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t result = nvs_open(APP_CFG_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK) {
        return result;
    }

    size_t user_required = username_len;
    result = nvs_get_str(handle, "mqtt_user", username, &user_required);
    if (result != ESP_OK) {
        nvs_close(handle);
        return result;
    }

    size_t pass_required = password_len;
    result = nvs_get_str(handle, "mqtt_pass", password, &pass_required);
    nvs_close(handle);
    return result;
}

bool app_config_is_service_mode(void)
{
    return s_service_mode;
}
