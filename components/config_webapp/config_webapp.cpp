#include "config_webapp.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "config_webapp";

typedef struct {
    const config_item_t *items;
    size_t item_count;
    char nvs_namespace[16];
    httpd_handle_t server;
} config_webapp_ctx_t;

static config_webapp_ctx_t s_ctx = {};

static bool is_ctx_ready()
{
    return s_ctx.items != nullptr && s_ctx.item_count > 0 && s_ctx.nvs_namespace[0] != '\0';
}

static const config_item_t *find_item(const char *key)
{
    if (!is_ctx_ready() || key == nullptr) {
        return nullptr;
    }

    for (size_t index = 0; index < s_ctx.item_count; ++index) {
        const config_item_t &item = s_ctx.items[index];
        if (strcmp(item.key, key) == 0) {
            return &item;
        }
    }
    return nullptr;
}

static esp_err_t open_nvs(nvs_open_mode_t mode, nvs_handle_t *out_handle)
{
    if (!is_ctx_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    return nvs_open(s_ctx.nvs_namespace, mode, out_handle);
}

static std::string html_escape(const std::string &input)
{
    std::string output;
    output.reserve(input.size());
    for (char ch : input) {
        switch (ch) {
            case '&': output += "&amp;"; break;
            case '<': output += "&lt;"; break;
            case '>': output += "&gt;"; break;
            case '"': output += "&quot;"; break;
            case '\'': output += "&#39;"; break;
            default: output.push_back(ch); break;
        }
    }
    return output;
}

static int hex_to_int(char ch)
{
    if (ch >= '0' && ch <= '9') {
        return ch - '0';
    }
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (ch >= 'a' && ch <= 'f') {
        return 10 + (ch - 'a');
    }
    return -1;
}

static std::string url_decode(const std::string &value)
{
    std::string out;
    out.reserve(value.size());
    for (size_t index = 0; index < value.size(); ++index) {
        if (value[index] == '+') {
            out.push_back(' ');
            continue;
        }
        if (value[index] == '%' && (index + 2) < value.size()) {
            int high = hex_to_int(value[index + 1]);
            int low = hex_to_int(value[index + 2]);
            if (high >= 0 && low >= 0) {
                out.push_back(static_cast<char>((high << 4) | low));
                index += 2;
                continue;
            }
        }
        out.push_back(value[index]);
    }
    return out;
}

static std::map<std::string, std::string> parse_form_encoded(const std::string &body)
{
    std::map<std::string, std::string> out;
    size_t start = 0;
    while (start < body.size()) {
        size_t end = body.find('&', start);
        if (end == std::string::npos) {
            end = body.size();
        }

        std::string pair = body.substr(start, end - start);
        size_t eq = pair.find('=');
        if (eq != std::string::npos) {
            std::string key = url_decode(pair.substr(0, eq));
            std::string value = url_decode(pair.substr(eq + 1));
            out[key] = value;
        } else {
            out[url_decode(pair)] = "";
        }

        start = end + 1;
    }
    return out;
}

static esp_err_t nvs_set_float(nvs_handle_t handle, const char *key, float value)
{
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

static esp_err_t nvs_get_float(nvs_handle_t handle, const char *key, float *value)
{
    size_t size = sizeof(*value);
    return nvs_get_blob(handle, key, value, &size);
}

static esp_err_t set_default_value_if_missing(nvs_handle_t nvs_handle, const config_item_t &item, bool *inserted)
{
    *inserted = false;
    esp_err_t result = ESP_OK;

    switch (item.type) {
        case CONFIG_VALUE_STRING: {
            size_t required_size = 0;
            result = nvs_get_str(nvs_handle, item.key, nullptr, &required_size);
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                const char *default_value = item.default_string != nullptr ? item.default_string : "";
                *inserted = true;
                return nvs_set_str(nvs_handle, item.key, default_value);
            }
            return result;
        }
        case CONFIG_VALUE_INT32: {
            int32_t value = 0;
            result = nvs_get_i32(nvs_handle, item.key, &value);
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                *inserted = true;
                return nvs_set_i32(nvs_handle, item.key, item.default_int);
            }
            return result;
        }
        case CONFIG_VALUE_FLOAT: {
            float value = 0;
            result = nvs_get_float(nvs_handle, item.key, &value);
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                *inserted = true;
                return nvs_set_float(nvs_handle, item.key, item.default_float);
            }
            return result;
        }
        case CONFIG_VALUE_BOOL: {
            uint8_t value = 0;
            result = nvs_get_u8(nvs_handle, item.key, &value);
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                *inserted = true;
                return nvs_set_u8(nvs_handle, item.key, item.default_bool ? 1 : 0);
            }
            return result;
        }
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t ensure_defaults_in_nvs()
{
    nvs_handle_t nvs_handle;
    esp_err_t result = nvs_open(s_ctx.nvs_namespace, NVS_READWRITE, &nvs_handle);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Nelze otevrit NVS namespace '%s': %s", s_ctx.nvs_namespace, esp_err_to_name(result));
        return result;
    }

    bool changed = false;
    for (size_t index = 0; index < s_ctx.item_count; ++index) {
        const config_item_t &item = s_ctx.items[index];
        bool inserted = false;
        result = set_default_value_if_missing(nvs_handle, item, &inserted);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Chyba pri praci s klicem '%s': %s", item.key, esp_err_to_name(result));
            nvs_close(nvs_handle);
            return result;
        }
        if (inserted) {
            changed = true;
        }
    }

    if (changed) {
        result = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return result;
}

static std::string read_value_for_html(nvs_handle_t nvs_handle, const config_item_t &item)
{
    switch (item.type) {
        case CONFIG_VALUE_STRING: {
            size_t required_size = 0;
            esp_err_t result = nvs_get_str(nvs_handle, item.key, nullptr, &required_size);
            if (result != ESP_OK || required_size == 0) {
                return item.default_string != nullptr ? item.default_string : "";
            }
            std::vector<char> buffer(required_size);
            result = nvs_get_str(nvs_handle, item.key, buffer.data(), &required_size);
            if (result != ESP_OK) {
                return item.default_string != nullptr ? item.default_string : "";
            }
            return buffer.data();
        }
        case CONFIG_VALUE_INT32: {
            int32_t value = item.default_int;
            nvs_get_i32(nvs_handle, item.key, &value);
            return std::to_string(value);
        }
        case CONFIG_VALUE_FLOAT: {
            float value = item.default_float;
            nvs_get_float(nvs_handle, item.key, &value);
            char out[32] = {0};
            snprintf(out, sizeof(out), "%.3f", value);
            return out;
        }
        case CONFIG_VALUE_BOOL: {
            uint8_t value = item.default_bool ? 1 : 0;
            nvs_get_u8(nvs_handle, item.key, &value);
            return value ? "1" : "0";
        }
        default:
            return "";
    }
}

static std::string build_config_page_html()
{
    nvs_handle_t nvs_handle;
    esp_err_t result = nvs_open(s_ctx.nvs_namespace, NVS_READONLY, &nvs_handle);

    std::string html;
    html.reserve(4096);
    html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>Konfigurace</title>";
    html += "<style>body{font-family:sans-serif;max-width:760px;margin:20px auto;padding:0 12px;}";
    html += "label{font-weight:600;display:block;margin-bottom:4px;}";
    html += "small{display:block;color:#666;margin-top:4px;}";
    html += "input[type=text],input[type=number]{width:100%;padding:8px;box-sizing:border-box;}";
    html += ".item{border:1px solid #ddd;border-radius:8px;padding:12px;margin-bottom:12px;}";
    html += ".actions{display:flex;gap:8px;flex-wrap:wrap;}";
    html += "button{padding:10px 14px;border:0;border-radius:8px;cursor:pointer;}";
    html += "</style></head><body>";
    html += "<h1>Konfigurace zařízení</h1>";
    html += "<form id='cfgForm' method='post' action='/config/save'>";

    for (size_t index = 0; index < s_ctx.item_count; ++index) {
        const config_item_t &item = s_ctx.items[index];
        std::string current_value = (result == ESP_OK) ? read_value_for_html(nvs_handle, item) : "";

        html += "<div class='item'>";
        html += "<label for='" + html_escape(item.key) + "'>" + html_escape(item.label != nullptr ? item.label : item.key) + "</label>";

        if (item.type == CONFIG_VALUE_STRING) {
            std::string default_value = item.default_string != nullptr ? item.default_string : "";
            html += "<input type='text' id='" + html_escape(item.key) + "' name='" + html_escape(item.key) + "' value='" + html_escape(current_value) + "'";
            html += " data-default-type='string' data-default='" + html_escape(default_value) + "'";
            if (item.max_string_len > 0) {
                html += " maxlength='" + std::to_string(item.max_string_len) + "'";
            }
            html += ">";
        } else if (item.type == CONFIG_VALUE_INT32) {
            std::string default_value = std::to_string(item.default_int);
            html += "<input type='number' step='1' id='" + html_escape(item.key) + "' name='" + html_escape(item.key) + "' value='" + html_escape(current_value) + "'";
            html += " data-default-type='int' data-default='" + html_escape(default_value) + "'";
            html += " min='" + std::to_string(item.min_int) + "' max='" + std::to_string(item.max_int) + "'>";
        } else if (item.type == CONFIG_VALUE_FLOAT) {
            char default_float_buffer[32] = {0};
            snprintf(default_float_buffer, sizeof(default_float_buffer), "%.3f", item.default_float);
            html += "<input type='number' step='any' id='" + html_escape(item.key) + "' name='" + html_escape(item.key) + "' value='" + html_escape(current_value) + "'";
            html += " data-default-type='float' data-default='" + html_escape(default_float_buffer) + "'";
            html += " min='" + std::to_string(item.min_float) + "' max='" + std::to_string(item.max_float) + "'>";
        } else if (item.type == CONFIG_VALUE_BOOL) {
            bool checked = (current_value == "1");
            html += "<input type='checkbox' id='" + html_escape(item.key) + "' name='" + html_escape(item.key) + "'";
            html += " data-default-type='bool' data-default='" + std::string(item.default_bool ? "1" : "0") + "'";
            if (checked) {
                html += " checked";
            }
            html += ">";
        }

        if (item.description != nullptr && item.description[0] != '\0') {
            html += "<small>" + html_escape(item.description) + "</small>";
        }
        html += "</div>";
    }

    html += "<div class='actions'>";
    html += "<button type='submit'>Uložit</button>";
    html += "<button type='button' onclick='window.location.href=\"/config\"'>Obnovit</button>";
    html += "<button type='button' onclick='loadFactoryDefaults()'>Načíst tovární nastavení</button>";
    html += "</div></form>";
    html += "<script>function loadFactoryDefaults(){";
    html += "var fields=document.querySelectorAll('[data-default-type]');";
    html += "for(var i=0;i<fields.length;i++){var el=fields[i];var t=el.getAttribute('data-default-type');var d=el.getAttribute('data-default')||'';";
    html += "if(t==='bool'){el.checked=(d==='1');}else{el.value=d;}}}";
    html += "</script></body></html>";
    if (result == ESP_OK) {
        nvs_close(nvs_handle);
    }
    return html;
}

static esp_err_t config_get_handler(httpd_req_t *req)
{
    UBaseType_t stack_words = uxTaskGetStackHighWaterMark(nullptr);
    if (stack_words < 256) {
        ESP_LOGW(TAG, "Nizka rezerva stacku v GET handleru: %u words", static_cast<unsigned>(stack_words));
    }

    std::string html = build_config_page_html();
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html.c_str(), static_cast<ssize_t>(html.size()));
}

static bool parse_bool_value(const std::string &value)
{
    if (value == "1" || value == "on" || value == "true" || value == "TRUE") {
        return true;
    }
    return false;
}

static esp_err_t save_form_to_nvs(const std::map<std::string, std::string> &params)
{
    nvs_handle_t nvs_handle;
    esp_err_t result = nvs_open(s_ctx.nvs_namespace, NVS_READWRITE, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    for (size_t index = 0; index < s_ctx.item_count; ++index) {
        const config_item_t &item = s_ctx.items[index];
        auto found = params.find(item.key);

        if (item.type == CONFIG_VALUE_BOOL) {
            bool bool_value = (found != params.end()) ? parse_bool_value(found->second) : false;
            result = nvs_set_u8(nvs_handle, item.key, bool_value ? 1 : 0);
            if (result != ESP_OK) {
                nvs_close(nvs_handle);
                return result;
            }
            continue;
        }

        if (found == params.end()) {
            continue;
        }

        const std::string &value = found->second;
        if (item.type == CONFIG_VALUE_STRING) {
            std::string trimmed = value;
            if (item.max_string_len > 0 && trimmed.size() > item.max_string_len) {
                trimmed = trimmed.substr(0, item.max_string_len);
            }
            result = nvs_set_str(nvs_handle, item.key, trimmed.c_str());
            if (result != ESP_OK) {
                nvs_close(nvs_handle);
                return result;
            }
        } else if (item.type == CONFIG_VALUE_INT32) {
            char *end_ptr = nullptr;
            long parsed = strtol(value.c_str(), &end_ptr, 10);
            if (end_ptr == nullptr || *end_ptr != '\0') {
                nvs_close(nvs_handle);
                return ESP_ERR_INVALID_ARG;
            }
            int32_t int_value = static_cast<int32_t>(parsed);
            int_value = std::max(item.min_int, std::min(item.max_int, int_value));
            result = nvs_set_i32(nvs_handle, item.key, int_value);
            if (result != ESP_OK) {
                nvs_close(nvs_handle);
                return result;
            }
        } else if (item.type == CONFIG_VALUE_FLOAT) {
            char *end_ptr = nullptr;
            float parsed = strtof(value.c_str(), &end_ptr);
            if (end_ptr == nullptr || *end_ptr != '\0') {
                nvs_close(nvs_handle);
                return ESP_ERR_INVALID_ARG;
            }
            float float_value = std::max(item.min_float, std::min(item.max_float, parsed));
            result = nvs_set_float(nvs_handle, item.key, float_value);
            if (result != ESP_OK) {
                nvs_close(nvs_handle);
                return result;
            }
        }
    }

    result = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return result;
}

static esp_err_t config_save_handler(httpd_req_t *req)
{
    UBaseType_t stack_words = uxTaskGetStackHighWaterMark(nullptr);
    if (stack_words < 256) {
        ESP_LOGW(TAG, "Nizka rezerva stacku v POST handleru: %u words", static_cast<unsigned>(stack_words));
    }

    if (req->content_len <= 0 || req->content_len > 8192) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Neplatna data");
    }

    std::vector<char> body(req->content_len + 1, 0);
    int total_read = 0;
    while (total_read < req->content_len) {
        int bytes = httpd_req_recv(req, body.data() + total_read, req->content_len - total_read);
        if (bytes <= 0) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cteni pozadavku selhalo");
        }
        total_read += bytes;
    }

    std::string body_string(body.data(), total_read);
    std::map<std::string, std::string> params = parse_form_encoded(body_string);
    esp_err_t result = save_form_to_nvs(params);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Ulozeni konfigurace selhalo: %s", esp_err_to_name(result));
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Ulozeni konfigurace selhalo");
    }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/config");
    return httpd_resp_send(req, "", HTTPD_RESP_USE_STRLEN);
}

esp_err_t config_webapp_start(const char *nvs_namespace,
                              const config_item_t *items,
                              size_t item_count,
                              uint16_t http_port)
{
    if (nvs_namespace == nullptr || items == nullptr || item_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(nvs_namespace) > 15) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.server != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t index = 0; index < item_count; ++index) {
        if (items[index].key == nullptr || strlen(items[index].key) == 0 || strlen(items[index].key) > 15) {
            ESP_LOGE(TAG, "Neplatny klic konfigurace na indexu %u", static_cast<unsigned>(index));
            return ESP_ERR_INVALID_ARG;
        }
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.items = items;
    s_ctx.item_count = item_count;
    strncpy(s_ctx.nvs_namespace, nvs_namespace, sizeof(s_ctx.nvs_namespace) - 1);

    esp_err_t result = ensure_defaults_in_nvs();
    if (result != ESP_OK) {
        return result;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = http_port;
    config.max_uri_handlers = 8;
    config.stack_size = 10240;

    result = httpd_start(&s_ctx.server, &config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server nelze spustit: %s", esp_err_to_name(result));
        return result;
    }

    httpd_uri_t config_get_uri = {
        .uri = "/config",
        .method = HTTP_GET,
        .handler = config_get_handler,
        .user_ctx = nullptr,
    };

    httpd_uri_t config_save_uri = {
        .uri = "/config/save",
        .method = HTTP_POST,
        .handler = config_save_handler,
        .user_ctx = nullptr,
    };

    result = httpd_register_uri_handler(s_ctx.server, &config_get_uri);
    if (result != ESP_OK) {
        httpd_stop(s_ctx.server);
        s_ctx.server = nullptr;
        return result;
    }

    result = httpd_register_uri_handler(s_ctx.server, &config_save_uri);
    if (result != ESP_OK) {
        httpd_stop(s_ctx.server);
        s_ctx.server = nullptr;
        return result;
    }

    ESP_LOGI(TAG, "Config web app bezi na /config");
    return ESP_OK;
}

esp_err_t config_webapp_get_i32(const char *key, int32_t *value)
{
    if (key == nullptr || value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const config_item_t *item = find_item(key);
    if (item == nullptr || item->type != CONFIG_VALUE_INT32) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READONLY, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    result = nvs_get_i32(nvs_handle, key, value);
    nvs_close(nvs_handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        *value = item->default_int;
        return ESP_OK;
    }
    return result;
}

esp_err_t config_webapp_get_float(const char *key, float *value)
{
    if (key == nullptr || value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const config_item_t *item = find_item(key);
    if (item == nullptr || item->type != CONFIG_VALUE_FLOAT) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READONLY, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    result = nvs_get_float(nvs_handle, key, value);
    nvs_close(nvs_handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        *value = item->default_float;
        return ESP_OK;
    }
    return result;
}

esp_err_t config_webapp_get_bool(const char *key, bool *value)
{
    if (key == nullptr || value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const config_item_t *item = find_item(key);
    if (item == nullptr || item->type != CONFIG_VALUE_BOOL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READONLY, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    uint8_t raw = 0;
    result = nvs_get_u8(nvs_handle, key, &raw);
    nvs_close(nvs_handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        *value = item->default_bool;
        return ESP_OK;
    }
    if (result != ESP_OK) {
        return result;
    }
    *value = (raw != 0);
    return ESP_OK;
}

esp_err_t config_webapp_get_string(const char *key, char *buffer, size_t buffer_len)
{
    if (key == nullptr || buffer == nullptr || buffer_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const config_item_t *item = find_item(key);
    if (item == nullptr || item->type != CONFIG_VALUE_STRING) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READONLY, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    size_t required_size = buffer_len;
    result = nvs_get_str(nvs_handle, key, buffer, &required_size);
    nvs_close(nvs_handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        const char *fallback = item->default_string != nullptr ? item->default_string : "";
        if (strlen(fallback) + 1 > buffer_len) {
            return ESP_ERR_NVS_INVALID_LENGTH;
        }
        strcpy(buffer, fallback);
        return ESP_OK;
    }
    return result;
}

esp_err_t config_webapp_set_i32(const char *key, int32_t value)
{
    if (key == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const config_item_t *item = find_item(key);
    if (item == nullptr || item->type != CONFIG_VALUE_INT32) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t clamped = std::max(item->min_int, std::min(item->max_int, value));

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    result = nvs_set_i32(nvs_handle, key, clamped);
    if (result == ESP_OK) {
        result = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return result;
}

esp_err_t config_webapp_set_float(const char *key, float value)
{
    if (key == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const config_item_t *item = find_item(key);
    if (item == nullptr || item->type != CONFIG_VALUE_FLOAT) {
        return ESP_ERR_INVALID_ARG;
    }

    float clamped = std::max(item->min_float, std::min(item->max_float, value));

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    result = nvs_set_float(nvs_handle, key, clamped);
    if (result == ESP_OK) {
        result = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return result;
}

esp_err_t config_webapp_set_bool(const char *key, bool value)
{
    if (key == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const config_item_t *item = find_item(key);
    if (item == nullptr || item->type != CONFIG_VALUE_BOOL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    result = nvs_set_u8(nvs_handle, key, value ? 1 : 0);
    if (result == ESP_OK) {
        result = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return result;
}

esp_err_t config_webapp_set_string(const char *key, const char *value)
{
    if (key == nullptr || value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const config_item_t *item = find_item(key);
    if (item == nullptr || item->type != CONFIG_VALUE_STRING) {
        return ESP_ERR_INVALID_ARG;
    }

    std::string normalized = value;
    if (item->max_string_len > 0 && normalized.size() > item->max_string_len) {
        normalized = normalized.substr(0, item->max_string_len);
    }

    nvs_handle_t nvs_handle;
    esp_err_t result = open_nvs(NVS_READWRITE, &nvs_handle);
    if (result != ESP_OK) {
        return result;
    }

    result = nvs_set_str(nvs_handle, key, normalized.c_str());
    if (result == ESP_OK) {
        result = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return result;
}
