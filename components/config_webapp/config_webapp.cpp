#include "config_webapp.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config_store.h"

static const char *TAG = "config_webapp";
static bool s_has_restart_info = false;
static config_webapp_restart_info_t s_restart_info = {
    .boot_count = 0,
    .last_reason = ESP_RST_UNKNOWN,
    .last_restart_unix = 0,
};
static bool s_has_network_info = false;
static config_webapp_network_info_t s_network_info = {
    .is_ap_mode = false,
    .active_ssid = nullptr,
};
static std::string s_network_ssid_storage;

typedef struct {
    httpd_handle_t server;
} config_webapp_ctx_t;

static config_webapp_ctx_t s_ctx = {};

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

static const char *reset_reason_to_str(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_POWERON: return "Power-on";
        case ESP_RST_EXT: return "External pin";
        case ESP_RST_SW: return "Software restart";
        case ESP_RST_PANIC: return "Kernel panic";
        case ESP_RST_INT_WDT: return "Interrupt watchdog";
        case ESP_RST_TASK_WDT: return "Task watchdog";
        case ESP_RST_WDT: return "Other watchdog";
        case ESP_RST_DEEPSLEEP: return "Wake from deep sleep";
        case ESP_RST_BROWNOUT: return "Brownout";
        case ESP_RST_SDIO: return "SDIO reset";
        case ESP_RST_USB: return "USB reset";
        case ESP_RST_JTAG: return "JTAG reset";
        case ESP_RST_EFUSE: return "eFuse error reset";
        case ESP_RST_PWR_GLITCH: return "Power glitch";
        case ESP_RST_CPU_LOCKUP: return "CPU lockup";
        default: return "Unknown";
    }
}

static std::string format_unix_time(int64_t unix_time)
{
    if (unix_time <= 0) {
        return "neznamy";
    }

    time_t t = static_cast<time_t>(unix_time);
    struct tm tm_info = {};
    if (localtime_r(&t, &tm_info) == nullptr) {
        return "neznamy";
    }

    char buffer[32] = {0};
    if (strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_info) == 0) {
        return "neznamy";
    }
    return buffer;
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

static std::string read_value_for_html(const config_item_t &item)
{
    switch (item.type) {
        case CONFIG_VALUE_STRING: {
            char buffer[256] = {0};
            config_store_get_string_item(&item, buffer, sizeof(buffer));
            return buffer;
        }
        case CONFIG_VALUE_INT32: {
            int32_t value = config_store_get_i32_item(&item);
            return std::to_string(value);
        }
        case CONFIG_VALUE_FLOAT: {
            float value = config_store_get_float_item(&item);
            char out[32] = {0};
            snprintf(out, sizeof(out), "%.3f", value);
            return out;
        }
        case CONFIG_VALUE_BOOL: {
            bool value = config_store_get_bool_item(&item);
            return value ? "1" : "0";
        }
        default:
            return "";
    }
}

static std::string build_config_page_html()
{
    const esp_app_desc_t *app_desc = esp_app_get_description();
    const std::string project_name = (app_desc != nullptr && app_desc->project_name[0] != '\0') ? app_desc->project_name : "projekt";

    std::string html;
    html.reserve(4096);
    html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>" + html_escape(project_name) + " - Konfigurace</title>";
    html += "<style>body{font-family:sans-serif;max-width:760px;margin:20px auto;padding:0 12px;}";
    html += "label{font-weight:600;display:block;margin-bottom:4px;}";
    html += "small{display:block;color:#666;margin-top:4px;}";
    html += "input[type=text],input[type=number]{width:100%;padding:8px;box-sizing:border-box;}";
    html += ".item{border:1px solid #ddd;border-radius:8px;padding:12px;margin-bottom:12px;}";
    html += ".actions{display:flex;gap:8px;flex-wrap:wrap;}";
    html += "button{padding:10px 14px;border:0;border-radius:8px;cursor:pointer;}";
    html += "</style></head><body>";
    html += "<h1>Konfigurace zařízení - " + html_escape(project_name) + "</h1>";
    html += "<p><a href='/'>← Zpět na systémový přehled</a></p>";
    html += "<form id='cfgForm' method='post' action='/config/save'>";

    const size_t item_count = config_store_item_count();
    for (size_t index = 0; index < item_count; ++index) {
        const config_item_t *item_ptr = config_store_item_at(index);
        if (item_ptr == nullptr) {
            continue;
        }
        const config_item_t &item = *item_ptr;
        std::string current_value = read_value_for_html(item);

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
    return html;
}

static std::string build_root_page_html()
{
    esp_chip_info_t chip_info = {};
    esp_chip_info(&chip_info);

    const esp_app_desc_t *app_desc = esp_app_get_description();
    const std::string project_name = (app_desc != nullptr && app_desc->project_name[0] != '\0') ? app_desc->project_name : "projekt";
    uint32_t uptime_seconds = static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);

    std::string html;
    html.reserve(3000);
    html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>" + html_escape(project_name) + " - Systémový přehled</title>";
    html += "<style>body{font-family:sans-serif;max-width:760px;margin:20px auto;padding:0 12px;}";
    html += ".card{border:1px solid #ddd;border-radius:8px;padding:12px;margin-bottom:12px;}";
    html += "h1,h2{margin-top:0;}";
    html += "ul{padding-left:18px;margin:0;}";
    html += "li{margin-bottom:6px;}";
    html += "a.button{display:inline-block;padding:10px 14px;border-radius:8px;border:1px solid #333;text-decoration:none;color:#111;}";
    html += "</style></head><body>";
    html += "<h1>Systémový přehled - " + html_escape(project_name) + "</h1>";

    if (s_has_network_info) {
        html += "<div class='card'><h2>Síťový režim</h2><ul>";
        html += "<li>Aktivní režim: <strong>" + std::string(s_network_info.is_ap_mode ? "AP (konfigurační hotspot)" : "STA (klient)") + "</strong></li>";
        if (!s_network_ssid_storage.empty()) {
            html += "<li>SSID: <strong>" + html_escape(s_network_ssid_storage) + "</strong></li>";
        }
        html += "</ul></div>";
    }

    if (s_has_restart_info) {
        html += "<div class='card'><h2>Restarty</h2><ul>";
        html += "<li>Počet restartů: <strong>" + std::to_string(s_restart_info.boot_count) + "</strong></li>";
        html += "<li>Důvod posledního restartu: <strong>" + std::string(reset_reason_to_str(static_cast<esp_reset_reason_t>(s_restart_info.last_reason))) + "</strong></li>";
        html += "<li>Čas posledního restartu: <strong>" + format_unix_time(s_restart_info.last_restart_unix) + "</strong></li>";
        html += "</ul></div>";
    }

    html += "<div class='card'><h2>Systémové informace</h2><ul>";
    html += "<li>Projekt: <strong>" + html_escape(project_name) + "</strong></li>";
    html += "<li>Verze aplikace: <strong>" + std::string(app_desc->version) + "</strong></li>";
    html += "<li>ESP-IDF: <strong>" + std::string(esp_get_idf_version()) + "</strong></li>";
    html += "<li>Chip model: <strong>ESP32</strong></li>";
    html += "<li>Jádra CPU: <strong>" + std::to_string(chip_info.cores) + "</strong></li>";
    html += "<li>Revize čipu: <strong>" + std::to_string(chip_info.revision) + "</strong></li>";
    html += "<li>Volná heap: <strong>" + std::to_string(static_cast<unsigned long>(esp_get_free_heap_size())) + " B</strong></li>";
    html += "<li>Minimum heap: <strong>" + std::to_string(static_cast<unsigned long>(esp_get_minimum_free_heap_size())) + " B</strong></li>";
    html += "<li>Uptime: <strong>" + std::to_string(uptime_seconds) + " s</strong></li>";
    html += "</ul></div>";

    html += "<p><a class='button' href='/config'>Otevřít konfiguraci</a></p>";
    html += "</body></html>";
    return html;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    std::string html = build_root_page_html();
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html.c_str(), static_cast<ssize_t>(html.size()));
}

static esp_err_t captive_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    const char *html =
        "<!doctype html><html><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='0; url=http://192.168.4.1/'>"
        "<title>Captive portal</title>"
        "</head><body>"
        "<script>window.location.replace('http://192.168.4.1/');</script>"
        "<a href='http://192.168.4.1/'>Otevrit konfiguraci</a>"
        "</body></html>";
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t captive_head_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    return httpd_resp_send(req, nullptr, 0);
}

static esp_err_t captive_windows_ncsi_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, "NCSI captive portal", HTTPD_RESP_USE_STRLEN);
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
    for (size_t index = 0; index < config_store_item_count(); ++index) {
        const config_item_t *item_ptr = config_store_item_at(index);
        if (item_ptr == nullptr) {
            continue;
        }
        const config_item_t &item = *item_ptr;
        auto found = params.find(item.key);
        esp_err_t result = ESP_OK;

        if (item.type == CONFIG_VALUE_BOOL) {
            bool bool_value = (found != params.end()) ? parse_bool_value(found->second) : false;
            result = config_store_set_bool_item(&item, bool_value);
            if (result != ESP_OK) {
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
            result = config_store_set_string_item(&item, trimmed.c_str());
            if (result != ESP_OK) {
                return result;
            }
        } else if (item.type == CONFIG_VALUE_INT32) {
            char *end_ptr = nullptr;
            long parsed = strtol(value.c_str(), &end_ptr, 10);
            if (end_ptr == nullptr || *end_ptr != '\0') {
                return ESP_ERR_INVALID_ARG;
            }
            int32_t int_value = static_cast<int32_t>(parsed);
            int_value = std::max(item.min_int, std::min(item.max_int, int_value));
            result = config_store_set_i32_item(&item, int_value);
            if (result != ESP_OK) {
                return result;
            }
        } else if (item.type == CONFIG_VALUE_FLOAT) {
            char *end_ptr = nullptr;
            float parsed = strtof(value.c_str(), &end_ptr);
            if (end_ptr == nullptr || *end_ptr != '\0') {
                return ESP_ERR_INVALID_ARG;
            }
            float float_value = std::max(item.min_float, std::min(item.max_float, parsed));
            result = config_store_set_float_item(&item, float_value);
            if (result != ESP_OK) {
                return result;
            }
        }
    }

    return ESP_OK;
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

    auto restart_task = [](void *arg) {
        vTaskDelay(pdMS_TO_TICKS(250));
        ESP_LOGI(TAG, "Restartuji zarizeni po ulozeni konfigurace");
        esp_restart();
        vTaskDelete(nullptr);
    };

    xTaskCreate(restart_task, "cfg_restart", 2048, nullptr, 5, nullptr);

    const char *html =
        "<!doctype html><html><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Uloženo</title>"
        "<style>body{font-family:sans-serif;max-width:640px;margin:24px auto;padding:0 12px;}</style>"
        "</head><body>"
        "<h1>Konfigurace uložena</h1>"
        "<p>Zařízení se restartuje. Za chvíli proběhne nové načtení stránky konfigurace.</p>"
        "<p>Pokud by se stránka neobnovila sama, otevřete znovu <a href='/config'>/config</a>.</p>"
        "<script>setTimeout(function(){window.location.href='/config';},1200);</script>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

esp_err_t config_webapp_start(uint16_t http_port,
                              const config_webapp_restart_info_t *restart_info,
                              const config_webapp_network_info_t *network_info)
{
    if (s_ctx.server != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!config_store_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = ESP_OK;

    s_has_restart_info = (restart_info != nullptr);
    if (s_has_restart_info) {
        s_restart_info = *restart_info;
    } else {
        memset(&s_restart_info, 0, sizeof(s_restart_info));
    }

    s_has_network_info = (network_info != nullptr);
    s_network_ssid_storage.clear();
    if (s_has_network_info) {
        s_network_info = *network_info;
        if (network_info->active_ssid != nullptr) {
            s_network_ssid_storage = network_info->active_ssid;
            s_network_info.active_ssid = s_network_ssid_storage.c_str();
        }
    } else {
        memset(&s_network_info, 0, sizeof(s_network_info));
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = http_port;
    config.max_uri_handlers = 16;
    config.stack_size = 10240;
    config.uri_match_fn = httpd_uri_match_wildcard;

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

    httpd_uri_t root_get_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = nullptr,
    };

    httpd_uri_t captive_android_uri = {
        .uri = "/generate_204",
        .method = HTTP_GET,
        .handler = captive_redirect_handler,
        .user_ctx = nullptr,
    };

    httpd_uri_t captive_android_alt_uri = {
        .uri = "/gen_204",
        .method = HTTP_GET,
        .handler = captive_redirect_handler,
        .user_ctx = nullptr,
    };

    httpd_uri_t captive_apple_uri = {
        .uri = "/hotspot-detect.html",
        .method = HTTP_GET,
        .handler = captive_redirect_handler,
        .user_ctx = nullptr,
    };

    httpd_uri_t captive_windows_uri = {
        .uri = "/fwlink",
        .method = HTTP_GET,
        .handler = captive_redirect_handler,
        .user_ctx = nullptr,
    };

    httpd_uri_t captive_windows_txt_uri = {
        .uri = "/connecttest.txt",
        .method = HTTP_GET,
        .handler = captive_redirect_handler,
        .user_ctx = nullptr,
    };

    httpd_uri_t captive_windows_ncsi_uri = {
        .uri = "/ncsi.txt",
        .method = HTTP_GET,
        .handler = captive_windows_ncsi_handler,
        .user_ctx = nullptr,
    };

    httpd_uri_t captive_head_fallback_uri = {
        .uri = "/*",
        .method = HTTP_HEAD,
        .handler = captive_head_handler,
        .user_ctx = nullptr,
    };

    httpd_uri_t captive_fallback_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = captive_redirect_handler,
        .user_ctx = nullptr,
    };

    result = httpd_register_uri_handler(s_ctx.server, &root_get_uri);
    if (result != ESP_OK) {
        httpd_stop(s_ctx.server);
        s_ctx.server = nullptr;
        return result;
    }

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

    result = httpd_register_uri_handler(s_ctx.server, &captive_android_uri);
    if (result != ESP_OK) {
        httpd_stop(s_ctx.server);
        s_ctx.server = nullptr;
        return result;
    }

    result = httpd_register_uri_handler(s_ctx.server, &captive_android_alt_uri);
    if (result != ESP_OK) {
        httpd_stop(s_ctx.server);
        s_ctx.server = nullptr;
        return result;
    }

    result = httpd_register_uri_handler(s_ctx.server, &captive_apple_uri);
    if (result != ESP_OK) {
        httpd_stop(s_ctx.server);
        s_ctx.server = nullptr;
        return result;
    }

    result = httpd_register_uri_handler(s_ctx.server, &captive_windows_uri);
    if (result != ESP_OK) {
        httpd_stop(s_ctx.server);
        s_ctx.server = nullptr;
        return result;
    }

    result = httpd_register_uri_handler(s_ctx.server, &captive_windows_txt_uri);
    if (result != ESP_OK) {
        httpd_stop(s_ctx.server);
        s_ctx.server = nullptr;
        return result;
    }

    result = httpd_register_uri_handler(s_ctx.server, &captive_windows_ncsi_uri);
    if (result != ESP_OK) {
        httpd_stop(s_ctx.server);
        s_ctx.server = nullptr;
        return result;
    }

    result = httpd_register_uri_handler(s_ctx.server, &captive_fallback_uri);
    if (result != ESP_OK) {
        httpd_stop(s_ctx.server);
        s_ctx.server = nullptr;
        return result;
    }

    result = httpd_register_uri_handler(s_ctx.server, &captive_head_fallback_uri);
    if (result != ESP_OK) {
        httpd_stop(s_ctx.server);
        s_ctx.server = nullptr;
        return result;
    }

    ESP_LOGI(TAG, "Config web app bezi na /config (captive portal redirect na http://192.168.4.1/)");
    return ESP_OK;
}

esp_err_t config_webapp_stop(void)
{
    if (s_ctx.server == nullptr) {
        return ESP_OK;
    }

    httpd_handle_t server = s_ctx.server;
    s_ctx.server = nullptr;
    return httpd_stop(server);
}

esp_err_t config_webapp_prepare(const char *nvs_namespace)
{
    if (nvs_namespace == nullptr || nvs_namespace[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (!config_store_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t config_webapp_get_i32(const char *key, int32_t *value)
{
    if (key == nullptr || value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    *value = config_store_get_i32(key);
    return ESP_OK;
}

esp_err_t config_webapp_get_float(const char *key, float *value)
{
    if (key == nullptr || value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    *value = config_store_get_float(key);
    return ESP_OK;
}

esp_err_t config_webapp_get_bool(const char *key, bool *value)
{
    if (key == nullptr || value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    *value = config_store_get_bool(key);
    return ESP_OK;
}

esp_err_t config_webapp_get_string(const char *key, char *buffer, size_t buffer_len)
{
    if (key == nullptr || buffer == nullptr || buffer_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    config_store_get_string(key, buffer, buffer_len);
    return ESP_OK;
}

esp_err_t config_webapp_set_i32(const char *key, int32_t value)
{
    return config_store_set_i32(key, value);
}

esp_err_t config_webapp_set_float(const char *key, float value)
{
    return config_store_set_float(key, value);
}

esp_err_t config_webapp_set_bool(const char *key, bool value)
{
    return config_store_set_bool(key, value);
}

esp_err_t config_webapp_set_string(const char *key, const char *value)
{
    return config_store_set_string(key, value);
}
