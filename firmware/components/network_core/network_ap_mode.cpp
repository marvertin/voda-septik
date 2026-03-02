#include "network_ap_mode.h"

#include <cstring>
#include <errno.h>

#include <lwip/inet.h>
#include <lwip/sockets.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "network_ap_mode";
static TaskHandle_t s_captive_dns_task_handle = nullptr;

static size_t decode_dns_qname(const uint8_t *buffer, size_t buffer_len, size_t start, char *out, size_t out_len)
{
    if (buffer == nullptr || out == nullptr || out_len == 0 || start >= buffer_len) {
        return 0;
    }

    size_t read_pos = start;
    size_t out_pos = 0;

    while (read_pos < buffer_len) {
        uint8_t label_len = buffer[read_pos++];
        if (label_len == 0) {
            break;
        }

        if ((read_pos + label_len) > buffer_len) {
            return 0;
        }

        if (out_pos != 0 && out_pos < (out_len - 1)) {
            out[out_pos++] = '.';
        }

        for (size_t i = 0; i < label_len && out_pos < (out_len - 1); ++i) {
            out[out_pos++] = static_cast<char>(buffer[read_pos + i]);
        }
        read_pos += label_len;
    }

    out[out_pos] = '\0';
    return read_pos;
}

static void captive_dns_task(void *arg)
{
    (void)arg;

    static const uint8_t captive_ip[4] = {192, 168, 4, 1};

    while (true) {
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Captive DNS: nelze vytvorit socket: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int reuse = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in bind_addr = {};
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        bind_addr.sin_port = htons(53);

        if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
            ESP_LOGE(TAG, "Captive DNS: bind na port 53 selhal: errno=%d", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        ESP_LOGI(TAG, "Captive DNS server bezi na 0.0.0.0:53 -> 192.168.4.1 (cekam dotazy)");

        while (true) {
            uint8_t request[512] = {0};
            sockaddr_in source_addr = {};
            socklen_t source_len = sizeof(source_addr);
            int len = recvfrom(sock,
                               request,
                               sizeof(request),
                               0,
                               (struct sockaddr *)&source_addr,
                               &source_len);
            if (len <= 0) {
                ESP_LOGW(TAG, "Captive DNS: recvfrom selhal/ukoncen, len=%d errno=%d", len, errno);
                break;
            }
            if (len < 12) {
                ESP_LOGW(TAG, "Captive DNS: prijat kratky paket (%d B)", len);
                continue;
            }

            uint16_t qdcount = (static_cast<uint16_t>(request[4]) << 8) | request[5];
            if (qdcount == 0) {
                ESP_LOGW(TAG, "Captive DNS: dotaz bez question sekce");
                continue;
            }

            char qname[128] = {0};
            size_t question_end = 12;
            size_t qname_end = decode_dns_qname(request,
                                                static_cast<size_t>(len),
                                                question_end,
                                                qname,
                                                sizeof(qname));
            if (qname_end == 0) {
                ESP_LOGW(TAG, "Captive DNS: nelze dekodovat qname");
                continue;
            }
            question_end = qname_end;

            if (question_end + 3 >= static_cast<size_t>(len)) {
                ESP_LOGW(TAG, "Captive DNS: nekompletni question cast");
                continue;
            }

            uint16_t qtype = (static_cast<uint16_t>(request[question_end]) << 8) | request[question_end + 1];
            uint16_t qclass = (static_cast<uint16_t>(request[question_end + 2]) << 8) | request[question_end + 3];

            char source_ip[16] = {0};
            inet_ntoa_r(source_addr.sin_addr, source_ip, sizeof(source_ip));
            ESP_LOGD(TAG,
                     "Captive DNS dotaz: src=%s qname='%s' qtype=%u qclass=%u len=%d",
                     source_ip,
                     (qname[0] != '\0') ? qname : "(empty)",
                     static_cast<unsigned>(qtype),
                     static_cast<unsigned>(qclass),
                     len);

            question_end += 4;

            uint8_t response[512] = {0};
            size_t response_len = question_end;
            if (response_len + 16 > sizeof(response)) {
                continue;
            }

            memcpy(response, request, response_len);

            response[2] = 0x81;
            response[3] = 0x80;
            response[6] = 0x00;
            response[7] = 0x01;
            response[8] = 0x00;
            response[9] = 0x00;
            response[10] = 0x00;
            response[11] = 0x00;

            response[response_len + 0] = 0xC0;
            response[response_len + 1] = 0x0C;
            response[response_len + 2] = 0x00;
            response[response_len + 3] = 0x01;
            response[response_len + 4] = 0x00;
            response[response_len + 5] = 0x01;
            response[response_len + 6] = 0x00;
            response[response_len + 7] = 0x00;
            response[response_len + 8] = 0x00;
            response[response_len + 9] = 0x3C;
            response[response_len + 10] = 0x00;
            response[response_len + 11] = 0x04;
            response[response_len + 12] = captive_ip[0];
            response[response_len + 13] = captive_ip[1];
            response[response_len + 14] = captive_ip[2];
            response[response_len + 15] = captive_ip[3];
            response_len += 16;

            int sent = sendto(sock,
                              response,
                              response_len,
                              0,
                              (struct sockaddr *)&source_addr,
                              source_len);
            if (sent < 0) {
                ESP_LOGW(TAG, "Captive DNS: sendto selhalo errno=%d", errno);
            } else {
                ESP_LOGD(TAG, "Captive DNS odpoved odeslana: %d B -> %s (A 192.168.4.1)", sent, source_ip);
            }
        }

        close(sock);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void start_captive_dns_server(void)
{
    if (s_captive_dns_task_handle != nullptr) {
        return;
    }

    BaseType_t task_result = xTaskCreate(captive_dns_task,
                                         "captive_dns",
                                         4096,
                                         nullptr,
                                         4,
                                         &s_captive_dns_task_handle);
    if (task_result != pdPASS) {
        s_captive_dns_task_handle = nullptr;
        ESP_LOGE(TAG, "Captive DNS: nelze vytvorit task");
    }
}

esp_err_t network_ap_mode_start(const char *ap_ssid, const char *ap_password)
{
    if (ap_ssid == nullptr || strlen(ap_ssid) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.ap.ssid, ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(ap_ssid);
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;

    if (ap_password != nullptr && strlen(ap_password) > 0) {
        strncpy((char *)wifi_config.ap.password, ap_password, sizeof(wifi_config.ap.password) - 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nelze nastavit AP mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nelze aplikovat AP konfiguraci: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nelze spustit WiFi v AP modu: %s", esp_err_to_name(ret));
        return ret;
    }

    start_captive_dns_server();

    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif != nullptr) {
        esp_netif_ip_info_t ip_info = {};
        if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) {
            ESP_LOGI(TAG, "AP mode: SSID=%s, IP=" IPSTR, ap_ssid, IP2STR(&ip_info.ip));
        }
    }

    return ESP_OK;
}
