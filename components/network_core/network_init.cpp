#include "network_init.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <cstring>
#include <errno.h>

#include <lwip/inet.h>
#include <lwip/sockets.h>

#include "network_mqtt_config.h"
#include "network_event.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define WIFI_RECONNECT_DELAY_MIN_MS 1000
#define WIFI_RECONNECT_DELAY_MAX_MS 60000

#define MQTT_CONNECTED_BIT BIT0
#define MQTT_LWT_TOPIC_MAX_LEN 128
#define MQTT_LWT_MESSAGE_MAX_LEN 64
#define MQTT_CLIENT_ID_MAX_LEN 32

static const char *TAG = "network";

static EventGroupHandle_t s_wifi_event_group;
static bool s_network_base_inited = false;
static bool s_sta_handlers_registered = false;
static int8_t s_last_rssi = INT8_MIN;
static esp_timer_handle_t s_network_publish_retry_timer = nullptr;
static esp_timer_handle_t s_wifi_reconnect_timer = nullptr;
static uint32_t s_wifi_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MIN_MS;

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static EventGroupHandle_t s_mqtt_event_group = NULL;
static bool s_mqtt_connected = false;
static bool s_lwt_enabled = false;
static int s_lwt_qos = 1;
static bool s_lwt_retain = true;
static char s_mqtt_status_topic[MQTT_LWT_TOPIC_MAX_LEN] = {0};
static char s_lwt_message[MQTT_LWT_MESSAGE_MAX_LEN] = {0};
static char s_mqtt_client_id[MQTT_CLIENT_ID_MAX_LEN] = {0};
static TaskHandle_t s_captive_dns_task_handle = nullptr;

static network_state_callback_t s_state_callback = nullptr;
static void *s_state_callback_ctx = nullptr;
static network_event_callback_t s_event_callback = nullptr;
static void *s_event_callback_ctx = nullptr;

static void publish_network_event(bool wifi_up_hint);

static void schedule_wifi_reconnect(void);
static void start_captive_dns_server(void);

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
            ESP_LOGI(TAG,
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
                ESP_LOGI(TAG, "Captive DNS odpoved odeslana: %d B -> %s (A 192.168.4.1)", sent, source_ip);
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

static void wifi_reconnect_cb(void *arg)
{
    (void)arg;
    ESP_LOGW(TAG, "WiFi odpojeno, zkousim reconnect (backoff=%lu ms)", (unsigned long)s_wifi_reconnect_delay_ms);
    esp_wifi_connect();
}

static void schedule_wifi_reconnect(void)
{
    if (s_wifi_reconnect_timer == nullptr) {
        return;
    }

    esp_timer_stop(s_wifi_reconnect_timer);
    esp_timer_start_once(s_wifi_reconnect_timer, (uint64_t)s_wifi_reconnect_delay_ms * 1000ULL);

    if (s_wifi_reconnect_delay_ms < WIFI_RECONNECT_DELAY_MAX_MS) {
        uint32_t next_delay = s_wifi_reconnect_delay_ms * 2;
        s_wifi_reconnect_delay_ms = (next_delay > WIFI_RECONNECT_DELAY_MAX_MS)
                                   ? WIFI_RECONNECT_DELAY_MAX_MS
                                   : next_delay;
    }
}

static void ensure_client_id_generated(void)
{
    if (s_mqtt_client_id[0] != '\0') {
        return;
    }

    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(s_mqtt_client_id,
                 sizeof(s_mqtt_client_id),
                 "esp32-%02x%02x%02x",
                 mac[3],
                 mac[4],
                 mac[5]);
    } else {
        snprintf(s_mqtt_client_id, sizeof(s_mqtt_client_id), "esp32-client");
    }
}

static void delayed_network_publish_cb(void *arg)
{
    (void)arg;
    publish_network_event(true);
}

static void publish_network_event(bool wifi_up_hint)
{
    wifi_ap_record_t ap_info = {};
    bool ap_info_ok = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
    bool wifi_up = wifi_up_hint || ap_info_ok;

    int8_t last_rssi = INT8_MIN;
    if (ap_info_ok) {
        last_rssi = ap_info.rssi;
        s_last_rssi = ap_info.rssi;
    } else if (wifi_up && s_last_rssi != INT8_MIN) {
        last_rssi = s_last_rssi;
    }

    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip_info = {};
    bool ip_ready = (sta_netif != NULL)
                 && (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK)
                 && (ip_info.ip.addr != 0);
    uint32_t ip_addr = ip_ready ? ip_info.ip.addr : 0;

    if (s_state_callback != nullptr) {
        network_state_t state = {
            .wifi_up = wifi_up,
            .ip_ready = ip_ready,
            .mqtt_ready = s_mqtt_connected,
            .last_rssi = last_rssi,
            .ip_addr = ip_addr,
            .timestamp_us = esp_timer_get_time(),
        };
        s_state_callback(&state, s_state_callback_ctx);

        if (s_event_callback != nullptr) {
            network_event_t event = network_event_make(state.wifi_up,
                                                       state.ip_ready,
                                                       state.mqtt_ready,
                                                       state.last_rssi,
                                                       state.ip_addr);
            s_event_callback(&event, s_event_callback_ctx);
        }
        return;
    }

    if (s_event_callback != nullptr) {
        network_event_t event = network_event_make(wifi_up,
                                                   ip_ready,
                                                   s_mqtt_connected,
                                                   last_rssi,
                                                   ip_addr);
        s_event_callback(&event, s_event_callback_ctx);
    }
}

static esp_err_t network_platform_init(void)
{
    if (s_network_base_inited) {
        return ESP_OK;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if (s_network_publish_retry_timer == nullptr) {
        esp_timer_create_args_t timer_args = {
            .callback = &delayed_network_publish_cb,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "net_pub_retry",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_network_publish_retry_timer));
    }

    if (s_wifi_reconnect_timer == nullptr) {
        esp_timer_create_args_t reconnect_timer_args = {
            .callback = &wifi_reconnect_cb,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "wifi_reconnect",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&reconnect_timer_args, &s_wifi_reconnect_timer));
    }

    s_network_base_inited = true;
    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    auto schedule_retry_publish = []() {
        if (s_network_publish_retry_timer == nullptr) {
            return;
        }
        esp_timer_stop(s_network_publish_retry_timer);
        esp_timer_start_once(s_network_publish_retry_timer, 300 * 1000);
    };

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi spusteno, probiha pripojeni...");
        publish_network_event(false);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi pripojeno na AP, cekam na IP");
        s_wifi_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MIN_MS;
        if (s_wifi_reconnect_timer != nullptr) {
            esp_timer_stop(s_wifi_reconnect_timer);
        }
        publish_network_event(true);
        schedule_retry_publish();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        schedule_wifi_reconnect();
        publish_network_event(false);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Ziskana IP adresa:" IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MIN_MS;
        if (s_wifi_reconnect_timer != nullptr) {
            esp_timer_stop(s_wifi_reconnect_timer);
        }
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        publish_network_event(true);
        schedule_retry_publish();
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT pripojeno");
            s_mqtt_connected = true;
            xEventGroupSetBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
            publish_network_event(true);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT odpojeno");
            s_mqtt_connected = false;
            xEventGroupClearBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
            publish_network_event(true);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT chyba");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Posledni chyba stacku: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "Cislo chyby TLS: 0x%x", event->error_handle->esp_tls_stack_err);
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGE(TAG, "Broker odmitl pripojeni");
            }
            break;

        default:
            break;
    }
}

esp_err_t network_register_state_callback(network_state_callback_t callback, void *ctx)
{
    s_state_callback = callback;
    s_state_callback_ctx = ctx;
    return ESP_OK;
}

esp_err_t network_register_event_callback(network_event_callback_t callback, void *ctx)
{
    s_event_callback = callback;
    s_event_callback_ctx = ctx;
    return ESP_OK;
}

esp_err_t network_init_sta(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL || strlen(ssid) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(network_platform_init());

    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
    }
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    esp_netif_create_default_wifi_sta();

    if (!s_sta_handlers_registered) {
        esp_event_handler_instance_t instance_any_id;
        esp_event_handler_instance_t instance_got_ip;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            &instance_any_id));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            &instance_got_ip));
        s_sta_handlers_registered = true;
    }

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi inicializace dokoncena. Pripojuji se k SSID:%s", ssid);

    return ESP_OK;
}

esp_err_t network_init_ap(const char *ap_ssid, const char *ap_password)
{
    if (ap_ssid == NULL || strlen(ap_ssid) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(network_platform_init());
    esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.ap.ssid, ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(ap_ssid);
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;

    if (ap_password != NULL && strlen(ap_password) > 0) {
        strncpy((char *)wifi_config.ap.password, ap_password, sizeof(wifi_config.ap.password) - 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    start_captive_dns_server();

    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif != NULL) {
        esp_netif_ip_info_t ip_info = {};
        if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) {
            ESP_LOGI(TAG, "AP mode: SSID=%s, IP=" IPSTR, ap_ssid, IP2STR(&ip_info.ip));
        }
    }

    return ESP_OK;
}

bool network_wait_connected(uint32_t timeout_ms)
{
    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           ticks);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Pripojeno k WiFi siti");
        return true;
    }
    if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Selhalo pripojeni k WiFi");
        return false;
    }

    ESP_LOGW(TAG, "Timeout pri cekani na WiFi pripojeni");
    return false;
}

esp_err_t network_mqtt_start_ex(const char *broker_uri,
                                const char *username,
                                const char *password,
                                const network_mqtt_lwt_config_t *lwt_config)
{
    if (s_mqtt_client != NULL) {
        ESP_LOGW(TAG, "MQTT jiz inicializovan");
        return ESP_OK;
    }

    if (!network_mqtt_config_prepare(broker_uri, username, password)) {
        ESP_LOGE(TAG, "Neplatne MQTT URI: '%s'", broker_uri != NULL ? broker_uri : "(null)");
        ESP_LOGE(TAG, "Ocekavam format mqtt://host:port nebo mqtts://host:port");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG,
             "MQTT connect cfg: uri='%s', user='%s', password_set=%s",
             broker_uri,
             (username != NULL && username[0] != '\0') ? username : "(none)",
             (password != NULL && password[0] != '\0') ? "yes" : "no");

    s_lwt_enabled = false;
    s_mqtt_status_topic[0] = '\0';
    s_lwt_message[0] = '\0';
    s_lwt_qos = 1;
    s_lwt_retain = true;

    if (lwt_config != NULL && lwt_config->enabled) {
        if (lwt_config->status_topic == NULL || lwt_config->status_topic[0] == '\0') {
            ESP_LOGE(TAG, "LWT je zapnute, ale status topic neni vyplnen");
            return ESP_ERR_INVALID_ARG;
        }

        strncpy(s_mqtt_status_topic, lwt_config->status_topic, sizeof(s_mqtt_status_topic) - 1);
        s_mqtt_status_topic[sizeof(s_mqtt_status_topic) - 1] = '\0';

        const char *message = "offline";
        strncpy(s_lwt_message, message, sizeof(s_lwt_message) - 1);
        s_lwt_message[sizeof(s_lwt_message) - 1] = '\0';

        s_lwt_qos = (lwt_config->qos < 0) ? 0 : ((lwt_config->qos > 2) ? 2 : lwt_config->qos);
        s_lwt_retain = lwt_config->retain;
        s_lwt_enabled = true;

        ESP_LOGI(TAG,
                 "MQTT LWT cfg: status_topic='%s', offline_msg='%s', qos=%d, retain=%s",
                 s_mqtt_status_topic,
                 s_lwt_message,
                 s_lwt_qos,
                 s_lwt_retain ? "yes" : "no");
    }

    s_mqtt_event_group = xEventGroupCreate();
    if (s_mqtt_event_group == NULL) {
        ESP_LOGE(TAG, "Nelze vytvorit MQTT event group");
        return ESP_ERR_NO_MEM;
    }

    ensure_client_id_generated();

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = network_mqtt_config_uri();
    mqtt_cfg.session.keepalive = 15;
    mqtt_cfg.network.disable_auto_reconnect = false;
    mqtt_cfg.credentials.client_id = s_mqtt_client_id;
    mqtt_cfg.credentials.username = network_mqtt_config_username_or_null();
    mqtt_cfg.credentials.authentication.password = network_mqtt_config_password_or_null();
    if (s_lwt_enabled) {
        mqtt_cfg.session.last_will.topic = s_mqtt_status_topic;
        mqtt_cfg.session.last_will.msg = s_lwt_message;
        mqtt_cfg.session.last_will.qos = s_lwt_qos;
        mqtt_cfg.session.last_will.retain = s_lwt_retain;
    }

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Nelze inicializovat MQTT klienta");
        return ESP_FAIL;
    }

    esp_err_t ret = esp_mqtt_client_register_event(s_mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nelze registrovat MQTT event handler");
        return ret;
    }

    ret = esp_mqtt_client_start(s_mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nelze spustit MQTT klienta");
        return ret;
    }

    ESP_LOGI(TAG, "MQTT klient inicializovan: %s", network_mqtt_config_uri());
    return ESP_OK;
}

esp_err_t network_mqtt_start(const char *broker_uri, const char *username, const char *password)
{
    return network_mqtt_start_ex(broker_uri, username, password, NULL);
}

bool network_mqtt_wait_connected(uint32_t timeout_ms)
{
    if (s_mqtt_event_group == NULL) {
        ESP_LOGE(TAG, "MQTT event group neni inicializovana");
        return false;
    }

    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    EventBits_t bits = xEventGroupWaitBits(s_mqtt_event_group, MQTT_CONNECTED_BIT, pdFALSE, pdFALSE, ticks);
    return (bits & MQTT_CONNECTED_BIT) != 0;
}

bool network_mqtt_is_connected(void)
{
    return s_mqtt_connected;
}

esp_mqtt_client_handle_t network_mqtt_client(void)
{
    return s_mqtt_client;
}

const char *network_mqtt_status_topic(void)
{
    if (s_mqtt_status_topic[0] == '\0') {
        return NULL;
    }

    return s_mqtt_status_topic;
}
