#include "network_init.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include <cstring>

#include "network_ap_mode.h"
#include "network_mqtt_config.h"
#include "network_event.h"

#define WIFI_RECONNECT_DELAY_MIN_MS 1000
#define WIFI_RECONNECT_DELAY_MAX_MS 60000

#define MQTT_LWT_TOPIC_MAX_LEN 128
#define MQTT_LWT_MESSAGE_MAX_LEN 64
#define MQTT_CLIENT_ID_MAX_LEN 32

static const char *TAG = "network";

static bool s_network_base_inited = false;
static bool s_sta_handlers_registered = false;
static int8_t s_last_rssi = INT8_MIN;
static esp_timer_handle_t s_network_publish_retry_timer = nullptr;
static esp_timer_handle_t s_wifi_reconnect_timer = nullptr;
static uint32_t s_wifi_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MIN_MS;
static uint32_t s_wifi_reconnect_attempts = 0;
static uint32_t s_wifi_reconnect_successes = 0;
static bool s_wifi_reconnect_pending = false;
static bool s_wifi_up = false;
static bool s_ip_ready = false;
static uint32_t s_ip_addr = 0;

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static bool s_mqtt_connected = false;
static bool s_lwt_enabled = false;
static int s_lwt_qos = 1;
static bool s_lwt_retain = true;
static char s_mqtt_status_topic[MQTT_LWT_TOPIC_MAX_LEN] = {0};
static char s_lwt_message[MQTT_LWT_MESSAGE_MAX_LEN] = {0};
static char s_mqtt_client_id[MQTT_CLIENT_ID_MAX_LEN] = {0};
static bool s_ap_mode_active = false;

static network_event_callback_t s_event_callback = nullptr;
static void *s_event_callback_ctx = nullptr;

static void publish_network_event(void);

static void schedule_wifi_reconnect(void);

static void wifi_reconnect_cb(void *arg)
{
    (void)arg;
    if (s_ap_mode_active) {
        return;
    }
    s_wifi_reconnect_attempts++;
    s_wifi_reconnect_pending = true;
    ESP_LOGW(TAG, "WiFi odpojeno, zkousim reconnect (backoff=%lu ms)", (unsigned long)s_wifi_reconnect_delay_ms);
    esp_wifi_connect();
}

static void schedule_wifi_reconnect(void)
{
    if (s_ap_mode_active) {
        return;
    }
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
    publish_network_event();
}

static void publish_network_event(void)
{
    network_event_t event = network_event_make(s_ap_mode_active,
                                               s_wifi_up,
                                               s_ip_ready,
                                               s_mqtt_connected,
                                               s_last_rssi,
                                               s_ip_addr,
                                               s_wifi_reconnect_attempts,
                                               s_wifi_reconnect_successes);

    if (s_event_callback != nullptr) {
        s_event_callback(&event, s_event_callback_ctx);
    }
}

static void network_platform_init(void)
{
    if (s_network_base_inited) {
        return;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());

    ret = esp_event_loop_create_default();
    if (ret == ESP_ERR_INVALID_STATE) {
        ret = ESP_OK;
    }
    ESP_ERROR_CHECK(ret);

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
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    auto schedule_retry_publish = []() {
        if (s_ap_mode_active) {
            return;
        }
        if (s_network_publish_retry_timer == nullptr) {
            return;
        }
        esp_timer_stop(s_network_publish_retry_timer);
        esp_timer_start_once(s_network_publish_retry_timer, 300 * 1000);
    };

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        s_wifi_reconnect_pending = false;
        s_wifi_up = false;
        s_ip_ready = false;
        s_ip_addr = 0;
        s_last_rssi = INT8_MIN;
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi spusteno, probiha pripojeni...");
        publish_network_event();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi pripojeno na AP, cekam na IP");
        s_wifi_up = true;
        s_ip_ready = false;
        s_ip_addr = 0;
        s_wifi_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MIN_MS;
        if (s_wifi_reconnect_timer != nullptr) {
            esp_timer_stop(s_wifi_reconnect_timer);
        }
        publish_network_event();
        schedule_retry_publish();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        s_wifi_up = false;
        s_ip_ready = false;
        s_ip_addr = 0;
        s_last_rssi = (disc != nullptr) ? disc->rssi : INT8_MIN;
        schedule_wifi_reconnect();
        publish_network_event();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Ziskana IP adresa:" IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_up = true;
        s_ip_ready = true;
        s_ip_addr = event->ip_info.ip.addr;
        wifi_ap_record_t ap_info = {};
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_last_rssi = ap_info.rssi;
        }
        if (s_wifi_reconnect_pending) {
            s_wifi_reconnect_successes++;
            s_wifi_reconnect_pending = false;
        }
        s_wifi_reconnect_delay_ms = WIFI_RECONNECT_DELAY_MIN_MS;
        if (s_wifi_reconnect_timer != nullptr) {
            esp_timer_stop(s_wifi_reconnect_timer);
        }
        publish_network_event();
        schedule_retry_publish();
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            if (s_ap_mode_active) {
                s_mqtt_connected = false;
                publish_network_event();
                break;
            }
            ESP_LOGI(TAG, "MQTT pripojeno");
            s_mqtt_connected = true;
            publish_network_event();
            break;

        case MQTT_EVENT_DISCONNECTED:
            if (s_ap_mode_active) {
                s_mqtt_connected = false;
                publish_network_event();
                break;
            }
            ESP_LOGW(TAG, "MQTT odpojeno");
            s_mqtt_connected = false;
            publish_network_event();
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

    network_platform_init();

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

    s_ap_mode_active = false;
    s_wifi_up = false;
    s_ip_ready = false;
    s_ip_addr = 0;
    s_last_rssi = INT8_MIN;

    ESP_LOGI(TAG, "WiFi inicializace dokoncena. Pripojuji se k SSID:%s", ssid);

    return ESP_OK;
}

esp_err_t network_init_ap(const char *ap_ssid, const char *ap_password)
{
    network_platform_init();
    esp_err_t ap_ret = network_ap_mode_start(ap_ssid, ap_password);
    if (ap_ret != ESP_OK) {
        ESP_LOGE(TAG, "Nelze spustit AP konfiguracni rezim: %s", esp_err_to_name(ap_ret));
        return ap_ret;
    }

    if (s_wifi_reconnect_timer != nullptr) {
        esp_timer_stop(s_wifi_reconnect_timer);
    }
    if (s_network_publish_retry_timer != nullptr) {
        esp_timer_stop(s_network_publish_retry_timer);
    }

    s_ap_mode_active = true;
    s_mqtt_connected = false;
    s_wifi_reconnect_pending = false;
    if (s_mqtt_client != NULL) {
        esp_mqtt_client_stop(s_mqtt_client);
    }

    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t ip_info = {};
    bool ap_ip_ok = (ap_netif != NULL) && (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) && (ip_info.ip.addr != 0);
    s_wifi_up = true;
    s_ip_ready = ap_ip_ok;
    s_ip_addr = ap_ip_ok ? ip_info.ip.addr : 0;
    s_last_rssi = INT8_MIN;

    publish_network_event();

    return ESP_OK;
}

esp_err_t network_mqtt_start_ex(const char *broker_uri,
                                const char *username,
                                const char *password,
                                const network_mqtt_lwt_config_t *lwt_config)
{
    if (s_ap_mode_active) {
        ESP_LOGW(TAG, "MQTT start preskocen: aktivni AP konfiguracni rezim");
        return ESP_ERR_INVALID_STATE;
    }

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
