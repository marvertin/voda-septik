#include "wifi_init.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <string.h>

// WiFi konfigurace - pokud jsou vyplněny, uloží se do flash a použijí se
// Můžete je pak vymazat a budou se načítat z flash
#define WIFI_SSID      ""  // Je ve Flashi když je prázdný, protože se načítá z NVS
#define WIFI_PASSWORD  "" // Je ve Flashi když je prázdný, protože se načítá z NVS
#define WIFI_MAX_RETRY 5

#define NVS_NAMESPACE "wifi_config"
#define NVS_SSID_KEY "ssid"
#define NVS_PASS_KEY "password"

static const char *TAG = "wifi";

// Event bity
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// Uložení WiFi credentials do NVS
static esp_err_t save_wifi_credentials(const char* ssid, const char* password)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Chyba při otevírání NVS pro zápis");
        return ret;
    }

    ret = nvs_set_str(nvs_handle, NVS_SSID_KEY, ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Chyba při ukládání SSID do NVS");
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_set_str(nvs_handle, NVS_PASS_KEY, password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Chyba při ukládání hesla do NVS");
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WiFi credentials uloženy do flash");
    }
    return ret;
}

// Načtení WiFi credentials z NVS
static esp_err_t load_wifi_credentials(char* ssid, size_t ssid_len, char* password, size_t pass_len)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi credentials nejsou v NVS");
        return ret;
    }

    size_t required_size = ssid_len;
    ret = nvs_get_str(nvs_handle, NVS_SSID_KEY, ssid, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SSID není v NVS");
        nvs_close(nvs_handle);
        return ret;
    }

    required_size = pass_len;
    ret = nvs_get_str(nvs_handle, NVS_PASS_KEY, password, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Heslo není v NVS");
        nvs_close(nvs_handle);
        return ret;
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "WiFi credentials načteny z flash");
    return ESP_OK;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi spuštěno, probíhá připojení...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Opakování připojení k AP, pokus %d/%d", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Selhalo připojení k WiFi");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Získána IP adresa:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_init_sta(void)
{
    esp_err_t ret;

    // Inicializace NVS (nutné pro WiFi)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Vytvořit event group pro synchronizaci
    s_wifi_event_group = xEventGroupCreate();

    // Inicializace TCP/IP stacku
    ESP_ERROR_CHECK(esp_netif_init());

    // Vytvořit výchozí event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Inicializace WiFi s výchozí konfigurací
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Registrace event handlerů
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

    // Konfigurace WiFi - načtení credentials
    char ssid[32] = {0};
    char password[64] = {0};

    // Zkontroluj, jestli jsou definovány v kódu (a nejsou prázdné)
    if (strlen(WIFI_SSID) > 0 && strlen(WIFI_PASSWORD) > 0) {
        strncpy(ssid, WIFI_SSID, sizeof(ssid) - 1);
        strncpy(password, WIFI_PASSWORD, sizeof(password) - 1);
        ESP_LOGI(TAG, "Používám WiFi credentials z kódu");
        
        // Ulož do NVS pro příští použití
        save_wifi_credentials(ssid, password);
    } else {
        // Pokus se načíst z NVS
        ESP_LOGI(TAG, "WiFi credentials nejsou v kódu, načítám z flash");
        ret = load_wifi_credentials(ssid, sizeof(ssid), password, sizeof(password));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Nelze načíst WiFi credentials! Definujte WIFI_SSID a WIFI_PASSWORD v kódu.");
            return ESP_FAIL;
        }
    }

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi inicializace dokončena. Připojuji se k SSID:%s", ssid);

    return ESP_OK;
}

bool wifi_wait_connected(uint32_t timeout_ms)
{
    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            ticks);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Připojeno k WiFi síti");
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Selhalo připojení k WiFi");
        return false;
    } else {
        ESP_LOGW(TAG, "Timeout při čekání na WiFi připojení");
        return false;
    }
}
