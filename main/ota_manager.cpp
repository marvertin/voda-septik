#include "ota_manager.h"

#include <string.h>

extern "C" {
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

static const char *TAG = "ota_manager";
static constexpr size_t OTA_URL_MAX_LEN = 384;
static constexpr size_t OTA_HTTP_BUF_SIZE = 4096;

struct ota_task_params_t {
    char url[OTA_URL_MAX_LEN];
};

static bool s_ota_in_progress = false;
static portMUX_TYPE s_ota_mux = portMUX_INITIALIZER_UNLOCKED;

static bool ota_is_in_progress(void)
{
    bool in_progress = false;
    taskENTER_CRITICAL(&s_ota_mux);
    in_progress = s_ota_in_progress;
    taskEXIT_CRITICAL(&s_ota_mux);
    return in_progress;
}

static void ota_set_in_progress(bool in_progress)
{
    taskENTER_CRITICAL(&s_ota_mux);
    s_ota_in_progress = in_progress;
    taskEXIT_CRITICAL(&s_ota_mux);
}

static void ota_task(void *param)
{
    ota_task_params_t *task_params = static_cast<ota_task_params_t *>(param);
    esp_http_client_handle_t client = nullptr;
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = nullptr;
    uint8_t buffer[OTA_HTTP_BUF_SIZE] = {0};
    int status_code = 0;
    int http_status = 0;
    bool ota_started = false;
    bool ota_finished = false;

    ESP_LOGI(TAG, "OTA startuji z URL: %s", task_params->url);

    esp_http_client_config_t http_cfg = {};
    http_cfg.url = task_params->url;
    http_cfg.timeout_ms = 15000;
    http_cfg.keep_alive_enable = true;

    client = esp_http_client_init(&http_cfg);
    if (client == nullptr) {
        ESP_LOGE(TAG, "esp_http_client_init selhal");
        goto cleanup;
    }

    if (esp_http_client_open(client, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Nelze otevrit HTTP spojeni pro OTA");
        goto cleanup;
    }

    status_code = esp_http_client_fetch_headers(client);
    if (status_code < 0) {
        ESP_LOGW(TAG, "HTTP hlavicky bez content length (%d), pokracuji streamem", status_code);
    }

    http_status = esp_http_client_get_status_code(client);
    if (http_status != 200) {
        ESP_LOGE(TAG, "HTTP status pro OTA neni 200, ale %d", http_status);
        goto cleanup;
    }

    update_partition = esp_ota_get_next_update_partition(nullptr);
    if (update_partition == nullptr) {
        ESP_LOGE(TAG, "Nenalezena update partition");
        goto cleanup;
    }

    if (esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin selhal");
        goto cleanup;
    }
    ota_started = true;

    while (true) {
        int read_len = esp_http_client_read(client, reinterpret_cast<char *>(buffer), sizeof(buffer));
        if (read_len < 0) {
            ESP_LOGE(TAG, "Chyba pri cteni OTA streamu");
            goto cleanup;
        }

        if (read_len == 0) {
            break;
        }

        if (esp_ota_write(ota_handle, buffer, (size_t)read_len) != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write selhal");
            goto cleanup;
        }
    }

    if (esp_ota_end(ota_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end selhal");
        goto cleanup;
    }
    ota_finished = true;

    if (esp_ota_set_boot_partition(update_partition) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition selhal");
        goto cleanup;
    }

    ESP_LOGW(TAG, "OTA uspesne dokonceno, rebootuji do noveho firmware");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();

cleanup:
    if (client != nullptr) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
    }

    if (ota_started && !ota_finished) {
        (void)esp_ota_abort(ota_handle);
    }

    ota_set_in_progress(false);
    delete task_params;
    vTaskDelete(nullptr);
}

esp_err_t ota_manager_start_from_url(const char *url)
{
    if (url == nullptr || url[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (strncmp(url, "http://", 7) != 0 && strncmp(url, "https://", 8) != 0) {
        ESP_LOGW(TAG, "OTA URL musi zacinat na http:// nebo https://");
        return ESP_ERR_INVALID_ARG;
    }

    if (ota_is_in_progress()) {
        ESP_LOGW(TAG, "OTA uz probiha, novy request odmtnut");
        return ESP_ERR_INVALID_STATE;
    }

    ota_task_params_t *params = new ota_task_params_t();
    if (params == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    strncpy(params->url, url, sizeof(params->url) - 1);
    params->url[sizeof(params->url) - 1] = '\0';

    ota_set_in_progress(true);
    if (xTaskCreate(ota_task,
                    "ota_task",
                    configMINIMAL_STACK_SIZE * 10,
                    params,
                    5,
                    nullptr) != pdPASS) {
        ota_set_in_progress(false);
        delete params;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t ota_manager_confirm_running_firmware(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    esp_err_t state_result = esp_ota_get_state_partition(running, &state);
    if (state_result != ESP_OK) {
        ESP_LOGW(TAG, "esp_ota_get_state_partition selhal: %s", esp_err_to_name(state_result));
        return state_result;
    }

    ESP_LOGI(TAG, "Aktualni OTA stav bezi partition: %d", (int)state);

    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t mark_result = esp_ota_mark_app_valid_cancel_rollback();
        if (mark_result == ESP_OK) {
            ESP_LOGW(TAG, "Firmware potvrzen, rollback zrusen");
        } else {
            ESP_LOGE(TAG, "Potvrzeni firmware selhalo: %s", esp_err_to_name(mark_result));
        }
        return mark_result;
    }

    ESP_LOGI(TAG, "Firmware neni v PENDING_VERIFY, potvrzeni neni potreba");
    return ESP_OK;
}
