#ifndef MQTT_INIT_H
#define MQTT_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_err.h>
#include <stdbool.h>

/**
 * @brief Inicializuje MQTT a připojí se k brokerovi
 * 
 * @param broker_uri URI brokerů (např. "mqtt://192.168.1.100:1883")
 * @return ESP_OK při úspěchu
 */
esp_err_t mqtt_init(const char *broker_uri, const char *username, const char *password);

/**
 * @brief Publikuje zprávu na MQTT topic
 * 
 * @param topic MQTT topic
 * @param data Data k publikování
 * @param retain Retain flag
 * @return ESP_OK při úspěchu
 */
esp_err_t mqtt_publish(const char *topic, const char *data, bool retain);

/**
 * @brief Čeká na připojení k MQTT brokerovi
 * 
 * @param timeout_ms Timeout v milisekundách
 * @return true pokud je připojeno
 */
bool mqtt_wait_connected(uint32_t timeout_ms);

/**
 * @brief Vrací stav připojení k MQTT
 * 
 * @return true pokud je připojeno
 */
bool mqtt_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif // MQTT_INIT_H
