# network_core

Obecna ESP-IDF komponenta pro:
- inicializaci Wi-Fi (STA/AP),
- lifecycle MQTT klienta,
- publikaci MQTT zprav,
- callback s aktualnim stavem site.

Komponenta je zamerne bez aplikačně specifické logiky (zadne `sensor_events`, `state_manager`, apod.).

## Verejne API

Hlavičky:
- `include/network_init.h`
- `include/mqtt_publish.h`

Hlavni funkce:
- `network_register_state_callback(...)`
- `network_init_sta(...)`
- `network_init_ap(...)`
- `network_wait_connected(...)`
- `network_mqtt_start(...)`
- `network_mqtt_wait_connected(...)`
- `network_mqtt_is_connected()`
- `mqtt_publish(...)`
- `mqtt_is_connected()`

## Integrace do projektu

1. Pridat komponentu do `components/network_core`.
2. V `main/CMakeLists.txt` pridat zavislost `network_core` do `REQUIRES`.
3. V aplikaci zavolat inicializaci:

```c
#include "network_init.h"
#include "mqtt_publish.h"

static void on_network_state(const network_state_t *state, void *ctx)
{
    (void)ctx;
    // app-specificka reakce na wifi/ip/mqtt zmenu
}

void app_main(void)
{
    ESP_ERROR_CHECK(network_register_state_callback(on_network_state, NULL));

    ESP_ERROR_CHECK(network_init_sta("my-ssid", "my-pass"));
    network_wait_connected(10000);

    ESP_ERROR_CHECK(network_mqtt_start("mqtt://broker:1883", "user", "pass"));

    if (mqtt_is_connected()) {
        mqtt_publish("my/topic", "hello", true);
    }
}
```

## Poznamky

- URI musi mit format `mqtt://host:port` nebo `mqtts://host:port`.
- Callback dostava agregovany stav (`wifi_up`, `ip_ready`, `mqtt_ready`, `last_rssi`, `ip_addr`, `timestamp_us`).
- Komponenta neimplementuje zadny aplikační state machine; ten patri do host aplikace.
