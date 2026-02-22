# network_core

Obecna ESP-IDF komponenta pro:
- inicializaci Wi-Fi (STA/AP),
- lifecycle MQTT klienta,
- publikaci MQTT zprav,
- callback se sitovym eventem.

Komponenta je zamerne bez aplikačně specifické logiky (zadne `sensor_events`, `state_manager`, apod.).

## Verejne API

Hlavičky:
- `include/network_init.h`
- `include/mqtt_publish.h`

Hlavni funkce:
- `network_register_event_callback(...)`
- `network_init_sta(...)`
- `network_init_ap(...)`
- `network_mqtt_start(...)`
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

static void on_network_event(const network_event_t *event, void *ctx)
{
    (void)ctx;
    // app-specificka reakce na zmenu sitoveho stavu
    // event->level / event->last_rssi / event->ip_addr / reconnect countery
}

void app_main(void)
{
    ESP_ERROR_CHECK(network_register_event_callback(on_network_event, NULL));

    ESP_ERROR_CHECK(network_init_sta("my-ssid", "my-pass"));

    ESP_ERROR_CHECK(network_mqtt_start("mqtt://broker:1883", "user", "pass"));

    if (mqtt_is_connected()) {
        mqtt_publish("my/topic", "hello", true);
    }
}
```

## Poznamky

- URI musi mit format `mqtt://host:port` nebo `mqtts://host:port`.
- Callback dostava `network_event_t` (level, RSSI, IP, reconnect_attempts, reconnect_successes).
- Komponenta neimplementuje zadny aplikační state machine; ten patri do host aplikace.

### `network_event_t` reference

| Pole | Vyznam | Jednotka / format |
| --- | --- | --- |
| `level` | Agregovana uroven konektivity (`SYS_NET_*`). | enum |
| `last_rssi` | Posledni znama sila Wi-Fi signalu. | dBm |
| `ip_addr` | IPv4 adresa v `uint32_t` (network byte order). | `0xAABBCCDD` |
| `reconnect_attempts` | Pocet pokusu o Wi-Fi reconnect od startu. | pocet |
| `reconnect_successes` | Pocet uspesnych reconnectu (ziskana IP po reconnect pokusu). | pocet |
