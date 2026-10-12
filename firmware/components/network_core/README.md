# network_core

ESP-IDF komponenta pro Wi-Fi STA/AP, MQTT lifecycle, LWT, základní publikaci a callback se síťovým snapshotem. Ověřeno proti kódu 12. 10. 2026; projekt ji sestavuje s ESP-IDF v5.5.2. Neobsahuje aplikační `sensor_events` ani `state_manager`.

## Veřejné API

| Hlavička | API |
| --- | --- |
| `include/network_init.h` | `network_register_event_callback`, `network_init_sta`, `network_init_ap`, `network_mqtt_start`, `network_mqtt_start_ex`, `network_init_with_mqtt_ex`, `network_mqtt_is_connected`, `network_mqtt_client`, `network_mqtt_status_topic` |
| `include/network_event.h` | typy `system_network_level_t`, `network_event_t`, funkce `network_event_level` a `network_event_make` |
| `include/mqtt_publish.h` | `mqtt_publish`, `mqtt_publish_online_status`, `mqtt_is_connected` |

Varianta `_ex` přijímá `network_mqtt_lwt_config_t` (`enabled`, `status_topic`, `qos`, `retain`); offline payload je `offline`. URI musí začínat `mqtt://` nebo `mqtts://` a mít neprázdný host. Validace není plná kontrola URL ani garance připojení TLS.

Inicializace nečeká synchronně na Wi-Fi/IP/MQTT. Dostupnost se zjišťuje callbackem nebo `network_mqtt_is_connected()`. Wi-Fi reconnect se plánuje timerem s exponenciální prodlevou 1–60 s; při získání IP se backoff obnovuje. Snapshot se posílá při změnách a také periodicky přibližně po 10 s.

## Integrace

Přidej `network_core` do komponentových závislostí a callback registruj před spuštěním sítě. Příklad vyžaduje odpovídající přihlašování; hodnoty níže jsou pouze zástupné:

```c
#include "network_init.h"
#include "mqtt_publish.h"
#include "esp_err.h"

static void on_network_event(const network_event_t *event, void *ctx)
{
    (void)ctx;
    if (event && event->level == SYS_NET_MQTT_READY) {
        (void)mqtt_publish("example/status", "online", true);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(network_register_event_callback(on_network_event, NULL));
    ESP_ERROR_CHECK(network_init_sta("SSID", "WIFI_PASSWORD"));
    ESP_ERROR_CHECK(network_mqtt_start("mqtt://BROKER:1883", "USER", "MQTT_PASSWORD"));
}
```

Callback se vyvolává ze síťové obsluhy; nedělej v něm dlouhé blokující operace. V host aplikaci `voda-septik` jej `network_event_bridge` převádí na události fronty a teprve `state_manager` řeší displeje, discovery a publikaci.

## `network_event_t`

| Pole | Význam |
| --- | --- |
| `level` | `SYS_NET_DOWN`, `SYS_NET_WIFI_ONLY`, `SYS_NET_IP_ONLY`, `SYS_NET_MQTT_READY`, `SYS_NET_AP_CONFIG` |
| `last_rssi` | poslední RSSI v dBm; nedostupný snapshot může mít `INT8_MIN` |
| `ip_addr` | IPv4 hodnota převzatá z ESP netif |
| `reconnect_attempts` | pokusy o Wi-Fi reconnect od startu |
| `reconnect_successes` | úspěšné získání IP po reconnect pokusu |
| `wifi_connects`, `wifi_disconnects` | Wi-Fi připojení a odpojení |
| `mqtt_connects`, `mqtt_disconnects` | MQTT připojení a odpojení |

`network_mqtt_client()` poskytuje aktivní handle pro registraci MQTT event handlerů; před použitím ověř, že není NULL. `mqtt_publish` používá QoS 1 a daný retain flag; neřeší aplikační topic registr, dostupnost, deduplikaci ani cache. Tyto funkce patří do `main/mqtt_publisher_task.cpp`.

## AP režim

`network_init_ap` přepne síť do AP režimu, odstaví MQTT/STA reconnect a spustí captive DNS. Vlastní HTTP web nezajišťuje tato komponenta; host aplikace jej spouští přes `webapp_startup` / `config_webapp`.

Aplikační propojení popisuje [mapa projektu](../../../.agents/PROJECT_INDEX.md).
