# zalevaci-nadrz

## Struktura mqtt topiků.

```
home/water_tank/
 ├── state/
 │    ├── volume_l
 │    ├── flow_l_min
 │    ├── total_pumped_l
 │    ├── temp_water_c
 │    ├── temp_shaft_c
 │    ├── pressure_bar
 │    ├── filter_delta_bar
 │    └── pump/
 │         ├── running
 │         ├── power_w
 │         ├── current_a
 │         ├── voltage_v
 │         └── energy_kwh
 │
 ├── diag/
 │    ├── wifi_rssi_dbm
 │    ├── uptime_s
 │    ├── free_heap_b
 │    └── mqtt_reconnects
 │
 ├── event/
 │    ├── reboot_reason
 │    └── reboot_counter
 │
 ├── status
 │
 ├── cmd/
 │
 └── debug/
      ├── raw/
      └── intermediate/
```      
home/water_tank/state/heartbeat

## Publikační pravidla

| Kategorie | QoS | Retain |
| --- | ---: | :---: |
| state | 1 | ano |
| status | 1 | ano |
| event | 1 | ne |
| diag | 1 | ano |
| debug | 0 | ne |
| cmd | 1 | ne |

## Příkazy přes MQTT


* home/water_tank/cmd/reboot
* home/water_tank/cmd/reset_total
* home/water_tank/cmd/service_mode

## Poslat last will.

esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri = "mqtt://mqtt.home.arpa",
    .credentials.username = "esp32_tank",
    .credentials.authentication.password = "tajne",
    .session.last_will.topic = "home/water_tank/status",
    .session.last_will.msg = "offline",
    .session.last_will.qos = 1,
    .session.last_will.retain = 1,
};


typedef enum {
    NET_WIFI_DISCONNECTED,
    NET_WIFI_CONNECTED,

    NET_IP_LOST,
    NET_IP_ACQUIRED,

    NET_MQTT_DISCONNECTED,
    NET_MQTT_CONNECTED
} network_event_type_t;

typedef enum {
    SYS_NET_DOWN,
    SYS_NET_WIFI_ONLY,
    SYS_NET_IP_ONLY,
    SYS_NET_MQTT_READY,
    SYS_NET_AP_CONFIG
} system_network_level_t;

## Architektura site (po refaktoru)

- `components/network_core/network_init.*`
    - Orchestrace site: WiFi init (STA/AP), MQTT lifecycle, event handlery, retry timery (neblokujici).
    - Sbira runtime data (wifi/ip/mqtt) a publikuje callback `network_event_t`.

- `components/network_core/network_event.*`
    - Definice `network_event_t` + vyhodnoceni `system_network_level_t`.
    - Vyroba sitoveho eventu primo uvnitr `network_core` vcetne reconnect counteru.

- `components/network_core/network_mqtt_config.*`
    - Validace `mqtt_uri` a bezpecne ulozeni `uri/user/pass` pro MQTT klienta.
    - Poskytuje read-only pristup na pripravenou konfiguraci.

- `components/network_core/mqtt_publish.*`
    - Samostatna vrstva pro publikaci zprav (`mqtt_publish`) mimo init/lifecycle.
    - Pouziva aktivni MQTT klient handle z `network_init`.

- `main/network_event_bridge.*`
    - Odebira `network_event_t` callback z `network_core`.
    - Bali jej do `app_event_t` a publikuje `EVT_NETWORK` do interni event queue.