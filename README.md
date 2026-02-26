# zalevaci-nadrz

## Dokumentace

- [OTA aktualizace](docs/ota.md)
- [Logovani a log levely](docs/logging.md)

## Struktura mqtt topiků.

Poznamka (migrace):
- Konfiguracni polozka `mqtt_topic` byla odstranena.
- Korenny topic je nyni pevne dany v registru topiku (`main/mqtt_topics.*`) jako `zalivka/nadrz`.

```
zalivka/nadrz                       Týká se zálivky a nádrže na dešťovku + vodu ze studny

├── stav/
│    ├── objem                      [l] Aktuální objem vody v nádrži. HA: sensor (state_class: measurement)
│    ├── prutok                     [l/min] Aktuální průtok vody. HA: sensor (state_class: measurement)
│    ├── cerpano_celkem             [l] Celkové vyčerpané množství vody od počátku. HA: sensor (state_class: total_increasing)
│    ├── teplota_voda               [°C] Aktuální teplota vody v nádrži. HA: sensor (device_class: temperature)
│    ├── teplota_nadrz              [°C] Aktuální teplota vzduchu v šachtě u potrubí v blízkosti tlakové nádoby. HA: sensor (device_class: temperature)
│    ├── tlak_pred_filtrem          [bar] Aktuální tlak vody před filtrem. HA: sensor (device_class: pressure)
│    ├── tlak_za_filtrem            [bar] Aktuální tlak vody za filtrem. HA: sensor (device_class: pressure)
│    ├── rozdil_tlaku_filtru        [bar] Rozdíl tlaku před a za filtrem. HA: sensor (state_class: measurement)
│    ├── zanesenost_filtru_percent  [%] Odhad zanesení filtru v procentech (odvozeno z rozdílu tlaků). HA: sensor
│    └── pumpa/
│         ├── bezi                  [bool] Příznak ano/ne (0/1), zda čerpadlo běží. HA: binary_sensor (device_class: running)
│         ├── vykon_cinny_w         [W] Aktuální činný výkon. HA: sensor (device_class: power)
│         ├── jalovy_vykon_var      [var] Aktuální jalový výkon. HA: sensor
│         ├── cosfi                 [-] Aktuální účiník. HA: sensor
│         ├── proud_a               [A] Aktuální proud protékaný čerpadlem. HA: sensor (device_class: current)
│         ├── napeti_v              [V] Aktuální napětí na čerpadle. HA: sensor (device_class: voltage)
│         ├── energie_cinna_kwh     [kWh] Celkové množství spotřebované činné energie. HA: sensor (device_class: energy, state_class: total_increasing)
│         └── energie_jalova_kvarh  [kvarh] Celkové množství spotřebované jalové energie. HA: sensor (state_class: total_increasing)
│
├── system/
│    ├── status                     [-] online/offline (řešeno přes LWT). HA: binary_sensor (device_class: connectivity)
│    ├── boot_mode                  [text] Rezim startu firmware: `ota` (pending verify) / `normal`
│    ├── ota/event                  [text] Prubeh OTA operace (start/download/rebooting/confirmed/error_...)
│    ├── ota/progress               [%] Prubeh OTA stahovani a zapisu (0-100)
│    ├── reboot_reason              [text] Důvod posledního rebootu (esp_reset_reason). HA: sensor
│    ├── reboot_counter             [count] Počet rebootů od instalace. HA: sensor (state_class: total_increasing)
│    └── last_disconnect_duration_s [s] Doba posledního výpadku spojení. HA: sensor
│
├── diag/
│    ├── fw_version                 [text] Verze firmware. HA: sensor
│    ├── build_timestamp            [datetime] Datum a čas sestavení firmware. HA: sensor
│    ├── git_hash                   [text] Git hash verze firmware. HA: sensor
│    ├── uptime_s                   [s] Jak dlouho zařízení běží. HA: sensor (state_class: total_increasing)
│    ├── wifi_rssi_dbm              [dBm] Síla WiFi signálu. HA: sensor (device_class: signal_strength)
│    ├── wifi_reconnect_try         [count] Počet pokusů o reconnect k WiFi. HA: sensor (state_class: total_increasing)
│    ├── wifi_reconnect_success     [count] Počet úspěšných reconnectů k WiFi. HA: sensor (state_class: total_increasing)
│    ├── mqtt_reconnects            [count] Počet reconnectů k MQTT brokeru. HA: sensor (state_class: total_increasing)
│    ├── last_mqtt_rc               [code] Poslední návratový kód MQTT klienta. HA: sensor
│    ├── heap_free_b                [B] Aktuální počet volných bajtů na heapu. HA: sensor (device_class: data_size)
│    ├── heap_min_free_b            [B] Nejmenší zaznamenaná hodnota volného heapu od startu. HA: sensor (device_class: data_size)
│    ├── esp_vcc_mv                 [mV] Napájecí napětí ESP. HA: sensor (device_class: voltage)
│    └── nvs_errors                 [count] Počet chyb při práci s NVS. HA: sensor (state_class: total_increasing)
│
├── cmd/
│    ├── reboot                     [-] Reboot zařízení, čímž se vypnou všechny debugy. HA: button
│    ├── webapp/start               [-] Nastartování konfigurační aplikace. Implicitně je vypnutá, po startu se sama po 2 hodinách vypne. HA: button
│    ├── webapp/stop                [-] Vypnutí webové aplikace. HA: button
│    ├── debug/start                [-] Nastartování debug režimu, začnou se produkovat debug data. HA: button
│    ├── debug/stop                 [-] Zastavení debug režimu. HA: button
│    ├── log/level                  [text] Nastaveni log levelu per tag (payload: tag=level)
│    ├── ota/start                  [url] URL na binarni obraz firmware (http/https)
│    └── ota/confirm                [-] Rucni potvrzeni nahraneho firmware po overeni funkcnosti
│
└── debug


## Ovladani pres Mosquitto CLI

Podrobny navod k logovani je v dokumentu [Logovani a log levely](docs/logging.md).

Pro pohodlne ovladani je v projektu interaktivni skript:

```bash
./tools/mqtt_cmd_cli.sh
```

Skript:
- pouziva default parametry: `mqtt.home.arpa:1883`, uzivatel `ha`, `qos=1`, root `zalivka/nadrz`,
- cte heslo ze souboru `~/.zalevaci-nadrz/mqtt_password`,
- kdyz soubor neexistuje, zepta se na heslo a ulozi ho (`chmod 600`),
- nabidne menu pro command topiky (`reboot`, `webapp/start`, `debug/start|stop`, ...).

Volitelne lze prepsat parametry pres promenne prostredi:

```bash
MQTT_HOST=mqtt.home.arpa MQTT_PORT=1883 MQTT_USER=ha TOPIC_ROOT=zalivka/nadrz ./tools/mqtt_cmd_cli.sh
```

Pokud je potreba smazat stare retained command zpravy na brokeru, pouzij:

```bash
./tools/clear_retained_cmds.sh
```

Skript vycisti retained zpravy na `zalivka/nadrz/cmd/*`.

Nize jsou priklady pro `mosquitto_sub` a `mosquitto_pub`.

Nejdriv si nastav promenne (uprav podle sveho brokeru):

```bash
MQTT_HOST=192.168.1.10
MQTT_PORT=1883
MQTT_USER="uzivatel"
MQTT_PASS="heslo"
TOPIC_ROOT="zalivka/nadrz"
```

### Odběr vsech dat ze zarizeni

```bash
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -v -t "$TOPIC_ROOT/#"
```

### Odeslani commandu

`cmd/reboot` (restart zarizeni):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -t "$TOPIC_ROOT/cmd/reboot" -m "1"
```

`cmd/webapp/start` (spusti konfiguracni web):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -t "$TOPIC_ROOT/cmd/webapp/start" -m "1"
```

`cmd/webapp/stop` (zastavi konfiguracni web):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -t "$TOPIC_ROOT/cmd/webapp/stop" -m "1"
```

`cmd/debug/start` (zapne debug publikaci):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -t "$TOPIC_ROOT/cmd/debug/start" -m "1"
```

`cmd/debug/stop` (vypne debug publikaci):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -t "$TOPIC_ROOT/cmd/debug/stop" -m "1"
```

`cmd/ota/start` (spusti OTA z URL):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 -t "$TOPIC_ROOT/cmd/ota/start" -m "http://192.168.1.10:8000/zalevaci-nadrz.bin"
```

`cmd/ota/confirm` (potvrdi novy firmware po rebootu):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 -t "$TOPIC_ROOT/cmd/ota/confirm" -m "1"
```

### Odběr debug vetve

```bash
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -v -t "$TOPIC_ROOT/debug/#"
```


## OTA helper skript

Podrobny navod k OTA je v dokumentu [OTA aktualizace](docs/ota.md).

V projektu je interaktivni helper, ktery:
- spusti jednoduchy HTTP server nad vybranym `.bin` souborem,
- posle MQTT command `cmd/ota/start` s URL na firmware,
- po rebootu se zepta, zda je firmware v poradku,
- pri potvrzeni posle `cmd/ota/confirm`.

Spusteni:

```bash
./tools/ota_flow_cli.sh
```


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