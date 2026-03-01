# voda-septik

## Dokumentace

- [OTA aktualizace](docs/ota.md)
- [Logovani a log levely](docs/logging.md)

## Kalibrace tlaku (webapp)

Modul tlaku (`main/tlak.cpp`) meri dve 4-20 mA cidla (pred filtrem / za filtrem) a hodnoty mapuje pres kalibracni polozky dostupne v konfiguracni webapp.

- `tlk_b_raw_4ma` (default `745`) - ADC RAW hodnota pred filtrem odpovidajici 4 mA.
- `tlk_b_raw_20ma` (default `3722`) - ADC RAW hodnota pred filtrem odpovidajici 20 mA.
- `tlk_b_p_min` (default `0.0`) - tlak pred filtrem [bar] pro 4 mA.
- `tlk_b_p_max` (default `10.0`) - tlak pred filtrem [bar] pro 20 mA.
- `tlk_a_raw_4ma` (default `745`) - ADC RAW hodnota za filtrem odpovidajici 4 mA.
- `tlk_a_raw_20ma` (default `3722`) - ADC RAW hodnota za filtrem odpovidajici 20 mA.
- `tlk_a_p_min` (default `0.0`) - tlak za filtrem [bar] pro 4 mA.
- `tlk_a_p_max` (default `10.0`) - tlak za filtrem [bar] pro 20 mA.
- `tlk_ema_alpha` (default `0.55`) - spolecny EMA filtr pro obe cidla.
- `tlk_hyst_bar` (default `0.02`) - spolecna hystereze [bar] pro obe cidla.
- `tlk_sample_ms` (default `100`) - spolecna perioda mereni [ms] pro obe cidla.
- `tlk_round_dec` (default `2`) - spolecny pocet desetinnych mist publikovaneho tlaku.
- `tlk_dp_100` (default `1.0`) - rozdil tlaku [bar], ktery odpovida 100 % zanesenosti filtru.

Publikovane MQTT hodnoty:
- `stav/tlak/pred_filtrem_bar`
- `stav/tlak/za_filtrem_bar`
- `stav/tlak/rozdil_filtru_bar`
- `stav/tlak/zanesenost_filtru_percent`

Doporuceny postup prvotniho nastaveni:
1. Pro kazde cidlo zvlast zmerit referencni body 4 mA/20 mA a upravit `tlk_b_*` a `tlk_a_*`.
2. Nastavit realny rozsah cidel (vychozi 0-10 bar) pomoci `tlk_b_p_min/max` a `tlk_a_p_min/max`.
3. Podle dynamiky provozu doladit spolecnou filtraci (`tlk_ema_alpha`, `tlk_hyst_bar`, `tlk_sample_ms`, `tlk_round_dec`).
4. Podle provozu filtru doladit `tlk_dp_100`, aby procenta zaneseni odpovidala realnemu stavu.

Kalibracni checklist (rychly postup):
- [ ] Zapnout debug pro tag `TLAK` a overit, ze hodnoty `raw` nekolisaji nestandardne.
- [ ] Pri referencnim bode 4 mA/20 mA nastavit kalibraci pred filtrem (`tlk_b_*`).
- [ ] Pri referencnim bode 4 mA/20 mA nastavit kalibraci za filtrem (`tlk_a_*`).
- [ ] Overit, ze tlaky odpovidaji manometru, pripadne doladit `tlk_b_p_min/max` a `tlk_a_p_min/max`.
- [ ] Doladit spolecnou filtraci (`tlk_ema_alpha`, `tlk_hyst_bar`, `tlk_sample_ms`, `tlk_round_dec`) podle pozadovane dynamiky.
- [ ] Pri znamem zanesenem stavu filtru doladit `tlk_dp_100`.

## Kalibrace objemu (webapp)

Modul objemu (`main/zasoba.cpp`) interne zmeri vysku hladiny (kalibrace `lvl_*`) a do MQTT/HA publikuje objem i vysku hladiny.

Kalibracni polozky:
- `lvl_raw_min` (default `540`) - ADC RAW hodnota odpovidajici minimalni hladine.
- `lvl_raw_max` (default `950`) - ADC RAW hodnota odpovidajici maximalni hladine.
- `lvl_h_min` (default `0.0`) - vyska hladiny [m] pro `lvl_raw_min`.
- `lvl_h_max` (default `0.290`) - vyska hladiny [m] pro `lvl_raw_max`.
- `tank_area_m2` (default `5.4`) - pudorysna plocha nadrze [m²] (pro nadrz 2 × 2.7 m).

Pouzity prepocet:
- vyska z kalibrace RAW: linearni mapovani mezi `lvl_raw_min/lvl_raw_max` a `lvl_h_min/lvl_h_max`
- objem: `objem_l = vyska_m * tank_area_m2 * 1000`

Publikovany MQTT vystup:
- `stav/zasoba/objem_l` [l]
- `stav/zasoba/hladina_m` [m]

Prakticky postup:
1. Nejdriv zkalibrovat vysku (`lvl_*`) podle realnych referencnich bodu.
2. Nastavit skutecnou plochu nadrze v `tank_area_m2`.
3. Overit v HA, ze `stav/zasoba/objem_l` a `stav/zasoba/hladina_m` odpovidaji realnemu stavu nadrze.

## Struktura mqtt topiků.

Poznamka (migrace):
- Konfiguracni polozka `mqtt_topic` byla odstranena.
- Korenny topic je nyni pevne dany v registru topiku (`main/mqtt_topics.*`) jako `voda/septik`.

```
voda/septik                         Voda v nadrzi (byvaly septik), vcetne dalsich budoucih pouziti (napr. bazen)

├── stav/
│    ├── teplota/
│    │    ├── voda                  [°C] Aktuální teplota vody v nádrži. HA: sensor (device_class: temperature)
│    │    └── vzduch                [°C] Aktuální teplota vzduchu v šachtě u potrubí v blízkosti tlakové nádoby. HA: sensor (device_class: temperature)
│    ├── zasoba/
│    │    ├── objem_l               [l] Aktuální objem vody v nádrži. HA: sensor (state_class: measurement)
│    │    └── hladina_m             [m] Aktuální výška hladiny vody v nádrži. HA: sensor (state_class: measurement)
│    ├── cerpani/
│    │    ├── prutok_l_min          [l/min] Aktuální průtok vody. HA: sensor (state_class: measurement)
│    │    ├── cerpano_celkem_l      [l] Celkové vyčerpané množství vody od počátku. HA: sensor (state_class: total_increasing)
│    │    └── pumpa/
│    │         ├── bezi             [bool] Příznak ano/ne (0/1), zda čerpadlo běží. HA: binary_sensor (device_class: running)
│    │         ├── vykon_cinny_w    [W] Aktuální činný výkon. HA: sensor (device_class: power)
│    │         ├── jalovy_vykon_var [var] Aktuální jalový výkon. HA: sensor
│    │         ├── cosfi            [-] Aktuální účiník. HA: sensor
│    │         ├── proud_a          [A] Aktuální proud protékaný čerpadlem. HA: sensor (device_class: current)
│    │         ├── napeti_v         [V] Aktuální napětí na čerpadle. HA: sensor (device_class: voltage)
│    │         ├── energie_cinna_kwh [kWh] Celkové množství spotřebované činné energie. HA: sensor (device_class: energy, state_class: total_increasing)
│    │         └── energie_jalova_kvarh [kvarh] Celkové množství spotřebované jalové energie. HA: sensor (state_class: total_increasing)
│    └── tlak/
│         ├── pred_filtrem_bar      [bar] Aktuální tlak vody před filtrem. HA: sensor (device_class: pressure)
│         ├── za_filtrem_bar        [bar] Aktuální tlak vody za filtrem. HA: sensor (device_class: pressure)
│         ├── rozdil_filtru_bar     [bar] Rozdíl tlaku před a za filtrem. HA: sensor (state_class: measurement)
│         └── zanesenost_filtru_percent [%] Odhad zanesení filtru v procentech (odvozeno z rozdílu tlaků). HA: sensor
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
│    ├── nvs_errors                 [count] Počet chyb při práci s NVS. HA: sensor (state_class: total_increasing)
│    └── teplota_scan               [json] Prubezny report nalezenych DS18B20 adres, teplot a mapovani na konfiguraci
│
├── cmd/
│    ├── reboot                     [-] Reboot zařízení, čímž se vypnou všechny debugy. HA: button
│    ├── webapp/start               [-] Nastartování konfigurační aplikace. Implicitně je vypnutá, po startu se sama po 2 hodinách vypne. HA: button
│    ├── webapp/stop                [-] Vypnutí webové aplikace. HA: button
│    ├── debug/start                [-] Nastartování debug režimu, začnou se produkovat debug data. HA: button
│    ├── debug/stop                 [-] Zastavení debug režimu. HA: button
│    ├── log/level                  [text] Nastaveni log levelu per tag (payload: tag=level)
│    ├── ota/start                  [url] URL na binarni obraz firmware (http/https)
│    ├── ota/confirm                [-] Rucni potvrzeni nahraneho firmware po overeni funkcnosti
│    └── teplota/scan               [bool] Zapnuti/vypnuti periodickeho skenovani DS18B20 adres (1=true, 0=false)
│
└── debug


## Ovladani pres Mosquitto CLI

Podrobny navod k logovani je v dokumentu [Logovani a log levely](docs/logging.md).

Pro pohodlne ovladani je v projektu jednotny interaktivni skript:

```bash
zalevaci
```

Pokud jeste nemas aktivni `direnv`, udelej jednorazove:

```bash
direnv allow
```

Potom bude `tools/` automaticky v `PATH` pri vstupu do adresare projektu a prikaz `zalevaci` bude dostupny primo.

Napoveda a neinteraktivni rezim:

```bash
zalevaci --help
```

Priklady (vhodne pro skripty/CI):

```bash
# command publish
zalevaci cmd reboot

# log level
zalevaci log --tag mqtt_cmd --level DEBUG

# vycisteni retained cmd topiku
zalevaci clear-retained

# vypis nalezenych log tagu
zalevaci tags
```

Skript:
- pouziva default parametry: `mqtt.home.arpa:1883`, uzivatel `ha`, `qos=1`, root `voda/septik`,
- cte heslo ze souboru `~/.voda-septik/mqtt_password`,
- kdyz soubor neexistuje, zepta se na heslo a ulozi ho (`chmod 600`),
- nabidne menu pro command topiky (`reboot`, `webapp/start`, `debug/start|stop`, ...),
- umi nastavit log level (vcetne volby tagu nalezenych ve zdrojacich),
- umi zapnout/vypnout scan teplotnich cidel (`cmd/teplota/scan`),
- umi vycistit retained command topiky,
- umi projit OTA flow (HTTP server + `cmd/ota/start` + potvrzeni).

Volitelne lze prepsat parametry pres promenne prostredi:

```bash
MQTT_HOST=mqtt.home.arpa MQTT_PORT=1883 MQTT_USER=ha TOPIC_ROOT=voda/septik zalevaci
```

Pokud je potreba smazat stare retained command zpravy na brokeru, pouzij volbu `Vycistit retained cmd/*` v menu skriptu `zalevaci`.

Nize jsou priklady pro `mosquitto_sub` a `mosquitto_pub`.

Nejdriv si nastav promenne (uprav podle sveho brokeru):

```bash
MQTT_HOST=192.168.1.10
MQTT_PORT=1883
MQTT_USER="uzivatel"
MQTT_PASS="heslo"
TOPIC_ROOT="voda/septik"
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

`cmd/webapp` (spusti konfiguracni web):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -t "$TOPIC_ROOT/cmd/webapp" -m "on"
```

`cmd/webapp` (zastavi konfiguracni web):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -t "$TOPIC_ROOT/cmd/webapp" -m "off"
```

`cmd/debug` (zapne debug publikaci):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -t "$TOPIC_ROOT/cmd/debug" -m "on"
```

`cmd/debug` (vypne debug publikaci):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -t "$TOPIC_ROOT/cmd/debug" -m "off"
```

`cmd/ota/start` (spusti OTA z URL):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 -t "$TOPIC_ROOT/cmd/ota/start" -m "http://192.168.1.10:8000/voda-septik.bin"
```

`cmd/ota/confirm` (potvrdi novy firmware po rebootu):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 -t "$TOPIC_ROOT/cmd/ota/confirm" -m "1"
```

`cmd/teplota/scan` (zapne periodicky scan DS18B20 adres a report na `diag/teplota_scan`):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 -t "$TOPIC_ROOT/cmd/teplota/scan" -m "1"
```

`cmd/teplota/scan` (vypne scan):

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 -t "$TOPIC_ROOT/cmd/teplota/scan" -m "0"
```

### Odběr debug vetve

```bash
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -v -t "$TOPIC_ROOT/debug/#"
```


## OTA helper skript

Podrobny navod k OTA je v dokumentu [OTA aktualizace](docs/ota.md).

V projektu je interaktivni helper (soucast `zalevaci`), ktery:
- spusti jednoduchy HTTP server nad vybranym `.bin` souborem,
- posle MQTT command `cmd/ota/start` s URL na firmware,
- po rebootu se zepta, zda je firmware v poradku,
- pri potvrzeni posle `cmd/ota/confirm`.

Spusteni:

```bash
zalevaci
```

V menu zvol `OTA flow`.


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