# Diagnostika firmware

Ověřeno proti aktivním zdrojům 12. 10. 2026. Diagnostika rozlišuje chybu platformy, chybu měření, síťový výpadek a explicitně zapnuté ladění.

## Tvrdé chyby

`APP_ERROR_CHECK` hlásí unikátní `Exxx`, zavolá handler pro log/TM1637 a následně `ESP_ERROR_CHECK`. Používá se zejména při inicializaci základní platformy, registraci konfigurace, vytváření tasků/front a obsluze watchdogu. Úplná inventura 141 aktivních kódů je v [error-codes.md](error-codes.md).

Běžný výpadek Wi-Fi/MQTT není sám o sobě tvrdou chybou. ADS1115, teploty a elektroměr mají vlastní měkkou diagnostiku; nepřítomný LCD deaktivuje zobrazování místo ukončení aplikace. Průtok vyžaduje správný flash oddíl a při jeho inicializační chybě používá tvrdou chybu.

Po chybovém resetu aplikace ukazuje rychlou LED sekvenci přibližně 10 s. Watchdog je nastavený na 5 s a `trigger_panic=false`; nepřisuzuj každému watchdog hlášení automatický restart.

## Měření a nedostupnost

`state_manager` převádí chybné teploty, hladinu/objem, tlaky a elektrické veličiny na `EMPTY` MQTT události. Publisher tím označí individuální availability jako `offline`, nikoli nulovou hodnotu. Platná hodnota obnoví `online`. Starý retained payload může zůstat na brokeru, HA jej má posuzovat podle dostupnosti.

- Průtok 0 je platný stav po 3 s bez impulsu. Současná průtoková větev pro neplatné float hodnoty nepoužívá stejnou EMPTY větev jako ostatní měření; jde o známý rozdíl v implementaci.
- Stav pumpy je `bezi` / `stoji` podle hranice 5 W, při neplatném výkonu `chyba`.
- `system/status` používá globální MQTT LWT `offline`; HA discovery však u jednotlivých entit aktuálně nespojuje globální LWT s individuální availability. Celkový síťový výpadek a porucha čidla tedy mají různé mechanismy.
- Fronty uchovávají poslední vzorky ADC pomocí overwrite, aby pomalý konzument nečetl nahromaděné staré vzorky.

## Provozní diagnostika MQTT

| Oblast | Topicy pod `voda/septik/` |
| --- | --- |
| Verze a paměť | `diag/fw_version`, `diag/git_hash`, `diag/build_timestamp`, `diag/uptime_s`, `diag/heap_free_b`, `diag/heap_min_free_b` |
| Wi-Fi/MQTT | `diag/wifi_rssi_dbm`, `diag/wifi_reconnect_try`, `diag/wifi_reconnect_success`, `diag/wifi/connects`, `diag/wifi/disconnects`, `diag/mqtt/connects`, `diag/mqtt/disconnects`, `diag/mqtt_reconnects` |
| NVS | `diag/nvs_errors` |
| ADS1115 | `diag/ads1115/status`, `diag/ads1115/read_errors`, `diag/ads1115/last_ok_age_s` |
| Modbus | `diag/modbus/kws/status`, `timeouts`, `crc_errors`, `read_errors`, `last_ok_age_s` pod stejnou větví |
| Teploty | `diag/teplota/status`, `bus_errors`, `crc_errors`, `read_errors`, `last_ok_age_s` pod stejnou větví |
| Fronty | `diag/events/sensor_queue_drops`, `diag/mqtt/publish_queue_drops` |
| ROM scan | `diag/teplota_scan` (JSON, jen při zapnutém scanu) |

Status měřicích modulů je `ok` / `chyba`. Čítače a stáří posledního měření pocházejí ze snapshotů modulů. Diagnostika se publikuje při síťových událostech/telemetrii; telemetrie je plánovaná nejvýše po 10 s nebo při změně stavu. Opakované senzorické warningy jsou omezené přibližně na jeden log za 60 s.

`diag/last_mqtt_rc` je v registru, ale runtime proměnná není aktualizovaná a zůstává 0. `diag/esp_vcc_mv` v aktuálním registru není.

## Boot metadata

`system/reboot_counter` je počet startů (nikoli reconnectů), uložený jako `sys_meta/boot_count_v2`. Zvyšuje se jednou při startu `state_manager`. `system/reboot_reason` je číselný `esp_reset_reason_t`; `system/boot_mode` rozlišuje pending OTA (`ota`) a běžný obraz (`normal`).

`restart_info.cpp` vrací chyby NVS volajícímu; `state_manager` loguje chybu a zvyšuje `diag/nvs_errors`, nikoli automaticky shazuje firmware. Pokud systémový čas není platný, čas restartu se ukládá jako 0. Aktivní zdroje neobsahují vlastní SNTP inicializaci.

## Debug

Explicitní `cmd/debug=on` zapíná podrobnou větev `debug/*` a automaticky se vypne po dvou hodinách. RAW hodnoty, kalibrace a filtry patří sem; dlouhodobé čítače chyb patří do `diag/*`. Sériové log levely se nastavují samostatně a jejich maximum sestavení je nyní INFO.

Další informace: [logování](logging.md), [MQTT registr](../../.agents/MQTT_INDEX.md), [konfigurace](../../.agents/CONFIG_INDEX.md).

## Identifikace verze a nasazení

`diag/fw_version` pochází z `esp_app_desc_t.version`, nyní odvozené z Git tagů. `diag/git_hash` však aktuálně publikuje pouze prefix této verze před první pomlčkou a u tagovaných buildů není hash. Pro určení provozovaného commitu používej úplnou verzi a evidenci nasazení; postup je v [pravidlech vývoje a nasazení](vyvoj-a-nasazeni.md).
