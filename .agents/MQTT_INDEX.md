# Mapa MQTT

Ověřeno proti kódu 12. 10. 2026. Zdrojem pravdy jsou [mqtt_topics.h](../firmware/main/mqtt_topics.h) a [mqtt_topics.cpp](../firmware/main/mqtt_topics.cpp). Kořen je pevně `voda/septik`; cesty v tabulkách jsou relativní k němu. Registr má **67 topiců: 60 publikačních a 7 příkazových**.

## Publikace a dostupnost

- `mqtt_topic_id_t` musí odpovídat pořadí `MQTT_TOPIC_TABLE`; počet hlídá `static_assert`.
- Běžná aplikační publikace používá `mqtt_publisher_enqueue_*`: fronta 32 položek, cache poslední hodnoty a odesílání změn. Stejná příchozí hodnota se znovu odešle po 60 s od poslední publikace; nejde o samostatný časovač publikující bez nových událostí.
- Po MQTT reconnectu se cache odešle znovu; discovery a boot diagnostika se publikují při přechodu do `SYS_NET_MQTT_READY`.
- `EMPTY` publikační událost pošle dostupnost `offline` a ponechá poslední retained hodnotu. Platná hodnota pošle `online` a vlastní payload.
- Pro `stav/X` je dostupnost `availability/X`; například `voda/septik/availability/zasoba/objem_m3`. Pro ostatní topicy se přidává `/availability` na konec celé cesty.
- `system/status` má vlastní retained LWT (`offline`) a online stav. HA entity kromě tohoto topicu nyní používají individuální dostupnost; discovery nekombinuje individuální dostupnost s globálním LWT. Celkový výpadek zařízení tedy není totožný s chybou konkrétního senzoru.
- Publikační wrapper používá QoS 1. Přímá debug publikace používá QoS 0 bez retain; DS18B20 scan se odesílá přímo přes wrapper bez 128B cache publisheru.
- Číselné hodnoty publisher serializuje na šest desetinných míst. Textový buffer má 128 B včetně ukončení; MQTT command payloady se rovněž zkracují na nejvýše 127 znaků.

## Publikační topicy

| Cesta | Payload | QoS | Retain |
| --- | --- | --- | --- |
| `stav/zasoba/objem_m3` | number | 1 | ano |
| `stav/zasoba/hladina_m` | number | 1 | ano |
| `stav/cerpani/prutok_l_min` | number | 1 | ano |
| `stav/cerpani/cerpano_celkem_l` | number | 1 | ano |
| `stav/teplota/voda` | number | 1 | ano |
| `stav/teplota/vzduch` | number | 1 | ano |
| `stav/tlak/pred_filtrem_bar` | number | 1 | ano |
| `stav/tlak/za_filtrem_bar` | number | 1 | ano |
| `stav/tlak/rozdil_filtru_bar` | number | 1 | ano |
| `stav/zanesenost_filtru_percent` | number | 1 | ano |
| `stav/cerpani/pumpa/stav` | text | 1 | ano |
| `elektro/cerpadlo/napeti_v` | number | 1 | ano |
| `elektro/cerpadlo/proud_a` | number | 1 | ano |
| `elektro/cerpadlo/vykon_cinny_w` | number | 1 | ano |
| `elektro/cerpadlo/vykon_jalovy_var` | number | 1 | ano |
| `elektro/cerpadlo/vykon_zdanlivy_va` | number | 1 | ano |
| `elektro/cerpadlo/frekvence_hz` | number | 1 | ano |
| `elektro/cerpadlo/cosfi` | number | 1 | ano |
| `elektro/cerpadlo/energie_cinna_kwh` | number | 1 | ano |
| `elektro/cerpadlo/energie_jalova_kvarh` | number | 1 | ano |
| `system/status` | text | 1 | ano |
| `system/boot_mode` | text | 1 | ano |
| `system/ota/event` | text | 1 | ne |
| `system/ota/progress` | number | 1 | ne |
| `system/reboot_reason` | text | 1 | ano |
| `system/reboot_counter` | number | 1 | ano |
| `system/last_disconnect_duration_s` | number | 1 | ano |
| `diag/fw_version` | text | 1 | ano |
| `diag/build_timestamp` | text | 1 | ano |
| `diag/git_hash` | text | 1 | ano |
| `diag/uptime_s` | number | 1 | ano |
| `diag/wifi_rssi_dbm` | number | 1 | ano |
| `diag/wifi_reconnect_try` | number | 1 | ano |
| `diag/wifi_reconnect_success` | number | 1 | ano |
| `diag/wifi/connects` | number | 1 | ano |
| `diag/wifi/disconnects` | number | 1 | ano |
| `diag/mqtt/connects` | number | 1 | ano |
| `diag/mqtt/disconnects` | number | 1 | ano |
| `diag/mqtt_reconnects` | number | 1 | ano |
| `diag/last_mqtt_rc` | number | 1 | ano |
| `diag/heap_free_b` | number | 1 | ano |
| `diag/heap_min_free_b` | number | 1 | ano |
| `diag/nvs_errors` | number | 1 | ano |
| `diag/ads1115/status` | text | 1 | ano |
| `diag/ads1115/read_errors` | number | 1 | ano |
| `diag/ads1115/last_ok_age_s` | number | 1 | ano |
| `diag/modbus/kws/status` | text | 1 | ano |
| `diag/modbus/kws/timeouts` | number | 1 | ano |
| `diag/modbus/kws/crc_errors` | number | 1 | ano |
| `diag/modbus/kws/read_errors` | number | 1 | ano |
| `diag/modbus/kws/last_ok_age_s` | number | 1 | ano |
| `diag/teplota/status` | text | 1 | ano |
| `diag/teplota/bus_errors` | number | 1 | ano |
| `diag/teplota/crc_errors` | number | 1 | ano |
| `diag/teplota/read_errors` | number | 1 | ano |
| `diag/teplota/last_ok_age_s` | number | 1 | ano |
| `diag/events/sensor_queue_drops` | number | 1 | ano |
| `diag/mqtt/publish_queue_drops` | number | 1 | ano |
| `diag/teplota_scan` | json | 1 | ne |

## Příkazy

Všechny příkazy jsou `SUBSCRIBE_ONLY`, QoS 1, bez retain. Firmware přijaté retained příkazy ignoruje.

| Cesta | Payload a chování |
| --- | --- |
| `cmd/reboot` | `1`, `true`, `on`, `yes`, `now` nebo prázdný payload vyvolá restart. |
| `cmd/webapp` | `on` / `off` (také běžné bool varianty); zapne/vypne konfigurační web. |
| `cmd/debug` | `on` / `off`; debug MQTT se při zapnutí automaticky vypne za 2 hodiny. |
| `cmd/log/level` | `TAG=LEVEL`, `TAG:LEVEL`, `TAG LEVEL` nebo samotný `LEVEL`; úrovně NONE až VERBOSE, čísla 0–5. |
| `cmd/ota/start` | URL obrazu `http://…` nebo `https://…`; praktický limit je 127 znaků command bufferu. |
| `cmd/ota/confirm` | Pravdivý nebo prázdný payload potvrzuje běžící pending OTA image. |
| `cmd/teplota/scan` | Pravdivý payload zapne pravidelný report; `0` / `off` vypne. Prázdný payload zapíná. |

## Jednotky a význam

- `stav/zasoba/objem_m3` je **m³**; `hladina_m` je m. Starý `objem_l` není v registru.
- Průtok je l/min, celkové čerpání je l. Nulový průtok je platná hodnota.
- Tlak je bar, zanesení filtru %; správná cesta je `stav/zanesenost_filtru_percent`, bez mezisegmentu `tlak`.
- `stav/cerpani/pumpa/stav` je `bezi`, `stoji` nebo `chyba`, odvozené z činného výkonu (hranice 5 W). Není to povel pro relé.
- Energie elektroměru přichází do `state_manager` ve Wh/varh a před publikací se dělí 1000 na kWh/kvarh.
- `system/reboot_counter` znamená počet startů, ukládá se do `sys_meta/boot_count_v2` a zvyšuje se jednou při startu `state_manager`.
- `system/reboot_reason` je textové číslo `esp_reset_reason_t`; `last_disconnect_duration_s` měří výpadek konektivity po ztrátě stavu MQTT ready.
- `diag/last_mqtt_rc` je registrovaný, ale proměnná ve `state_manager` je nyní inicializovaná na 0 a bez další aktualizace. Nevydávej jej za skutečný poslední chybový kód klienta.

## Home Assistant a debug

- `mqtt_ha_discovery.cpp` publikuje retained konfigurace pro publikační topicy pod `homeassistant/<component>/<object_id>/config` (object ID je `voda_septik_` + normalizovaná celá cesta topicu; ID zařízení je `voda_septik_esp32`).
- `system/status` je connectivity binary sensor; číselné jednotky a třídy se odvozují z cest. `diag/*` a většina `system/*` jsou diagnostické entity.
- Průtok již nemá `expire_after=30`: nedostupnost se řídí availability.
- `diag/teplota_scan` je JSON se seznamem `found`; HA konfigurace zobrazuje jejich počet a JSON atributy. Přímý scan publish nyní nevytváří odpovídající individuální availability, přestože ji discovery nastavuje; samotné přijetí reportu proto nezaručuje dostupnou HA entitu.
- Debug není součástí centrálního registru. Aktivní větve jsou `debug/prutok`, `debug/teplota`, `debug/tlak`, `debug/tlak_cfg`, `debug/filtr`, `debug/zasoba` a `debug/cfg/zasoba`. Zprávy obsahují zdrojový soubor, řádek a funkci a mají buffer 192 B.
- Při změně názvů topiců řeš i staré retained discovery konfigurace na brokeru; nová konfigurace sama neodstraní starou entitu.

Postupy: [provozní README](../firmware/README.md), [logování](../firmware/docs/logging.md), [OTA](../firmware/docs/ota.md).

## Identifikace verze a nasazení

`diag/fw_version` pochází z `esp_app_desc_t.version`, nyní odvozené z Git tagů. `diag/git_hash` však aktuálně publikuje pouze prefix této verze před první pomlčkou a u tagovaných buildů není hash. Pro určení provozovaného commitu používej úplnou verzi a evidenci nasazení; postup je v [pravidlech vývoje a nasazení](../firmware/docs/vyvoj-a-nasazeni.md).

## Nové měření filtru

`stav/filtr/mereni`: JSON, QoS v registru 1, retain ano; publisher používá společný MQTT wrapper jako ostatní topicy. Payload: `valid`, `boot_count`, `measurement_id`, `pressure_diff_bar`, `flow_l_min`, `pressure_diff_stddev_bar`, `sample_count`. Bez absolutního času a procent. Průtok je průměr při měření, nikoli aktuální průtok. Záporné Δp zůstává podepsané.

Před prvním měřením po restartu `valid=false`, ID 0, hodnoty null, počet 0. Po každém přijatém 10s okně ID vzroste i při stejné hodnotě. Bez vhodného průtoku a při vadných datech zůstává poslední výsledek; publisher jej obnovuje po 60 s při nových tlakových událostech a po reconnectu. HA discovery vystavuje dvojici boot:measurement jako stav a JSON jako atributy. HA trvalý výsledek ignoruje invalid zprávy, duplicity a starší dvojice, čas přijetí ukládá pouze u nové dvojice.

Starý `stav/zanesenost_filtru_percent` zůstává v registru kvůli stabilitě ID, firmware jej již nepublikuje a při discovery odstraní jeho starou retained konfiguraci i hodnotu. Nahrazuje jej HA package `firmware/ha/homeassistant/packages/filter.yaml`. Instantní `stav/tlak/rozdil_filtru_bar` je nadále okamžitá diagnostika; pro zanesení používej výhradně nové filtrované měření. Textová kapacita publisheru zvýšena na 384 B.
