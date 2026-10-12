# MQTT Index

Korenny topic je pevne `voda/septik`. Zdroj pravdy je `firmware/main/mqtt_topics.h` a `firmware/main/mqtt_topics.cpp`.

## Pravidla z kodu

- `mqtt_topic_id_t` v `mqtt_topics.h` musi poradne odpovidat `MQTT_TOPIC_TABLE` v `mqtt_topics.cpp`.
- `static_assert` hlida pocet polozek tabulky proti `COUNT`.
- Publish pouzivej pres `mqtt_publisher_enqueue_*`, ne primo pres `mqtt_publish`, pokud nejde o specialni pripad.
- `mqtt_publisher_task` publikuje availability per topic; `EMPTY` payload znamena unavailable.
- Command topicy jsou `SUBSCRIBE_ONLY` a obsluhuje je `mqtt_commands.cpp`.
- Home Assistant discovery se generuje z tabulky a nazvu v `mqtt_ha_discovery.cpp`.

## Publish topicy

Stav a mereni:

- `stav/zasoba/objem_m3` - objem nadrze podle aktualni tabulky topicu.
- `stav/zasoba/hladina_m` - vyska hladiny.
- `stav/cerpani/prutok_l_min` - aktualni prutok.
- `stav/cerpani/cerpano_celkem_l` - celkove cerpano.
- `stav/teplota/voda` - teplota vody.
- `stav/teplota/vzduch` - teplota vzduchu.
- `stav/tlak/pred_filtrem_bar` - tlak pred filtrem.
- `stav/tlak/za_filtrem_bar` - tlak za filtrem.
- `stav/tlak/rozdil_filtru_bar` - rozdil tlaku na filtru.
- `stav/zanesenost_filtru_percent` - zanesenost filtru podle aktualni tabulky topicu.
- `stav/cerpani/pumpa/stav` - textovy stav cerpadla.

Elektromer:

- `elektro/cerpadlo/napeti_v`
- `elektro/cerpadlo/proud_a`
- `elektro/cerpadlo/vykon_cinny_w`
- `elektro/cerpadlo/vykon_jalovy_var`
- `elektro/cerpadlo/vykon_zdanlivy_va`
- `elektro/cerpadlo/frekvence_hz`
- `elektro/cerpadlo/cosfi`
- `elektro/cerpadlo/energie_cinna_kwh`
- `elektro/cerpadlo/energie_jalova_kvarh`

System:

- `system/status` - LWT/online status.
- `system/boot_mode` - `ota` nebo `normal`.
- `system/ota/event`
- `system/ota/progress`
- `system/reboot_reason`
- `system/reboot_counter`
- `system/last_disconnect_duration_s`

Diagnostika:

- `diag/fw_version`
- `diag/build_timestamp`
- `diag/git_hash`
- `diag/uptime_s`
- `diag/wifi_rssi_dbm`
- `diag/wifi_reconnect_try`
- `diag/wifi_reconnect_success`
- `diag/wifi/connects`
- `diag/wifi/disconnects`
- `diag/mqtt/connects`
- `diag/mqtt/disconnects`
- `diag/mqtt_reconnects`
- `diag/last_mqtt_rc`
- `diag/heap_free_b`
- `diag/heap_min_free_b`
- `diag/nvs_errors`
- `diag/ads1115/status`
- `diag/ads1115/read_errors`
- `diag/ads1115/last_ok_age_s`
- `diag/modbus/kws/status`
- `diag/modbus/kws/timeouts`
- `diag/modbus/kws/crc_errors`
- `diag/modbus/kws/read_errors`
- `diag/modbus/kws/last_ok_age_s`
- `diag/teplota/status`
- `diag/teplota/bus_errors`
- `diag/teplota/crc_errors`
- `diag/teplota/read_errors`
- `diag/teplota/last_ok_age_s`
- `diag/events/sensor_queue_drops`
- `diag/mqtt/publish_queue_drops`
- `diag/teplota_scan`

## Command topicy

- `cmd/reboot` - reboot zarizeni.
- `cmd/webapp` - start/stop konfiguracni webapp.
- `cmd/debug` - start/stop debug rezimu, auto-off po 2 hodinach.
- `cmd/log/level` - payload `tag=LEVEL`, `tag:LEVEL`, `tag LEVEL`, nebo samotny `LEVEL`.
- `cmd/ota/start` - payload URL firmware image.
- `cmd/ota/confirm` - potvrzeni pending OTA image.
- `cmd/teplota/scan` - zapnuti/vypnuti periodickeho DS18B20 scan reportu.

## Pozor na nesoulady

- `firmware/README.md` misty uvadi starsi/novejsi nazvy, napriklad `stav/zasoba/objem_l` a `stav/tlak/zanesenost_filtru_percent`; aktualni kod ma `stav/zasoba/objem_m3` a `stav/zanesenost_filtru_percent`.
- Pri zmene topicu zkontroluj i `mqtt_ha_discovery.cpp`, dokumentaci a helper `firmware/tools/zalevaci`.
