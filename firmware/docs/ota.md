# OTA aktualizace firmware

Aktuální helper je `firmware/tools/zalevaci`, nikoli odstraněný `ota_flow_cli.sh`. Všechny příklady spouštěj z `firmware/`, po aktivaci ESP-IDF v5.5.2. Stav ověřen proti kódu 12. 10. 2026.

## Interaktivní postup

1. Sestav obraz pomocí `idf.py build`.
2. Spusť `./tools/zalevaci` a vyber `OTA flow`.
3. Vyber broker, obraz `build/voda-septik.bin` a IP počítače dostupnou z ESP; výchozí HTTP port je 8000.
4. Helper spustí HTTP server, pošle `cmd/ota/start` a přes `mosquitto_sub` čeká na `system/boot_mode=ota` (výchozí timeout 300 s).
5. Po startu nového firmware ověř měření, MQTT a displeje. Potvrzení otázky `y` / `yes` pošle `cmd/ota/confirm`.
6. Po otázce helper vždy pošle restart: potvrzený obraz zůstane aktivní; bez potvrzení se při dalším bootu uplatní rollback. Nakonec ukončí HTTP server.

Připojení lze změnit globálními parametry před subcommand nebo menu, například `./tools/zalevaci --host BROKER --port 1883`. Nástroj potřebuje `mosquitto_pub` i `mosquitto_sub`; heslo načítá z `~/.voda-septik/mqtt_password`.

## Neinteraktivní použití

```sh
./tools/zalevaci ota --help
./tools/zalevaci --host BROKER ota --bin build/voda-septik.bin --host-ip IP_POCITACE --no-reboot
```

Nahraď `BROKER` a `IP_POCITACE` skutečnými adresami. Parametry:

- `--http-port` mění HTTP port (výchozí 8000).
- `--timeout` mění čekání na pending OTA boot (výchozí 300 s).
- `--confirm` potvrzuje obraz automaticky po zjištění `boot_mode=ota`; samo neověřuje senzory ani provozní funkčnost.
- Bez `--confirm` se obraz nepotvrdí. Bez `--no-reboot` helper po čekání posílá další restart, a tím u nepotvrzeného obrazu vyvolá návrat na předchozí verzi.
- S `--no-reboot` lze ověřit zařízení a ručně poslat potvrzení nebo restart přes `cmd`.

## Ruční postup

Připrav obraz na HTTP serveru dostupném z ESP. Z běžícího zařízení spusť OTA:

```sh
./tools/zalevaci cmd ota/start http://IP_SERVERU:8000/voda-septik.bin
```

Sleduj `voda/septik/system/ota/#` a `voda/septik/system/boot_mode`. Po ověření nového obrazu:

```sh
./tools/zalevaci cmd ota/confirm 1
./tools/zalevaci cmd reboot 1
```

Příkazy posílej bez retain. URL v aktuálním command handleru musí vejít do 127 znaků (vlastní OTA manager má větší buffer). Používej adresu dostupnou ze sítě ESP; `localhost` počítače z ESP dostupný není.

## Implementace a rollback

- `mqtt_commands.cpp` předává URL do `ota_manager_start_from_url`.
- `ota_manager.cpp` akceptuje prefix HTTP/HTTPS, vyžaduje HTTP status 200 a zapisuje sekvenčně do následujícího OTA slotu. Pro HTTPS nemá výslovně nastavený certifikát ani certificate bundle; samotná akceptace URL nedokládá funkční HTTPS přenos.
- `partitions.csv` obsahuje `ota_0` a `ota_1`, každý 0x180000 B. Aktualizuje se obraz aplikace, nikoli tabulka oddílů, bootloader nebo provozní data.
- Události zahrnují `start`, `download`, `rebooting`, `confirmed`, `confirm_not_needed` a chyby jednotlivých kroků. Progress je 0–100 %, bez retain; při známé délce se hlásí přibližně po 5 procentních bodech.
- Po úspěšném zápisu se změní boot slot a zařízení restartuje.
- `state_manager` publikuje `system/boot_mode=ota` pro `ESP_OTA_IMG_PENDING_VERIFY`, jinak `normal`.
- `cmd/ota/confirm` volá `esp_ota_mark_app_valid_cancel_rollback`. Není implementované automatické potvrzení podle senzorů ani vlastní časovač, který by po určité době sám restartoval nepověřený obraz.
- Rollback je zapnutý v `sdkconfig` i `sdkconfig.defaults`; musí jej podporovat skutečně nahraný bootloader. Před zásahy do flash zálohuj NVS a datové oddíly, pokud je chceš zachovat.

Související: [obnova prostředí](../../README.md), [MQTT registr](../../.agents/MQTT_INDEX.md).
