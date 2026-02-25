# OTA aktualizace firmware

## Jak to uzivatelsky pouzivat

Nejjednodussi postup je pouzit interaktivni helper:

1. Vygeneruj firmware (`idf.py build`).
2. Spust skript:

```bash
./tools/ota_flow_cli.sh
```

3. Ve skriptu vyber:
   - MQTT broker a prihlaseni,
   - soubor firmware (`build/zalevaci-nadrz.bin`),
   - sitovou adresu, odkud si ESP muze stahnout `.bin`.
4. Skript spusti lokalni HTTP server, posle `cmd/ota/start` a vypisuje OTA eventy.
5. Po rebootu noveho firmware skript pocka na signal bootu OTA image (`system/boot_mode = ota`) a teprve potom se zepta na potvrzeni.
6. Pokud vse funguje, potvrd novy firmware (skript posle `cmd/ota/confirm`).
7. Po vyhodnoceni potvrzovaci otazky skript vzdy pocka 0.5 s a posle `cmd/reboot` (jak pri potvrzeni, tak i bez potvrzeni).

Rucni postup bez helper skriptu:

```bash
# start OTA
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 \
  -t "zalivka/nadrz/cmd/ota/start" \
  -m "http://IP_SERVERU:8000/zalevaci-nadrz.bin"

# sledovani OTA eventu
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -v \
  -t "zalivka/nadrz/system/ota/#" -t "zalivka/nadrz/system/boot_mode"

# potvrzeni po uspesnem nabootovani a overeni funkcnosti
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 \
  -t "zalivka/nadrz/cmd/ota/confirm" -m "1"
```

## Jak to funguje uvnitr

- OTA se spousti MQTT commandem `zalivka/nadrz/cmd/ota/start`.
- Command handler v `main/mqtt_commands.cpp` vola OTA manager (`ota_manager_start_from_url`).
- OTA manager (`main/ota_manager.cpp`) stahuje image pres HTTP(S), zapisuje ji do OTA partition a po dokonceni restartuje zarizeni.
- Pri behu publikuje stav do:
  - `zalivka/nadrz/system/ota/event`
  - `zalivka/nadrz/system/ota/progress`
- Po startu noveho firmware se publikuje `zalivka/nadrz/system/boot_mode`:
  - `ota` = firmware je ve stavu pending verify
  - `normal` = standardni, potvrzeny start
- Potvrzeni probiha commandem `zalivka/nadrz/cmd/ota/confirm`, ktery vola `ota_manager_confirm_running_firmware()`.
- Pokud firmware neni potvrzen a je aktivni rollback, bootloader vrati predchozi potvrzenou verzi.

## Poznamky k provozu

- OTA URL musi byt dostupna ze stejne site jako ESP.
- Pro rollback je potreba mit zapnuty bootloader rollback (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`) a zmenu mit opravdu nahranou do zarizeni.
- Doporuceny postup je vzdy potvrzovat firmware az po realnem testu funkcnosti (senzory, MQTT, ovladani).
