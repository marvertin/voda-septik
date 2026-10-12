# Project Index

Tento index je orientacni mapa repozitare pro rychle navazani prace.

## Koren

- `README.md` - obnova projektu, instalace prostredi, sestaveni a zaloha provoznich dat.

- `firmware/` - ESP-IDF firmware pro zarizeni voda/septik.
- `hardware/` - KiCad projekt `Voda septik`, schemata, PCB, vlastni footprinty a symboly.
- `.github/workflows/build.yml` - CI build firmware pri zmenach ve `firmware/**`, ESP-IDF `v5.5.2`, target `esp32`.
- `.gitignore` - aktualne ignoruje pouze `.DS_Store`.

## Firmware build

- `firmware/CMakeLists.txt` - ESP-IDF project wrapper, `project(voda-septik)`.
- `firmware/main/CMakeLists.txt` - registruje aplikační zdroje a zavislosti.
- `firmware/sdkconfig.defaults` - zapina rollback OTA a podporu BOYA flash.
- `firmware/partitions.csv` - partition table.
- `firmware/dependencies.lock` - lock managed komponent.

## Firmware dokumentace

- `firmware/README.md` - provozni dokumentace, MQTT strom, kalibrace tlaku a zasoby, helper `zalevaci`.
- `firmware/docs/ota.md` - OTA tok pres MQTT command a HTTP download.
- `firmware/docs/logging.md` - zmena log levelu pres MQTT a debug doporuceni.
- `firmware/docs/diagnostika.md` - hranice mezi stavem, diag, debug a tvrdou chybou.
- `firmware/docs/error-codes.md` - pravidla a rozsahy `Exxx` kodu.

## Aplikacni entrypoint

- `firmware/main/voda-septik.cpp`
  - startuje BOOT tlacitko, chybovou LED sekvenci po error resetu, TM1637 status display,
  - inicializuje NVS a task watchdog,
  - registruje konfiguracni sekce `Sit`, `System`, `Teplota`, `Zasoba`, `Tlak`, `Elektromer`, `Prutokomer`,
  - startuje Wi-Fi/MQTT, LWT, MQTT publish task, MQTT command subscriber,
  - startuje LCD, `state_manager`, ADS1115 a senzoricke tasky.
- `firmware/main/voda-septik.c` - C bridge na C++ `cpp_app_main`.

## Hlavni runtime moduly

- `state_manager.*` - centralni konzument `sensor_events`, publikuje MQTT stav/diag, ridi dostupnost entit a status indikatory.
- `sensor_events.*` - FreeRTOS fronta aplikacnich udalosti ze senzoru do state manageru.
- `mqtt_topics.*` - centralni registry MQTT topicu, smeru, typu payloadu, QoS a retain.
- `mqtt_publisher_task.*` - fronta publikace, cache poslednich hodnot, periodicky refresh, availability topicy.
- `mqtt_commands.*` - subscriber pro command topicy: reboot, webapp, debug, log level, OTA, DS18B20 scan.
- `mqtt_ha_discovery.*` - generovani Home Assistant MQTT discovery konfiguraci z registru topicu.
- `network_event_bridge.*` - prevod udalosti z `network_core` do stavu aplikace.
- `debug_mqtt.*` - debug publikace pod `debug/*`, rizena globalnim `g_debug_enabled`.
- `ota_manager.*` - OTA download, progress/event publikace a potvrzeni pending image.

## Senzory a periferie

- `ads1115.*` - sdileny ADC zdroj pro tlak a hladinu, vystavuje overwrite fronty pro posledni sample.
- `tlak.*` - tlak pred/za filtrem, EMA/hystereze, vypocet rozdilu a zanesenosti filtru.
- `zasoba.*` - hladina a prepocet na objem nadrze.
- `teplota.*` - DS18B20 voda/vzduch, scan report, diag chyb.
- `prutokomer.*` - pulzni prutokomer, aktualni prutok a celkove cerpano.
- `elektromer.*` - KWS-303L pres Modbus/RS485, elektricke hodnoty cerpadla.
- `lcd.*` - HD44780 LCD pres I2C.
- `status_display.*` - TM1637 stavovy displej a indikace poruch.
- `boot_button.*` - dlouhy stisk GPIO0 pro konfiguracni AP/webapp.
- `pins.h` - mapa GPIO pinu.

## Vlastni komponenty

- `components/config_store` - registry konfiguracnich polozek, NVS read/write API.
- `components/config_webapp` - jednoducha HTTP konfiguracni webapp nad `config_store`.
- `components/network_core` - reusable Wi-Fi/MQTT lifecycle a `mqtt_publish`.
- `components/error_check` - `APP_ERROR_CHECK` wrapper pro tvrdou chybu s kodem.
- `components/tm1637_startup_animation` - TM1637 startovni animace.

## Hardware

- `hardware/Voda septik.kicad_pro` - KiCad projekt.
- `hardware/Voda septik.kicad_sch` - schema.
- `hardware/Voda septik.kicad_pcb` - PCB.
- `hardware/martin.pretty/` a `hardware/zaciname.pretty/` - vlastni footprinty.
- `hardware/martin.kicad_sym`, `hardware/0martin.kicad_sym` - vlastni symboly.
