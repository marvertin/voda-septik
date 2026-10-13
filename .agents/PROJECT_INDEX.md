# Mapa projektu

Ověřeno proti aktivním zdrojům 12. 10. 2026. Repo obsahuje monitoring vody v nádrži (bývalý septik), čerpání, tlaků před/za filtrem, teplot a elektrických veličin čerpadla. Název helperu `zalevaci` ani položka `auto_mode` nedokládají implementované řízení závlahy: aktuální aktivní firmware měří, publikuje a odvozuje stav pumpy z výkonu; GPIO pro její spínání není v `pins.h`.

## Dokumentace a zdroje pravdy

- [Kořenové README](../README.md): instalace a obnova.
- [Provozní README](../firmware/README.md): kalibrace a obsluha.
- [Hardware README](../hardware/README.md): knihovny a propojení periferií.
- [Konfigurace](CONFIG_INDEX.md): 38 položek v 7 sekcích.
- [MQTT](MQTT_INDEX.md): 67 topiců, availability a discovery.
- [Pravidla větví, nasazení a verzování](../firmware/docs/vyvoj-a-nasazeni.md).
- [Diagnostika](../firmware/docs/diagnostika.md), [logování](../firmware/docs/logging.md), [OTA](../firmware/docs/ota.md), [kódy chyb](../firmware/docs/error-codes.md).

Zdrojem pravdy je kód a explicitní CMake seznam zdrojů. Dokumentaci externích komponent v `managed_components/` udržuje jejich dodavatel.

## Sestavení a prostředí

- `firmware/CMakeLists.txt`: projekt `voda-septik`, CMake minimálně 3.22; načítá IDF přes `IDF_PATH`.
- `firmware/main/CMakeLists.txt`: aktivní aplikační zdroje, včetně C bridge `voda-septik.c`.
- ESP-IDF v5.5.2, target `esp32`, flash 4 MB. CI má stejnou verzi a publikuje firmware, bootloader a partition table.
- `sdkconfig` je verzovaný; `sdkconfig.defaults` přidává rollback OTA a podporu BOYA flash. `partitions.csv` má dva OTA sloty po 0x180000 B, NVS a čtyři vlastní datové oddíly.
- Sedm registry komponent je uložených v `managed_components` spolu s lock souborem: ads111x 1.1.14, esp_idf_lib_helpers 1.4.0, hd44780 1.3.0, i2cdev 2.1.1, onewire 1.1.0, pcf8574 1.0.10, tm1637 1.0.1.
- Projektový `.envrc` načítá `~/.espressif/v5.5.2/esp-idf/export.sh` a přidává `tools/` do PATH. Je volitelný a určený pro macOS/Linux.
- Běžný `build/` je ignorovaný. `.vscode/settings.json` je lokální; firmware lze sestavit bez něj.

## Start aplikace

`app_main` v C bridge volá `cpp_app_main`:

1. Spustí BOOT task, indikaci případného chybového resetu a status displej; vypíše oddíly.
2. Inicializuje NVS a watchdog (5 s, oba idle core, `trigger_panic=false`). U NVS bez volných stránek nebo s jinou verzí může provést erase NVS.
3. Vytvoří aplikační frontu 32 událostí a síťový bridge.
4. Připraví NVS `app_cfg`, sedm sekcí konfigurace a konfigurační web.
5. Načte Wi-Fi/MQTT nastavení a spustí síť s LWT. `MQTT_BENCH_MODE=false` znamená normální provoz; testovací broker není používán.
6. Spustí MQTT publisher (fronta 32), command handler a LCD.
7. Spustí `state_manager`, průtok, ADS1115, hladinu, tlaky, elektroměr a teploty.

## Tok dat

```text
ADS1115 → poslední vzorky ve dvou jednoprvkových frontách → tlak / zásoba ┐
GPIO ISR → průtok                                                    │
1-Wire → teploty                                                      ├→ sensor_events → state_manager → LCD / TM1637
UART2 / RS485 → elektroměr                                            │                            └→ MQTT publisher → broker
network_core → network_event_bridge → síťové změny / telemetrie        ┘
MQTT příkazy → mqtt_commands → web / debug / logování / OTA / scan
```

Události jsou `EVT_SENSOR`, `EVT_NETWORK_STATE_CHANGE`, `EVT_NETWORK_TELEMETRY`, `EVT_TICK` (poslední je definovaný, ale aktivní kód nemá producenta). Nepoužívej historický název `EVT_NETWORK`.

## Aplikační moduly (`firmware/main/`)

| Modul | Zodpovědnost a důležité vazby |
| --- | --- |
| `ads1115.*` | Jediný vlastník ADC: I2C0, 0x48, PGA ±2,048 V, ALERT/RDY. AIN1 tlak za, AIN2 tlak před, AIN3 hladina. Tlaky 250 ms / 128 SPS, hladina 2 s / 16 SPS. Fronty posledních vzorků používají overwrite. |
| `tlak.*` | RAW → bar, TrimmedMean<7,1>, EMA, hystereze, zaokrouhlení; okamžitý rozdíl tlaků; nezaokrouhlené kalibrované páry pro monitor filtru. Chybný kanál reprezentuje NaN. |
| `zasoba.*` | RAW → výška → objem v m³, TrimmedMean<5,1>, EMA a směrová hystereze (`directional_hysteresis.hpp`). Výchozí plocha 5,4 m². |
| `prutokomer.*` | GPIO33 ISR a kruhová historie 128 časů impulsů, odhad průtoku v okně cca 1,5 s, vyhodnocení 200 ms, nula po 3 s bez impulsu. Výchozí 38 impulsů/l. Celkový objem přetrvává po celých litrech. |
| `flash_monotonic_counter.*` | Zápisy bitů do `flow_data0`, základ a marker rolloveru v NVS `flash_ctr`; obnovuje rozpracovaný rollover. Zálohuj data i příslušné NVS společně. |
| `teplota.*` | DS18B20 přes onewire, čtení scratchpadu a CRC přímo v modulu. Konverze 800 ms + prodleva 1000 ms; čtení tedy není přesná perioda 1 s. Discovery cca 30 s, zapnutý scan report cca 5 s. Dvě ROM adresy z konfigurace. |
| `elektromer.*` | KWS-303L přes Modbus RTU, UART2 9600 8N1 / MAX3485, funkce 0x03; prodleva cyklu 5 s, timeout čtení 200 ms. Devět elektrických veličin, CRC a čítače chyb. |
| `sensor_events.*` | Typované senzorické a síťové události, fronta a čítač zahozených publish událostí. |
| `network_event_bridge.*` | Převádí callback `network_event_t` na dvě události. Telemetrie nejvýše každých 10 s nebo při změně síťové úrovně. |
| `state_manager.*` | Konzumuje události, převádí jednotky elektroměru, aktualizuje displeje, boot metadata, diagnostiku a MQTT dostupnost. Pumpa `bezi` od 5 W, jinak `stoji`, při neplatném výkonu `chyba`. |
| `mqtt_topics.*` | Centrální registr cest, směru, payloadu, QoS a retain. |
| `mqtt_publisher_task.*` | Fronta, poslední hodnoty, potlačení shodných zpráv, obnovovací publikace při nové události po 60 s, flush po reconnectu. |
| `mqtt_commands.*` | Registrace subscriberu a dispatch sedmi příkazů; ignoruje retained commandy, časově omezuje debug na 2 hodiny. |
| `mqtt_ha_discovery.*` | Retained HA konfigurace všech publikačních topiců, názvy, jednotky, individuální availability. |
| `debug_mqtt.*` | Přímá QoS 0 publikace do `debug/*`, mimo publisher cache, řízená `g_debug_enabled`. |
| `ota_manager.*` | HTTP klient a sekvenční zápis do druhého OTA slotu; reboot a ruční potvrzení pending image. |
| `network_config.*`, `system_config.*` | Definice a registrace konfigurace; síťové výchozí údaje nezveřejňuj. |
| `restart_info.*` | NVS `sys_meta`, počet startů a důvod resetu; chyby vrací volajícímu bez `APP_ERROR_CHECK`. Nesynchronizovaný čas se ukládá jako 0. |
| `webapp_startup.*` | HTTP web na portu 80, AP / MQTT zapnutí, návrat po restartu z uložení konfigurace, auto-stop 2 hodiny. |
| `boot_button.*` | Běžný stisk GPIO0, debounce 40 ms. Aplikační callback přepne do AP jednou za běh. |
| `lcd.*` | HD44780 / PCF8574 0x27, samostatné I2C1, fronta 8; nepřítomný LCD pouze deaktivuje výstup. |
| `status_display.*` | TM1637 a LED: síť, aktivita MQTT, točící indikace průtoku, segmenty chyb senzorů a hlášení `Exxx`. |
| `trimmed_mean.hpp`, `directional_hysteresis.hpp` | Pomocné filtry pro měření; parametry neurčují registr konfigurace samy o sobě. |

## Vlastní komponenty

- `config_store`: typový registr, sekce, limity a NVS.
- `config_webapp`: HTML formulář, typová validace, uložení a restart; captive portal URL.
- `network_core`: Wi-Fi STA/AP, reconnect 1–60 s, MQTT lifecycle/LWT, callback a základní publish wrapper. Aplikační logika je mimo komponentu.
- `error_check`: společný fatalní wrapper a možnost registrovat handler pro zobrazení kódu.
- `tm1637_startup_animation`: knihovní animace; provázání volajícího ověřuj ve `status_display.cpp`.

## Hardware a provozní pomocníci

KiCad schéma i PCB jsou v `hardware/`; mapu GPIO a knihoven shrnuje [hardware/README.md](../hardware/README.md). Schéma není náhradou za `pins.h` při posuzování aktivního firmware.

`firmware/tools/zalevaci` má interaktivní menu i subcommandy `cmd`, `log`, `clear-retained`, `tags`, `ota`. Podporuje běžný i bench broker; jeho `--bench` nepřepíná firmware, pouze cíl CLI. Heslo je mimo Git.

`firmware/ha/docker-compose.yml` je volitelná konfigurace Mosquitto a Home Assistantu. Provozní data a uživatelská `.storage` jsou mimo Git; pohyblivé image tagy nepřipínají totožnou verzi.

## Známé hranice současného stavu

- `interval_s`, `auto_mode`, `tepl_max` nejsou čtené pro řízení aktivního runtime.
- Sériové logy jsou v `sdkconfig` omezené na INFO; MQTT debug je samostatný.
- `diag/last_mqtt_rc` zůstává 0, protože jeho runtime proměnná není aktualizovaná.
- HA discovery používá individuální availability bez kombinace s globálním LWT. NaN průtoku nemá stejnou EMPTY větev jako teplota/tlak/hladina/elektroměr.
- Manifest `main/idf_component.yml` stále deklaruje široké `idf >=4.1.0`, ale ověřené sestavení a CI jsou pouze na 5.5.2; tuto deklaraci neber jako ověřenou podporu IDF 4.1.
- Přímý DS18B20 scan publish neaktualizuje individuální availability použitou v jeho HA discovery.
- Knihovna KiCad `PCM_Espressif` není uložená ani verzově připnutá v projektu; její tabulka footprintů nemusí být globálně připojená.
- Statická kontrola zdrojů a předchozí čisté sestavení nejsou ověřením skutečného chování periferií na zařízení.

`filter_monitor.*`: samostatný monitor volaný ze state_manager pro surový průtok a dvojice kalibrovaných tlaků před EMA/hysterezí/zaokrouhlením. `filter_monitor_config.cpp`: 5 položek sekce Tlak. Časy událostí jsou monotónní od startu; HA doplňuje čas příjmu nového měření.
