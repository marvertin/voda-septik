# Diagnostika firmware

Tento dokument popisuje hranici mezi tvrdou chybou firmware, běžnou uživatelskou telemetrií, diagnostikou a explicitním debug režimem.

## Tvrdé chyby `Exxx`

Tvrdá chyba je určena pro stav, ve kterém firmware nemůže bezpečně pokračovat kvůli chybě implementace, rozbité základní platformě nebo zásadnímu konfliktu hardware. Při tvrdé chybě se hlásí unikátní kód `Exxx`; pravidla přidělování jsou v `docs/error-codes.md`.

Mezi tvrdé chyby patří:

- inicializace NVS, task watchdogu, event loopu, WiFi driveru a základní sítě,
- registrace konfigurace a MQTT topiců, pokud jde o chybu firmware,
- vytvoření základních tasků a front,
- konflikt pinů nebo konfigurace, který nemůže vzniknout běžným provozem.

Mezi tvrdé chyby nepatří:

- dočasné nepřipojení WiFi nebo MQTT,
- neodpovídající senzor,
- nefunkční displej,
- chybějící data, pokud lze stav vyjádřit jako nedostupný údaj.

## Hlavní MQTT a Home Assistant

Hlavní topicy (`stav/*`, `elektro/*`, `system/*`) nesou údaje, které mají smysl pro uživatele. Nemají obsahovat servisní podrobnosti typu CRC chyba, počet timeoutů nebo interní RAW hodnoty.

Pokud údaj není k dispozici, publikuje se nedostupnost přes dostupnost dané entity, ne náhradní hodnota s diagnostickým významem. Poslední platná hodnota se nemá používat jako maskování poruchy.

## DIAG topicy

`diag/*` je určeno pro řešení problémů s částmi systému: WiFi/MQTT, pamětí, NVS, I2C, ADS1115, 1-Wire teplotami, Modbus elektroměrem a podobně.

DIAG nesmí zahlcovat broker ani log, ani když je část hardware odpojená. Preferovaný tvar je periodicky publikovaný stav, čítač chyb a stáří posledního platného měření. DIAG nemá přenášet každý jednotlivý chybný pokus.

Příklady vhodných DIAG údajů:

- `diag/ads1115/status`, `diag/ads1115/read_errors`, `diag/ads1115/last_ok_age_s`,
- `diag/modbus/kws/status`, `diag/modbus/kws/timeouts`, `diag/modbus/kws/crc_errors`, `diag/modbus/kws/last_ok_age_s`,
- `diag/teplota/status`, `diag/teplota/read_errors`, `diag/teplota/last_ok_age_s`,
- `diag/events/sensor_queue_drops`, `diag/mqtt/publish_queue_drops`.

## DEBUG režim

DEBUG je explicitně zapínaný režim pro ladění detailů, zejména filtrů, hystereze, kalibrací a RAW hodnot. V tomto režimu je přijatelné vyšší množství dat.

Debug výstupy patří do `debug/*` a mají být vypnuté v běžném provozu. Provozní diagnostika, která je potřeba dlouhodobě, patří do `diag/*`, ne do `debug/*`.

## Aktuální audit

Současný návrh hlavních topiců odpovídá pravidlům: běžné hodnoty jdou do `stav/*` a `elektro/*`, chybějící měření se ve `state_manager` posílá jako nedostupná entita.

Tvrdé chyby jsou většinou použité správně: WiFi inicializace je fatalní, výpadek připojení nikoliv; ADS1115 se při chybě zařízení chová jako nedostupný zdroj dat; displeje neukončují firmware.

Místa vhodná k úpravě:

- `restart_info.cpp`: chyba zápisu restart metadat do NVS by neměla shodit firmware; stačí zvýšit `diag/nvs_errors`.
- `elektromer.cpp`: při odpojeném Modbus elektroměru se dnes může logovat více varování v každém cyklu. Lepší je agregovat chyby do čítačů a logovat jen změnu stavu nebo periodický souhrn.
- `teplota.cpp`: opakované chyby čtení teploty by měly být rate-limitované a viditelné přes DIAG čítač.
- `ads1115.cpp`, `zasoba.cpp`, `tlak.cpp`: RAW/filtrační detaily patří do DEBUG; do DIAG patří stav ADC a počty chyb čtení.
- `diag/esp_vcc_mv`: topic je definovaný, ale není plněný; buď doplnit měření, nebo topic odstranit z HA discovery.
