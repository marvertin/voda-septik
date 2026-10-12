# Mapa konfigurace

Ověřeno proti aktivním zdrojům 12. 10. 2026. Zdrojem pravdy jsou definice `config_item_t` a registrace v `main/voda-septik.cpp`. Všechny cesty níže jsou relativní ke `firmware/`.

## Uložení a použití

- Sedm sekcí (`Sit`, `System`, `Teplota`, `Zasoba`, `Tlak`, `Elektromer`, `Prutokomer`) obsahuje celkem 32 položek.
- `components/config_store` uchovává registr a typové get/set API nad NVS namespace `app_cfg`; float se ukládá jako blob.
- Chybějící hodnota používá výchozí hodnotu z kódu. Aktuální hodnoty uložené v zařízení nejsou totožné s výchozími hodnotami a nejsou zálohované Gitem.
- Konfigurační web validuje typy a meze, zapisuje hodnoty a po uložení restartuje zařízení. Síť i kalibrace se načítají při inicializaci příslušných modulů.
- Registr umožňuje nejvýše 64 položek a 16 sekcí. Klíče NVS mají limit 15 znaků.
- Přístupové údaje z `network_config.cpp` nekopíruj do dokumentace ani výstupů nástrojů. Níže jsou záměrně vynechané jejich výchozí hodnoty.

## Síť

Zdroj: [main/network_config.cpp](../firmware/main/network_config.cpp).

| Klíč | Typ | Výchozí hodnota | Meze | Význam |
| --- | --- | --- | --- | --- |
| `wifi_ssid` | string | nezveřejněno | max. 31 znaků | SSID site, ke ktere se ma zarizeni pripojit. |
| `wifi_pass` | string | nezveřejněno | max. 63 znaků | Heslo Wi-Fi; prázdná hodnota sama nezapíná AP. STA vyžaduje WPA2. |
| `mqtt_uri` | string | nezveřejněno | max. 127 znaků | Adresa MQTT brokeru, napr. mqtt://mqtt:1883. |
| `mqtt_user` | string | nezveřejněno | max. 63 znaků | Uzivatelske jmeno pro pripojeni k MQTT brokeru. |
| `mqtt_pass` | string | nezveřejněno | max. 127 znaků | Heslo pro pripojeni k MQTT brokeru. |

## Systém

Zdroj: [main/system_config.cpp](../firmware/main/system_config.cpp).

| Klíč | Typ | Výchozí hodnota | Meze | Význam |
| --- | --- | --- | --- | --- |
| `interval_s` | int32 | 30 | 5 až 3600 | Registrovaná perioda; aktuální runtime ji nepoužívá. |
| `auto_mode` | bool | true | true / false | Registrovaný přepínač; automatické řízení není implementované. |

## Teplota

Zdroj: [main/teplota.cpp](../firmware/main/teplota.cpp).

| Klíč | Typ | Výchozí hodnota | Meze | Význam |
| --- | --- | --- | --- | --- |
| `tepl_max` | float | 35.0 | -20.0 až 90.0 | Registrovaný práh; aktuální runtime jej nepoužívá. |
| `temp_addr_water` | string | konkrétní ROM v kódu | max. 18 znaků | ROM adresa DS18B20 pro vodni sondu (hex, napr. 28FF1A2B3C4D5E6F). |
| `temp_addr_air` | string | konkrétní ROM v kódu | max. 18 znaků | ROM adresa DS18B20 pro vzduchovou sondu (hex, napr. 28FF1A2B3C4D5E6F). |

## Zásoba

Zdroj: [main/zasoba.cpp](../firmware/main/zasoba.cpp).

| Klíč | Typ | Výchozí hodnota | Meze | Význam |
| --- | --- | --- | --- | --- |
| `lvl_ads_raw_min` | int32 | 6400 | 0 až 32767 | ADS1115 RAW hodnota odpovidajici minimalni hladine. |
| `lvl_ads_raw_max` | int32 | 32000 | 1 až 32767 | ADS1115 RAW hodnota odpovidajici maximalni hladine. |
| `lvl_h_min` | float | 0.0 | 0.0 až 5.0 | Vyska hladiny pro minimalni hodnotu senzoru. |
| `lvl_h_max` | float | 1.900 | 0.0 až 5.0 | Vyska hladiny pro maximalni hodnotu senzoru. |
| `tank_area_m2` | float | 5.4 | 0.1 až 50.0 | Pudorysna plocha nadrze pouzita pro prepocet vysky na objem. |
| `lvl_ema_alpha` | float | 0.25 | 0.01 až 1.0 | Koeficient EMA filtru na vysce hladiny (0-1). |
| `lvl_hyst_m` | float | 0.002 | 0.0 až 0.05 | Mrtve pasmo hystereze vysky hladiny v metrech. |
| `lvl_round_dec` | int32 | 2 | 1 až 3 | Pocet desetinnych mist pro publikovany objem (1-3; 1=desetiny, 2=setiny, 3=tisiciny). |

## Tlak

Zdroj: [main/tlak.cpp](../firmware/main/tlak.cpp).

| Klíč | Typ | Výchozí hodnota | Meze | Význam |
| --- | --- | --- | --- | --- |
| `tlk_b_ads_r4` | int32 | 6400 | 0 až 32767 | ADS1115 RAW hodnota (pred filtrem) odpovidajici vstupu 4 mA. |
| `tlk_b_ads_r20` | int32 | 32000 | 1 až 32767 | ADS1115 RAW hodnota (pred filtrem) odpovidajici vstupu 20 mA. |
| `tlk_b_p_min` | float | 0.0 | -1.0 až 16.0 | Tlak pred filtrem odpovidajici 4 mA. |
| `tlk_b_p_max` | float | 10.0 | -1.0 až 16.0 | Tlak pred filtrem odpovidajici 20 mA. |
| `tlk_a_ads_r4` | int32 | 6400 | 0 až 32767 | ADS1115 RAW hodnota (za filtrem) odpovidajici vstupu 4 mA. |
| `tlk_a_ads_r20` | int32 | 32000 | 1 až 32767 | ADS1115 RAW hodnota (za filtrem) odpovidajici vstupu 20 mA. |
| `tlk_a_p_min` | float | 0.0 | -1.0 až 16.0 | Tlak za filtrem odpovidajici 4 mA. |
| `tlk_a_p_max` | float | 10.0 | -1.0 až 16.0 | Tlak za filtrem odpovidajici 20 mA. |
| `tlk_ema_alpha` | float | 0.55 | 0.01 až 1.0 | Spolecny koeficient EMA filtru tlaku (0-1). |
| `tlk_hyst_bar` | float | 0.02 | 0.0 až 1.0 | Spolecne mrtve pasmo hystereze tlaku v barech. |
| `tlk_round_dec` | int32 | 2 | 1 až 3 | Spolecny pocet desetinnych mist pro publikovany tlak (1-3). |
| `tlk_dp_100` | float | 1.0 | 0.01 až 20.0 | Rozdil tlaku, ktery odpovida 100% zanesenosti filtru. |

## Elektroměr

Zdroj: [main/elektromer.cpp](../firmware/main/elektromer.cpp).

| Klíč | Typ | Výchozí hodnota | Meze | Význam |
| --- | --- | --- | --- | --- |
| `kws_addr` | int32 | 1 | 1 až 247 | Adresa Modbus slave elektromeru KWS-303L (1-247). |

## Průtokoměr

Zdroj: [main/prutokomer.cpp](../firmware/main/prutokomer.cpp).

| Klíč | Typ | Výchozí hodnota | Meze | Význam |
| --- | --- | --- | --- | --- |
| `flow_pulses_l` | int32 | 38 | 1 až 200 | Kalibracni konstanta prutokomeru: kolik impulsu odpovida jednomu litru. |

## Důležité odlišnosti od starších dokumentů

- RAW kalibrace tlaku mají názvy `tlk_b_ads_r4`, `tlk_b_ads_r20`, `tlk_a_ads_r4`, `tlk_a_ads_r20`; hladina používá `lvl_ads_raw_min` a `lvl_ads_raw_max`.
- Výchozí ADC body jsou 6400 a 32000: 4–20 mA přes 100Ω shunt při rozsahu ADS1115 ±2,048 V. Výchozí maximum hladiny je 1,9 m, nikoli 0,290 m.
- `tlk_sample_ms` ani `mqtt_topic` nejsou registrované položky. Periodu ADC určuje `ads1115.cpp`; kořen MQTT je pevně `voda/septik`.
- `interval_s`, `auto_mode` a `tepl_max` se registrují a ukládají, ale aktivní aplikace je nyní nečte pro řízení měření, závlahy nebo ochrany. Neodvozuj z jejich popisků implementovanou automatiku.
- Adresy DS18B20 jsou konkrétní pro instalovaná čidla. Při výměně použij `cmd/teplota/scan`, report `diag/teplota_scan` a adresu v přesném formátu reportovaném firmware.

## Konfigurační web

- `cmd/webapp` s payloadem `on` / `off` zapíná / vypíná HTTP server na portu 80. Opakované zapnutí obnovuje časovač dvouhodinového automatického vypnutí.
- BOOT (GPIO0) reaguje na běžný stisk po odrušení 40 ms; přepne jednou za běh do otevřeného AP `voda-septik-config` a spustí web. Nepotřebuje dlouhé podržení.
- V AP režimu je obvyklá adresa `http://192.168.4.1/`; přesnou adresu vypíše firmware. STA režim používá IP zařízení v místní síti.
- Stránka konfigurace je `GET /config`, uložení `POST /config/save`. AP nabízí captive DNS a přesměrování kontrolních URL klientů.
- Po softwarovém restartu vyvolaném uložením konfigurace se web znovu spustí po získání IP podle jednorázového NVS markeru `app_cfg/web_restart`. Běžný start v STA jej automaticky nezapíná.

Související provozní postup: [firmware/README.md](../firmware/README.md).
