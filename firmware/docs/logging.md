# Logování a debug MQTT

Ověřeno proti aktivnímu kódu a `sdkconfig` 12. 10. 2026. Rozlišuj **sériové ESP logy** a **samostatné MQTT debug zprávy**.

## Sériové logy

Příkaz `voda/septik/cmd/log/level` volá `esp_log_level_set`. Přijímá `TAG=LEVEL`, `TAG:LEVEL`, `TAG LEVEL`, nebo samotný `LEVEL` pro globální `*`. Úrovně: NONE/0, ERROR/1, WARN/2, INFO/3, DEBUG/4, VERBOSE/5 (také TRACE).

Aktuální `sdkconfig` má `CONFIG_LOG_DEFAULT_LEVEL=3`, `CONFIG_LOG_MAXIMUM_LEVEL=3` a dynamické řízení. Výchozí i maximální zahrnutá úroveň je **INFO**. Nastavení DEBUG/VERBOSE přes MQTT samo nevrátí logy vynechané při překladu; pro jejich skutečné zahrnutí je potřeba změnit maximum sestavení a vytvořit nový firmware. Tento dokument konfiguraci nemění.

Z `firmware/`:

```sh
./tools/zalevaci log --tag mqtt_commands --level INFO
./tools/zalevaci log --tag tlak --level WARN
./tools/zalevaci log --tag '*' --level WARN
./tools/zalevaci tags
```

Platné tagy zahrnují `mqtt_commands`, `mqtt_publisher_task`, `state_manager`, `network_init`, `tlak`, `zasoba`, `ads1115`, `teplota`, `elektromer`, `prutokomer`, `ota_manager`, `webapp_startup`. Používej názvy z kódu; staré příklady `mqtt_cmd` nebo `TLAK` nejsou aktuální tagy.

Nastavení log levelu je za běhu v RAM a není uložené jako konfigurační položka. MQTT reconnect znovu nenastavuje všechny úrovně; po restartu platí nastavení sestavení. Sériový výstup sleduj pomocí `idf.py -p PORT monitor`.

## Debug zprávy přes MQTT

```sh
./tools/zalevaci cmd debug on
./tools/zalevaci cmd debug off
```

`cmd/debug` řídí `g_debug_enabled`, není to nastavení úrovně ESP logů. Zapnutí obnoví dvouhodinový časovač; po uplynutí nebo restartu je debug vypnutý.

Zprávy vznikají přes `DEBUG_PUBLISH`, přímo přes MQTT klienta s **QoS 0 a bez retain**, pouze při připojení. Buffer je 192 B a prefix obsahuje zdrojový soubor, řádek a funkci. Kořen je `voda/septik/debug/`.

| Větev | Obsah |
| --- | --- |
| `prutok` | průtok, celkový objem a pulsy |
| `teplota` | výsledek čtení teplotních čidel |
| `tlak_cfg` | kalibrace a parametry tlaků |
| `tlak` | RAW hodnoty a zpracování tlakových kanálů |
| `filtr` | rozdíl tlaků a zanesení |
| `cfg/zasoba` | kalibrace a parametry hladiny |
| `zasoba` | RAW, výška a objem v m³ |

Pro odběr použij `mosquitto_sub` s připojením ke svému brokeru a topicem `voda/septik/debug/#`. MQTT heslo je v lokálním souboru helperu, nikoli v tomto dokumentu.

## Kalibrace a diagnostika

Pro tlak používej `tlk_b_ads_r4`, `tlk_b_ads_r20`, `tlk_a_ads_r4`, `tlk_a_ads_r20` a fyzikální meze `tlk_b_p_*` / `tlk_a_p_*`. Výchozí RAW body jsou 6400/32000. Pro hladinu používej `lvl_ads_raw_min/max`, `lvl_h_min/max` a `tank_area_m2`; MQTT objem je `objem_m3`, ne litry.

Samotný ESP log level DEBUG nezapíná debug MQTT. Běžné dlouhodobé chyby a stav zařízení sleduj přes `diag/*`; debug slouží pro podrobnější krátké ověření filtrů a kalibrace.

Retained zprávy na command topicích lze vyčistit pomocí:

```sh
./tools/zalevaci clear-retained
```

Firmware je při příjmu ignoruje. Tento příkaz nemaže retained discovery ani měření.

Související: [kalibrace](../README.md), [konfigurace](../../.agents/CONFIG_INDEX.md), [diagnostika](diagnostika.md).
