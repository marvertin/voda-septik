# Firmware voda-septik

Firmware monitoruje nádrž, čerpání, teploty, tlaky filtru a elektrické veličiny čerpadla. Publikuje přes MQTT a automaticky vytváří entity v Home Assistantu. Stav pumpy odvozuje z naměřeného činného výkonu; aktivní firmware neobsahuje ovládání relé čerpadla.

Dokumentace ověřena proti kódu 12. 10. 2026. Instalaci ESP-IDF a obnovu z GitHubu popisuje [kořenové README](../README.md).

## Dokumentace

- [Architektura a moduly](../.agents/PROJECT_INDEX.md)
- [Konfigurační položky, výchozí hodnoty a meze](../.agents/CONFIG_INDEX.md)
- [Úplný MQTT registr](../.agents/MQTT_INDEX.md)
- [Hardware, piny a knihovny](../hardware/README.md)
- [Vývoj, nasazování a verzování](docs/vyvoj-a-nasazeni.md)
- [Ověřené nasazení v2.3.0](docs/nasazeni-v2.3.0.md)
- [OTA](docs/ota.md), [logování](docs/logging.md), [diagnostika](docs/diagnostika.md), [chybové kódy](docs/error-codes.md)

## Sestavení

ESP-IDF **v5.5.2**, target **esp32**, flash **4 MB**. Z tohoto adresáře, v aktivovaném prostředí IDF:

```sh
idf.py --version
idf.py build
```

Výstup je `build/voda-septik.bin`. `sdkconfig`, `sdkconfig.defaults`, `partitions.csv` a přesné závislosti jsou v Gitu. BOOT tlačítko při běhu slouží ke konfiguraci; sériové nahrání zajišťuje `idf.py -p PORT flash monitor`.

## Konfigurační web

Web lze zapnout příkazem `cmd/webapp` (`on`) v místní síti nebo stiskem BOOT (GPIO0), který přepne do AP `voda-septik-config` a spustí captive portal. Stačí běžný stisk; dlouhé podržení není implementované. V AP použij `http://192.168.4.1/` nebo adresu z logu. V STA použij IP zařízení.

Web běží na portu 80, automaticky se vypne po 2 hodinách; opakované zapnutí obnovuje časovač. Uložení konfigurace zapisuje NVS a restartuje ESP. Po tomto softwarovém restartu se web opět zapne, jakmile zařízení získá IP. Běžný start v STA jej sám nezapíná.

Položky `interval_s`, `auto_mode` a `tepl_max` jsou registrované, ale současná aplikace je nepoužívá pro časování, závlahu nebo ochranu. Měření mají periody pevně určené v příslušných modulech.

## Kalibrace tlaků

ADS1115 měří proudové smyčky přes 100Ω shunt s PGA ±2,048 V. Výchozí RAW je **6400 při 4 mA** a **32000 při 20 mA**. Nejde o staré ADC body 745/3722.

| Před filtrem | Za filtrem | Význam / výchozí hodnota |
| --- | --- | --- |
| `tlk_b_ads_r4` | `tlk_a_ads_r4` | RAW při 4 mA: 6400 |
| `tlk_b_ads_r20` | `tlk_a_ads_r20` | RAW při 20 mA: 32000 |
| `tlk_b_p_min` | `tlk_a_p_min` | dolní tlak: 0 bar |
| `tlk_b_p_max` | `tlk_a_p_max` | horní tlak: 10 bar |

Společné nastavení: `tlk_ema_alpha=0.55`, `tlk_hyst_bar=0.02`, `tlk_round_dec=2`, `tlk_dp_100=1.0`. Úplné meze jsou v mapě konfigurace. `tlk_sample_ms` neexistuje; producent ADC čte tlaky po 250 ms.

Postup:

1. U každého čidla změř referenční body a nastav odpovídající RAW a fyzikální rozsah.
2. Ověř tlak podle skutečného referenčního měření.
3. Dolaď filtraci, hysterezi a práh rozdílu `tlk_dp_100` podle filtru.
4. Ulož konfiguraci a po restartu ověř oba kanály i MQTT.

Výstupy: `stav/tlak/pred_filtrem_bar`, `stav/tlak/za_filtrem_bar`, `stav/tlak/rozdil_filtru_bar`, **`stav/zanesenost_filtru_percent`**. Zanesení se odvozuje z rozdílu a omezuje na 0–100 %; není samostatné přímé měření.

## Kalibrace hladiny a objemu

| Položka | Výchozí hodnota | Význam |
| --- | --- | --- |
| `lvl_ads_raw_min` | 6400 | RAW referenčního minima |
| `lvl_ads_raw_max` | 32000 | RAW referenčního maxima |
| `lvl_h_min` | 0 m | výška v minimu |
| `lvl_h_max` | 1,9 m | výška v maximu |
| `tank_area_m2` | 5,4 m² | plocha nádrže |
| `lvl_unpump_cm` | 10 cm | minimální hladina pro čerpání ode dna (0–500 cm) |
| `lvl_ema_alpha` | 0,25 | EMA |
| `lvl_hyst_m` | 0,002 m | směrová hystereze |
| `lvl_round_dec` | 2 | desetinná místa objemu |

RAW se lineárně převede na výšku, filtruje a převádí vztahy:

```text
objem_m3 = max(0, výška_m − lvl_unpump_cm / 100) × tank_area_m2
objem_l  = objem_m3 × 1000    (pouze případný přepočet mimo MQTT registr)
```

Objem udává využitelnou zásobu. Publikovaná i zobrazovaná výška zůstává skutečná, bez odečtu. Parametr nastav ve webu v sekci Zásoba; ukládá se do NVS a použije se po restartu.

Nejprve zkalibruj výšku, potom nastav skutečnou plochu a ověř výsledný objem. ADC kanál hladiny se čte po 2 s. Aktuální MQTT výstupy jsou **`stav/zasoba/objem_m3` [m³]** a `stav/zasoba/hladina_m` [m]; topic `objem_l` již není registrovaný.

## Průtok, teploty a elektroměr

- Průtok: `flow_pulses_l` má výchozí hodnotu 38 impulsů/l, meze 1–200. Vyhodnocení po 200 ms používá historii impulsů; po 3 s bez impulsu je průtok 0. Celkový objem se ukládá po celých litrech do `flow_data0` a NVS `flash_ctr`; zlomky litru nemusí přežít restart.
- DS18B20: při výměně čidel zapni `cmd/teplota/scan`, sleduj `diag/teplota_scan` a nastav `temp_addr_water` / `temp_addr_air` podle nalezených ROM. Potom scan vypni. Výchozí ROM odpovídají konkrétní instalaci.
- KWS-303L: Modbus RTU 9600 8N1, UART2; `kws_addr` je výchozí 1, povolené 1–247. Stav pumpy je `bezi` při výkonu alespoň 5 W, jinak `stoji`, nebo `chyba` při neplatném výkonu.

## MQTT a Home Assistant

Kořen je pevně `voda/septik`. Hlavní větve: `stav/*`, `elektro/cerpadlo/*`, `system/*`, `diag/*`, `cmd/*` a volitelné `debug/*`. V registru je 59 publikačních a 7 příkazových topiců; úplná tabulka, QoS a retain jsou v [mapě MQTT](../.agents/MQTT_INDEX.md).

Po připojení k brokeru firmware publikuje retained discovery do `homeassistant/.../config`, zařízení má ID `voda_septik_esp32`. Platná měření a nedostupnost používají individuální availability. Poslední retained hodnota může na brokeru zůstat i po označení nedostupnosti. Discovery nyní nekombinuje tuto dostupnost s globálním LWT `system/status`.

Průtok nemá `expire_after=30`; nulový průtok není výpadek senzoru. Po přejmenování topiců odstraň staré retained discovery konfigurace podle konkrétní entity, jinak mohou v HA zůstat staré názvy.

## Ovládání nástrojem `zalevaci`

Z `firmware/` spusť `./tools/zalevaci`; při aktivním `direnv` je dostupný i příkaz `zalevaci`. Nástroj vyžaduje Python 3 a pro MQTT operace `mosquitto_pub`, pro odběr/OTA také `mosquitto_sub`.

```sh
./tools/zalevaci --help
./tools/zalevaci tags
./tools/zalevaci cmd webapp on
./tools/zalevaci cmd webapp off
./tools/zalevaci cmd debug on
./tools/zalevaci log --tag mqtt_commands --level INFO
./tools/zalevaci cmd teplota/scan 1
./tools/zalevaci cmd teplota/scan 0
./tools/zalevaci clear-retained
```

`cmd reboot` skutečně restartuje zařízení. Commandy posílej bez retain; firmware retained příkazy ignoruje. `clear-retained` odstraňuje pouze retained zprávy na sedmi známých command topicích, nikoli discovery nebo měření.

Globální parametry (`--host`, `--port`, `--user`, `--pass-file`, `--bench`) patří před subcommand. Výchozí produkční broker helperu je `mqtt.veve:1883`, bench je `amur.veve:1884`. Parametr `--bench` mění cíl CLI, nepřepíná firmware do bench režimu. Oba názvy jsou lokální pro původní síť.

Heslo helper načítá z `~/.voda-septik/mqtt_password` nebo vlastního `--pass-file`; chybějící heslo si vyžádá a uloží s oprávněním 600. Neukládej ho do Gitu.

## Provozní data a zálohy

V Gitu jsou zdroje a výchozí konfigurace, nikoli aktuální NVS kalibrace, čítače v ESP nebo lokální MQTT heslo. Pro obnovu zařízení udržuj i zálohu datových oddílů a záznam provozní konfigurace. Podrobnosti a obnova Home Assistantu jsou v [kořenovém README](../README.md).
