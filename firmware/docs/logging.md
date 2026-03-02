# Logovani a zmena log levelu pres MQTT

## Jak to uzivatelsky pouzivat

### Rychle nastaveni jedne hodnoty

Pro zmenu log levelu konkretniho tagu publikuj command na topic:

- topic: `voda/septik/cmd/log/level`
- payload: `TAG=LEVEL`

Pro zmenu defaultni urovne pro vsechny tagy lze poslat i jen samotny `LEVEL`.

Priklad:

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 \
  -t "voda/septik/cmd/log/level" \
  -m "mqtt_commands=DEBUG"
```

Povolene urovne:

- `NONE` (0)
- `ERROR` (1)
- `WARN` (2)
- `INFO` (3)
- `DEBUG` (4)
- `VERBOSE` (5, lze i `TRACE`)

Lze nastavit i globalni uroven:

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 \
  -t "voda/septik/cmd/log/level" \
  -m "*=WARN"
```

Nebo ekvivalentne bez tagu:

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 \
  -t "voda/septik/cmd/log/level" \
  -m "WARN"
```

### Interaktivni skript

Jednotny helper spustis prikazem:

```bash
zalevaci
```

Napoveda a neinteraktivni prikazy:

```bash
zalevaci --help
```

V menu je volba `Nastavit log level`.

Skript umi najit tagy ve zdrojacich (`main`, `components`, `managed_components`) a nabidnout je interaktivne.

Priklad pouziti:

```bash
# 1) spust helper
zalevaci
# 2) vyber "Nastavit log level"
# 3) vyber tag (nebo *) a uroven

# nebo neinteraktivne:
zalevaci log --tag mqtt_cmd --level DEBUG
```

Skript si stejne jako ostatni CLI helpery umi nacist heslo ze souboru `~/.voda-septik/mqtt_password`.

### Vycisteni starych retained commandu

Pokud broker drzi stare retained zpravy na `cmd/*`, vycisti je jednim prikazem:

```bash
zalevaci
```

V menu vyber `Vycistit retained cmd/*`. Skript posle pro vsechny command topicy retained null zpravu (`-r -n`) a tim je na brokeru smaze.

## Jak to funguje uvnitr

- MQTT topic `cmd/log/level` je registrovany v centralni tabulce topicu (`main/mqtt_topics.h/.cpp`).
- Subscriber (`main/mqtt_commands.cpp`) payload parsuje jako:
  - `tag=level`
  - `tag:level`
  - `tag level`
  - `level` (nastavi default `*`)
- Uroven se mapuje na `esp_log_level_t` a aplikuje pres `esp_log_level_set(tag, level)`.
- Po kazde uspesne zmene se loguje potvrzeni s tagem a finalni urovni.

### Sjednoceni tagu v projektu

- V hlavnich souborech je pouzivany jednotny styl `static const char *TAG = "..."`.
- U `main/blikaniled.cpp` uz neni per-loop prepis log levelu; log level je rizeny centralne (default + MQTT command).
- Ve `state_manager` se defaultni log level inicializuje jen jednou, aby MQTT nastaveni nebylo po reconnectu prepisovano.

## Doporuceni pro provoz

- V beznem provozu nech globalni uroven na `WARN`.
- Pro kratke ladeni zvys uroven jen na konkretnim tagu (napr. `mqtt_commands=DEBUG`).
- Po dodiagnostikovani vrat hodnoty zpet na `WARN`, at se nesnizuje vykon a nezahlcuje serial/MQTT vystup.

## Ladeni tlaku (4-20 mA)

Pro ladeni tlakove vetve nastav tag `tlak` na `DEBUG`:

```bash
zalevaci log --tag tlak --level DEBUG
```

V debug logu sleduj zejmena:

- `raw` hodnoty pred/za filtrem (stabilita a sum)
- prepocitany proud `i=... mA` (zda odpovida realnemu senzoru)
- tlak `p=... bar` pred/za filtrem
- rozdil `dP=... bar` a odvozenou `clog=...%`

Prakticky postup pri kalibraci:

1. Pri znamem bodu (4 mA / 20 mA) over `raw` a uprav `tlk_raw_4ma`, `tlk_raw_20ma`.
2. Over mapovani rozsahu cidla pres `tlk_p_min` a `tlk_p_max`.
3. Podle realneho stavu filtru dolad `tlk_dp_100`, aby procenta zaneseni sedela.

## Ladeni zasoby (hladina -> litry)

Pro ladeni vetve zasoby nastav tag `zasoba` na `DEBUG`:

```bash
zalevaci log --tag zasoba --level DEBUG
```

V `DEBUG_PUBLISH` topicu `debug/zasoba` sleduj zejmena:

- `raw` (filtrovana ADC hodnota)
- `hladina_m` (interne vypoctena vyska hladiny)
- `objem_l` (finalni hodnota publikovana do `stav/zasoba/objem_l`)
- `area_m2` (konfiguracni plocha nadrze)

Konfigurace pro prepocet na objem:

- `lvl_raw_min`, `lvl_raw_max`, `lvl_h_min`, `lvl_h_max` (kalibrace vysky)
- `tank_area_m2` (pudorysna plocha nadrze, default `5.4` pro 2 × 2.7 m)

Pouzity vztah:

- `objem_l = vyska_m * tank_area_m2 * 1000`
