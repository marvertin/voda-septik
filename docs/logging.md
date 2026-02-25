# Logovani a zmena log levelu pres MQTT

## Jak to uzivatelsky pouzivat

### Rychle nastaveni jedne hodnoty

Pro zmenu log levelu konkretniho tagu publikuj command na topic:

- topic: `zalivka/nadrz/cmd/log/level`
- payload: `TAG=LEVEL`

Priklad:

```bash
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 \
  -t "zalivka/nadrz/cmd/log/level" \
  -m "mqtt_cmd=DEBUG"
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
  -t "zalivka/nadrz/cmd/log/level" \
  -m "*=WARN"
```

### Interaktivni skripty

1) Jednoduche menu commandu:

```bash
./tools/mqtt_cmd_cli.sh
```

V menu je volba `log/level`.

2) Pokrocilejsi helper pro logy (umi najit tagy ve zdrojacich):

```bash
# vypis nalezenych tagu (prohleda main, components i managed_components)
./tools/mqtt_log_level_cli.py --list

# nastaveni konkretniho tagu
./tools/mqtt_log_level_cli.py --tag mqtt_cmd --level DEBUG

# nastaveni globalne
./tools/mqtt_log_level_cli.py --all --level WARN
```

Skript si stejne jako ostatni CLI helpery umi nacist heslo ze souboru `~/.zalevaci-nadrz/mqtt_password`.

### Vycisteni starych retained commandu

Pokud broker drzi stare retained zpravy na `cmd/*`, vycisti je jednim prikazem:

```bash
./tools/clear_retained_cmds.sh
```

Skript posle pro vsechny command topicy retained null zpravu (`-r -n`) a tim je na brokeru smaze.

## Jak to funguje uvnitr

- MQTT topic `cmd/log/level` je registrovany v centralni tabulce topicu (`main/mqtt_topics.h/.cpp`).
- Subscriber (`main/mqtt_commands.cpp`) payload parsuje jako:
  - `tag=level`
  - `tag:level`
  - `tag level`
- Uroven se mapuje na `esp_log_level_t` a aplikuje pres `esp_log_level_set(tag, level)`.
- Po kazde uspesne zmene se loguje potvrzeni s tagem a finalni urovni.

### Sjednoceni tagu v projektu

- V hlavnich souborech je pouzivany jednotny styl `static const char *TAG = "..."`.
- U `main/blikaniled.cpp` uz neni per-loop prepis log levelu; log level je rizeny centralne (default + MQTT command).
- Ve `state_manager` se defaultni log level inicializuje jen jednou, aby MQTT nastaveni nebylo po reconnectu prepisovano.

## Doporuceni pro provoz

- V beznem provozu nech globalni uroven na `WARN`.
- Pro kratke ladeni zvys uroven jen na konkretnim tagu (napr. `mqtt_cmd=DEBUG`).
- Po dodiagnostikovani vrat hodnoty zpet na `WARN`, at se nesnizuje vykon a nezahlcuje serial/MQTT vystup.
