# AGENTS.md

Pracovni instrukce pro agenty v repozitari `voda-septik`.

## Rychly obraz

- Projekt je rozdeleny na `firmware/` a `hardware/`.
- `firmware/` je ESP-IDF projekt pro target `esp32`, CI pouziva ESP-IDF `v5.5.2`.
- `hardware/` obsahuje KiCad projekt, schemata, PCB a vlastni knihovny footprintu/symbolu.
- Detailni mapy jsou v `.agents/`:
  - `.agents/PROJECT_INDEX.md`
  - `.agents/MQTT_INDEX.md`
  - `.agents/CONFIG_INDEX.md`

## Pravidla prace

- Hlavni aplikace startuje v `firmware/main/voda-septik.cpp`, kde se registruje konfigurace, startuje sit, MQTT, fronty a senzoricke tasky.
- Pred zmenami MQTT topiku vzdy upravuj centralni registr v `firmware/main/mqtt_topics.h` a `firmware/main/mqtt_topics.cpp`; Home Assistant discovery je na nem zavisle.
- Pred pridanim konfiguracni polozky ji zaregistruj pres `config_store_register_item`, zarad ji do spravne sekce v `cpp_app_main` a zkontroluj `.agents/CONFIG_INDEX.md`.
- Tvrdym chybam `APP_ERROR_CHECK("Exxx", ...)` pridelej unikatni kod podle `firmware/docs/error-codes.md`.
- Aktivni firmware je v `firmware/main` a `firmware/components`; `firmware/main/archive` ber jako referencni/legacy kod.
- `firmware/managed_components` jsou vendored/managed zavislosti. Neupravuj je, pokud neni zadani vyslovne o zavislosti.
- Pokud menis runtime chovani firmware, over aspon buildem z `firmware/`: `idf.py build`. Pro izolovany build lze pouzit schvaleny prikaz `idf.py -B build-codex build`.
- Necommituj ani neukladej build artefakty, `.DS_Store`, hesla mimo existujici projektove nastaveni ani lokalni vystupy z flash/monitoringu.

## Uzitecne prikazy

```bash
cd firmware
idf.py build
idf.py -B build-codex build
zalevaci --help
rg -n "\\bE\\d{3}\\b" firmware/main firmware/components
rg -n "TOPIC_ENTRY\\(" firmware/main/mqtt_topics.cpp
rg -n "config_store_register_item|static const config_item_t" firmware/main firmware/components
```
