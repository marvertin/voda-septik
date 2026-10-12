# AGENTS.md

Pracovní instrukce pro repozitář `voda-septik`. Stav dokumentace ověřen proti kódu 12. 10. 2026.

## Orientace

- `README.md` – obnova prostředí, sestavení, Git a zálohy.
- `firmware/README.md` – provoz, konfigurace, kalibrace a MQTT ovládání.
- `hardware/README.md` – KiCad, knihovny, piny, ADC a vazba na firmware.
- `.agents/PROJECT_INDEX.md` – moduly, start aplikace a tok dat.
- `.agents/MQTT_INDEX.md` – úplný registr topiců, dostupnost a command payloady.
- `.agents/CONFIG_INDEX.md` – všechny konfigurační položky, výchozí hodnoty a meze.

## Pravidla práce

- Firmware používá ESP-IDF **v5.5.2**, target **esp32**; shodnou verzi má CI. Změnu IDF neprováděj jako vedlejší efekt dokumentace.
- Aktivní aplikaci určuje explicitní seznam v `firmware/main/CMakeLists.txt`: C bridge volá `cpp_app_main` v `main/voda-septik.cpp`. `main/archive/` je referenční kód.
- Před změnou MQTT uprav centrální registr `mqtt_topics.h/.cpp`, zkontroluj HA discovery, publisher, command handler, helper i dokumentaci. Topic ID musí odpovídat pořadí tabulky.
- Novou konfiguraci definuj jako `config_item_t`, registruj v příslušné sekci při startu a aktualizuj `.agents/CONFIG_INDEX.md`. Uvedení popisku ve webu samo neznamená, že aplikace položku skutečně používá.
- Přístupové údaje z konfigurace a lokálního souboru `~/.voda-septik/mqtt_password` nekopíruj do dokumentů nebo výstupů.
- Kódy `APP_ERROR_CHECK("Exxx", ...)` musí být unikátní v aktivních zdrojích; inventura je v `firmware/docs/error-codes.md`.
- `firmware/managed_components/` jsou spravované knihovny uložené v Gitu. Bez výslovného zadání o závislostech je neupravuj, včetně jejich Markdown dokumentace.
- Runtime změny ověř sestavením z `firmware/`: `idf.py build`. Izolované sestavení lze spustit pomocí `idf.py -B build-codex build`; jeho výstupy nepřidávej do Gitu.
- U čisté dokumentace ověř lokální odkazy, skutečné API, klíče, jednotky a příkazy proti zdrojům. Nové sestavení není nutné, pokud se kód ani konfigurace nezměnily.
- Necommituj build výstupy, `.DS_Store`, lokální flash/monitor logy ani provozní hesla.

## Vývoj, větve, nasazení a verze

Závazný postup je v [firmware/docs/vyvoj-a-nasazeni.md](firmware/docs/vyvoj-a-nasazeni.md).

- Novou funkcionalitu vyvíjej na **develop**. Podle změny nasazuj lokálně přes USB nebo vzdáleně přes **OTA** a otestuj ji.
- Po úspěšném testu a potvrzení OTA posuň **main na konkrétní commit skutečně běžící v provozu**, nikoli automaticky na nejnovější netestovaný develop. U USB musí být obdobně ověřené nasazení a provoz.
- Opravu provozního problému dělej na **main**, aktualizuj patch verzi a **ihned ji nasaď**, otestuj a při OTA potvrď. Potom ji přenes do develop. Selhané/čekající nasazení nezapisuj jako dokončené.
- Čistou dokumentaci commituj na **obou větvích**; jsou-li srovnané, tentýž commit přes fast-forward. Při rozdílném runtime obsahu přenes pouze dokumentaci. Samotná dokumentace nezvyšuje release verzi a nenasazuje firmware.
- Před prací načti vzdálené větve/tagy a ověř pracovní strom; nepoužívej force-push jako náhradu za sladění historie.
- Verze aplikace nyní pochází z `git describe --always --tags --dirty`. Nové release tagy mají formát `vMAJOR.MINOR.PATCH`: patch opravy, minor kompatibilní funkce, major nekompatibilní změny. Verzový tag připrav na čistém commitu před finálním buildem; již zveřejněné tagy neposouvej.
- Eviduj úplné SHA commitu, tag/popisek, SHA-256 binárního obrazu a potvrzení/test nasazení. Nepovažuj `diag/git_hash` za skutečné SHA: aktuálně vrací prefix verze před první pomlčkou.
- Změna IDF není zvýšení aplikační verze. Nevytvářej novou release verzi jen kvůli těmto instrukcím.
- Dokumentační HEAD main může být před posledním nasazeným obrazem; přesný provozovaný commit určuje evidence/tag. Odeslání MQTT commandu nebo úspěšné CI není důkazem potvrzeného OTA.

## Rychlá kontrola

```sh
cd firmware
idf.py --version
idf.py build
./tools/zalevaci --help
```

Při změnách hardwaru začni v `main/pins.h`, `main/ads1115.cpp`, `main/lcd.cpp`, `main/elektromer.cpp` a KiCad schématu. Zachovej obě samostatné I2C sběrnice a datové oddíly flash čítače.
