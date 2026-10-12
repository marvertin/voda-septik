# Nasazení v2.3.0 – využitelná zásoba vody

Dne 12. 10. 2026 přibližně ve 22:06–22:10 (Europe/Prague) nasazeno a ověřeno na produkčním zařízení `voda-septik.veve` (192.168.2.33), MQTT `voda/septik`.

- Zdrojový commit: `2e119cea2a3e3ca8e9a48b8807293f14ccec4882`.
- Anotovaný release tag a verze obrazu: `v2.3.0`.
- SHA-256 `voda-septik.bin`: `6d9726f45113a50a6bc501eb6fbc672093f798f4e9a7b23710208ff91b0c2970`.
- Sestavení: ESP-IDF v5.5.2, ESP32, čistý commit; `idf.py reconfigure build` úspěšné, verze ověřena v `project_description.json` i zařízení.
- Předchozí skutečná verze: `v2.2-1-ge96e916`; změny zdrojů mezi ní a výchozím HEAD byly pouze dokumentace.
- Přenos: OTA přes místní HTTP server; stažení 100 %, událost `rebooting`, následně `system/boot_mode=ota` a `diag/fw_version=v2.3.0`.
- Po ověření měření odeslán `cmd/ota/confirm=1`; skutečně přijata událost `system/ota/event=confirmed`.
- Další restarty při uložení konfigurace: stále `v2.3.0`, `system/boot_mode=normal`, MQTT online, ADS1115/Modbus/teploty `ok`.

## Ověření funkce

Skutečná C++ funkce lokálně ověřena pro zápornou/nulovou výšku, výšku pod mezí, přesně na mezi a nad mezí; pro nastavení 0, 10 a 20 cm, zaokrouhlení i neplatné měření NaN. Využitelná zásoba pod mezí je 0; NaN zůstává neplatným měřením.

Konfigurační web v sekci Zásoba obsahuje `lvl_unpump_cm`, jednotka cm, výchozí hodnota 10, meze 0–500. Bez staré NVS položky bylo skutečně použito 10 cm. Testovací změna na 20 cm byla uložena přes web a přežila restart; živá měření odpovídala `(hladina_m − 0.20) × 5.4`. Poté obnoveno 10 cm a ověřeno jeho přetrvávání po dalším restartu. Původní kalibrace RAW 9335/29493, výška 0.330/1.930 m a plocha 5.400 m² zůstaly zachovány.

Po posledním restartu automaticky ověřeno 17 živých dvojic MQTT hladina/zásoba proti `round(max(0, hladina_m − 0.10) × 5.4, 2)`. Výška se publikuje skutečná; odečet je pouze ve výpočtu objemu. Filtry po restartu postupně dobíhají k ustálenému měření jako v předchozí verzi.

Závěrečné ustálené měření: skutečná hladina **0.926085 m**, využitelná zásoba **4.46 m³**, `diag/nvs_errors=0`, verze `v2.3.0`, boot `normal`. Před nasazením při téměř stejné hladině 0.926170 m byla zásoba 5.00 m³.

Provozní nastavení při dokončení: **10 cm**. Dočasně zapnutý debug i konfigurační web byly vypnuty. Větev main posunuta fast-forward na přesný nasazený commit; následné dokumentační commity nemění tento obraz. Nasazení je **potvrzené a ověřené**.
