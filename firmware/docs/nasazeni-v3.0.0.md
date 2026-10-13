# Nasazení v3.0.0 – měření diskového filtru

Potvrzeno 2026-10-13T20:06:55+02:00, zařízení `voda-septik.veve` (192.168.2.33), MQTT `voda/septik`.

- Zdrojový commit: `ead3876f6d7ac378c25ca0afd4f2f687e9d51f21`.
- Neměnný anotovaný tag vytvořen před finálním buildem: `v3.0.0`.
- SHA-256 obrazu: `96556a646126e09e51df4cc8a247b5d82f3c2ca0dfe4e7bfe61893d926088497`.
- ESP-IDF v5.5.2, finální `idf.py reconfigure build` z čistého commitu/tagu; verze ověřena v metadatech a zařízení.
- OTA z v2.3.0, živě ověřen pending boot, šest platných oken a uchování bez vody. Až poté odesláno potvrzení; zachycena skutečná událost `confirmed`.
- Po následném restartu: v3.0.0, `boot_mode=normal`, boot_count 34, senzory ok, NVS errors 0. Nové měření správně začíná `valid=false`, ID 0, hodnoty null.
- Main posunut na přesný otestovaný zdrojový commit. Tato následná evidence je dokumentační změna a nemění běžící obraz.

## Živé výsledky

| ID | Δp [bar] | Průtok [l/min] | Směrodatná odchylka [bar] | Ponechané vzorky |
| --- | --- | --- | --- | --- |
| 1 | 1.233499 | 47.9544 | 0.012976 | 33 |
| 2 | 1.227628 | 48.4617 | 0.010078 | 33 |
| 3 | 1.224515 | 48.2942 | 0.010657 | 33 |
| 4 | 1.214003 | 48.4748 | 0.009106 | 33 |
| 5 | 1.2145 | 48.3589 | 0.013349 | 33 |
| 6 | 1.212914 | 48.0336 | 0.008104 | 33 |

Při zastavení vody poslední platné měření **č. 8**, Δp **1.209789 bar**, průtok při měření **48.8792 l/min**, σ **0.01453 bar**. Po čerstvě doložené nule byla během 75 s zachycena obnovená publikace naprosto shodného JSON včetně ID. Pro rozhodování o vodě byly vyloučeny retained zprávy (`mosquitto_sub -R`).

## Ostatní ověření a hranice

- Host testy monitoru prošly: hranice průtoku, navazující okna, uchování výsledku, čištění, podepsané Δp, ořez špiček, velký šum, společné tlakové přechody, zastaralý průtok, výpadky/NaN, kolísání průtoku, restart a korekce offsetu.
- HA package prošel parsováním YAML/Jinja a izolovanými testy nových/duplicitních/starších dvojic, neplatné zprávy po restartu, pásma necitlivosti a hystereze. **V provozním HA není nasazen ani ověřen**, protože místo konfigurace nebylo dosud určeno. Procenta a příznak čištění v provozním HA jsou tedy stále nedokončené.
- Všechna šest platných oken byla přijata; sensor queue drops=0. MQTT publish queue drops byl 48 po startu a 55 po rozběhu vody, dále při měření stabilní. Po ověřovacím restartu opět bootový náraz 47. Zprávy nových výsledků 1–8 i držené hodnoty byly doručeny; kapacita publikační fronty může při nárazové publikaci dalších veličin zahazovat enqueue a zaslouží samostatné řešení. Volná paměť při měření přibližně 84 kB.
- Meze kvality a hydraulické meze HA jsou prozatímní nastavitelné hodnoty; offset čidel je 0, kalibrace společným tlakem nebyla provedena. Vysoké Δp samo není důkaz konkrétního stupně zanesení bez ověření kalibrace a mezí konkrétního filtru.
- Hlasové výzvy Martinovi byly obnoveny po 20:00; po měření zazněla výzva Heleně. Z telemetrie je doloženo zastavení vody, nikoli návštěva Heleny nebo návrat Martina domů.
- **Push neproveden.** Automatická kontrola odmítla export do neověřeného origin bez explicitního souhlasu. Develop, main a tag jsou zatím lokální; čeká se na souhlas uživatele.
