# Chybové kódy `Exxx`

Ověřeno proti aktivním zdrojům 12. 10. 2026. `APP_ERROR_CHECK` při neúspěšném výsledku hlásí kód přes registrovaný handler a následně volá `ESP_ERROR_CHECK`. Handler displeje může nejprve zobrazovat kód na TM1637.

## Pravidla přidělování

- Formát je `E` a tři číslice; kód musí být unikátní pro konkrétní místo v aktivně překládaném firmware.
- Používej rozsah daného modulu; nově přidávané kódy zapisuj i sem. Historické mezery jsou přípustné, nepovažuj je automaticky za volné kódy.
- Archiv `main/archive/`, příklady knihoven a dokumentační ukázky se do aktivní inventury nepočítají.
- Selhání základní platformy, front a tasků je tvrdá chyba. Běžné odpojení sítě nebo senzoru se řeší diagnostikou a nedostupností.

## Rozsahy

| Rozsah | Modul |
| --- | --- |
| E1xx | start aplikace |
| E2xx | network_core |
| E3xx | konfigurace a config_store |
| E4xx | historicky restart_info; aktuálně žádný aktivní kód |
| E5xx | MQTT commands a publisher |
| E6xx | state_manager |
| E7xx | průtok, tlak, hladina, teplota, ADS1115 |
| E8xx | elektroměr |
| E9xx | rezerva |

## Aktuální inventura

V aktivních modulech je **141 unikátních kódů**. Cesty jsou relativní ke `firmware/`.

| Soubor | Použité kódy |
| --- | --- |
| `components/config_store/config_store.cpp` | `E301`, `E302`, `E303`, `E304`, `E305`, `E306`, `E307`, `E308`, `E309`, `E310`, `E311`, `E312`, `E313`, `E314`, `E315`, `E316`, `E317`, `E318` |
| `components/network_core/network_init.cpp` | `E201`, `E202`, `E203`, `E204`, `E205`, `E206`, `E207`, `E208`, `E209`, `E210`, `E211`, `E212`, `E213` |
| `main/ads1115.cpp` | `E780`, `E781`, `E782`, `E783`, `E784` |
| `main/elektromer.cpp` | `E841`, `E842`, `E843`, `E844`, `E845`, `E846`, `E847`, `E849`, `E850`, `E851`, `E866`, `E867` |
| `main/mqtt_commands.cpp` | `E501`, `E502`, `E503` |
| `main/mqtt_publisher_task.cpp` | `E504`, `E505`, `E506`, `E507`, `E508` |
| `main/network_config.cpp` | `E319`, `E320`, `E321`, `E322`, `E323` |
| `main/prutokomer.cpp` | `E706`, `E707`, `E708`, `E709`, `E711`, `E712`, `E713`, `E714` |
| `main/state_manager.cpp` | `E601`, `E602`, `E603`, `E604`, `E605`, `E606`, `E607`, `E608`, `E609`, `E610`, `E611`, `E612`, `E613`, `E614` |
| `main/system_config.cpp` | `E324`, `E325` |
| `main/teplota.cpp` | `E771`, `E772`, `E773`, `E774`, `E775`, `E776`, `E777` |
| `main/tlak.cpp` | `E723`, `E724`, `E725`, `E726`, `E727`, `E728`, `E729`, `E730`, `E731`, `E732`, `E733`, `E734`, `E735`, `E737`, `E738`, `E739`, `E740` |
| `main/voda-septik.cpp` | `E101`, `E102`, `E103`, `E104`, `E105`, `E106`, `E107`, `E108`, `E109`, `E110`, `E111`, `E112`, `E113`, `E114`, `E116`, `E117`, `E118`, `E119`, `E120` |
| `main/zasoba.cpp` | `E756`, `E757`, `E758`, `E759`, `E760`, `E761`, `E762`, `E763`, `E764`, `E765`, `E766`, `E767`, `E768` |

`restart_info.cpp` nyní vrací chyby volajícímu; `state_manager` je loguje a zvyšuje `diag/nvs_errors`. Původní tvrzení o fatalním zápisu restart metadat již neplatí.

Kontroluj kódy v aktivních `.cpp`/`.c` souborech, nikoli pouze prostým hledáním v celém repozitáři (zahrnovalo by i tuto tabulku a archiv). U config_store se kódy mohou předávat jeho pomocné funkci `check_or_abort`.
