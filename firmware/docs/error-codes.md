# Error kódy (`Exxx`) – mapování a pravidla

## Pravidla

- Formát kódu je `E` + 3 číslice.
- První číslice určuje modul (např. `E1xx` = start aplikace, `E8xx` = elektroměr).
- V rámci modulu se kódy přidělují postupně (`E101`, `E102`, ...); mezera je přípustná jen po odstraněném kódu.
- Kódy musí být v aktivně překládaném firmware unikátní. Archivované zdroje v `main/archive` se do kontroly produkčního firmware nepočítají.
- Jeden kód smí označovat právě jedno konkrétní místo volání `APP_ERROR_CHECK`.
- `cpp_app_main` v `main/voda-septik.cpp` začíná na `E101`.

## Mapa modulů

| Modul | Soubory | Rozsah |
|---|---|---|
| `cpp_app_main` | `main/voda-septik.cpp` | `E101–E199` |
| `network_core` | `components/network_core/network_init.cpp` | `E201–E299` |
| `config` | `components/config_store/config_store.cpp`, `main/network_config.cpp`, `main/system_config.cpp` | `E301–E399` |
| `restart_info` | `main/restart_info.cpp` | `E401–E499` |
| `mqtt_runtime` | `main/mqtt_commands.cpp`, `main/mqtt_publisher_task.cpp` | `E501–E599` |
| `state_manager` | `main/state_manager.cpp` | `E601–E699` |
| `sensor_stack` | `main/prutokomer.cpp`, `main/tlak.cpp`, `main/zasoba.cpp`, `main/teplota.cpp`, `main/ads1115.cpp` | `E701–E799` |
| `elektromer` | `main/elektromer.cpp` | `E801–E899` |
| rezerva | zatím nepoužito | `E901–E999` |

## Kontrola duplicit

Použij projektové hledání na výraz `\bE\d{3}\b` a ověř, že každý kód v aktivních zdrojích je použit právě jednou. Při kontrole ignoruj dokumentaci s příklady a `main/archive`.
