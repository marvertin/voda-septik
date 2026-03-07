# Error kódy (`Exxx`) – mapování a pravidla

## Pravidla

- Formát kódu je `E` + 3 číslice.
- První číslice určuje modul (rozsah `E1xx` až `E9xx`).
- V rámci modulu se kódy přidělují postupně bez mezer (`E101`, `E102`, ...).
- Kódy musí být v projektu unikátní.
- `cpp_app_main` v `main/voda-septik.cpp` začíná na `E101`.

## Mapa modulů

| Modul | Soubory | Rozsah |
|---|---|---|
| `cpp_app_main` | `main/voda-septik.cpp` | `E101–E199` |
| `network_core` | `components/network_core/network_init.cpp` | `E201–E299` |
| `config_store` | `components/config_store/config_store.cpp` | `E301–E399` |
| `restart_info` | `main/restart_info.cpp` | `E401–E499` |
| `mqtt_runtime` | `main/mqtt_commands.cpp`, `main/mqtt_publisher_task.cpp` | `E501–E599` |
| `state_manager` | `main/state_manager.cpp` | `E601–E699` |
| `sensor_stack` | `main/adc_shared.cpp`, `main/prutokomer.cpp`, `main/teplota.cpp`, `main/tlak.cpp`, `main/tlak2.cpp`, `main/zasoba.cpp` | `E701–E799` |
| `config_items` | `main/network_config.cpp`, `main/system_config.cpp` | `E801–E899` |
| `display_lcd` | `main/lcd.cpp` | `E901–E999` |

## Kontrola duplicit

Použij projektové hledání na výraz `\bE\d{3}\b` a ověř, že každý kód je použit právě jednou.