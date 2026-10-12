# voda-septik

Repozitář obsahuje ESP-IDF firmware a KiCad návrh hardware. Provozní dokumentace je v [firmware/README.md](firmware/README.md).

## Struktura

- `firmware/main/` – aplikace; `main/archive/` obsahuje historický referenční kód.
- `firmware/components/` – vlastní komponenty.
- `firmware/managed_components/` – externí knihovny, jejichž obsah je uložený v Gitu.
- `firmware/dependencies.lock` – přesné verze externích závislostí.
- `firmware/sdkconfig` – úplná konfigurace sestavení pro ESP32.
- `firmware/sdkconfig.defaults` – doplňující výchozí nastavení.
- `firmware/partitions.csv` – rozdělení 4MB flash paměti.
- `firmware/tools/zalevaci` – nástroj pro MQTT příkazy, diagnostiku a OTA.
- `firmware/ha/` – konfigurace Home Assistantu a Mosquitto pro Docker Compose.
- `hardware/` – KiCad projekt, schéma, PCB a vlastní knihovny; mapa v [hardware/README.md](hardware/README.md).
- `AGENTS.md` a `.agents/` – pracovní instrukce a mapy projektu.

## Verze a větve

- Firmware: **ESP-IDF v5.5.2**, target **esp32**. Stejnou verzi používá automatické sestavení v `.github/workflows/build.yml`.
- Hardware byl upravován v **KiCadu 9**.
- Nová funkcionalita vzniká na `develop`, nasazuje se přes USB/OTA a testuje. Po potvrzení OTA se `main` posune na skutečně provozovaný commit. Provozní opravy vznikají na `main` a ihned se nasazují; následně se přenesou do develop. Čistá dokumentace patří na obě větve bez zvýšení release verze. Úplný postup a mechanismus Git verzí jsou v [pravidlech vývoje a nasazení](firmware/docs/vyvoj-a-nasazeni.md).

Obnovu začni z `main`, pokud nepotřebuješ konkrétní vývojovou verzi. IDF ani verze knihoven se při obnově automaticky neaktualizují na novější řadu.

## Obnova na čistém počítači

### 1. Získání projektu

Nainstaluj Git a zajisti přístup ke GitHub repozitáři. Na macOS/Linuxu:

```sh
mkdir -p ~/Projects
cd ~/Projects
git clone git@github.com:marvertin/voda-septik.git
cd voda-septik
git switch main
```

Pokud nemáš nastavený SSH klíč, použij HTTPS adresu `https://github.com/marvertin/voda-septik.git` a přihlas se ke GitHubu. SSH klíče nejsou součástí tohoto repozitáře.

### 2. Instalace ESP-IDF

Použij [oficiální instalační postup pro ESP-IDF v5.5.2](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32/get-started/index.html) včetně systémových předpokladů pro svůj operační systém.

Pro ruční instalaci na macOS/Linuxu do umístění očekávaného projektovým `.envrc`:

```sh
mkdir -p ~/.espressif/v5.5.2
git clone --branch v5.5.2 --recursive https://github.com/espressif/esp-idf.git ~/.espressif/v5.5.2/esp-idf
cd ~/.espressif/v5.5.2/esp-idf
./install.sh esp32
```

Na Windows vyber ve vhodném instalátoru nebo rozšíření ESP-IDF verzi **v5.5.2** a otevři jeho připravený terminál. Projektové `.envrc` je určené pro macOS/Linux, není podmínkou sestavení.

Instalace IDF, překladače a Python prostředí jsou obnovitelné nástroje a do projektového Gitu nepatří.

### 3. Sestavení

Na macOS/Linuxu aktivuj IDF pro aktuální terminál:

```sh
source ~/.espressif/v5.5.2/esp-idf/export.sh
cd ~/Projects/voda-septik/firmware
idf.py --version
idf.py build
```

Ověř, že první příkaz vypíše verzi **5.5.2**. Target `esp32` je již uložený v `sdkconfig`; při obnově není potřeba spouštět `idf.py set-target`, který resetuje konfiguraci.

Výstup je `firmware/build/voda-septik.bin`. Složka `build/` se vytváří znovu a nezálohuje se v Gitu. Po přesunu projektu odstraň staré `build/`, aby nezůstaly absolutní cesty z původního umístění.

Volitelně lze použít `direnv`: nainstaluj ho, zapoj do svého shellu a v `firmware/` spusť `direnv allow`. Projektový `.envrc` přidá i `firmware/tools` do `PATH`. Při jiném umístění IDF uprav jeho cestu nebo aktivuj IDF ručně.

### 4. Nahrání přes USB

Připoj ESP32 a zjisti jeho sériový port. Z `firmware/` spusť:

```sh
idf.py -p PORT flash monitor
```

Nahraď `PORT` skutečným portem, například `/dev/cu.usbserial-…` na macOS nebo `COM3` na Windows. Monitor se ukončuje pomocí `Ctrl+]`.

Pro nahrání používej projektovou tabulku oddílů a sestavený bootloader. Příkaz `erase-flash` smaže také nastavení a provozní hodnoty uložené v zařízení; pro běžnou obnovu počítače není potřeba.

## MQTT a OTA

Nástroj `tools/zalevaci` vyžaduje Python 3; pro MQTT operace také příkazy `mosquitto_pub` a pro OTA `mosquitto_sub`. Spouštěj jej ze složky `firmware/`:

```sh
./tools/zalevaci --help
./tools/zalevaci ota --help
```

Broker a přihlašování lze nastavit parametry nástroje nebo proměnnými prostředí, například `MQTT_HOST`, `MQTT_PORT`, `MQTT_USER` a `MQTT_PASS_FILE`. Výchozí adresy brokerů jsou lokální pro původní síť a na jiném místě je potřeba je změnit.

MQTT heslo si nástroj ukládá do `~/.voda-septik/mqtt_password`. Tento soubor **není v repozitáři**: heslo uchovej ve správci hesel nebo bezpečné záloze. Nástroj si při prvním použití vyžádá chybějící heslo. Heslo nezapisuj do README ani jej nepřidávej do Gitu.

OTA používá MQTT příkazy a HTTP(S) stažení obrazu. Nový firmware po ověření funkčnosti vyžaduje potvrzení kvůli rollbacku. Aktuální rozhraní helperu popisuje `./tools/zalevaci ota --help`; interaktivní i neinteraktivní postup a rollback popisuje [firmware/docs/ota.md](firmware/docs/ota.md).

## Obnova hardware v KiCadu

1. Nainstaluj KiCad 9 a jeho standardní knihovny.
2. Otevři `hardware/Voda septik.kicad_pro`.
3. Projektové tabulky `sym-lib-table` a `fp-lib-table` připojují knihovnu `martin` přes `${KIPRJMOD}`. Díky tomu nezávisí na absolutní cestě k projektu.
4. Přes správce zásuvných modulů a obsahu nainstaluj **Espressif KiCad Libraries**. Návrh používá knihovnu s názvem `PCM_Espressif`; ověř její registraci v tabulce symbolů i footprintů. Pokud chybí, připoj odpovídající nainstalovanou knihovnu pod tímto názvem.

Použité symboly a footprinty jsou vložené také ve schématu a PCB, takže existující návrh lze otevřít. Pro aktualizace z knihoven a vkládání dalších součástek je potřeba správně připojit externí knihovny. Verze balíčku Espressif zatím není v projektu připnutá.

Automatické zálohy KiCadu, zámky a osobní nastavení editoru se v Gitu neuchovávají.

## Co zálohovat mimo Git

Git uchovává zdroje a konfiguraci pro sestavení, nikoli celý provozní stav:

- MQTT heslo a případné další přístupové údaje uložené mimo repozitář.
- Nastavení, kalibrace a čítače v ESP32: NVS a datové oddíly zařízení nejsou součástí zdrojového kódu. Pro úplnou obnovu zařízení udržuj samostatný záznam nastavení nebo zálohu jeho datových oddílů.
- Při provozování Home Assistantu také jeho úplnou zálohu. Git obsahuje YAML konfiguraci, ale ne `.storage`, databázi, uživatele a nastavení vytvořená přes UI. Obdobně případná provozní data Mosquitto vyžadují vlastní zálohu.

Konfigurace Docker Compose používá pohyblivé značky obrazů `stable` a `2`; sama nezaručuje obnovu totožné verze služeb.

Lokální commity, necommitované soubory a stashe nejsou automaticky na GitHubu. Po dokončení změn je commitni a odešli do příslušné větve. Stav ověř pomocí `git status`, `git fetch origin` a `git branch -vv`.

## Ověření obnovitelnosti

Dne **12. 10. 2026** byl nový klon větve `main` z GitHubu sestaven bez původní složky `build/` a bez necommitovaných souborů projektu. Sestavení s ESP-IDF **5.5.2** pro **ESP32** prošlo. Test použil již nainstalované nástroje; nová instalace vývojového prostředí ani flashování fyzického zařízení nebyly součástí testu.
