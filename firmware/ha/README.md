# Home Assistant a MQTT v provozu

Podklady odpovídají provozu na **amur**, ověřeno 13. 10. 2026: Home Assistant **2026.2.3**, Mosquitto **2.1.2**. Obrazy jsou připnuté na tyto verze, nikoli na pohyblivé `stable` / `2`. Toto není úplná soukromá záloha existující instalace; Git obsahuje konfiguraci a postup vytvoření služby, nikoli hesla, účty nebo databázi.

## Uspořádání

- `docker-compose.homeassistant.yml`: samostatný projekt `homeassistant`, síť hostitele, TZ Europe/Prague, alias `mqtt` → 127.0.0.1, lokální čas hostitele. Port HA je 8123.
- `docker-compose.mqtt.yml`: samostatný projekt `mqtt`, externí Docker síť `veve`, porty 1883/9001, perzistentní data a logy.
- `homeassistant/`: verzovaná konfigurace HA včetně automatizací, skriptů, scén, blueprintů a `packages/filter.yaml`.
- `mosquitto/config/mosquitto.conf`: autentizace přes soubor `passwords`, anonymní přístup zakázán, perzistence zapnuta.

Relativní mounty jsou upravené pro tuto složku. Na amuru má HA konfiguraci `/srv/docker/homeassistant/config` a MQTT `/srv/docker/mqtt/config`, data a logy v sousedních adresářích. Stávající provoz se tímto commitem nepřesouvá ani nepřenasazuje.

## Vytvoření nové instalace

Předpoklady: Linux s Docker Engine a Docker Compose v2+, nastavený čas Europe/Prague, dostupné porty 1883 a 8123. Síť hostitele a mount `/etc/localtime` odpovídají Linuxu na amuru; postup není určený pro Docker Desktop na Macu. DNS jméno brokeru používané ESP32 (v provozu `mqtt.veve`) musí ukazovat na nový server, nebo je nutné upravit konfiguraci ESP32.

Spouštěj z této složky:

```sh
mkdir -p mosquitto/data mosquitto/log
cp homeassistant/secrets.yaml.example homeassistant/secrets.yaml
# Pokud síť veve ještě neexistuje:
docker network create veve
```

Při obnově použij původní soukromý soubor `mosquitto/config/passwords`. Pro novou instalaci vytvoř uživatele `ha` interaktivně (heslo se neobjeví v příkazu ani v Gitu):

```sh
docker run --rm -it --user 0 \
  -v "$PWD/mosquitto/config:/mosquitto/config" \
  --entrypoint mosquitto_passwd eclipse-mosquitto:2.1.2 \
  -c /mosquitto/config/passwords ha
```

`-c` vytváří nový soubor a přepíše případný existující: používej jej pouze při první instalaci. Další účty přidávej stejným příkazem bez `-c`. Pokud ESP32 zachovává staré přihlašovací údaje, použij odpovídající heslo nebo aktualizuj jeho konfiguraci. Hashovaný soubor hesel je také soukromý a není v Gitu.

Připrav oprávnění pro uživatele Mosquitto (UID/GID 1883 v tomto obrazu):

```sh
sudo chown -R 1883:1883 mosquitto/data mosquitto/log
sudo chown 1883:1883 mosquitto/config/passwords
sudo chmod 600 mosquitto/config/passwords

docker compose -p mqtt -f docker-compose.mqtt.yml config --quiet
docker compose -p homeassistant -f docker-compose.homeassistant.yml config --quiet
docker compose -p mqtt -f docker-compose.mqtt.yml up -d
docker compose -p homeassistant -f docker-compose.homeassistant.yml up -d
```

Otevři `http://ADRESA_SERVERU:8123`, vytvoř vlastní účet a přes **Nastavení → Zařízení a služby → Přidat integraci → MQTT** připoj broker `mqtt` (nebo `127.0.0.1`), port 1883, uživatele `ha` a vytvořené heslo. MQTT integrace, účty a nastavení vytvořená v UI jsou uloženy v `.storage`; samotné YAML je nenahradí. ESP32 publikuje HA discovery automaticky při připojení; při obnově po ztrátě MQTT dat může být nutné znovu připojit ESP32.

Port 9001 je v Compose publikovaný stejně jako v provozu, ale WebSocket listener v mosquitto.conf je vypnutý. Konfigurace HTTP trusted_proxies odpovídá proxy na loopbacku amuru; pokud používáš jinou proxy, uprav její skutečné adresy.

## Kontrola a výpočet zanesení

```sh
docker exec homeassistant python -m homeassistant --script check_config --config /config
docker compose -p mqtt -f docker-compose.mqtt.yml logs --tail 30
docker compose -p homeassistant -f docker-compose.homeassistant.yml logs --tail 30
```

Package filtru se načítá automaticky z `configuration.yaml`. Uchovává poslední platné měření přes restart, ignoruje neplatná/duplicitní/starší měření, doplňuje čas přijetí a počítá procenta a příznak čištění. Meze v `homeassistant/packages/filter.yaml` jsou **prozatímní**: pásmo necitlivosti 0.05 bar, čištění 1.0 bar, zrušení 0.8 bar. Nastav je podle kalibrace a konkrétního filtru.

Nový HA nemá žádnou předchozí platnou hodnotu: výsledek bude neznámý až do prvního ustáleného průtoku nejméně 25 l/min a přijetí dokončeného okna. Obnovená instalace má výsledek z vlastní zálohy; vytvoření nového systému není obnovení historie. Ověření původního nasazení a archivního přehrání je v [evidenci v3.0.0](../docs/nasazeni-v3.0.0.md).

## Úplná obnova existujícího provozu

Pro přesnou obnovu jsou navíc potřeba **soukromé zálohy mimo Git**:

- celý `/srv/docker/homeassistant/config` včetně `.storage`, `secrets.yaml`, databáze a případných dalších uživatelských souborů;
- `/srv/docker/mqtt/config` včetně `passwords` a `/srv/docker/mqtt/data` (retained zprávy), případně logy;
- použité Compose definice, DNS/proxy a případná konfigurace firewallu.

Použij plnou zálohu HA vytvořenou jeho zálohovací funkcí, nebo konzistentní souborovou zálohu při zastavených kontejnerech. Nekopíruj živou SQLite databázi bez odpovídajících WAL dat. Zálohy obsahují přístupové údaje a ukládej je soukromě; nedávej je do repozitáře. Po obnově obnov také vlastníky a oprávnění. Nezaměňuj tuto složku s verzovanou zálohou `.storage`.

`.gitignore` v této složce vylučuje hesla, databáze, interní úložiště, zálohy a provozní data. Soubor `secrets.yaml.example` je pouze prázdná šablona. Hesla v historii Gitu tento mechanismus neodstraňuje; tato změna nezapisuje žádná nová provozní hesla.
