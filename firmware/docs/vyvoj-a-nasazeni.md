# Vývoj, nasazení a verze firmware

Pravidla uživatele a aktuální mechanismus verzování, ověřené 12. 10. 2026. Tato pravidla platí pro budoucí práci; zápis pravidel sám neznamená, že byl nový firmware nasazen.

## Význam větví

- **`develop`**: dočasná větev pro novou funkcionalitu a její testování. Vytvoř ji z aktuální main při začátku vývoje; po úplném začlenění do main a ověřeném nasazení ji smaž lokálně i vzdáleně.
- **`main`**: provozní kód. Funkcionalita z develop sem patří až po úspěšném nasazení, testu a při OTA také potvrzení nového obrazu.
- Čistá dokumentace se ukládá na obě větve. Takový commit nemění běžící obraz, přesto mění Git HEAD a může změnit popis verze při příštím sestavení.

`main` proto označuje provozní linii zdrojů, ale kvůli dokumentačním commitům nemusí jeho aktuální HEAD být doslova commit použitý při posledním sestavení. Přesný provozovaný commit se určuje z evidence nasazení a neměnného tagu, nikoli jen z názvu větve.

## Nová funkcionalita

1. Načti vzdálený stav, ověř pracovní strom a vytvoř `develop` z aktuální `main`, pokud neexistuje; pracuj na `develop`.
2. Implementuj a commitni změnu, aktualizuj související dokumentaci a zvol odpovídající verzi.
3. Sestav z čistého, konkrétního commitu s ESP-IDF v5.5.2. Poznamenej úplné SHA commitu, verzový tag/popisek a kontrolní součet výsledného `.bin`.
4. Podle rozsahu změny nahraj lokálně přes USB nebo vzdáleně přes OTA. Lokální nahrání použij zejména při změně bootloaderu/tabulky oddílů nebo pokud OTA není dostupné; samotná OTA aktualizuje aplikaci.
5. Ověř funkce dotčené změnou a základní provoz (měření, MQTT/HA, dostupnost a přetrvávání potřebných nastavení).
6. Při OTA ověř `system/boot_mode=ota`, skutečnou verzi nového obrazu a funkčnost, potom jej potvrď přes `cmd/ota/confirm`. Ověř dokončení potvrzení (`confirmed`, případně po dalším restartu `boot_mode=normal`). Odeslání commandu samo není důkaz úspěchu.
7. **Teprve po úspěšném testu a potvrzení OTA posuň `main` na přesný commit běžící v provozu.** U USB nasazení je ekvivalentem úspěšný provozní test a ověření obrazu.
8. Neposouvej `main` na pozdější netestovaný HEAD develop, pokud se od sestavení změnil. Přednostně použij fast-forward; pokud není možný, nejprve vyřeš historii a sestav/testuj výsledek. Nepoužívej force-push jako zkratku.
9. Odešli příslušné větve a tag na GitHub a zaznamenej nasazení. Neproběhl-li test nebo potvrzení, `main` na novou funkcionalitu neposouvej.

Test s `--confirm` v neinteraktivním OTA helperu potvrzuje obraz automaticky po zjištění pending bootu; nenahrazuje ověření funkcí. Podrobný postup je v [ota.md](ota.md).

## Oprava problému v provozu

1. Opravu prováděj na **`main`** proti skutečně provozované verzi. Nepřidávej k ní netestovanou funkcionalitu z develop.
2. Aktualizuj patch verzi, commitni, sestav z čistého stromu a proveď potřebné rychlé kontroly.
3. **Opravu ihned nasaď** vhodným způsobem (USB nebo OTA), otestuj ji; u OTA potvrď fungující obraz. Zaznamenej výsledek a skutečně nasazený commit.
4. Pokud `develop` existuje, opravu následně přenes i do `develop`, aby ji další vývoj neztratil. Jsou-li větve srovnané, stačí fast-forward; při rozpracovaném vývoji začleň opravu merge nebo odpovídajícím cherry-pickem a ověř výsledek.
5. Pokud oprava nasadit nebo potvrdit nejde, označ ji jako čekající na nasazení. Nepovažuj commit na main ani úspěšný CI build za důkaz, že již běží v zařízení. Poslední potvrzené nasazení zůstává určující.

U hotfixu se commit na main vytváří před nasazením; tato krátká mezifáze je výjimka z pravidla přesného provozního commitu. Nezaměňuj ji s úspěšným dokončením nasazení.

## Čistě dokumentační změny

- Aktualizuj dokumentaci na `main`; na `develop` ji přenes jen pokud tato vývojová větev existuje.
- Jsou-li větve srovnané, použij jeden commit a obě na něj posuň přes fast-forward.
- Jsou-li odlišné, přenes pouze dokumentační změnu; neslučuj do main neověřenou funkcionalitu jen kvůli dokumentaci.
- Samotná dokumentace nemá zvyšovat release verzi firmware ani vyvolávat nahrání nového obrazu.

## Jak se verze nyní stanovuje

`firmware/CMakeLists.txt` nedefinuje `PROJECT_VER` ani `project(... VERSION ...)`, `firmware/version.txt` neexistuje a `CONFIG_APP_PROJECT_VER_FROM_CONFIG` je vypnuté.

ESP-IDF v5.5.2 proto při konfiguraci projektu používá:

```sh
git describe --always --tags --dirty
```

Výsledek se stane `PROJECT_VER` a je vložený do `esp_app_desc_t.version`. `state_manager.cpp` jej publikuje jako `voda/septik/diag/fw_version`; datum a čas sestavení publikuje do `diag/build_timestamp`.

| Příklad | Význam |
| --- | --- |
| `v2.2` | HEAD přesně na příslušném tagu |
| `v2.2-3-g8b3f543` | tři commity za tagem v2.2, zkrácené SHA 8b3f543 |
| `v2.2-3-g8b3f543-dirty` | sledované soubory mají necommitované změny |
| `8b3f543` | hash jako fallback, pokud není dosažitelný tag |

IDF používá i nečíselné a lehké tagy, protože má `--tags` a nemá omezení `--match v*`. Historické pomocné tagy proto mohou ovlivnit výsledný popisek. Nekontrolovaný tag na release commitu může změnit verzi dalšího sestavení. `-dirty` nezaručuje detekci všech lokálních neversionovaných vstupů; finální obraz sestavuj z čistého checkoutu, ideálně čerstvého klonu.

Při změně commitu/tagu spusť `idf.py reconfigure` a `idf.py build` (případně čisté sestavení). Ověř `project_version` v `build/project_description.json` a metadata výsledného obrazu; starší obsah build adresáře není důkazem aktuální verze. Po tagování již sestavený `.bin` svou verzi nezmění.

## Pravidla aktualizace verzí

Pro nové release tagy používej **`vMAJOR.MINOR.PATCH`**. Historické `v2.2` ponech beze změny; pro rozhodování o dalším čísle odpovídá řadě 2.2.0.

- **PATCH**: oprava chyby bez plánované nekompatibility (např. oprava provozu řady 2.2 → `v2.2.1`).
- **MINOR**: nová zpětně kompatibilní funkcionalita (`v2.3.0`).
- **MAJOR**: nekompatibilní změna, například MQTT rozhraní, NVS formátu nebo provozních požadavků bez zachované kompatibility (`v3.0.0`).

Příklady jsou pravidla, nikoli pokyn tyto konkrétní tagy právě vytvořit. Před volbou čísla vždy načti existující tagy a ověř poslední skutečně nasazenou verzi.

- Verze firmware není verze ESP-IDF. IDF 5.5.2 se při zvýšení aplikační verze automaticky nemění.
- Verzový tag vytvoř na čistém konkrétním commitu **před finálním sestavením**. Používej anotované tagy. Tag označuje obraz/kandidáta; úspěšné nasazení dokládá až test a evidence potvrzení.
- Pokud je kandidát označený prerelease tagem (např. `v2.3.0-rc.1`), do provozu eviduj tuto skutečnou verzi. Dodatečný finální tag nezmění verzi již nahraného obrazu; pro obraz s finálním názvem je potřeba nové sestavení a nasazení.
- Již zveřejněné verzové tagy nepřesouvej ani znovu nepoužívej pro jiný obsah. Opravený kandidát má nové číslo/tag.
- Před použitím automatického názvu vždy porovnej `git describe --always --tags --dirty` s očekávaným tagem. Nepoužívej ruční `PROJECT_VER` nebo `version.txt` jako nezdokumentované přepsání současného mechanismu.
- Tagy odešli explicitně (`git push origin TAG`); push větve sám jejich zálohu nezaručuje.

## Evidence nasazení

Pro každé skutečné nasazení zaznamenej: zařízení, datum, úplné SHA commitu, verzový tag/popisek, SHA-256 obrazu, USB/OTA, výsledek testu a potvrzení OTA. Evidence musí vycházet ze skutečného zařízení, ne pouze z lokálního build adresáře.

Pro lokální kontrolu před nasazením:

```sh
git status --short
git rev-parse HEAD
git describe --always --tags --dirty
shasum -a 256 firmware/build/voda-septik.bin
```

Na Windows použij ekvivalentní nástroj pro SHA-256. Po změně kódu/tagu musí být `.bin` skutečně znovu sestavený.

## Známé mezery současné implementace

- **`diag/git_hash` není spolehlivý hash commitu.** `state_manager.cpp` zkopíruje `app_desc->version` a uřízne text na první pomlčce; z `v2.2-3-g8b3f543` vyrobí `v2.2`. Pro určení provozovaného commitu používej úplné `diag/fw_version` spolu s evidencí sestavení/nasazení. Na přesném tagu se SHA určuje z neměnného tagu.
- CI používá `actions/checkout@v4` bez výslovného nastavení plné historie a tagů. Release obraz potřebuje ověřené tagy/historii; verze automatického artefaktu nemusí odpovídat lokálnímu `git describe`. Současný workflow také nemá trigger na tag push a čistá změna tagu sama nesestaví nový obraz.
- `main`, starý pomocný tag `N_A_S_A_Z_E_N_O` ani poslední lokální `build/` samy nedokládají aktuální stav fyzického zařízení. Při tomto dokumentačním průchodu nebylo zařízení kontaktováno, tagy verzí se neposouvaly a nový firmware se nenahrával.

Tyto mezery jsou zdokumentované; jejich oprava by byla změnou kódu/CI a řídí se stejným pravidlem vývoje, testování a nasazení.
