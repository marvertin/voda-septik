# Hardware voda-septik

Stav dokumentace ověřen 12. 10. 2026. Návrh je v KiCadu 9 a související runtime firmware je v `../firmware/`.

## Soubory a knihovny

- `Voda septik.kicad_pro` – projekt; `Voda septik.kicad_sch` – schéma; `Voda septik.kicad_pcb` – deska.
- `martin.kicad_sym` a `martin.pretty/` jsou připojené jako projektová knihovna `martin` pomocí `${KIPRJMOD}`.
- `0martin.kicad_sym` a `zaciname.pretty/` jsou také v Gitu, ale současné projektové tabulky je nepřipojují jako samostatné knihovny.
- Standardní KiCad knihovny používané návrhem zahrnují `Device`, `Connector`, `Analog_ADC`, `Display_Character`, `power`, `Capacitor_THT`, `Connector_PinHeader_2.54mm`, `Diode_THT`, `LED_THT`, `Resistor_THT`.
- Externí **Espressif KiCad Libraries** nainstaluj přes správce zásuvných modulů a obsahu a ověř registraci názvu `PCM_Espressif` v tabulce symbolů i footprintů. Balíček není v repozitáři ani připnutý na konkrétní verzi.
- Schéma/PCB obsahují vložené kopie použitých symbolů/footprintů. To umožňuje otevřít stávající návrh, ale pro aktualizace a nové součástky je potřeba dostupná zdrojová knihovna.
- `.kicad_prl`, zámky a automatické zálohy jsou ignorované; `.bak`, stará schématická záloha, CSV a `fp-info-cache` jsou historicky verzované pomocné soubory.

## GPIO podle aktivního firmware

Zdrojem této tabulky je [pins.h](../firmware/main/pins.h); jde o konfiguraci programu, nikoli nezávislé elektrické ověření zapojení desky.

| Konstanta | GPIO | Poznámka v kódu |
| --- | --- | --- |
| `STATUS_LED_GPIO` | 16 | D9 LED |
| `TM1637_CLK_GPIO` | 17 | J4 CLK |
| `TM1637_DIO_GPIO` | 18 | J4 DIO |
| `DISPLAY_I2C_SDA_GPIO` | 22 | J3 SDA |
| `DISPLAY_I2C_SCL_GPIO` | 23 | J3 SCL |
| `ADS1115_I2C_SDA_GPIO` | 14 | U3 SDA |
| `ADS1115_I2C_SCL_GPIO` | 13 | U3 SCL |
| `ADS1115_ALERT_RDY_GPIO` | 4 | U3 ALERT/RDY |
| `FLOW_SENSOR_GPIO` | 33 | /prutok |
| `TEMPERATURE_SENSOR_GPIO` | 32 | /teplomery |
| `RS485_RX_GPIO` | 27 | MAX3485 TXD |
| `RS485_TX_GPIO` | 26 | MAX3485 RXD |
| `RS485_EN_GPIO` | 25 | MAX3485 EN |
| `BOOT_BUTTON_GPIO` | 0 | GPIO0 |

## Sběrnice a ADC

- ADS1115: I2C0, adresa **0x48**, 100 kHz, SDA14/SCL13, ALERT/RDY4. Jediný ADC task přepíná single-ended kanály: AIN1 tlak za filtrem, AIN2 tlak před filtrem, AIN3 hladina; AIN0 je rezerva.
- LCD HD44780 / PCF8574: samostatné I2C1, adresa **0x27**, SDA22/SCL23. Nezaměňuj ji s ADC sběrnicí.
- Proudové smyčky: firmware předpokládá **100Ω shunt**, PGA ADS1115 **±2,048 V**, LSB 62,5 µV; 4 mA → 0,4 V → RAW6400, 20 mA → 2,0 V → RAW32000. Kalibrace se ukládá v zařízení.
- RS485 / KWS-303L: UART2, **9600 8N1**, RX27/TX26, řízení směru 25, výchozí slave adresa 1.
- DS18B20: sdílená 1-Wire na GPIO32; voda a vzduch se rozlišují podle ROM adres.
- Průtok: náběžná hrana na GPIO33, výchozí 38 impulsů/l; nejde o analogový vstup.
- TM1637: CLK17/DIO18; BOOT je GPIO0, status LED GPIO16.

## Flash paměť

ESP32 má v projektu konfiguraci 4 MB. Zdrojem je [partitions.csv](../firmware/partitions.csv):

| Oddíl | Offset | Velikost | Účel |
| --- | --- | --- | --- |
| `nvs` | 0x9000 | 0x6000 | konfigurace `app_cfg`, restart metadata `sys_meta`, základ čítače `flash_ctr` |
| `otadata` | 0xF000 | 0x2000 | volba OTA obrazu |
| `phy_init` | 0x11000 | 0x1000 | PHY data |
| `flow_data0` | 0x12000 | 0x2000 | bitový flash čítač celých litrů |
| `user_data1` | 0x14000 | 0x2000 | rezervovaný datový oddíl |
| `user_data2` | 0x16000 | 0x2000 | rezervovaný datový oddíl |
| `user_data3` | 0x18000 | 0x2000 | rezervovaný datový oddíl |
| `ota_0` | 0x20000 | 0x180000 | první obraz aplikace |
| `ota_1` | 0x1A0000 | 0x180000 | druhý obraz aplikace |

Při obnově čítače nestačí samotný `flow_data0`: potřebuje i základ/rollover marker z NVS `flash_ctr`. Při přemapování oddílů, změně ADC rozsahu, adres nebo GPIO aktualizuj firmware i tuto mapu. Dokumentační průchod neprováděl měření skutečné desky.
