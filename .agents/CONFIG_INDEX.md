# Configuration Index

Zdroj pravdy jsou `static const config_item_t ..._ITEM` definice a jejich registrace pres `config_store_register_item`.

## Registrace sekci

Sekce se zakladaji v `firmware/main/voda-septik.cpp` pred startem webapp:

- `Sit` - `network_config_register_config_items`
- `System` - `system_config_register_config_items`
- `Teplota` - `teplota_register_config_items`
- `Zasoba` - `zasoba_register_config_items`
- `Tlak` - `tlak_register_config_items`
- `Elektromer` - `elektromer_register_config_items`
- `Prutokomer` - `prutokomer_register_config_items`

## Sit

Soubor: `firmware/main/network_config.cpp`

- `wifi_ssid` - string, SSID.
- `wifi_pass` - string, heslo k Wi-Fi.
- `mqtt_uri` - string, MQTT broker URI.
- `mqtt_user` - string, MQTT uzivatel.
- `mqtt_pass` - string, MQTT heslo.

Poznamka: vychozi hodnoty v kodu obsahuji provozni-looking udaje. Nesir je do novych dokumentu ani logu, pokud to neni nutne.

## System

Soubor: `firmware/main/system_config.cpp`

- `interval_s` - int32, perioda mereni/publikace.
- `auto_mode` - bool, automaticky rezim.

## Teplota

Soubor: `firmware/main/teplota.cpp`

- `tepl_max` - float, maximalni teplota.
- `temp_addr_water` - string, ROM adresa DS18B20 vody.
- `temp_addr_air` - string, ROM adresa DS18B20 vzduchu.

## Zasoba

Soubor: `firmware/main/zasoba.cpp`

- `lvl_ads_raw_min` - int32, ADS1115 RAW pro minimalni hladinu.
- `lvl_ads_raw_max` - int32, ADS1115 RAW pro maximalni hladinu.
- `lvl_h_min` - float, vyska hladiny pro minimum.
- `lvl_h_max` - float, vyska hladiny pro maximum.
- `tank_area_m2` - float, pudorysna plocha nadrze.
- `lvl_ema_alpha` - float, EMA koeficient hladiny.
- `lvl_hyst_m` - float, hystereze hladiny v metrech.
- `lvl_round_dec` - int32, desetinna mista publikovane hodnoty.

## Tlak

Soubor: `firmware/main/tlak.cpp`

- `tlk_b_ads_r4` - int32, RAW pred filtrem pri 4 mA.
- `tlk_b_ads_r20` - int32, RAW pred filtrem pri 20 mA.
- `tlk_b_p_min` - float, tlak pred filtrem pri 4 mA.
- `tlk_b_p_max` - float, tlak pred filtrem pri 20 mA.
- `tlk_a_ads_r4` - int32, RAW za filtrem pri 4 mA.
- `tlk_a_ads_r20` - int32, RAW za filtrem pri 20 mA.
- `tlk_a_p_min` - float, tlak za filtrem pri 4 mA.
- `tlk_a_p_max` - float, tlak za filtrem pri 20 mA.
- `tlk_ema_alpha` - float, EMA koeficient tlaku.
- `tlk_hyst_bar` - float, hystereze tlaku.
- `tlk_round_dec` - int32, desetinna mista publikovane hodnoty.
- `tlk_dp_100` - float, rozdil tlaku pro 100 % zanesenosti.

## Elektromer

Soubor: `firmware/main/elektromer.cpp`

- `kws_addr` - int32, Modbus adresa KWS-303L.

## Prutokomer

Soubor: `firmware/main/prutokomer.cpp`

- `flow_pulses_l` - int32, pocet pulsu na litr.

## Webapp a NVS

- `components/config_store` drzi registry polozek a typove get/set API nad NVS.
- `components/config_webapp` vykresluje polozky podle registru, validuje typy a uklada hodnoty.
- NVS namespace pro aplikacni konfiguraci je `app_cfg`.
