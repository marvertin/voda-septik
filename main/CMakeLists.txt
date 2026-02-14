idf_component_register(SRCS "zalevaci-nadrz.cpp" "blikaniled.cpp" "lcd-demo.cpp" "prutokomer-demo.cpp" "teplota-demo.cpp" "hladina-demo.cpp" "lcd.cpp" "wifi_init.cpp" "mqtt_init.cpp" "zalevaci-nadrz.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio onewire esp_adc esp_wifi nvs_flash esp_netif config_webapp
                    PRIV_REQUIRES esp_timer cxx mqtt)
