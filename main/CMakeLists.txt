idf_component_register(SRCS "zalevaci-nadrz.cpp" "app-config.cpp" "restart_info.cpp" "sensor_events.cpp" "state_manager.cpp" "blikaniled.cpp" "lcd-demo.cpp" "prutokomer.cpp" "teplota-demo.cpp" "hladina-demo.cpp" "lcd.cpp" "network_init.cpp" "network_state_machine.cpp" "network_mqtt_config.cpp" "mqtt_publish.cpp" "flash_monotonic_counter.cpp" "zalevaci-nadrz.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio onewire esp_adc esp_wifi nvs_flash esp_netif config_webapp
                    PRIV_REQUIRES esp_timer cxx mqtt app_update)
