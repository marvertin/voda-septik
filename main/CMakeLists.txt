idf_component_register(SRCS "zalevaci-nadrz.cpp" "status_display.cpp" "app-config.cpp" "restart_info.cpp" "sensor_events.cpp" "state_manager.cpp" "network_event_bridge.cpp" "webapp_startup.cpp" "blikaniled.cpp" "lcd-demo.cpp" "prutokomer.cpp" "teplota-demo.cpp" "hladina-demo.cpp" "lcd.cpp" "flash_monotonic_counter.cpp" "boot_button.cpp" "zalevaci-nadrz.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio onewire esp_adc esp_wifi nvs_flash esp_netif config_webapp network_core error_check tm1637_startup_animation
                    PRIV_REQUIRES esp_timer cxx mqtt app_update)
