idf_component_register(
    SRCS "network_init.cpp" "network_ap_mode.cpp" "network_mqtt_config.cpp" "mqtt_publish.cpp" "network_event.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif nvs_flash mqtt esp_timer error_check
)
