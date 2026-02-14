idf_component_register(
    SRCS "config_webapp.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_http_server esp_app_format esp_timer
)
