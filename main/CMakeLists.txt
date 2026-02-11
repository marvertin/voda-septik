idf_component_register(SRCS "zalevaci-nadrz.c" "blikaniled.c" "lcd-demo.c" "prutokomer-demo.c" "teplota-demo.c" "hladina-demo.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio onewire esp_adc
                    PRIV_REQUIRES esp_timer)
