idf_component_register(SRCS "xag_wifi_component.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi nvs_flash xag_nvs_component LedCode)

