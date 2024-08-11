idf_component_register(SRCS "xag_mqtt_component.c"
                    REQUIRES xag_nvs_component app_update esp_event esp_http_client esp_https_ota esp_netif nvs_flash esp_wifi driver mqtt json esp_system 
                    INCLUDE_DIRS "include"
                    )
