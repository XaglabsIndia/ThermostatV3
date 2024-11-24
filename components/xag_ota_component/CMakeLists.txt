idf_component_register(SRCS "xag_ota_component.c"
                       INCLUDE_DIRS "include"
                       REQUIRES LedCode  esp_http_client esp_https_ota nvs_flash app_update json xag_mqtt_component xag_nvs_component
                       EMBED_FILES "${CMAKE_CURRENT_LIST_DIR}/server_certs/ca_cert.pem")
