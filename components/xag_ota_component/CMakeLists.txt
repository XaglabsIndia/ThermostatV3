idf_component_register(SRCS "xag_ota_component.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_client esp_https_ota nvs_flash app_update xag_mqtt_component
                       EMBED_FILES "${CMAKE_CURRENT_LIST_DIR}/server_certs/ca_cert.pem")
