idf_component_register(SRCS "ButtonPress.c" "Main.c" "LoraComs.c"
                    INCLUDE_DIRS "include"
                    REQUIRES  freertos driver MultiButton esp_timer provision xag_wifi_component xag_mqtt_component xag_ota_component esp_littlefs hdc1080 )

# Specify the partition table
idf_build_set_property(PARTITION_TABLE_FILE ${CMAKE_SOURCE_DIR}/partitions.csv)