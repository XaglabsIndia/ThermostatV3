idf_component_register(SRCS "ButtonPress.c" "Main.c" "LoraComs.c" "ULPFile.c"
                    INCLUDE_DIRS "include"
                    REQUIRES  freertos driver MultiButton esp_timer provision xag_wifi_component xag_mqtt_component xag_ota_component esp_littlefs hdc1080 ulp)

# Specify the partition table
idf_build_set_property(PARTITION_TABLE_FILE ${CMAKE_SOURCE_DIR}/partitions.csv)
set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_s_sources "ulpF/main.S"
                    "ulpF/hdc1080.S"
                    "ulpF/LoRa.S" 
                    "ulpF/lora_tx.s"                  
                    "ulpF/spi.S"
                    "ulpF/i2c.S"
                    "ulpF/stack.S"
                    "ulpF/conditions.S"
                        )
set(ulp_exp_dep_srcs 
"ULPFile.c" 
)
ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")