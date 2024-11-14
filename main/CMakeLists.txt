idf_component_register(SRCS "ButtonPress.c" "Main.c" "LoraComs.c" "hello_world_main.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES GxEPD2 Adafruit_GFX Adafruit_BusIO  freertos driver MultiButton esp_timer provision xag_wifi_component xag_mqtt_component xag_ota_component esp_littlefs hdc1080)
