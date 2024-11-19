idf_component_register(
    SRCS "LedTask.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer
)