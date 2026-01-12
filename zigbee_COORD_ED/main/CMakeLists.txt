idf_component_register(
    SRCS "esp_zigbee_coord_ed.c"
    INCLUDE_DIRS "."
    REQUIRES 
        esp-zigbee-lib
        nvs_flash
        esp_timer
    )
