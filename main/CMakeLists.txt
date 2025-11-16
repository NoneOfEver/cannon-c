idf_component_register(SRCS "main.c" "ble.c"
                       REQUIRES esp_adc nvs_flash bt
                       INCLUDE_DIRS ".")

target_link_libraries(${COMPONENT_LIB} m)
