idf_component_register(SRCS "nmea_gps.c" "nmea_parse.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_uart esp_timer esp_system)
