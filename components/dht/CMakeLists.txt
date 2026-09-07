idf_component_register(SRCS "dht.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio esp_timer esp_hw_support hal soc esp_rom)
