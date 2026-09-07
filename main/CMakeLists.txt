idf_component_register(SRCS "main.c"
                            "app_power.c"
                            "app_wifi.c"
                            "app_mqtt.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "certs/mqtt_ca.pem"
                       REQUIRES nvs_flash
                                esp_wifi
                                esp_netif
                                esp_event
                                lwip
                                mqtt
                                esp_timer
                                esp_driver_gpio
                                esp_driver_i2c
                                esp_driver_uart
                                bh1750
                                bmp280
                                dht
                                modbus_rtu
                                nmea_gps)
