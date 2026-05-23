idf_component_register(SRCS "main.c"
                              "uart_link.c"
                              "msg_ring.c"
                              "wifi.c"
                              "web.c"
                       INCLUDE_DIRS "." "../../common/iridium_protocol"
                       PRIV_REQUIRES esp_driver_uart esp_driver_gpio
                                     esp_wifi nvs_flash esp_http_server
                                     spiffs esp_netif esp_event
                                     esp_timer esp_hw_support)

# Embed SPIFFS image generated from spiffs_data/.
spiffs_create_partition_image(storage ../spiffs_data FLASH_IN_PROJECT)
