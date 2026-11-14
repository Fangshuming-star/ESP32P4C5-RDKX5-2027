idf_component_register(SRCS "wifi_ota.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_wifi esp_event esp_netif nvs_flash
                                     esp_http_client app_update esp_hosted)
