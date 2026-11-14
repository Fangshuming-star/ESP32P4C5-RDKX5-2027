idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_hosted
                       PRIV_REQUIRES esp_wifi nvs_flash esp_event esp_netif esp_https_ota app_update esp_http_client mbedtls)
