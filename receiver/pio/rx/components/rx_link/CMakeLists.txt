idf_component_register(
    SRCS "ble_rx.c" "wifi_rx.c"
    INCLUDE_DIRS "."
    REQUIRES kk bt esp_wifi esp_netif
)
