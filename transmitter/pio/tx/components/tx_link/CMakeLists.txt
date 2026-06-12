idf_component_register(
    SRCS "ble_tx.c" "gesture_center.c" "imu_tx.c" "tx_gesture.c" "tx_mount.c"
    INCLUDE_DIRS "."
    REQUIRES kk bt esp32_bno08x
)
