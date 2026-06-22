idf_component_register(
    SRCS "ble_tx.c" "gesture_center.c" "imu_decouple.c" "imu_tx.c" "motion_detect.c"
         "tx_gesture.c" "tx_mount.c" "tx_track.c" "tx_ota.c"
    INCLUDE_DIRS "."
    REQUIRES kk bt esp32_bno08x app_update
)
