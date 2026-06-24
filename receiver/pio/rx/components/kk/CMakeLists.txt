idf_component_register(
    SRCS
        "src/time.c"
        "src/led.c"
        "src/repair.c"
        "src/gesture_cfg.c"
        "src/tx_track_cfg.c"
        "src/imu_mount.c"
        "src/storage.c"
        "src/rx_profile.c"
        "src/telemetry.c"
        "src/fw_version.c"
        "src/ppm.c"
        "src/rc_out.c"
        "src/servo_follow.c"
        "src/head_track.c"
        "src/ota_image.c"
        "src/rx_ota.c"
        "src/rx_web.c"
        "src/rx_web_page.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer nvs_flash esp_http_server esp_wifi esp_netif esp_driver_gptimer esp_driver_ledc app_update
)
