set(KK_COMMON_SRCS
    "src/time.c"
    "src/led.c"
    "src/repair.c"
    "src/gesture_cfg.c"
    "src/tx_track_cfg.c"
    "src/imu_mount.c"
    "src/storage.c"
    "src/telemetry.c"
    "src/fw_version.c"
    "src/ota_image.c"
    "src/rc_proto.c"
)

set(KK_RX_SRCS
    "src/rx_profile.c"
    "src/ppm.c"
    "src/rc_out.c"
    "src/servo_follow.c"
    "src/head_track.c"
    "src/rx_ota.c"
    "src/rx_web.c"
    "src/rx_web_page.c"
)

# RX 工程根目录放 .kk_rx_project 空标记（TX 无此文件）
if(EXISTS "${CMAKE_SOURCE_DIR}/.kk_rx_project")
    idf_component_register(
        SRCS ${KK_COMMON_SRCS} ${KK_RX_SRCS}
        INCLUDE_DIRS "include"
        REQUIRES driver esp_timer nvs_flash esp_driver_gptimer esp_driver_ledc esp_app_format
                 esp_http_server esp_wifi esp_netif app_update
    )
else()
    idf_component_register(
        SRCS ${KK_COMMON_SRCS}
        INCLUDE_DIRS "include"
        REQUIRES driver esp_timer nvs_flash esp_driver_gptimer esp_driver_ledc esp_app_format
    )
endif()
