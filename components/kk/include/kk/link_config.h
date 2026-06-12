#pragma once

#define KK_BLE_NAME_RX           "KK-TRACK-RX"
#define KK_BLE_NAME_TX           "KK-TRACK-TX"

#define KK_WIFI_SSID             "TRACK-KK"
#define KK_WIFI_PASS             "12345678"
#define KK_WIFI_CHANNEL          1
#define KK_WIFI_MAX_STA          2
#define KK_WIFI_BOOT_WAIT_MS     (60UL * 1000UL)  /* no STA join -> AP off */
#define KK_WIFI_STABLE_DELAY_MS  3000             /* after PPM on, before AP */
#define KK_WIFI_IDLE_MS          (30UL * 1000UL)  /* no HTTP traffic -> AP off */
#define KK_WIFI_TX_POWER_QDBM    24               /* 6 dBm ~= 30% of 20 dBm max */

#define KK_BLE_SVC_UUID16        0xFFF0
#define KK_BLE_CH_LINK_UUID16    0xFFF1

#define KK_UDP_PORT              4210
#define KK_PREFS_NS              "kk_link"
#define KK_PREFS_RX_NS           "kk_rx"

#define KK_BLE_SCAN_SEC          8
#define KK_BLE_CONNECT_MS        15000
#define KK_BLE_RETRY_MS          2000   /* 未配对扫描失败后退避 */
#define KK_BLE_RECONNECT_MS      400    /* 已配对断线后尽快重连 */
#define KK_BLE_WIFI_RETRY_MS     2000
#define KK_WIFI_JOIN_DELAY_MS    4000   /* RX 开 AP 后留给手机/网页 */
#define KK_WIFI_JOIN_MS          12000
#define KK_BLE_TEL_MS            20     /* TX→RX 遥测周期，与 PPM 50Hz 对齐 */
#define KK_DIAG_LOG_MS           2000UL /* 状态日志周期；与遥测/PPM 解耦，不阻塞热路径 */

/* TX BNO085 GAME_ROTATION_VECTOR 报告间隔（us）；10000=100Hz */
#define KK_TX_IMU_REPORT_US         10000UL
/* 1=仅 IMU 串口调试，暂不启 BLE（避免 connect 日志打断） */
#define KK_TX_IMU_DEBUG_SERIAL      0

#define KK_REPAIR_HOLD_MS        5000
/* IMU 归零：短按 Rest_M（松开时判定） */
#define KK_BTN_SHORT_MIN_MS      30
#define KK_BTN_SHORT_MAX_MS      800

/* 0=正式模式（BLE 连上后才开 WiFi AP） */
#define KK_RX_WIFI_TEST_ALWAYS_ON  0

#define KK_BLE_REPAIR_CMD        "REPAIR"
#define KK_BLE_CENTER_CMD        "CENTER"
