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

/* TX 陀螺主导 Y↔P 解耦（编译保留；运行由网页 TRK 开关） */
#define KK_TX_GYRO_DECOUPLE             1
#define KK_TX_GYRO_DECOUPLE_STRENGTH    0.72f
#define KK_TX_GYRO_DECOUPLE_DOM_START   0.52f
#define KK_TX_GYRO_DECOUPLE_DOM_FULL    0.80f
#define KK_TX_GYRO_DECOUPLE_ROLL_RELAX  22.0f
#define KK_TX_GYRO_DECOUPLE_DR_RELAX    2.5f
#define KK_TX_GYRO_DECOUPLE_BOTH_GYRO   72.0f
#define KK_DEC_SUP_ATTACK               0.42f   /* 解耦抑制爬升（每帧，100Hz） */
#define KK_DEC_SUP_RELEASE              0.16f   /* 解耦抑制回落 */
#define KK_DEC_MIN_GAIN                 0.18f   /* 次轴最低增益，避免完全锁死 */
#define KK_DEC_DOM_SPAN                 0.22f   /* 检测阈值→满抑制跨度 */

/* TX 移动检测（走路/大动作 → hold + 静止自动置零） */
#define KK_MOB_GYRO_HEAD_DOM_SHARE      0.60f
#define KK_MOB_GYRO_BODY_AXIS_DPS       48.0f
#define KK_MOB_GYRO_BODY_TOTAL_DPS      90.0f
#define KK_MOB_GYRO_BODY_ROLL_DPS       28.0f
#define KK_MOB_LIN_ACCEL_MPS2           2.5f
#define KK_MOB_LIN_ACCEL_SOFT_MPS2      1.2f
#define KK_MOB_LIN_EMA_NEW              0.32f   /* 线加速度低通 */
#define KK_MOB_LIN_PEAK_DECAY           0.93f   /* 峰值衰减（步伐检测） */
#define KK_MOB_TRIGGER_MS               350UL
#define KK_MOB_TRIGGER_DECAY_DIV        2U      /* 触发计时慢衰减，避免单帧漏检清零 */
#define KK_MOB_SETTLE_MS                1500UL
#define KK_MOB_SETTLE_DECAY_DIV         2U      /* 静止计时遇扰动慢减，不一次清零 */
#define KK_MOB_CENTER_RAMP_MS           220UL   /* 检测到移动后回中的过渡时间 */
#define KK_MOB_SETTLE_GYRO_DPS          14.0f
#define KK_MOB_STABILITY_REPORT_US      50000UL
#define KK_MOB_LIN_ACCEL_REPORT_US      50000UL
#define KK_TX_IMU_STALL_RECOVER_MS      3000UL
#define KK_TX_IMU_STALL_FAIL_MAX         3U

#define KK_REPAIR_HOLD_MS        5000
/* IMU 归零：短按 Rest_M（松开时判定） */
#define KK_BTN_SHORT_MIN_MS      30
#define KK_BTN_SHORT_MAX_MS      800

/* 0=正式模式（BLE 连上后才开 WiFi AP） */
#define KK_RX_WIFI_TEST_ALWAYS_ON  0

#define KK_BLE_REPAIR_CMD        "REPAIR"
#define KK_BLE_CENTER_CMD        "CENTER"

/* RX 失控保护：遥测丢失 / BLE 断连 → PPM 回 offset 个体中位并保持 */
#define KK_RX_FS_TEL_LOST_MS     300UL   /* 无遥测超过此值进 failsafe */
#define KK_RX_FS_RECOVER_MS      500UL   /* 恢复跟踪前需连续有效遥测 */
#define KK_RX_FS_RAMP_MS         220UL   /* 回中过渡，与移动检测一致 */

/* TX：逻辑 Roll 左右快速摆动 → 回正稳定后自动回中（与物理短按相同） */
/* 摆动角度/超时由 RX 网页配置，经 BLE GES 同步；见 gesture_cfg.h */
#define KK_TX_ROLL_NEUT_DEG            10.0f
#define KK_TX_ROLL_SETTLE_MS           700UL
#define KK_TX_ROLL_SETTLE_TIMEOUT_MS   1500UL
#define KK_TX_ROLL_GESTURE_COOLDOWN_MS 4000UL
