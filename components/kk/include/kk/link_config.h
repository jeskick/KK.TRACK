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
#define KK_BLE_CH_OTA_UUID16     0xFFF2

/* OTA：网页上传 RX.1.2.3.bin / TX.1.2.3.bin；编译输出在 ota/ 同格式 */
#define KK_OTA_FN_RX               "RX.*.bin"
#define KK_OTA_FN_TX               "TX.*.bin"
#define KK_OTA_HTTP_BUF            1024
#define KK_OTA_TX_HTTP_BUF         256   /* TX 中继：小缓冲减轻 WiFi+BLE 并发内存压力 */
#define KK_OTA_BLE_CHUNK_MIN       20
#define KK_OTA_BLE_CHUNK_MAX       244   /* 单 mbuf 块 256B；也受 ATT MTU-3 限制 */
#define KK_OTA_BLE_PACE_MS         15UL  /* notify 间隔，减轻 WiFi+BLE 并发与 RX 复位风险 */
#define KK_OTA_TX_RDY_MS           20000UL
#define KK_OTA_TX_CHUNK_WAIT_MS    25000UL /* 单 HTTP 块 BLE 中继最长等待 */
#define KK_OTA_TX_RELAY_TOTAL_MS   (10UL * 60UL * 1000UL) /* 整包 TX OTA 上限 */
#define KK_OTA_TX_STALL_MS         90000UL /* TX 侧无 chunk 则放弃 */
#define KK_OTA_TX_FINISH_MS        45000UL /* 等 TX OTA,DONE 或断链 */
#define KK_OTA_TX_IMAGE_MAX        1740800UL /* TWO_OTA_LARGE 单槽上限 */
#define KK_OTA_IMAGE_MAGIC         0xE9U     /* ESP-IDF app image 首字节 */
/* esp_app_desc 魔数与 CMake project() 名；用于首包校验，杜绝 RX/TX 固件刷反 */
#define KK_OTA_APP_DESC_MAGIC      0xABCD5432U
#define KK_OTA_PROJ_RX             "kk_rx"
#define KK_OTA_PROJ_TX             "kk_tx"
#define KK_OTA_RX_IMAGE_MIN        (32U * 1024U)
#define KK_OTA_TX_IMAGE_MIN        (32U * 1024U)
#define KK_OTA_BOOT_CONFIRM_MS     15000UL   /* 新镜像稳定运行后再 cancel rollback */
#define KK_OTA_REBOOT_DELAY_MS     800UL

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
#define KK_BLE_TEL_BACKOFF_MS    60     /* 遥测写入失败(链路拥塞)时的退避，给 BLE 缓冲排空 */
#define KK_DIAG_LOG_MS           2000UL /* 状态日志周期；与遥测/PPM 解耦，不阻塞热路径 */

/*
 * ===== 调试日志主题开关（编译期，见 kk/kk_log.h）=====
 * 平时全 0：运行时只输出"事件/状态变化"日志（链路、配对、PPM、WiFi、OTA、IMU 故障…），
 * 一次变化一条，绝不周期刷屏，干净直观。要聚焦调试某功能时把对应主题置 1 重新编译，
 * 该功能处会以 KK_DBG_STREAM_MS 为周期实时输出细节，调完置回 0。
 * 正式版(release/nolog)已整体去日志，这些开关不影响正式版体积与性能。
 */
#define KK_DBG_STREAM_MS         100u  /* 实时调试流刷新周期(ms) */
#define KK_DBG_DECOUPLE_MS       50u   /* 解耦分析流周期(ms)，比通用流更快以抓瞬态耦合 */
#define KK_DBG_POSE              0     /* TX/RX: 姿态/舵机输出实时流 */
#define KK_DBG_DECOUPLE          0     /* TX: 轴解耦链路全量(raw/geo/out/gyro/sup) */
#define KK_DBG_MOTION            1     /* TX: 移动检测每拍判据(body/ho/nod/lin/st/trg/set) */
#define KK_DBG_LINK              0     /* RX: 遥测节拍/链路新鲜度 */

/*
 * 端到端信号链 — 各层职责分离，勿在错误层做限速：
 *
 *   L0 舵机物理：KK_SERVO_SEC_PER_60 → 最大角速度，是程序跟随的唯一速度上限
 *   L1 TX 感知：BNO085 100Hz → 安装/零位 → 逻辑 yaw/pitch（几何，不含舵机速度）
 *   L2 TX 语义：轴解耦(仅抑制耦合分量)、移动暂停、手势（语义，不改舵机响应曲线）
 *   L3 链路：BLE 遥测 50Hz，传输目标角
 *   L4 RX 跟随：servo_follow — 抖动死区 + 以 L0 为上限的斜率跟随（加减速自然柔和）
 *   L5 RX 映射：角度→PPM 线性即时，无额外时间滤波
 *   L6 PPM 50Hz(20ms) → 舵机
 *
 * 舵机典型满载 0.10–0.14 s/60°；取 0.12s 为设计值。
 * 50Hz 时单帧最大转角 ≈ 60°/(0.12s×50Hz) ≈ 10°/帧。
 */

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
#define KK_DEC_SUP_ATTACK               0.28f   /* 抑制渐强：柔和，减轻干涩 */
#define KK_DEC_SUP_RELEASE              0.22f   /* 抑制渐弱：主导轴恢复跟手 */
#define KK_DEC_MIN_GAIN                 0.22f
#define KK_DEC_YAW_MIN_GAIN_PITCH       0.05f   /* 俯仰主导时干扰 yaw 可近零 */
#define KK_DEC_PITCH_DOM_GYRO_RATIO     1.15f
#define KK_DEC_YAW_DOM_GYRO_RATIO       1.15f
#define KK_DEC_COUPLING_YAW_SUP         0.92f   /* 点头时无 yaw 陀螺支撑的 yaw 增量 */
#define KK_DEC_COUPLING_PITCH_SUP       0.42f   /* 转头时无 pitch 陀螺支撑（较轻） */
#define KK_DEC_YAW_TO_PITCH_SCALE       0.38f   /* 偏航→俯仰抑制强度比例（弱侧） */
#define KK_DEC_ROLL_PITCH_LEAK_DEG      12.0f   /* 中等 roll 下 pitch→yaw 泄漏补偿 */
#define KK_DEC_ROLL_LEAK_MAX_DEG        48.0f   /* 超过则视为侧戴/奇异区，不再加强抑 yaw */
#define KK_DEC_INTENT_GYRO_MIN          16.0f   /* 主观轴：陀螺超过此值且与 Δ角 同向 */
#define KK_DEC_INTENT_GYRO_STRONG       26.0f   /* 强陀螺：无需 Δ角 对齐也视为真运动 */
#define KK_DEC_INTENT_DEG_MIN           0.10f
#define KK_DEC_DOM_SPAN                 0.22f

/*
 * 几何法之上的跨轴干扰过滤（变增益“误差低通”，仅 KK_IMU_DECOUPLE_GEOMETRIC=1 时启用）：
 * 解决“主观转头(yaw)时俯仰(pitch)跟着动、反之亦然”的物理交叉耦合（头戴轴向不固定所致）。
 * 思路：每轴以系数 k 向几何真值收敛——主观驱动轴 k≈1 全跟手；当“对侧”陀螺占比超过
 * dom 阈值时，本轴 k 被压低（按 UI strength），把短暂的耦合冲动滤掉。总角速率低于
 * ACT_MIN(静止)时两轴 k→1，输出收敛到真值，零静态滞后、无冻结漂移（保持丝滑稳定）。
 * dom 阈值与 strength 复用网页 decouple_dom_x10 / decouple_str_x100。
 */
#define KK_XDEC_ACT_MIN_DPS            12.0f  /* 总角速率(dps)低于此视为静止/微动，不抑制(抬高以抑低速主导翻转抖动) */
#define KK_XDEC_DOM_SPAN               0.22f  /* 主导占比 smoothstep 跨度 */
#define KK_XDEC_SUP_ATTACK             0.45f  /* 抑制渐强：快，及时挡住起步耦合 */
#define KK_XDEC_SUP_RELEASE            0.12f  /* 抑制渐弱：慢，避免主导切换时闪烁 */
#define KK_XDEC_K_MIN                  0.03f  /* (旧模型遗留，绝对保持模型已不用) */
#define KK_XDEC_REF_FREE               0.15f  /* 抑制量低于此视为该轴"自由"，持续锁存几何真值为参照 */
#define KK_XDEC_REL_HOLD_MS            160u   /* 释放保持窗：换向时两轴速度同时过零会瞬时跌破 ACT_MIN，
                                              * 此窗内维持上一拍抑制目标(不松开/不重锁 ref)，桥过速度凹陷消除换向毛刺；
                                              * 仅真正持续静止超过此窗才放行，收敛回真值(零静态滞后)。 */

/* TX 移动检测（走路/ sudden 位移 → hold + 静止自动置零；头部大角度转/点不触发） */
#define KK_MOB_GYRO_HEAD_DOM_SHARE      0.62f  /* yaw/pitch 陀螺占比超此视为头部主观运动，一律豁免 */
#define KK_MOB_GYRO_BODY_AXIS_DPS       52.0f
#define KK_MOB_GYRO_BODY_TOTAL_DPS      95.0f
#define KK_MOB_GYRO_BODY_ROLL_DPS       32.0f
#define KK_MOB_GYRO_BODY_ROLL_SHARE     0.38f  /* roll 路径：roll 须为陀螺主导分量，排除转头耦合 */
#define KK_MOB_ROT_LIN_K                0.38f  /* 扣除转头假加速度( pitch+yaw 合成角速率 ) */
#define KK_MOB_LIN_ACCEL_MPS2           3.5f   /* 平移线加速度硬阈(已扣 rot 后) */
#define KK_MOB_LIN_ACCEL_SOFT_MPS2      1.6f
#define KK_MOB_HEAD_ACTIVE_DPS          28.0f  /* 单轴角速率超此视为正在转头/点头 */
#define KK_MOB_HEAD_PAIR_DPS            42.0f  /* pitch+yaw 合成角速率(双轴同时动) */
#define KK_MOB_WALK_LT_MIN              4.8f   /* 头在动时仍放行：脚步/平移够大 */
#define KK_MOB_GRAV_MAX_HEAD_DPS        45.0f  /* 重力判站/坐时头轴不应在快速转 */
#define KK_MOB_POSTURE_LT_MIN           2.5f   /* 站/坐：平移+重力联合下限 */
#define KK_MOB_TRIGGER_HARD_MS          300UL  /* 硬平移(lt≥硬阈)时更快触发 */
#define KK_MOB_TRIGGER_HARD_MUL         2U     /* 硬平移时触发计时倍率 */
#define KK_MOB_GRAV_TILT_DEG            18.0f  /* 逻辑系重力相对基线倾角→站/坐/大位移 */
#define KK_MOB_GRAV_BASE_EMA            0.035f /* 静止时重力基线慢跟踪 */
#define KK_MOB_GRAV_BASE_FREEZE_DPS     22.0f  /* 头在动时冻结重力基线，防 gt 随转头虚增 */
#define KK_MOB_LIN_EMA_NEW              0.28f
#define KK_MOB_LIN_PEAK_DECAY           0.92f
#define KK_MOB_TRIGGER_MS               620UL
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

/* RX 失控保护：正常使用不开 WiFi；断连或遥测停 → 立刻回 offset 中位 */
#define KK_RX_FS_TEL_LOST_MS     1200UL  /* 遥测丢失（TX 死机/假连接）；50Hz 下约 60 帧无包 */
#define KK_RX_FS_STALE_HOLD_MS   800UL   /* 短间隙保持上一帧，不回中 */
#define KK_RX_FS_RECOVER_MS      300UL   /* 恢复跟踪前需连续有效遥测 */
#define KK_RX_FS_RAMP_MS         220UL   /* 遗留：失控已改 snap，保留供 ht_failsafe_step */

/* L0 舵机物理规格（RX servo_follow 速度上限来源） */
#define KK_SERVO_SEC_PER_60          0.12f
#define KK_SERVO_MAX_DEG_PER_S       (60.0f / KK_SERVO_SEC_PER_60)
#define KK_SERVO_JITTER_DEADBAND_DEG 0.15f
#define KK_SERVO_FOLLOW_NOMINAL_DT_S (KK_BLE_TEL_MS / 1000.0f)

/* TX：逻辑 Roll 左右快速摆动 → 回正稳定后自动回中（与物理短按相同） */
/* 摆动角度/超时由 RX 网页配置，经 BLE GES 同步；见 gesture_cfg.h */
#define KK_TX_ROLL_NEUT_DEG            10.0f
#define KK_TX_ROLL_SETTLE_MS           700UL
#define KK_TX_ROLL_SETTLE_TIMEOUT_MS   2500UL
#define KK_TX_ROLL_GESTURE_COOLDOWN_MS 4000UL
/* 手势回中：须 Roll 轴主导，避免 Pitch/Yaw 耦合误触发 */
#define KK_TX_GESTURE_GYRO_ROLL_DPS    22.0f
#define KK_TX_GESTURE_POLL_MS          20UL
/* |Roll| 接近垂直：欧拉奇异区，不启动新手势 */
#define KK_IMU_GIMBAL_ROLL_DEG         55.0f
#define KK_IMU_POSE_CLAMP_DEG          90.0f

/*
 * 轴解耦算法选择（TX，编译期）：
 *   1 = 几何法：由头部前向轴朝向向量直接求 yaw/pitch，天生与 roll 无关、
 *       全姿态连续（无 55° 奇异跳变），不累积漂移。推荐。
 *   0 = 旧版：欧拉角 + 陀螺主导启发式抑制（imu_decouple.c）。保留以便随时回退对比。
 * 网页 decouple 开关含义：开=应用几何解耦；关=输出原始欧拉角（耦合，便于对比）。
 */
#define KK_IMU_DECOUPLE_GEOMETRIC      1
