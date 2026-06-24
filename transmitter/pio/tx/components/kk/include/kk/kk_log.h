#pragma once

/*
 * KK 统一日志工具（头文件即用，无 .c）。
 *
 * 设计原则（TX/RX 通用）：
 *   1) 事件/状态驱动：状态机、链路、配对、保存、OTA、IMU 故障等"变化"时打一条即可，
 *      绝不周期刷屏。用 KK_EVT（已在变化点调用）或 KK_EVT_CHG（轮询点自动判边沿）。
 *   2) 聚焦调试：要调某个功能，把对应主题(KK_DBG_*)置 1 重编译，相关处用 KK_DBG()
 *      以 KK_DBG_STREAM_MS 为周期实时输出细节；调完置回 0。默认全关，运行时零开销。
 *
 * 级别说明：本工程所有"应用日志"统一走 ESP_LOGW(WARN)，与既有代码一致——
 *   调试版(esp32-c3 debug) 保留 WARN 故可见；正式版(release/nolog) 已整体去日志，
 *   下面所有宏自动变为空，不占体积、不阻塞串口。
 */

#include <limits.h>
#include <stdint.h>

#include "esp_log.h"
#include "kk/time.h"

/* 事件日志：在状态真正变化的位置直接调用，一次一条。 */
#define KK_EVT(tag, fmt, ...) ESP_LOGW(tag, fmt, ##__VA_ARGS__)

/*
 * 边沿事件日志：可放在每帧轮询的位置，仅当整型 value 相对上一次变化时打印一条。
 * label 必须是唯一的合法标识符（每个调用点用于记忆上次值的静态变量名后缀）。
 *   KK_EVT_CHG(TAG, ble, s_ble_on, "[BLE] %s", s_ble_on ? "up" : "down");
 */
#define KK_EVT_CHG(tag, label, value, fmt, ...)                                                    \
    do {                                                                                           \
        static long kk__chg_##label = LONG_MIN;                                                    \
        const long kk__v = (long)(value);                                                          \
        if (kk__v != kk__chg_##label) {                                                            \
            kk__chg_##label = kk__v;                                                               \
            ESP_LOGW(tag, fmt, ##__VA_ARGS__);                                                     \
        }                                                                                          \
    } while (0)

/*
 * 主题门控的限速实时流：enabled 为编译期 0/1 主题开关，period_ms 为刷新周期。
 * 主题关时 if(0) 整段被编译器消除（参数不在运行时求值），零开销；
 * 主题开时每 period_ms 最多打一条，避免把串口打满拖慢主循环。
 */
#define KK_DBG(enabled, period_ms, tag, fmt, ...)                                                  \
    do {                                                                                           \
        if (enabled) {                                                                             \
            static uint32_t kk__dbg_last;                                                          \
            if (kk_diag_due(&kk__dbg_last, (period_ms))) {                                         \
                ESP_LOGW(tag, fmt, ##__VA_ARGS__);                                                 \
            }                                                                                      \
        }                                                                                          \
    } while (0)
