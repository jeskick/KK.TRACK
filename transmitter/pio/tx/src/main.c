#include "ble_tx.h"
#include "gesture_center.h"
#include "imu_tx.h"
#include "motion_detect.h"
#include "tx_gesture.h"
#include "tx_ota.h"

#include "kk/board_tx.h"
#include "kk/kk_log.h"
#include "kk/led.h"
#include "kk/link_config.h"
#include "kk/repair.h"
#include "kk/storage.h"
#include "kk/telemetry.h"
#include "kk/time.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "KK.TX";

static char s_rx_mac[24];
static bool s_paired;
static bool s_ble_on;
static bool s_repair_busy;
static uint32_t s_next_ble_ms;
static uint32_t s_next_tel_ms;

static kk_btn_repair_t s_btn;
static kk_btn_short_t s_btn_short;
static kk_led_code_player_t s_led_code;

static bool s_imu_prep;
static bool s_imu_ok;
static uint8_t s_green_code;

static void repair_enter_local(bool notify_peer);
static void center_enter(bool notify_peer);

static void led_update(void)
{
    const bool imu_ok = s_imu_ok && kk_imu_tx_has_pose();
    kk_led_in_t in = {
        .ble_connected = s_ble_on,
        .wifi_active = false,
        .func_prepare = s_imu_prep,
        .func_ok = imu_ok && !s_imu_prep,
        .err_code = s_green_code,
    };
    kk_led_apply(PIN_LED_BLUE, PIN_LED_GREEN, &in, &s_led_code);
}

static void on_gesture_center(void)
{
    ESP_LOGW(TAG, "[CENTER] roll gesture");
    center_enter(true);
}

static void center_enter(bool notify_peer)
{
    if (kk_tx_ota_is_active()) {
        ESP_LOGW(TAG, "[CENTER] ignored during OTA");
        return;
    }
    if (s_imu_ok) {
        kk_imu_tx_rezero();
    }
    kk_gesture_center_suppress(kk_millis());
    ESP_LOGW(TAG, "[CENTER] IMU re-zero");
#if KK_TX_IMU_DEBUG_SERIAL
    printf("# re-zero\n");
#endif
#if !KK_TX_IMU_DEBUG_SERIAL
    if (notify_peer && kk_ble_tx_is_link_ready()) {
        ESP_LOGW(TAG, "[CENTER] notify RX");
        kk_ble_tx_send_center();
    }
#endif
}

static void on_center_from_rx(void)
{
    ESP_LOGW(TAG, "[CENTER] from RX");
    center_enter(false);
}

static void poll_btn(void)
{
    switch (kk_btn_multifunc_poll(PIN_BTN, &s_btn, &s_btn_short)) {
    case KK_BTN_EVT_REPAIR:
        repair_enter_local(true);
        break;
    case KK_BTN_EVT_SHORT:
        ESP_LOGW(TAG, "[BTN] short -> center");
        center_enter(true);
        break;
    default:
        break;
    }
}

#if !KK_TX_IMU_DEBUG_SERIAL

static void repair_enter_local(bool notify_peer)
{
    if (kk_tx_ota_is_active()) {
        ESP_LOGW(TAG, "[PAIR] ignored during OTA");
        return;
    }
    if (s_repair_busy) {
        return;
    }
    s_repair_busy = true;
    if (notify_peer) {
        ESP_LOGW(TAG, "[PAIR] notify RX -> disconnect");
        kk_ble_tx_send_repair();
    }
    kk_ble_tx_disconnect();
    ESP_LOGW(TAG, "[PAIR] clear NVS peer_mac");
    kk_storage_clear_peer();
    s_paired = false;
    s_rx_mac[0] = '\0';
    s_ble_on = false;
    s_next_ble_ms = 0;
    s_repair_busy = false;
    ESP_LOGW(TAG, "[PAIR] wait re-pair");
}

static void on_repair_from_rx(void)
{
    if (s_repair_busy) {
        return;
    }
    s_repair_busy = true;
    ESP_LOGW(TAG, "[PAIR] REPAIR from RX -> disconnect");
    kk_ble_tx_disconnect();
    ESP_LOGW(TAG, "[PAIR] clear NVS peer_mac");
    kk_storage_clear_peer();
    s_paired = false;
    s_rx_mac[0] = '\0';
    s_ble_on = false;
    s_next_ble_ms = 0;
    s_repair_busy = false;
    ESP_LOGW(TAG, "[PAIR] wait re-pair");
}

static void on_ble_lost(void)
{
    /* 本函数在 NimBLE host 任务的 GAP 回调里执行；flash 写入（esp_ota_abort）
     * 必须留到主循环 kk_tx_ota_poll() 中处理（安全不变量#5），此处只置请求标志。 */
    if (kk_tx_ota_is_active()) {
        ESP_LOGW(TAG, "[BLE] lost during OTA -> request abort");
        kk_tx_ota_request_abort();
    }
    s_ble_on = false;
    s_next_ble_ms = 0;
    ESP_LOGW(TAG, "[BLE] lost -> reconnect now");
}

static void poll_imu(void)
{
    if (kk_tx_ota_is_active()) {
        return;
    }
    if (!kk_imu_tx_poll()) {
        return;
    }
    if (!kk_imu_tx_has_pose()) {
        return;
    }

    static uint32_t s_gest_ms;
    const uint32_t now = kk_millis();
    if (now < s_gest_ms) {
        return;
    }
    s_gest_ms = now + KK_TX_GESTURE_POLL_MS;

    kk_gesture_center_poll(kk_imu_tx_roll_deg(), kk_imu_tx_pitch_deg(), kk_imu_tx_yaw_deg(),
                           kk_imu_tx_gyro_roll_dps(), kk_imu_tx_gyro_yaw_dps(), now);
}

static void tel_poll_imu(void)
{
    if (kk_tx_ota_is_active() || !kk_ble_tx_is_link_ready() || !kk_imu_tx_has_pose()) {
        return;
    }

    const uint32_t now = kk_millis();
    if (now < s_next_tel_ms) {
        return;
    }
    s_next_tel_ms = now + KK_BLE_TEL_MS;

    char payload[48];
    const float yaw = kk_imu_tx_yaw_deg();
    const float pitch = kk_imu_tx_pitch_deg();
    if (kk_tel_format_pose(payload, sizeof(payload), yaw, pitch)) {
        if (!kk_ble_tx_send_telemetry(payload)) {
            /* 写入失败多为链路拥塞/mbuf 暂不足：退避一拍给协议栈排空，
             * 避免以 20ms 节奏持续重试把缓冲彻底打爆（os_memblock 耗尽掉线）。 */
            s_next_tel_ms = now + KK_BLE_TEL_BACKOFF_MS;
        }
    }
}

static void imu_log_throttled(void)
{
    /* IMU 姿态故障/恢复：状态变化时打一条（边沿），不再每 2s 周期刷 */
    KK_EVT_CHG(TAG, imu_pose, kk_imu_tx_has_pose() ? 1 : 0, "[IMU] %s",
               kk_imu_tx_has_pose() ? "pose OK" : "FAULT waiting recover");

    if (!kk_imu_tx_has_pose()) {
        return;
    }

    /* 聚焦调试实时流（默认全关，置 KK_DBG_* = 1 重编译开启）：
     *   KK_DBG_DECOUPLE：解耦链路全量快照，列固定、便于复制到表格按规律分析：
     *     gY/gP = 逻辑系陀螺(dps，驱动主导判定)
     *     rawY/rawP = 原始欧拉角(耦合参照)
     *     geoY/geoP = 几何解耦输出(xdec 前)
     *     outY/outP = xdec 输出(最终解耦结果，即送往舵机的角度)
     *     supY/supP = xdec 抑制量 0..1(对侧主导时升高)
     *   分析要点：纯转 yaw 时看 geoP 动多少(几何/物理耦合) vs outP 动多少(过滤后残留)。 */
#if KK_DBG_DECOUPLE
    {
        kk_imu_tx_dbg_t d;
        kk_imu_tx_get_dbg(&d);
        KK_DBG(1, KK_DBG_DECOUPLE_MS, TAG,
               "[DEC] gY=%.0f gP=%.0f | rawY=%.1f rawP=%.1f | geoY=%.1f geoP=%.1f | outY=%.1f "
               "outP=%.1f | supY=%.2f supP=%.2f%s",
               d.gyro_yaw, d.gyro_pitch, d.raw_yaw, d.raw_pitch, d.geo_yaw, d.geo_pitch, d.out_yaw,
               d.out_pitch, d.sup_yaw, d.sup_pitch, kk_imu_tx_is_motion_paused() ? " HOLD" : "");
    }
#endif
    KK_DBG(KK_DBG_POSE, KK_DBG_STREAM_MS, TAG, "[POSE] Y=%d P=%d R=%d", (int)kk_imu_tx_yaw_deg(),
           (int)kk_imu_tx_pitch_deg(), (int)kk_imu_tx_roll_deg());

    /* KK_DBG_MOTION：移动检测每拍判据。分析"时好时坏"——看 body/ho/nod 三个判定与
     *   lin(对 3.2 硬阈/1.4 软阈)、st(=4 运动中)是否在边界翻动；trg 攒到 480 才回中(PAUSE)，
     *   set 攒到 1500 才重新归零恢复(RUN)。漏检=该 body=1 时却为 0；误触=静止/纯转头时 body=1。 */
#if KK_DBG_MOTION
    if (kk_imu_tx_motion_enabled()) {
        kk_motion_dbg_t m;
        kk_motion_detect_get_dbg(&m);
        KK_DBG(1, KK_DBG_STREAM_MS, TAG,
               "[MOB] %s lin=%.1f lt=%.1f gt=%.0f st=%d gP=%.0f gY=%.0f gR=%.0f ho=%d nod=%d ha=%d body=%d trg=%lu set=%lu",
               m.state ? "PAUSE" : "RUN", m.lin_metric, m.lin_trans, m.grav_tilt, (int)m.stability,
               m.g_pitch, m.g_yaw, m.g_roll, m.head_only ? 1 : 0, m.head_nod ? 1 : 0,
               m.head_active ? 1 : 0, m.body_motion ? 1 : 0, (unsigned long)m.trigger_ms,
               (unsigned long)m.settle_ms);
    }
#endif
}

static void ble_init_task(void *arg)
{
    (void)arg;
    esp_log_level_set("phy_init", ESP_LOG_ERROR);
    kk_ble_tx_init();
    vTaskDelete(NULL);
}

static void app_run_ble(void)
{
    xTaskCreate(ble_init_task, "ble_init", 12288, NULL, 3, NULL);

    while (1) {
        poll_btn();
        kk_ble_tx_poll();
        kk_tx_ota_poll();
        kk_tx_ota_poll_boot_confirm(kk_millis());
        poll_imu();
        if (!kk_tx_ota_is_active()) {
            kk_imu_tx_motion_poll();
        }
        tel_poll_imu();
        imu_log_throttled();

        if (!s_ble_on) {
            if (kk_ble_tx_host_ready() && kk_millis() >= s_next_ble_ms && !s_repair_busy) {
                const bool ok = s_paired ? kk_ble_tx_connect_mac(s_rx_mac, false)
                                         : kk_ble_tx_scan_pair();
                if (ok) {
                    s_ble_on = true;
                    s_paired = kk_storage_load_paired(s_rx_mac, sizeof(s_rx_mac));
                } else {
                    s_next_ble_ms = kk_millis() +
                                    (s_paired ? KK_BLE_RECONNECT_MS : KK_BLE_RETRY_MS);
                }
            }
            led_update();
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        if (!kk_ble_tx_is_connected()) {
            s_ble_on = false;
            s_next_ble_ms = 0;
        }

        led_update();
        if (kk_tx_ota_is_active()) {
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

#else /* KK_TX_IMU_DEBUG_SERIAL */

static void repair_enter_local(bool notify_peer)
{
    (void)notify_peer;
    if (s_repair_busy) {
        return;
    }
    s_repair_busy = true;
    ESP_LOGW(TAG, "[PAIR] clear NVS peer_mac (BLE off)");
    kk_storage_clear_peer();
    s_paired = false;
    s_rx_mac[0] = '\0';
    s_repair_busy = false;
    ESP_LOGW(TAG, "[PAIR] wait re-pair when BLE enabled");
}

#endif /* KK_TX_IMU_DEBUG_SERIAL */

#if KK_TX_IMU_DEBUG_SERIAL

static void app_run_imu_serial(void)
{
    printf("ms,Y(Yaw偏航),P(Pitch俯仰),R(Roll横滚)\n");

    while (1) {
        poll_btn();

        if (kk_imu_tx_poll()) {
            printf("%lu,%d,%d,%d\n", (unsigned long)kk_millis(), (int)kk_imu_tx_yaw_deg(),
                   (int)kk_imu_tx_pitch_deg(), (int)kk_imu_tx_roll_deg());
        }

        kk_tx_ota_poll_boot_confirm(kk_millis());
        led_update();
        taskYIELD();
    }
}

#endif /* KK_TX_IMU_DEBUG_SERIAL */

static void app_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    kk_tx_ota_init();
    kk_tx_ota_log_partitions();

    kk_led_pins_init(PIN_LED_BLUE, PIN_LED_GREEN);
    kk_led_code_init(&s_led_code);
    kk_btn_repair_init(&s_btn);
    kk_btn_short_init(&s_btn_short);

    gpio_reset_pin(PIN_BTN);
    gpio_set_direction(PIN_BTN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BTN, GPIO_PULLUP_ONLY);

    s_imu_prep = true;
    s_imu_ok = kk_imu_tx_init();
    s_imu_prep = false;
    if (!s_imu_ok) {
        s_green_code = KK_GCODE_IMU_SPI;
        ESP_LOGE(TAG, "IMU init failed");
    } else {
        s_green_code = KK_GCODE_NONE;
        kk_tx_gesture_load(NULL);
        kk_gesture_center_init(on_gesture_center);
        ESP_LOGW(TAG, "gesture center: roll swing -> settle -> auto center");
    }

#if KK_TX_IMU_DEBUG_SERIAL
    ESP_LOGW(TAG, "=== TX IMU serial debug (BLE off) ===");
    ESP_LOGW(TAG, "btn: tap=center  hold 5s=repair");
#else
    s_paired = kk_storage_load_paired(s_rx_mac, sizeof(s_rx_mac));
    ESP_LOGW(TAG, "=== Track.KK.TX (ESP-IDF) ===");
    ESP_LOGW(TAG, "telemetry: IMU yaw/pitch @%lums", (unsigned long)KK_BLE_TEL_MS);
    ESP_LOGW(TAG, "IMU轴 Y=偏航(左转+) P=俯仰(低头+) R=横滚(左下+)");
    ESP_LOGW(TAG, "btn: tap=center  hold 5s=repair  roll-swing=auto center");
    kk_ble_tx_set_on_disconnect(on_ble_lost);
    kk_ble_tx_set_on_repair_peer(on_repair_from_rx);
    kk_ble_tx_set_on_center_peer(on_center_from_rx);
    s_next_ble_ms = 0;
#endif
}

void app_main(void)
{
    app_init();

#if KK_TX_IMU_DEBUG_SERIAL
    app_run_imu_serial();
#else
    app_run_ble();
#endif
}
