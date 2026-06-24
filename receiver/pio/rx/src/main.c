#include "ble_rx.h"
#include "wifi_rx.h"

#include "kk/board_rx.h"
#include "kk/head_track.h"
#include "kk/kk_log.h"
#include "kk/led.h"
#include "kk/link_config.h"
#include "kk/repair.h"
#include "kk/rc_out.h"
#include "kk/rx_profile.h"
#include "kk/rx_ota.h"
#include "kk/tx_track_cfg.h"
#include "kk/rx_web.h"
#include "kk/storage.h"
#include "kk/telemetry.h"
#include "kk/time.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "KK.RX";

static const char *reset_reason_str(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
        return "POWERON";
    case ESP_RST_SW:
        return "SW";
    case ESP_RST_PANIC:
        return "PANIC";
    case ESP_RST_INT_WDT:
        return "INT_WDT";
    case ESP_RST_TASK_WDT:
        return "TASK_WDT";
    case ESP_RST_WDT:
        return "WDT";
    case ESP_RST_BROWNOUT:
        return "BROWNOUT";
    default:
        return "other";
    }
}

kk_rx_profile_t g_profile;

static bool s_ble_on;
static bool s_paired;
static char s_tx_mac[24];
static bool s_repair_busy;
static bool s_need_ap;
static bool s_web_on;
static bool s_web_pending;
static bool s_ppm_on;
static bool s_need_ppm;

static bool s_need_link_down;
static uint32_t s_ap_after_ms;
static uint32_t s_tx_sync_next_ms;
static uint8_t s_tx_sync_left;
static bool s_wifi_scheduled;
static bool s_wifi_was_on;

static kk_btn_repair_t s_btn;
static kk_btn_short_t s_btn_short;
static kk_led_code_player_t s_led_code;

static void led_boot_test(void)
{
    kk_led_boot_test(PIN_LED_BLUE, PIN_LED_GREEN);
    ESP_LOGW(TAG, "LED test: 0.8s blue@GPIO%d then 0.8s green@GPIO%d",
             PIN_LED_BLUE, PIN_LED_GREEN);
}

static void led_update(void)
{
    const bool btn_hold = kk_btn_repair_holding(PIN_BTN);
    kk_led_in_t in = {
        .ble_connected = btn_hold ? false : s_ble_on,
        /* 蓝灯快闪：BLE 已连且 WiFi AP 已开；按住按键时也快闪提示 */
        .wifi_active = btn_hold || (s_ble_on && kk_wifi_rx_is_on()),
        .err_code = KK_GCODE_NONE,
    };
    const uint32_t now = kk_millis();
    const uint32_t pkt_age = g_kk_tel.last_pkt_ms > 0 ? now - g_kk_tel.last_pkt_ms : 0;

    if (s_ppm_on && (!s_ble_on || kk_head_track_failsafe_active() ||
                     (g_kk_tel.last_pkt_ms > 0 && pkt_age > KK_RX_FS_TEL_LOST_MS))) {
        in.err_code = KK_GCODE_NO_DATA;
    }
    in.func_ok = s_ble_on && g_kk_tel.last_pkt_ms > 0 && pkt_age < KK_RX_FS_TEL_LOST_MS &&
                 !kk_head_track_failsafe_active();
    in.func_prepare = s_ble_on && !in.func_ok && in.err_code == KK_GCODE_NONE;

    kk_led_apply(PIN_LED_BLUE, PIN_LED_GREEN, &in, &s_led_code);

    if (s_ppm_on) {
        kk_rc_out_set_failsafe(in.err_code == KK_GCODE_NO_DATA);
    }
}

static void repair_enter(bool notify_peer)
{
    if (kk_rx_ota_is_active()) {
        ESP_LOGW(TAG, "[PAIR] ignored during OTA");
        return;
    }
    if (s_repair_busy) {
        return;
    }
    s_repair_busy = true;

    if (notify_peer) {
        ESP_LOGW(TAG, "[PAIR] notify TX -> disconnect");
        kk_ble_rx_disconnect_peer(true);
    } else if (kk_ble_rx_is_connected()) {
        ESP_LOGW(TAG, "[PAIR] disconnect (peer initiated)");
        kk_ble_rx_disconnect_peer(false);
    }

    s_ble_on = false;
    s_need_ap = false;
    s_wifi_scheduled = false;
    s_web_pending = false;
    s_need_ppm = false;
    s_ppm_on = false;
    kk_head_track_failsafe_reset();
    kk_rc_out_stop();
    if (s_web_on) {
        kk_rx_web_stop();
        s_web_on = false;
    }
    kk_wifi_rx_ap_stop();

    ESP_LOGW(TAG, "[PAIR] clear NVS peer_mac");
    kk_storage_clear_peer();
    s_paired = false;
    s_tx_mac[0] = '\0';
    kk_ble_rx_start_adv();
    s_repair_busy = false;
    ESP_LOGW(TAG, "[PAIR] wait re-pair");
}

static void sync_mount_to_tx(const kk_imu_mount_t *mount)
{
    if (mount) {
        kk_ble_rx_send_mount(mount);
    }
}

static void sync_gesture_to_tx(const kk_gesture_cfg_t *cfg)
{
    if (cfg) {
        kk_ble_rx_send_gesture(cfg);
    }
}

static void sync_track_to_tx(const kk_tx_track_cfg_t *cfg)
{
    if (cfg) {
        kk_ble_rx_send_track(cfg);
    }
}

static void sync_profile_to_tx(void)
{
    if (!s_ble_on || !kk_ble_rx_is_connected()) {
        return;
    }
    kk_imu_mount_t m;
    kk_rx_profile_mount_to_imu(&g_profile, &m);
    sync_mount_to_tx(&m);
    kk_gesture_cfg_t g;
    kk_rx_profile_gesture_to_cfg(&g_profile, &g);
    sync_gesture_to_tx(&g);
    kk_tx_track_cfg_t t;
    kk_rx_profile_track_to_cfg(&g_profile, &t);
    sync_track_to_tx(&t);
}

static void on_web_profile_saved(void)
{
    s_tx_sync_left = 3;
    s_tx_sync_next_ms = 0;
}

static void on_ota_prepare(void)
{
    if (s_ppm_on) {
        kk_head_track_offset_center(&g_profile);
    }
}

static esp_err_t ota_ble_tx_begin(size_t size)
{
    return kk_ble_rx_ota_tx_begin(size);
}

static esp_err_t ota_ble_tx_write(const uint8_t *data, size_t len)
{
    return kk_ble_rx_ota_tx_send(data, len);
}

static esp_err_t ota_ble_tx_finish(void)
{
    return kk_ble_rx_ota_tx_finish();
}

static void ota_ble_tx_abort(void)
{
    kk_ble_rx_ota_tx_cancel_wait();
    kk_ble_rx_ota_tx_abort();
}

static bool ota_ble_tx_ready(void)
{
    return kk_ble_rx_ota_tx_ready();
}

static const kk_rx_ota_tx_ops_t s_ota_tx_ops = {
    .ready = ota_ble_tx_ready,
    .begin = ota_ble_tx_begin,
    .write = ota_ble_tx_write,
    .finish = ota_ble_tx_finish,
    .abort = ota_ble_tx_abort,
};

static void on_ble_connect(void)
{
    s_ble_on = true;
    s_wifi_scheduled = false;
    if (!s_ppm_on) {
        s_need_ppm = true;
    }
    s_paired = kk_storage_load_paired(s_tx_mac, sizeof(s_tx_mac));
    ESP_LOGW(TAG, "[BLE] link up peer %s", s_tx_mac);
    sync_profile_to_tx();
}

static void on_ble_telemetry(void)
{
    if (!s_ppm_on) {
        s_need_ppm = true;
    }
}

static void on_ble_disconnect(void)
{
    s_ble_on = false;
    if (kk_rx_ota_is_tx_relay()) {
        ESP_LOGW(TAG, "[BLE] disconnected during TX OTA");
        kk_ble_rx_ota_tx_cancel_wait();
        kk_ble_rx_ota_tx_abort();
        kk_rx_ota_tx_abort();
        return;
    }
    s_need_ap = false;
    s_wifi_scheduled = false;
    s_web_pending = false;
    s_need_link_down = true;
    ESP_LOGW(TAG, "[BLE] disconnected");
}

static void schedule_wifi_after_stable(uint32_t now)
{
    if (s_wifi_scheduled || kk_wifi_rx_is_on() || !s_ppm_on || !s_ble_on) {
        return;
    }
    s_wifi_scheduled = true;
    s_need_ap = true;
    s_ap_after_ms = now + KK_WIFI_STABLE_DELAY_MS;
    ESP_LOGW(TAG, "[WiFi] schedule AP in %lums (BLE+PPM stable)",
             (unsigned long)KK_WIFI_STABLE_DELAY_MS);
}

static void on_repair_from_tx(void)
{
    if (kk_rx_ota_is_active()) {
        ESP_LOGW(TAG, "[PAIR] ignored during OTA");
        return;
    }
    ESP_LOGW(TAG, "[PAIR] REPAIR from TX");
    repair_enter(false); /* peer already notified; disconnect then clear */
}

static void center_enter(bool notify_peer)
{
    if (kk_rx_ota_is_active()) {
        ESP_LOGW(TAG, "[CENTER] ignored during OTA");
        return;
    }
    ESP_LOGW(TAG, "[CENTER] offset center");
    if (s_ppm_on) {
        kk_head_track_offset_center(&g_profile);
    } else {
        kk_head_track_reset();
    }
    if (notify_peer && kk_ble_rx_is_connected()) {
        ESP_LOGW(TAG, "[CENTER] notify TX");
        kk_ble_rx_send_center();
    }
}

static void on_center_from_tx(void)
{
    ESP_LOGW(TAG, "[CENTER] from TX");
    center_enter(false);
}

static void on_ble_ready(void)
{
    /* 仅 BLE 协议栈就绪；PPM 延后到 TX 真正连上，避免广播阶段崩溃 */
}

static void app_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    gpio_reset_pin(PIN_BTN);
    gpio_set_direction(PIN_BTN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BTN, GPIO_PULLUP_ONLY);

    kk_led_pins_init(PIN_LED_BLUE, PIN_LED_GREEN);
    led_boot_test();
    kk_led_code_init(&s_led_code);
    kk_rx_profile_load(&g_profile);
    kk_tel_reset();
    kk_head_track_reset();
    kk_btn_repair_init(&s_btn);
    kk_btn_short_init(&s_btn_short);

    s_tx_mac[0] = '\0';
    s_paired = kk_storage_load_paired(s_tx_mac, sizeof(s_tx_mac));
    ESP_LOGW(TAG, "=== Track.KK.RX (ESP-IDF) ===");
    ESP_LOGW(TAG, "reset reason=%s", reset_reason_str());
    if (s_paired) {
        ESP_LOGW(TAG, "paired tx %s", s_tx_mac);
        ESP_LOGW(TAG, "ble name=%s adv=hidden", KK_BLE_NAME_RX);
    } else {
        ESP_LOGW(TAG, "unpaired");
        ESP_LOGW(TAG, "ble name=%s adv=visible", KK_BLE_NAME_RX);
    }

    kk_wifi_rx_init();
    kk_rx_ota_init();
    kk_rx_ota_log_partitions();
    kk_rx_ota_set_tx_ops(&s_ota_tx_ops);
    kk_rx_web_set_mount_sync(sync_mount_to_tx);
    kk_rx_web_set_gesture_sync(sync_gesture_to_tx);
    kk_rx_web_set_track_sync(sync_track_to_tx);
    kk_rx_web_set_on_saved(on_web_profile_saved);
    kk_rx_web_set_ota_prepare(on_ota_prepare);
    kk_ble_rx_set_on_connect(on_ble_connect);
    kk_ble_rx_set_on_disconnect(on_ble_disconnect);
    kk_ble_rx_set_on_repair_peer(on_repair_from_tx);
    kk_ble_rx_set_on_center_peer(on_center_from_tx);
    kk_ble_rx_set_on_telemetry(on_ble_telemetry);
    kk_ble_rx_set_on_ready(on_ble_ready);
}

static void ble_init_task(void *arg)
{
    (void)arg;
    esp_log_level_set("phy_init", ESP_LOG_ERROR);
    kk_ble_rx_init();
    vTaskDelete(NULL);
}

void app_main(void)
{
    app_init();
    xTaskCreate(ble_init_task, "ble_init", 16384, NULL, 3, NULL);
    KK_EVT(TAG, "[RX] main loop start");

    while (1) {
        const uint32_t now = kk_millis();

        if (s_need_link_down) {
            s_need_link_down = false;
            if (!kk_rx_ota_is_active()) {
                if (s_web_on) {
                    kk_rx_web_stop();
                    s_web_on = false;
                }
                kk_wifi_rx_ap_stop();
            }
        }

        if (s_need_ppm) {
            s_need_ppm = false;
            kk_rc_out_begin((kk_rc_proto_t)g_profile.rc_proto);
            kk_head_track_failsafe_reset();
            kk_head_track_poll(&g_profile, s_ble_on, true, now);
            s_ppm_on = true;
            ESP_LOGW(TAG, "RC out on proto=%u", (unsigned)g_profile.rc_proto);
            schedule_wifi_after_stable(now);
        }

        if (!s_need_ppm && s_ppm_on) {
            schedule_wifi_after_stable(now);
        }

        if (s_need_ap && now >= s_ap_after_ms) {
            s_need_ap = false;
            kk_wifi_rx_ap_start();
            s_web_pending = true;
        }

        switch (kk_btn_multifunc_poll(PIN_BTN, &s_btn, &s_btn_short)) {
        case KK_BTN_EVT_REPAIR:
            ESP_LOGW(TAG, "[PAIR] hold 5s -> repair");
            repair_enter(true);
            break;
        case KK_BTN_EVT_SHORT:
            ESP_LOGW(TAG, "[BTN] short -> center");
            center_enter(true);
            break;
        default:
            break;
        }

        if (s_web_pending && kk_wifi_rx_ap_ready() && !s_web_on) {
            kk_rx_web_begin(&g_profile);
            s_web_on = true;
            s_web_pending = false;
            ESP_LOGW(TAG, "web server on");
        }

        if (s_ble_on && s_tx_sync_left > 0 && !kk_rx_ota_is_active()) {
            if (s_tx_sync_next_ms == 0 || now >= s_tx_sync_next_ms) {
                sync_profile_to_tx();
                s_tx_sync_left--;
                s_tx_sync_next_ms = now + 1000;
            }
        }

        kk_wifi_rx_idle_poll();

        const bool wifi_on = kk_wifi_rx_is_on();
        if (s_wifi_was_on && !wifi_on) {
            if (s_web_on) {
                kk_rx_web_stop();
                s_web_on = false;
            }
            s_web_pending = false;
            ESP_LOGW(TAG, "[WiFi] AP off -> web stopped");
        }
        s_wifi_was_on = wifi_on;

        if (wifi_on) {
            kk_wifi_rx_udp_poll();
        }

        kk_tel_poll_rx_voltage();
        kk_ble_rx_poll();
        if (!kk_rx_ota_is_active()) {
            kk_head_track_poll(&g_profile, s_ble_on, s_ppm_on, now);
        }
        led_update();

        kk_rx_ota_poll_boot_confirm(now);

        /* 链路连接状态变化时打一条（边沿，不周期刷） */
        KK_EVT_CHG(TAG, rx_ble, s_ble_on ? 1 : 0, "[BLE] %s", s_ble_on ? "connected" : "down");
        /* 聚焦调试：把 KK_DBG_POSE / KK_DBG_LINK 置 1 重编译可开实时流（默认关，零开销） */
        KK_DBG(KK_DBG_POSE, KK_DBG_STREAM_MS, TAG,
               "[POSE] yaw=%d pitch=%d lr=%u ud=%u ble=%d fs=%d", (int)g_kk_ht.yaw_f,
               (int)g_kk_ht.pitch_f, g_kk_ht.ppm_lr, g_kk_ht.ppm_ud, s_ble_on,
               kk_head_track_failsafe_active());
        KK_DBG(KK_DBG_LINK, KK_DBG_STREAM_MS, TAG, "[LINK] ble=%d ppm=%d wifi=%d last_pkt=%lu",
               s_ble_on, s_ppm_on, kk_wifi_rx_is_on(), (unsigned long)g_kk_tel.last_pkt_ms);

        vTaskDelay(pdMS_TO_TICKS(kk_rx_ota_is_active() ? 2 : 20));
    }
}
