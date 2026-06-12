#include "ble_tx.h"
#include "imu_tx.h"

#include "kk/board_tx.h"
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
static uint32_t s_imu_log_ms;

static kk_btn_repair_t s_btn;
static kk_btn_short_t s_btn_short;
static kk_led_code_player_t s_led_code;

static bool s_imu_prep;
static bool s_imu_ok;
static uint8_t s_green_code;

static void repair_enter_local(bool notify_peer);

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

static void center_enter(bool notify_peer)
{
    if (s_imu_ok) {
        kk_imu_tx_rezero();
    }
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
    s_ble_on = false;
    s_next_ble_ms = 0;
    ESP_LOGW(TAG, "[BLE] lost -> reconnect now");
}

static void poll_imu(void)
{
    kk_imu_tx_poll();
}

static void tel_poll_imu(void)
{
    if (!kk_ble_tx_is_link_ready() || !kk_imu_tx_has_pose()) {
        return;
    }

    const uint32_t now = kk_millis();
    if (now < s_next_tel_ms) {
        return;
    }
    s_next_tel_ms = now + KK_BLE_TEL_MS;

    char payload[48];
    if (kk_tel_format_pose(payload, sizeof(payload), kk_imu_tx_yaw_deg(),
                           kk_imu_tx_pitch_deg())) {
        kk_ble_tx_send_telemetry(payload);
    }
}

static void imu_log_throttled(void)
{
    if (!kk_imu_tx_has_pose() || !kk_diag_due(&s_imu_log_ms, KK_DIAG_LOG_MS)) {
        return;
    }
    ESP_LOGW(TAG, "[IMU] Y=%d P=%d R=%d",
             (int)kk_imu_tx_yaw_deg(), (int)kk_imu_tx_pitch_deg(),
             (int)kk_imu_tx_roll_deg());
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
        poll_imu();
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
        vTaskDelay(pdMS_TO_TICKS(2));
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
    printf("ms,yaw,pitch,roll\n");

    while (1) {
        poll_btn();

        if (kk_imu_tx_poll()) {
            printf("%lu,%.2f,%.2f,%.2f\n",
                   (unsigned long)kk_millis(),
                   kk_imu_tx_yaw_deg(),
                   kk_imu_tx_pitch_deg(),
                   kk_imu_tx_roll_deg());
        }

        led_update();
        taskYIELD();
    }
}

#endif /* KK_TX_IMU_DEBUG_SERIAL */

static void app_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

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
    }

#if KK_TX_IMU_DEBUG_SERIAL
    ESP_LOGW(TAG, "=== TX IMU serial debug (BLE off) ===");
    ESP_LOGW(TAG, "btn: tap=center  hold 5s=repair");
#else
    s_paired = kk_storage_load_paired(s_rx_mac, sizeof(s_rx_mac));
    ESP_LOGW(TAG, "=== Track.KK.TX (ESP-IDF) ===");
    ESP_LOGW(TAG, "telemetry: IMU yaw/pitch @%lums", (unsigned long)KK_BLE_TEL_MS);
    ESP_LOGW(TAG, "btn: tap=center  hold 5s=repair");
    kk_ble_tx_set_on_disconnect(on_ble_lost);
    kk_ble_tx_set_on_repair_peer(on_repair_from_rx);
    kk_ble_tx_set_on_center_peer(on_center_from_rx);
    s_next_ble_ms = 0;
    s_imu_log_ms = 0;
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
