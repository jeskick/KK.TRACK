#pragma once
#include <stdbool.h>
#include "esp_err.h"

#include "kk/gesture_cfg.h"
#include "kk/tx_track_cfg.h"
#include "kk/imu_mount.h"
typedef void (*kk_ble_rx_event_cb_t)(void);
void kk_ble_rx_init(void);
void kk_ble_rx_set_on_ready(kk_ble_rx_event_cb_t cb);
bool kk_ble_rx_is_connected(void);
void kk_ble_rx_start_adv(void);
void kk_ble_rx_disconnect_peer(bool notify_peer);
void kk_ble_rx_send_center(void);
void kk_ble_rx_send_mount(const kk_imu_mount_t *mount);
void kk_ble_rx_send_gesture(const kk_gesture_cfg_t *cfg);
void kk_ble_rx_send_track(const kk_tx_track_cfg_t *cfg);
bool kk_ble_rx_ota_tx_ready(void);
esp_err_t kk_ble_rx_ota_tx_begin(size_t size);
esp_err_t kk_ble_rx_ota_tx_send(const uint8_t *data, size_t len);
esp_err_t kk_ble_rx_ota_tx_finish(void);
void kk_ble_rx_ota_tx_abort(void);
void kk_ble_rx_ota_tx_cancel_wait(void);
void kk_ble_rx_set_on_connect(kk_ble_rx_event_cb_t cb);
void kk_ble_rx_set_on_disconnect(kk_ble_rx_event_cb_t cb);
void kk_ble_rx_set_on_repair_peer(kk_ble_rx_event_cb_t cb);
void kk_ble_rx_set_on_center_peer(kk_ble_rx_event_cb_t cb);
void kk_ble_rx_set_on_telemetry(kk_ble_rx_event_cb_t cb);
void kk_ble_rx_poll(void);
