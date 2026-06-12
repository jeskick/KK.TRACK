#pragma once
#include <stdbool.h>

#include "kk/imu_mount.h"
typedef void (*kk_ble_rx_event_cb_t)(void);
void kk_ble_rx_init(void);
void kk_ble_rx_set_on_ready(kk_ble_rx_event_cb_t cb);
bool kk_ble_rx_is_connected(void);
void kk_ble_rx_start_adv(void);
void kk_ble_rx_disconnect_peer(bool notify_peer);
void kk_ble_rx_send_center(void);
void kk_ble_rx_send_mount(const kk_imu_mount_t *mount);
void kk_ble_rx_set_on_connect(kk_ble_rx_event_cb_t cb);
void kk_ble_rx_set_on_disconnect(kk_ble_rx_event_cb_t cb);
void kk_ble_rx_set_on_repair_peer(kk_ble_rx_event_cb_t cb);
void kk_ble_rx_set_on_center_peer(kk_ble_rx_event_cb_t cb);
void kk_ble_rx_set_on_telemetry(kk_ble_rx_event_cb_t cb);
