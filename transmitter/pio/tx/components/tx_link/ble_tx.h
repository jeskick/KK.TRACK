#pragma once
#include <stdbool.h>
typedef void (*kk_ble_tx_event_cb_t)(void);
void kk_ble_tx_init(void);
bool kk_ble_tx_host_ready(void);
bool kk_ble_tx_is_connected(void);
bool kk_ble_tx_is_link_ready(void);
bool kk_ble_tx_scan_pair(void);
bool kk_ble_tx_connect_mac(const char *mac, bool save_mac);
bool kk_ble_tx_send_telemetry(const char *payload);
void kk_ble_tx_send_repair(void);
void kk_ble_tx_send_center(void);
void kk_ble_tx_disconnect(void);
void kk_ble_tx_set_on_disconnect(kk_ble_tx_event_cb_t cb);
void kk_ble_tx_set_on_repair_peer(kk_ble_tx_event_cb_t cb);
void kk_ble_tx_set_on_center_peer(kk_ble_tx_event_cb_t cb);
