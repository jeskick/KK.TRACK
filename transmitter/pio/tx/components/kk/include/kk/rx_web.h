#pragma once

#include "kk/gesture_cfg.h"
#include "kk/tx_track_cfg.h"
#include "kk/imu_mount.h"
#include "kk/rx_profile.h"

#include <stdint.h>

typedef void (*kk_rx_web_mount_sync_cb_t)(const kk_imu_mount_t *mount);
typedef void (*kk_rx_web_gesture_sync_cb_t)(const kk_gesture_cfg_t *cfg);
typedef void (*kk_rx_web_track_sync_cb_t)(const kk_tx_track_cfg_t *cfg);
typedef void (*kk_rx_web_saved_cb_t)(void);
typedef void (*kk_rx_web_ota_prepare_cb_t)(void);

typedef struct {
    bool ble;
    bool failsafe;
    bool motion_paused;
} kk_rx_web_live_status_t;

typedef void (*kk_rx_web_live_status_cb_t)(kk_rx_web_live_status_t *st);

void kk_rx_web_begin(kk_rx_profile_t *profile);
void kk_rx_web_set_mount_sync(kk_rx_web_mount_sync_cb_t cb);
void kk_rx_web_set_gesture_sync(kk_rx_web_gesture_sync_cb_t cb);
void kk_rx_web_set_track_sync(kk_rx_web_track_sync_cb_t cb);
void kk_rx_web_set_on_saved(kk_rx_web_saved_cb_t cb);
void kk_rx_web_set_ota_prepare(kk_rx_web_ota_prepare_cb_t cb);
void kk_rx_web_set_live_status_cb(kk_rx_web_live_status_cb_t cb);
void kk_rx_web_handle(void);
void kk_rx_web_stop(void);
void kk_rx_web_touch(void);
uint32_t kk_rx_web_last_http_ms(void);
