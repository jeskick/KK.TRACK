#pragma once

#include "kk/imu_mount.h"
#include "kk/rx_profile.h"

#include <stdint.h>

typedef void (*kk_rx_web_mount_sync_cb_t)(const kk_imu_mount_t *mount);

void kk_rx_web_begin(kk_rx_profile_t *profile);
void kk_rx_web_set_mount_sync(kk_rx_web_mount_sync_cb_t cb);
void kk_rx_web_handle(void);
void kk_rx_web_stop(void);
uint32_t kk_rx_web_last_http_ms(void);
