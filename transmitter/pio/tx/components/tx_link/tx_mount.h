#pragma once

#include "kk/imu_mount.h"

void kk_tx_mount_load(kk_imu_mount_t *out);
void kk_tx_mount_save(const kk_imu_mount_t *mount);
void kk_tx_mount_apply(const kk_imu_mount_t *mount);
const kk_imu_mount_t *kk_tx_mount_get(void);
