#pragma once

#include <stdbool.h>

#include "kk/imu_mount.h"

bool kk_imu_tx_init(void);
bool kk_imu_tx_poll(void);
bool kk_imu_tx_ready(void);
bool kk_imu_tx_has_pose(void);
float kk_imu_tx_roll_deg(void);
float kk_imu_tx_pitch_deg(void);
float kk_imu_tx_yaw_deg(void);
void kk_imu_tx_rezero(void);
void kk_imu_tx_set_mount(const kk_imu_mount_t *mount);
float kk_imu_tx_sensor_deg(uint8_t axis);
