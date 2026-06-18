#pragma once

#include <stdbool.h>

#include "kk/imu_mount.h"
#include "kk/tx_track_cfg.h"

bool kk_imu_tx_init(void);
bool kk_imu_tx_poll(void);
bool kk_imu_tx_ready(void);
bool kk_imu_tx_has_pose(void);
bool kk_imu_tx_is_motion_paused(void);
float kk_imu_tx_roll_deg(void);
float kk_imu_tx_pitch_deg(void);
float kk_imu_tx_yaw_deg(void);
float kk_imu_tx_gyro_roll_dps(void);
float kk_imu_tx_gyro_yaw_dps(void);
void kk_imu_tx_rezero(void);
void kk_imu_tx_set_mount(const kk_imu_mount_t *mount);
void kk_imu_tx_motion_sensors(bool enable);
void kk_imu_tx_motion_poll(void);
void kk_imu_tx_apply_track_cfg(const kk_tx_track_cfg_t *cfg);
float kk_imu_tx_sensor_deg(uint8_t axis);
bool kk_imu_tx_motion_enabled(void);
float kk_imu_tx_motion_lin_mps2(void);
int8_t kk_imu_tx_motion_stability(void);
uint32_t kk_imu_tx_motion_trigger_ms(void);
