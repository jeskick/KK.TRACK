#pragma once

#include <stdbool.h>

#include "kk/imu_mount.h"
#include "kk/tx_track_cfg.h"

/*
 * 解耦链路调试快照：每次姿态更新(update_mapped)刷新一帧，供 KK_DBG_DECOUPLE 实时流分析。
 * 链路顺序：raw(原始欧拉) -> geo(几何解耦,无 roll 耦合) -> out(xdec 跨轴过滤输出)。
 * gyro 为逻辑系角速率(驱动主导判定)；sup 为 xdec 抑制量 0..1(对侧主导时升高)。
 */
typedef struct {
    float raw_yaw, raw_pitch;   /* 原始欧拉角(耦合参照) */
    float geo_yaw, geo_pitch;   /* 几何解耦输出(xdec 前) */
    float out_yaw, out_pitch;   /* xdec 输出(最终送遥测前的解耦结果) */
    float gyro_yaw, gyro_pitch; /* 逻辑系陀螺速率 dps */
    float sup_yaw, sup_pitch;   /* xdec 抑制量 0..1 */
} kk_imu_tx_dbg_t;

void kk_imu_tx_get_dbg(kk_imu_tx_dbg_t *out);

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
