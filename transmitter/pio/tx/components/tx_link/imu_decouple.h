#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "kk/tx_track_cfg.h"

typedef struct {
    float out_yaw;
    float out_pitch;
    float prev_yaw;
    float prev_pitch;
    float prev_roll;
    float sup_yaw_f;
    float sup_pitch_f;
    bool seeded;
} kk_imu_decouple_t;

void kk_imu_decouple_reset(kk_imu_decouple_t *st, float yaw_deg, float pitch_deg);

void kk_imu_decouple_apply(kk_imu_decouple_t *st, float yaw_raw, float pitch_raw,
                           float roll_raw, float gyro_pitch_dps, float gyro_yaw_dps,
                           float gyro_roll_dps, const kk_tx_track_cfg_t *cfg, float *yaw_out,
                           float *pitch_out);

/*
 * 几何法之上的跨轴干扰过滤器（变增益误差低通）。
 * 输入 yaw_geo/pitch_geo 为几何解耦输出的绝对角；用逻辑系陀螺判定主导轴，
 * 把“对侧主导”期间本轴向真值的跟踪系数压低，从而过滤交叉耦合冲动；
 * 静止时两轴满跟踪、收敛到真值（无滞后、无漂移）。seeded=false 时首帧自动播种。
 */
typedef struct {
    bool seeded;
    float out_yaw;
    float out_pitch;
    float sup_yaw;   /* 平滑后的 yaw 抑制量 0..1（pitch 主导时升高） */
    float sup_pitch; /* 平滑后的 pitch 抑制量 0..1（yaw 主导时升高） */
    float ref_yaw;   /* 绝对保持参照：该轴自由时锁存几何真值，被抑制时冻结于动作前角度 */
    float ref_pitch;
    uint16_t rel_hold; /* 释放保持倒计时(样本数)：活动期刷满，换向速度过零的短暂凹陷期内维持抑制不松开 */
} kk_imu_xdec_t;

void kk_imu_xdec_reset(kk_imu_xdec_t *st);

void kk_imu_xdec_apply(kk_imu_xdec_t *st, float yaw_geo, float pitch_geo, float gyro_yaw_dps,
                       float gyro_pitch_dps, const kk_tx_track_cfg_t *cfg, float *yaw_out,
                       float *pitch_out);
