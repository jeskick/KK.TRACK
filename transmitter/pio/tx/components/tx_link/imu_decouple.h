#pragma once

#include <stdbool.h>

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
