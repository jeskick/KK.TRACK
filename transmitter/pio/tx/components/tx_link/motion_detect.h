#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef void (*kk_motion_rezero_cb_t)(void);

void kk_motion_detect_init(kk_motion_rezero_cb_t on_rezero);
void kk_motion_detect_reset(void);
void kk_motion_detect_set_enabled(bool enabled);
bool kk_motion_detect_is_enabled(void);
bool kk_motion_detect_is_paused(void);
uint32_t kk_motion_detect_trigger_ms(void);

void kk_motion_detect_apply(float in_yaw, float in_pitch, float gyro_pitch_dps,
                            float gyro_yaw_dps, float gyro_roll_dps,
                            int8_t stability_class, float lin_accel_mps2, uint32_t elapsed_ms,
                            float *out_yaw, float *out_pitch);
