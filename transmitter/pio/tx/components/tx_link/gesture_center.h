#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef void (*kk_gesture_center_cb_t)(void);

void kk_gesture_center_init(kk_gesture_center_cb_t on_fire);
void kk_gesture_center_poll(float roll_deg, float pitch_deg, float yaw_deg,
                            float gyro_roll_dps, float gyro_yaw_dps, uint32_t now_ms);
void kk_gesture_center_suppress(uint32_t now_ms);
bool kk_gesture_center_is_active(void);
bool kk_gesture_center_in_progress(void);
