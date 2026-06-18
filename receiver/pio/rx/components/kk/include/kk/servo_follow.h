#pragma once

#include <stdbool.h>

/**
 * RX 角度跟随器：IMU 目标角 → 舵机可执行的输出角。
 *
 * 舵机物理最大角速度是硬上限；慢速目标变化时一步到位（与舵机同步）；
 * 快速目标变化时以 max_deg_per_s 斜率跟随；小误差死区抑制抖动。
 * 不在 PPM 层再做额外限速。
 */
typedef struct {
    float max_deg_per_s;
    float jitter_deadband_deg;
} kk_servo_follow_cfg_t;

kk_servo_follow_cfg_t kk_servo_follow_default_cfg(void);

/** @param dt_s 距上次调用间隔（秒）；<=0 时保持 prev */
float kk_servo_follow_angle(float prev_deg, float target_deg, float dt_s,
                            const kk_servo_follow_cfg_t *cfg);
