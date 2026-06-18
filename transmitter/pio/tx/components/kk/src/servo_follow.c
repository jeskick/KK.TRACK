#include "kk/servo_follow.h"
#include "kk/link_config.h"

#include <math.h>

kk_servo_follow_cfg_t kk_servo_follow_default_cfg(void)
{
    kk_servo_follow_cfg_t cfg = {
        .max_deg_per_s = KK_SERVO_MAX_DEG_PER_S,
        .jitter_deadband_deg = KK_SERVO_JITTER_DEADBAND_DEG,
    };
    return cfg;
}

float kk_servo_follow_angle(float prev_deg, float target_deg, float dt_s,
                            const kk_servo_follow_cfg_t *cfg)
{
    if (!cfg) {
        return prev_deg;
    }
    if (!isfinite(target_deg)) {
        return prev_deg;
    }
    if (!isfinite(prev_deg)) {
        return target_deg;
    }
    if (dt_s <= 0.0f) {
        return prev_deg;
    }

    float err = target_deg - prev_deg;
    if (fabsf(err) < cfg->jitter_deadband_deg) {
        return prev_deg;
    }

    const float max_step = cfg->max_deg_per_s * dt_s;
    if (err > max_step) {
        return prev_deg + max_step;
    }
    if (err < -max_step) {
        return prev_deg - max_step;
    }
    return target_deg;
}
