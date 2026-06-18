#include "imu_decouple.h"

#include "kk/link_config.h"

#include <math.h>

/*
 * L2 TX 轴解耦 — 主导轴全速；干扰轴按 UI strength 抑制。
 *
 * 网页 decouple_str：次轴最多削弱比例（越大抑制越强），两轴共用。
 * 俯仰→偏航物理耦合更强 → pitch 主导时 yaw 抑制目标可顶满 1.0；
 * 偏航→俯仰较弱 → pitch 抑制目标 × KK_DEC_YAW_TO_PITCH_SCALE。
 *
 * 关键：pitch 主导时禁止 fake yaw_intent 把 yaw_gain 放行到 1.0。
 */

static float kk_dec_wrap_delta(float prev, float cur)
{
    float d = cur - prev;
    while (d > 180.0f) {
        d -= 360.0f;
    }
    while (d < -180.0f) {
        d += 360.0f;
    }
    return d;
}

static float kk_dec_smooth01(float x, float lo, float hi)
{
    if (x <= lo) {
        return 0.0f;
    }
    if (x >= hi) {
        return 1.0f;
    }
    const float t = (x - lo) / (hi - lo);
    return t * t * (3.0f - 2.0f * t);
}

static float kk_dec_ema_asym(float prev, float target, float attack, float release)
{
    const float k = target > prev ? attack : release;
    return prev + (target - prev) * k;
}

static bool kk_dec_axis_intent(float gyro_dps, float d_deg)
{
    const float g = fabsf(gyro_dps);
    if (g >= KK_DEC_INTENT_GYRO_STRONG) {
        return true;
    }
    if (g < KK_DEC_INTENT_GYRO_MIN) {
        return false;
    }
    if (fabsf(d_deg) < KK_DEC_INTENT_DEG_MIN) {
        return false;
    }
    return (d_deg * gyro_dps) > 0.0f;
}

/** 须陀螺主导该轴，避免欧拉耦合误判 */
static bool kk_dec_pitch_dominant(float ax, float az, float pitch_share, float dom_start,
                                  bool pitch_intent)
{
    if (pitch_intent) {
        return true;
    }
    if (pitch_share < dom_start) {
        return false;
    }
    return ax >= az * KK_DEC_PITCH_DOM_GYRO_RATIO;
}

static bool kk_dec_yaw_dominant(float ax, float az, float yaw_share, float dom_start,
                                bool yaw_intent)
{
    if (!yaw_intent) {
        return false;
    }
    if (yaw_share < dom_start) {
        return false;
    }
    return az >= ax * KK_DEC_YAW_DOM_GYRO_RATIO;
}

static float kk_dec_axis_gain(float sup_f, float strength, float min_gain)
{
    float g = 1.0f - strength * sup_f;
    if (g < min_gain) {
        g = min_gain;
    }
    if (g > 1.0f) {
        g = 1.0f;
    }
    return g;
}

void kk_imu_decouple_reset(kk_imu_decouple_t *st, float yaw_deg, float pitch_deg)
{
    if (!st) {
        return;
    }
    st->out_yaw = yaw_deg;
    st->out_pitch = pitch_deg;
    st->prev_yaw = yaw_deg;
    st->prev_pitch = pitch_deg;
    st->prev_roll = 0.0f;
    st->sup_yaw_f = 0.0f;
    st->sup_pitch_f = 0.0f;
    st->seeded = false;
}

void kk_imu_decouple_apply(kk_imu_decouple_t *st, float yaw_raw, float pitch_raw,
                           float roll_raw, float gyro_pitch_dps, float gyro_yaw_dps,
                           float gyro_roll_dps, const kk_tx_track_cfg_t *cfg, float *yaw_out,
                           float *pitch_out)
{
    if (!st || !yaw_out || !pitch_out) {
        return;
    }

#if !KK_TX_GYRO_DECOUPLE
    (void)cfg;
    (void)gyro_pitch_dps;
    (void)gyro_yaw_dps;
    (void)gyro_roll_dps;
    (void)roll_raw;
    *yaw_out = yaw_raw;
    *pitch_out = pitch_raw;
    return;
#endif

    if (!cfg || !cfg->decouple_en) {
        *yaw_out = yaw_raw;
        *pitch_out = pitch_raw;
        st->out_yaw = yaw_raw;
        st->out_pitch = pitch_raw;
        st->prev_yaw = yaw_raw;
        st->prev_pitch = pitch_raw;
        st->prev_roll = roll_raw;
        st->sup_yaw_f = 0.0f;
        st->sup_pitch_f = 0.0f;
        st->seeded = true;
        return;
    }

    const float strength = (float)cfg->decouple_str_x100 / 100.0f;
    const float dom_start = (float)cfg->decouple_dom_x10 / 100.0f;
    float dom_full = dom_start + KK_DEC_DOM_SPAN;
    if (dom_full > 0.90f) {
        dom_full = 0.90f;
    }

    if (!st->seeded) {
        st->out_yaw = yaw_raw;
        st->out_pitch = pitch_raw;
        st->prev_yaw = yaw_raw;
        st->prev_pitch = pitch_raw;
        st->prev_roll = roll_raw;
        st->sup_yaw_f = 0.0f;
        st->sup_pitch_f = 0.0f;
        st->seeded = true;
        *yaw_out = yaw_raw;
        *pitch_out = pitch_raw;
        return;
    }

    /* 侧戴/垂直：欧拉奇异区 — 旁路解耦，避免 sup≈1 把输出锁死在限位 */
    if (fabsf(roll_raw) >= KK_IMU_GIMBAL_ROLL_DEG) {
        st->sup_yaw_f = 0.0f;
        st->sup_pitch_f = 0.0f;
        st->out_yaw = yaw_raw;
        st->out_pitch = pitch_raw;
        st->prev_yaw = yaw_raw;
        st->prev_pitch = pitch_raw;
        st->prev_roll = roll_raw;
        *yaw_out = yaw_raw;
        *pitch_out = pitch_raw;
        return;
    }

    const float d_yaw = kk_dec_wrap_delta(st->prev_yaw, yaw_raw);
    const float d_pitch = kk_dec_wrap_delta(st->prev_pitch, pitch_raw);

    const float ax = fabsf(gyro_pitch_dps);
    const float az = fabsf(gyro_yaw_dps);
    const float sum = ax + az + 1.0f;
    const float pitch_share = ax / sum;
    const float yaw_share = az / sum;

    const bool pitch_intent = kk_dec_axis_intent(gyro_pitch_dps, d_pitch);
    const bool yaw_intent = kk_dec_axis_intent(gyro_yaw_dps, d_yaw);
    const bool pitch_dom =
        kk_dec_pitch_dominant(ax, az, pitch_share, dom_start, pitch_intent);
    const bool yaw_dom = kk_dec_yaw_dominant(ax, az, yaw_share, dom_start, yaw_intent);

    /* pitch→yaw：抑制目标随 UI str 顶满；yaw→pitch：乘较弱比例 */
    float yaw_sup_tgt = 0.0f;
    float pitch_sup_tgt = 0.0f;

    if (pitch_dom) {
        yaw_sup_tgt = kk_dec_smooth01(pitch_share, dom_start, dom_full);
        if (yaw_sup_tgt < KK_DEC_COUPLING_YAW_SUP) {
            yaw_sup_tgt = KK_DEC_COUPLING_YAW_SUP;
        }
        if (pitch_intent) {
            yaw_sup_tgt = 1.0f;
        }
        if (fabsf(roll_raw) >= KK_DEC_ROLL_PITCH_LEAK_DEG &&
            fabsf(roll_raw) < KK_DEC_ROLL_LEAK_MAX_DEG) {
            yaw_sup_tgt = 1.0f;
        }
    }

    if (yaw_dom) {
        pitch_sup_tgt = kk_dec_smooth01(yaw_share, dom_start, dom_full) * KK_DEC_YAW_TO_PITCH_SCALE;
        if (yaw_intent && pitch_sup_tgt < KK_DEC_COUPLING_PITCH_SUP) {
            pitch_sup_tgt = KK_DEC_COUPLING_PITCH_SUP;
        }
    }

    if (pitch_dom && yaw_dom) {
        if (pitch_share >= yaw_share) {
            yaw_sup_tgt = 1.0f;
            pitch_sup_tgt *= 0.5f;
        } else {
            pitch_sup_tgt = KK_DEC_COUPLING_PITCH_SUP;
            yaw_sup_tgt *= 0.5f;
        }
    }

    st->sup_yaw_f =
        kk_dec_ema_asym(st->sup_yaw_f, yaw_sup_tgt, KK_DEC_SUP_ATTACK, KK_DEC_SUP_RELEASE);
    st->sup_pitch_f =
        kk_dec_ema_asym(st->sup_pitch_f, pitch_sup_tgt, KK_DEC_SUP_ATTACK, KK_DEC_SUP_RELEASE);

    const float yaw_min = pitch_dom ? KK_DEC_YAW_MIN_GAIN_PITCH : KK_DEC_MIN_GAIN;
    float yaw_gain = kk_dec_axis_gain(st->sup_yaw_f, strength, yaw_min);
    float pitch_gain = kk_dec_axis_gain(st->sup_pitch_f, strength, KK_DEC_MIN_GAIN);

    /* 仅真主导轴放行；pitch 主导时绝不因 fake yaw_intent 放开 yaw */
    if (pitch_dom && !yaw_dom) {
        pitch_gain = 1.0f;
    } else if (pitch_intent && !yaw_dom) {
        pitch_gain = 1.0f;
    }

    if (yaw_dom && !pitch_dom) {
        yaw_gain = 1.0f;
    } else if (yaw_intent && !pitch_dom) {
        yaw_gain = 1.0f;
    }

    st->out_yaw += d_yaw * yaw_gain;
    st->out_pitch += d_pitch * pitch_gain;

    st->prev_yaw = yaw_raw;
    st->prev_pitch = pitch_raw;
    st->prev_roll = roll_raw;

    *yaw_out = st->out_yaw;
    *pitch_out = st->out_pitch;
}
