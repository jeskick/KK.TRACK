#include "imu_decouple.h"

#include "kk/link_config.h"

#include <math.h>

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

static float kk_dec_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static float kk_dec_ema_asym(float prev, float target, float attack, float release)
{
    const float k = target > prev ? attack : release;
    return prev + (target - prev) * k;
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

    const float d_yaw = kk_dec_wrap_delta(st->prev_yaw, yaw_raw);
    const float d_pitch = kk_dec_wrap_delta(st->prev_pitch, pitch_raw);
    const float d_roll = kk_dec_wrap_delta(st->prev_roll, roll_raw);

    const float ax = fabsf(gyro_pitch_dps);
    const float az = fabsf(gyro_yaw_dps);
    const float ar = fabsf(gyro_roll_dps);
    const float sum = ax + az + 1.0f;
    const float yaw_share = az / sum;
    const float pitch_share = ax / sum;

    const float yaw_sup_tgt = kk_dec_smooth01(pitch_share, dom_start, dom_full);
    const float pitch_sup_tgt = kk_dec_smooth01(yaw_share, dom_start, dom_full);

    st->sup_yaw_f =
        kk_dec_ema_asym(st->sup_yaw_f, yaw_sup_tgt, KK_DEC_SUP_ATTACK, KK_DEC_SUP_RELEASE);
    st->sup_pitch_f =
        kk_dec_ema_asym(st->sup_pitch_f, pitch_sup_tgt, KK_DEC_SUP_ATTACK, KK_DEC_SUP_RELEASE);

    float yaw_gain = 1.0f - strength * st->sup_yaw_f;
    float pitch_gain = 1.0f - strength * st->sup_pitch_f;

    if (yaw_gain < KK_DEC_MIN_GAIN) {
        yaw_gain = KK_DEC_MIN_GAIN;
    }
    if (pitch_gain < KK_DEC_MIN_GAIN) {
        pitch_gain = KK_DEC_MIN_GAIN;
    }

    float relax = 0.0f;
    if (fabsf(roll_raw) >= KK_TX_GYRO_DECOUPLE_ROLL_RELAX) {
        relax = kk_dec_clampf(relax + 0.28f, 0.0f, 1.0f);
    }
    if (fabsf(d_roll) >= KK_TX_GYRO_DECOUPLE_DR_RELAX) {
        relax = kk_dec_clampf(relax + 0.28f, 0.0f, 1.0f);
    }
    if (ar >= KK_TX_GYRO_DECOUPLE_ROLL_RELAX) {
        relax = kk_dec_clampf(relax + 0.22f, 0.0f, 1.0f);
    }
    if (ax >= KK_TX_GYRO_DECOUPLE_BOTH_GYRO && az >= KK_TX_GYRO_DECOUPLE_BOTH_GYRO) {
        relax = 1.0f;
    }
    yaw_gain += (1.0f - yaw_gain) * relax;
    pitch_gain += (1.0f - pitch_gain) * relax;

    st->out_yaw += d_yaw * yaw_gain;
    st->out_pitch += d_pitch * pitch_gain;

    st->prev_yaw = yaw_raw;
    st->prev_pitch = pitch_raw;
    st->prev_roll = roll_raw;

    *yaw_out = st->out_yaw;
    *pitch_out = st->out_pitch;
}
