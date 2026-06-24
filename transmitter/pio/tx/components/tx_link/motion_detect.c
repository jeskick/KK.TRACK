#include "motion_detect.h"
#include "gesture_center.h"

#include "kk/link_config.h"

#include "esp_log.h"
#include <math.h>

static const char *TAG = "kk.motion";

typedef enum {
    KK_MOB_RUN = 0,
    KK_MOB_PAUSE = 1,
} kk_mob_state_t;

static kk_motion_rezero_cb_t s_on_rezero;
static bool s_enabled;
static kk_mob_state_t s_state;
static float s_ramp_from_yaw;
static float s_ramp_from_pitch;
static uint32_t s_ramp_ms;
static uint32_t s_trigger_ms;
static uint32_t s_settle_ms;
static float s_lin_ema;
static float s_lin_peak;
static float s_grav_base[3];
static bool s_grav_base_ok;
static kk_motion_dbg_t s_dbg;

static float kk_mob_smoothstep(float t)
{
    if (t <= 0.0f) {
        return 0.0f;
    }
    if (t >= 1.0f) {
        return 1.0f;
    }
    return t * t * (3.0f - 2.0f * t);
}

static float kk_mob_lin_metric(float lin_accel_mps2)
{
    const float a = KK_MOB_LIN_EMA_NEW;
    s_lin_ema = s_lin_ema * (1.0f - a) + lin_accel_mps2 * a;
    if (lin_accel_mps2 > s_lin_peak) {
        s_lin_peak = lin_accel_mps2;
    } else {
        s_lin_peak *= KK_MOB_LIN_PEAK_DECAY;
    }
    if (s_lin_ema > s_lin_peak) {
        s_lin_peak = s_lin_ema;
    }
    return s_lin_ema > s_lin_peak ? s_lin_ema : s_lin_peak;
}

static bool kk_mob_head_active(float g_pitch, float g_yaw)
{
    const float ap = fabsf(g_pitch);
    const float az = fabsf(g_yaw);
    if (ap >= KK_MOB_HEAD_ACTIVE_DPS || az >= KK_MOB_HEAD_ACTIVE_DPS) {
        return true;
    }
    return sqrtf(ap * ap + az * az) >= KK_MOB_HEAD_PAIR_DPS;
}

static bool kk_mob_head_only(float g_pitch, float g_yaw, float total_dps)
{
    if (total_dps < 18.0f) {
        return false;
    }
    const float ax = fabsf(g_pitch);
    const float az = fabsf(g_yaw);
    const float sum = ax + az + 1.0f;
    const float yaw_share = az / sum;
    const float pitch_share = ax / sum;
    return yaw_share >= KK_MOB_GYRO_HEAD_DOM_SHARE || pitch_share >= KK_MOB_GYRO_HEAD_DOM_SHARE;
}

static bool kk_mob_head_nod(float g_pitch, float g_yaw, float lin_metric)
{
    const float ap = fabsf(g_pitch);
    const float az = fabsf(g_yaw);
    if (ap < 22.0f) {
        return false;
    }
    if (az > ap * 0.55f) {
        return false;
    }
    return lin_metric < KK_MOB_LIN_ACCEL_MPS2 + 0.8f;
}

/*
 * IMU 不在颈转中心：快速转头/点头会产生向心假线加速度 a≈r·ω²。
 * 用主导头轴角速率估算并扣除，剩余 lin_trans 才用于走路/位移判定。
 */
static float kk_mob_lin_trans(float g_pitch, float g_yaw, float lin_metric)
{
    const float head_omega = sqrtf(g_pitch * g_pitch + g_yaw * g_yaw);
    const float scaled = head_omega * 0.01f;
    const float rot_art = KK_MOB_ROT_LIN_K * scaled * scaled;
    float trans = lin_metric - rot_art;
    return trans > 0.0f ? trans : 0.0f;
}

static float kk_mob_vec_len3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static float kk_mob_grav_tilt_deg(const float grav_logic[3])
{
    if (!grav_logic) {
        return 0.0f;
    }
    const float glen = kk_mob_vec_len3(grav_logic);
    const float blen = kk_mob_vec_len3(s_grav_base);
    if (glen < 0.5f || blen < 0.5f) {
        return 0.0f;
    }
    float dot = (grav_logic[0] * s_grav_base[0] + grav_logic[1] * s_grav_base[1] +
                   grav_logic[2] * s_grav_base[2]) /
                  (glen * blen);
    if (dot > 1.0f) {
        dot = 1.0f;
    }
    if (dot < -1.0f) {
        dot = -1.0f;
    }
    return acosf(dot) * 57.2957795f;
}

static void kk_mob_grav_base_update(const float grav_logic[3], float lin_trans, float g_pitch,
                                    float g_yaw, float total_dps)
{
    const float horiz = sqrtf(g_pitch * g_pitch + g_yaw * g_yaw);
    if (horiz >= KK_MOB_GRAV_BASE_FREEZE_DPS) {
        return;
    }
    if (!grav_logic || lin_trans >= KK_MOB_LIN_ACCEL_SOFT_MPS2 || total_dps >= 25.0f) {
        return;
    }
    const float glen = kk_mob_vec_len3(grav_logic);
    if (glen < 0.5f) {
        return;
    }
    const float inv = 1.0f / glen;
    const float nx = grav_logic[0] * inv;
    const float ny = grav_logic[1] * inv;
    const float nz = grav_logic[2] * inv;
    if (!s_grav_base_ok) {
        s_grav_base[0] = nx;
        s_grav_base[1] = ny;
        s_grav_base[2] = nz;
        s_grav_base_ok = true;
        return;
    }
    const float a = KK_MOB_GRAV_BASE_EMA;
    s_grav_base[0] = s_grav_base[0] * (1.0f - a) + nx * a;
    s_grav_base[1] = s_grav_base[1] * (1.0f - a) + ny * a;
    s_grav_base[2] = s_grav_base[2] * (1.0f - a) + nz * a;
}

static bool kk_mob_body_motion(float g_pitch, float g_yaw, float g_roll, int8_t stability,
                               float lin_metric, const float grav_logic[3], float *lin_trans_out)
{
    const float ax = fabsf(g_pitch);
    const float az = fabsf(g_yaw);
    const float ar = fabsf(g_roll);
    const float total = sqrtf(ax * ax + az * az + ar * ar);
    const float horiz = sqrtf(ax * ax + az * az);
    const float lin_trans = kk_mob_lin_trans(g_pitch, g_yaw, lin_metric);
    const float grav_tilt = kk_mob_grav_tilt_deg(grav_logic);
    const bool head_active = kk_mob_head_active(g_pitch, g_yaw);

    if (lin_trans_out) {
        *lin_trans_out = lin_trans;
    }

    if (head_active) {
        if (lin_trans >= KK_MOB_WALK_LT_MIN) {
            return true;
        }
        if (grav_tilt >= KK_MOB_GRAV_TILT_DEG && lin_trans >= KK_MOB_POSTURE_LT_MIN &&
            horiz < KK_MOB_GRAV_MAX_HEAD_DPS) {
            return true;
        }
        return false;
    }

    if (kk_mob_head_nod(g_pitch, g_yaw, lin_metric) && lin_trans < KK_MOB_LIN_ACCEL_SOFT_MPS2) {
        return false;
    }

    if (grav_tilt >= KK_MOB_GRAV_TILT_DEG && lin_trans >= 0.8f) {
        return true;
    }

    if (lin_trans >= KK_MOB_LIN_ACCEL_MPS2) {
        return true;
    }

    if (stability == 4 && lin_trans >= KK_MOB_LIN_ACCEL_SOFT_MPS2) {
        return true;
    }

    const float roll_share = ar / (total + 1.0f);
    if (ar >= KK_MOB_GYRO_BODY_ROLL_DPS && total >= KK_MOB_GYRO_BODY_TOTAL_DPS &&
        roll_share >= KK_MOB_GYRO_BODY_ROLL_SHARE && lin_trans >= KK_MOB_LIN_ACCEL_SOFT_MPS2) {
        return true;
    }

    return false;
}

static bool kk_mob_is_settled(float g_pitch, float g_yaw, float g_roll, int8_t stability,
                              float lin_metric)
{
    if (stability == 3 || stability == 2) {
        return lin_metric < KK_MOB_LIN_ACCEL_SOFT_MPS2;
    }
    return fabsf(g_pitch) < KK_MOB_SETTLE_GYRO_DPS && fabsf(g_yaw) < KK_MOB_SETTLE_GYRO_DPS &&
           fabsf(g_roll) < KK_MOB_SETTLE_GYRO_DPS && lin_metric < KK_MOB_LIN_ACCEL_SOFT_MPS2;
}

static void kk_mob_trigger_decay(uint32_t elapsed_ms)
{
    if (s_trigger_ms <= elapsed_ms) {
        s_trigger_ms = 0;
        return;
    }
    s_trigger_ms -= elapsed_ms / KK_MOB_TRIGGER_DECAY_DIV;
}

static void kk_mob_settle_decay(uint32_t elapsed_ms)
{
    if (s_settle_ms <= elapsed_ms) {
        s_settle_ms = 0;
        return;
    }
    s_settle_ms -= elapsed_ms / KK_MOB_SETTLE_DECAY_DIV;
}

static void kk_mob_center_output(uint32_t elapsed_ms, float *out_yaw, float *out_pitch)
{
    s_ramp_ms += elapsed_ms;
    float t = (float)s_ramp_ms / (float)KK_MOB_CENTER_RAMP_MS;
    const float blend = kk_mob_smoothstep(t);
    *out_yaw = s_ramp_from_yaw * (1.0f - blend);
    *out_pitch = s_ramp_from_pitch * (1.0f - blend);
}

void kk_motion_detect_init(kk_motion_rezero_cb_t on_rezero)
{
    s_on_rezero = on_rezero;
    s_enabled = false;
    kk_motion_detect_reset();
}

void kk_motion_detect_reset(void)
{
    s_state = KK_MOB_RUN;
    s_ramp_from_yaw = 0.0f;
    s_ramp_from_pitch = 0.0f;
    s_ramp_ms = 0;
    s_trigger_ms = 0;
    s_settle_ms = 0;
    s_lin_ema = 0.0f;
    s_lin_peak = 0.0f;
    s_grav_base[0] = 0.0f;
    s_grav_base[1] = 0.0f;
    s_grav_base[2] = -1.0f;
    s_grav_base_ok = false;
}

void kk_motion_detect_set_enabled(bool enabled)
{
    s_enabled = enabled;
    if (!enabled) {
        kk_motion_detect_reset();
    } else {
        ESP_LOGW(TAG, "enabled");
    }
}

bool kk_motion_detect_is_enabled(void)
{
    return s_enabled;
}

bool kk_motion_detect_is_paused(void)
{
    return s_enabled && s_state == KK_MOB_PAUSE;
}

uint32_t kk_motion_detect_trigger_ms(void)
{
    return s_trigger_ms;
}

void kk_motion_detect_get_dbg(kk_motion_dbg_t *out)
{
    if (out) {
        *out = s_dbg;
    }
}

static void kk_mob_fill_dbg(float g_pitch, float g_yaw, float g_roll, int8_t stability,
                            float lin_metric, const float grav_logic[3], bool body)
{
    const float ax = fabsf(g_pitch);
    const float az = fabsf(g_yaw);
    const float ar = fabsf(g_roll);
    const float total = sqrtf(ax * ax + az * az + ar * ar);
    const float lin_trans = kk_mob_lin_trans(g_pitch, g_yaw, lin_metric);
    s_dbg.lin_metric = lin_metric;
    s_dbg.lin_trans = lin_trans;
    s_dbg.grav_tilt = kk_mob_grav_tilt_deg(grav_logic);
    s_dbg.g_pitch = g_pitch;
    s_dbg.g_yaw = g_yaw;
    s_dbg.g_roll = g_roll;
    s_dbg.stability = stability;
    s_dbg.head_only = kk_mob_head_only(g_pitch, g_yaw, total);
    s_dbg.head_nod = kk_mob_head_nod(g_pitch, g_yaw, lin_metric);
    s_dbg.head_active = kk_mob_head_active(g_pitch, g_yaw);
    s_dbg.body_motion = body;
    s_dbg.state = (uint8_t)s_state;
    s_dbg.trigger_ms = s_trigger_ms;
    s_dbg.settle_ms = s_settle_ms;
}

void kk_motion_detect_apply(float in_yaw, float in_pitch, float gyro_pitch_dps,
                            float gyro_yaw_dps, float gyro_roll_dps,
                            int8_t stability_class, float lin_accel_mps2,
                            const float grav_logic[3], uint32_t elapsed_ms,
                            float *out_yaw, float *out_pitch)
{
    if (!out_yaw || !out_pitch) {
        return;
    }

    if (!s_enabled) {
        *out_yaw = in_yaw;
        *out_pitch = in_pitch;
        return;
    }

    const float lin_metric = kk_mob_lin_metric(lin_accel_mps2);
    const float ax = fabsf(gyro_pitch_dps);
    const float az = fabsf(gyro_yaw_dps);
    const float ar = fabsf(gyro_roll_dps);
    const float total_dps = sqrtf(ax * ax + az * az + ar * ar);
    float lin_trans = 0.0f;

    if (s_state == KK_MOB_RUN) {
        kk_mob_grav_base_update(grav_logic,
                                kk_mob_lin_trans(gyro_pitch_dps, gyro_yaw_dps, lin_metric),
                                gyro_pitch_dps, gyro_yaw_dps, total_dps);
    }

    if (s_state == KK_MOB_PAUSE) {
        kk_mob_center_output(elapsed_ms, out_yaw, out_pitch);

        if (kk_mob_is_settled(gyro_pitch_dps, gyro_yaw_dps, gyro_roll_dps, stability_class,
                              lin_metric)) {
            s_settle_ms += elapsed_ms;
        } else {
            kk_mob_settle_decay(elapsed_ms);
        }

        kk_mob_fill_dbg(gyro_pitch_dps, gyro_yaw_dps, gyro_roll_dps, stability_class, lin_metric,
                        grav_logic, false);

        if (s_settle_ms >= KK_MOB_SETTLE_MS) {
            ESP_LOGW(TAG, "settled -> re-zero resume");
            s_settle_ms = 0;
            s_state = KK_MOB_RUN;
            s_ramp_ms = 0;
            s_ramp_from_yaw = 0.0f;
            s_ramp_from_pitch = 0.0f;
            s_lin_ema = 0.0f;
            s_lin_peak = 0.0f;
            s_grav_base_ok = false;
            if (s_on_rezero) {
                s_on_rezero();
            }
            *out_yaw = 0.0f;
            *out_pitch = 0.0f;
        }
        return;
    }

    *out_yaw = in_yaw;
    *out_pitch = in_pitch;

    const bool body = !kk_gesture_center_is_active() &&
                      kk_mob_body_motion(gyro_pitch_dps, gyro_yaw_dps, gyro_roll_dps,
                                         stability_class, lin_metric, grav_logic, &lin_trans);
    if (body) {
        uint32_t add = elapsed_ms;
        if (lin_trans >= KK_MOB_LIN_ACCEL_MPS2) {
            add *= KK_MOB_TRIGGER_HARD_MUL;
        }
        s_trigger_ms += add;
    } else {
        kk_mob_trigger_decay(elapsed_ms);
    }

    kk_mob_fill_dbg(gyro_pitch_dps, gyro_yaw_dps, gyro_roll_dps, stability_class, lin_metric,
                    grav_logic, body);

    const uint32_t trigger_need =
        lin_trans >= KK_MOB_LIN_ACCEL_MPS2 ? KK_MOB_TRIGGER_HARD_MS : KK_MOB_TRIGGER_MS;
    if (s_trigger_ms >= trigger_need && !kk_gesture_center_is_active()) {
        s_ramp_from_yaw = in_yaw;
        s_ramp_from_pitch = in_pitch;
        s_ramp_ms = 0;
        s_state = KK_MOB_PAUSE;
        s_trigger_ms = 0;
        s_settle_ms = 0;
        kk_mob_center_output(0, out_yaw, out_pitch);
        ESP_LOGW(TAG, "body motion -> logic 0 ramp Y=%d P=%d lt=%.1f gt=%.0f", (int)in_yaw,
                 (int)in_pitch, lin_trans, kk_mob_grav_tilt_deg(grav_logic));
    }
}
