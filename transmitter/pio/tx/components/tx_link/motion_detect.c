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

static bool kk_mob_body_motion(float g_pitch, float g_yaw, float g_roll, int8_t stability,
                               float lin_metric)
{
    const float ax = fabsf(g_pitch);
    const float az = fabsf(g_yaw);
    const float ar = fabsf(g_roll);
    const float total = sqrtf(ax * ax + az * az + ar * ar);

    if (kk_mob_head_nod(g_pitch, g_yaw, lin_metric)) {
        return false;
    }

    if (lin_metric >= KK_MOB_LIN_ACCEL_MPS2) {
        return true;
    }

    if (lin_metric < KK_MOB_LIN_ACCEL_SOFT_MPS2 && kk_mob_head_only(g_pitch, g_yaw, total)) {
        return false;
    }

    if (stability == 4 && lin_metric >= KK_MOB_LIN_ACCEL_SOFT_MPS2 &&
        !kk_mob_head_only(g_pitch, g_yaw, total)) {
        return true;
    }

    if (ar >= KK_MOB_GYRO_BODY_ROLL_DPS && total >= KK_MOB_GYRO_BODY_TOTAL_DPS &&
        lin_metric >= KK_MOB_LIN_ACCEL_SOFT_MPS2) {
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

void kk_motion_detect_apply(float in_yaw, float in_pitch, float gyro_pitch_dps,
                            float gyro_yaw_dps, float gyro_roll_dps,
                            int8_t stability_class, float lin_accel_mps2, uint32_t elapsed_ms,
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

    if (s_state == KK_MOB_PAUSE) {
        kk_mob_center_output(elapsed_ms, out_yaw, out_pitch);

        if (kk_mob_is_settled(gyro_pitch_dps, gyro_yaw_dps, gyro_roll_dps, stability_class,
                              lin_metric)) {
            s_settle_ms += elapsed_ms;
        } else {
            kk_mob_settle_decay(elapsed_ms);
        }

        if (s_settle_ms >= KK_MOB_SETTLE_MS) {
            ESP_LOGW(TAG, "settled -> re-zero resume");
            s_settle_ms = 0;
            s_state = KK_MOB_RUN;
            s_ramp_ms = 0;
            s_ramp_from_yaw = 0.0f;
            s_ramp_from_pitch = 0.0f;
            s_lin_ema = 0.0f;
            s_lin_peak = 0.0f;
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

    if (!kk_gesture_center_is_active() &&
        kk_mob_body_motion(gyro_pitch_dps, gyro_yaw_dps, gyro_roll_dps, stability_class,
                           lin_metric)) {
        s_trigger_ms += elapsed_ms;
    } else {
        kk_mob_trigger_decay(elapsed_ms);
    }

    if (s_trigger_ms >= KK_MOB_TRIGGER_MS && !kk_gesture_center_is_active()) {
        s_ramp_from_yaw = in_yaw;
        s_ramp_from_pitch = in_pitch;
        s_ramp_ms = 0;
        s_state = KK_MOB_PAUSE;
        s_trigger_ms = 0;
        s_settle_ms = 0;
        kk_mob_center_output(0, out_yaw, out_pitch);
        ESP_LOGW(TAG, "body motion -> logic 0 ramp from Y=%d P=%d lin=%.1f", (int)in_yaw,
                 (int)in_pitch, lin_metric);
    }
}
