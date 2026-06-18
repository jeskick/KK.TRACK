#include "kk/head_track.h"
#include "kk/link_config.h"
#include "kk/ppm.h"
#include "kk/servo_follow.h"
#include "kk/telemetry.h"
#include "kk/time.h"

#include "esp_log.h"
#include <math.h>

static const char *TAG = "kk.ht";

typedef enum {
    HT_RUN = 0,
    HT_FS_RAMP,
    HT_FS_HOLD,
} ht_fs_state_t;

static ht_fs_state_t s_fs_state;
static uint32_t s_fs_ramp_ms;
static uint16_t s_fs_from_lr;
static uint16_t s_fs_from_ud;
static uint32_t s_fs_recover_ms;
static uint32_t s_poll_last_ms;
static uint32_t s_apply_last_ms;

static const kk_servo_follow_cfg_t s_follow_cfg = {
    .max_deg_per_s = KK_SERVO_MAX_DEG_PER_S,
    .jitter_deadband_deg = KK_SERVO_JITTER_DEADBAND_DEG,
};

kk_head_track_t g_kk_ht;

static float ht_smoothstep(float t)
{
    if (t <= 0.0f) {
        return 0.0f;
    }
    if (t >= 1.0f) {
        return 1.0f;
    }
    return t * t * (3.0f - 2.0f * t);
}

/** 按实际间隔 dt 做舵机物理上限跟随；PPM 映射在其后即时完成 */
static float ht_follow_angle(float prev, float target, float dt_s)
{
    return kk_servo_follow_angle(prev, target, dt_s, &s_follow_cfg);
}

static float ht_follow_dt_s(uint32_t now_ms)
{
    float dt_s = KK_SERVO_FOLLOW_NOMINAL_DT_S;
    if (s_apply_last_ms > 0 && now_ms >= s_apply_last_ms) {
        const uint32_t elapsed_ms = now_ms - s_apply_last_ms;
        if (elapsed_ms > 0 && elapsed_ms <= 200U) {
            dt_s = (float)elapsed_ms / 1000.0f;
        }
    }
    s_apply_last_ms = now_ms;
    return dt_s;
}

static void ht_offset_center_ppm(const kk_rx_profile_t *cfg, uint16_t *lr, uint16_t *ud)
{
    *lr = kk_rx_yaw_angle_to_us(0.0f, cfg->offset_lr, cfg->yaw_servo_deg, cfg->scale_lr);
    *ud = kk_rx_angle_to_us(0.0f, cfg->offset_ud, cfg->scale_ud);
}

static void ht_write_ppm(const kk_rx_profile_t *cfg, uint16_t lr, uint16_t ud)
{
    g_kk_ht.ppm_lr = lr;
    g_kk_ht.ppm_ud = ud;
    kk_ppm_set_channel(kk_rx_ch_to_index(cfg->ch_lr), lr);
    kk_ppm_set_channel(kk_rx_ch_to_index(cfg->ch_ud), ud);
    kk_ppm_commit();
}

static bool ht_link_ok(bool ble_connected, uint32_t now_ms)
{
    if (!ble_connected) {
        return false;
    }
    if (g_kk_tel.last_pkt_ms == 0) {
        return false;
    }
    return (now_ms - g_kk_tel.last_pkt_ms) < KK_RX_FS_TEL_LOST_MS;
}

static uint32_t ht_pkt_age_ms(uint32_t now_ms)
{
    if (g_kk_tel.last_pkt_ms == 0) {
        return UINT32_MAX;
    }
    return now_ms - g_kk_tel.last_pkt_ms;
}

static void ht_hold_last_ppm(const kk_rx_profile_t *cfg)
{
    float yaw = cfg->rev_lr ? -g_kk_ht.yaw_f : g_kk_ht.yaw_f;
    float pitch = cfg->rev_ud ? -g_kk_ht.pitch_f : g_kk_ht.pitch_f;
    g_kk_ht.ppm_lr = kk_rx_yaw_angle_to_us(yaw, cfg->offset_lr, cfg->yaw_servo_deg, cfg->scale_lr);
    g_kk_ht.ppm_ud = kk_rx_angle_to_us(pitch, cfg->offset_ud, cfg->scale_ud);
    kk_ppm_set_channel(kk_rx_ch_to_index(cfg->ch_lr), g_kk_ht.ppm_lr);
    kk_ppm_set_channel(kk_rx_ch_to_index(cfg->ch_ud), g_kk_ht.ppm_ud);
    kk_ppm_commit();
}

static void ht_enter_failsafe(const kk_rx_profile_t *cfg)
{
    s_fs_from_lr = g_kk_ht.ppm_lr;
    s_fs_from_ud = g_kk_ht.ppm_ud;
    s_fs_ramp_ms = 0;
    s_fs_recover_ms = 0;
    s_fs_state = HT_FS_RAMP;
    ESP_LOGW(TAG, "link lost -> offset center ramp from lr=%u ud=%u", (unsigned)s_fs_from_lr,
             (unsigned)s_fs_from_ud);
}

static void ht_resume_track(void)
{
    g_kk_ht.yaw_f = g_kk_tel.yaw_deg;
    g_kk_ht.pitch_f = g_kk_tel.pitch_deg;
    s_fs_state = HT_RUN;
    s_fs_ramp_ms = 0;
    s_fs_recover_ms = 0;
    s_apply_last_ms = 0;
    ESP_LOGW(TAG, "link stable -> resume track");
}

static void ht_output_offset_center(const kk_rx_profile_t *cfg)
{
    g_kk_ht.yaw_f = 0.0f;
    g_kk_ht.pitch_f = 0.0f;
    uint16_t lr = 0;
    uint16_t ud = 0;
    ht_offset_center_ppm(cfg, &lr, &ud);
    ht_write_ppm(cfg, lr, ud);
}

static void ht_failsafe_step(const kk_rx_profile_t *cfg, uint32_t elapsed_ms, bool link_ok)
{
    uint16_t lr_tgt = 0;
    uint16_t ud_tgt = 0;
    ht_offset_center_ppm(cfg, &lr_tgt, &ud_tgt);

    if (s_fs_state == HT_FS_RAMP) {
        s_fs_ramp_ms += elapsed_ms;
        const float t = (float)s_fs_ramp_ms / (float)KK_RX_FS_RAMP_MS;
        const float blend = ht_smoothstep(t);
        const uint16_t lr =
            (uint16_t)((float)s_fs_from_lr + (float)((int)lr_tgt - (int)s_fs_from_lr) * blend);
        const uint16_t ud =
            (uint16_t)((float)s_fs_from_ud + (float)((int)ud_tgt - (int)s_fs_from_ud) * blend);
        g_kk_ht.yaw_f = 0.0f;
        g_kk_ht.pitch_f = 0.0f;
        ht_write_ppm(cfg, lr, ud);
        if (blend >= 1.0f) {
            s_fs_state = HT_FS_HOLD;
        }
    } else {
        ht_output_offset_center(cfg);
    }

    if (link_ok) {
        if (s_fs_state == HT_FS_HOLD) {
            s_fs_recover_ms += elapsed_ms;
            if (s_fs_recover_ms >= KK_RX_FS_RECOVER_MS) {
                ht_resume_track();
            }
        }
    } else {
        s_fs_recover_ms = 0;
    }
}

void kk_head_track_snap_telemetry(void)
{
    if (isfinite(g_kk_tel.yaw_deg)) {
        g_kk_ht.yaw_f = g_kk_tel.yaw_deg;
    }
    if (isfinite(g_kk_tel.pitch_deg)) {
        g_kk_ht.pitch_f = g_kk_tel.pitch_deg;
    }
}

void kk_head_track_reset(void)
{
    g_kk_ht.yaw_f = 0.0f;
    g_kk_ht.pitch_f = 0.0f;
    g_kk_ht.ppm_lr = KK_RX_PPM_CENTER;
    g_kk_ht.ppm_ud = KK_RX_PPM_CENTER;
    s_apply_last_ms = 0;
}

void kk_head_track_failsafe_reset(void)
{
    s_fs_state = HT_RUN;
    s_fs_ramp_ms = 0;
    s_fs_recover_ms = 0;
    s_poll_last_ms = 0;
    s_apply_last_ms = 0;
}

bool kk_head_track_failsafe_active(void)
{
    return s_fs_state != HT_RUN;
}

void kk_head_track_center(const kk_rx_profile_t *cfg)
{
    kk_head_track_reset();
    if (!cfg) {
        return;
    }
    const kk_yaw_servo_t yaw_srv = kk_rx_yaw_servo_params(cfg->yaw_servo_deg);
    kk_ppm_fill_center();
    kk_ppm_set_channel(kk_rx_ch_to_index(cfg->ch_lr), yaw_srv.center_us);
    kk_ppm_set_channel(kk_rx_ch_to_index(cfg->ch_ud), KK_RX_PPM_CENTER);
    kk_ppm_commit();
}

void kk_head_track_offset_center(const kk_rx_profile_t *cfg)
{
    if (!cfg) {
        return;
    }
    ht_output_offset_center(cfg);
}

void kk_head_track_apply(const kk_rx_profile_t *cfg)
{
    if (!cfg) {
        return;
    }

    const float dt_s = ht_follow_dt_s(kk_millis());
    g_kk_ht.yaw_f = ht_follow_angle(g_kk_ht.yaw_f, g_kk_tel.yaw_deg, dt_s);
    g_kk_ht.pitch_f = ht_follow_angle(g_kk_ht.pitch_f, g_kk_tel.pitch_deg, dt_s);

    float yaw = cfg->rev_lr ? -g_kk_ht.yaw_f : g_kk_ht.yaw_f;
    float pitch = cfg->rev_ud ? -g_kk_ht.pitch_f : g_kk_ht.pitch_f;
    g_kk_ht.ppm_lr = kk_rx_yaw_angle_to_us(yaw, cfg->offset_lr, cfg->yaw_servo_deg, cfg->scale_lr);
    g_kk_ht.ppm_ud = kk_rx_angle_to_us(pitch, cfg->offset_ud, cfg->scale_ud);

    kk_ppm_set_channel(kk_rx_ch_to_index(cfg->ch_lr), g_kk_ht.ppm_lr);
    kk_ppm_set_channel(kk_rx_ch_to_index(cfg->ch_ud), g_kk_ht.ppm_ud);
    kk_ppm_commit();
}

void kk_head_track_poll(const kk_rx_profile_t *cfg, bool ble_connected, bool ppm_active,
                        uint32_t now_ms)
{
    if (!cfg || !ppm_active) {
        return;
    }

    uint32_t elapsed_ms = 20;
    if (s_poll_last_ms > 0 && now_ms >= s_poll_last_ms) {
        elapsed_ms = now_ms - s_poll_last_ms;
        if (elapsed_ms == 0) {
            elapsed_ms = 1;
        }
    }
    s_poll_last_ms = now_ms;

    const bool link_ok = ht_link_ok(ble_connected, now_ms);
    const uint32_t pkt_age = ht_pkt_age_ms(now_ms);
    const bool tel_fresh = g_kk_tel.last_pkt_ms > 0 && pkt_age < KK_RX_FS_STALE_HOLD_MS;

    if (s_fs_state != HT_RUN) {
        if (ble_connected && tel_fresh) {
            ht_resume_track();
            kk_head_track_apply(cfg);
            return;
        }
        ht_failsafe_step(cfg, elapsed_ms, link_ok);
        return;
    }

    if (!ble_connected || (g_kk_tel.last_pkt_ms > 0 && pkt_age >= KK_RX_FS_TEL_LOST_MS)) {
        ht_enter_failsafe(cfg);
        ht_failsafe_step(cfg, elapsed_ms, link_ok);
        return;
    }
    if (pkt_age >= KK_RX_FS_STALE_HOLD_MS) {
        ht_hold_last_ppm(cfg);
        return;
    }
    kk_head_track_apply(cfg);
}
