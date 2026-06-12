#include "kk/head_track.h"
#include "kk/ppm.h"
#include "kk/telemetry.h"

#include <math.h>

kk_head_track_t g_kk_ht;

void kk_head_track_reset(void)
{
    g_kk_ht.yaw_f = 0.0f;
    g_kk_ht.pitch_f = 0.0f;
    g_kk_ht.ppm_lr = KK_RX_PPM_CENTER;
    g_kk_ht.ppm_ud = KK_RX_PPM_CENTER;
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

static float kk_head_jitter_step(float prev, float raw, float jitter_deg)
{
    if (jitter_deg <= 0.0f) {
        return raw;
    }
    if (fabsf(raw - prev) < jitter_deg) {
        return prev;
    }
    return raw;
}

void kk_head_track_apply(const kk_rx_profile_t *cfg)
{
    if (!cfg) {
        return;
    }
    const float jitter = (float)cfg->jitter_x10 / 10.0f;
    g_kk_ht.yaw_f = kk_head_jitter_step(g_kk_ht.yaw_f, g_kk_tel.yaw_deg, jitter);
    g_kk_ht.pitch_f = kk_head_jitter_step(g_kk_ht.pitch_f, g_kk_tel.pitch_deg, jitter);

    float yaw = cfg->rev_lr ? -g_kk_ht.yaw_f : g_kk_ht.yaw_f;
    float pitch = cfg->rev_ud ? -g_kk_ht.pitch_f : g_kk_ht.pitch_f;
    g_kk_ht.ppm_lr = kk_rx_yaw_angle_to_us(yaw, cfg->offset_lr, cfg->yaw_servo_deg, cfg->scale_lr);
    g_kk_ht.ppm_ud = kk_rx_angle_to_us(pitch, cfg->offset_ud, false, KK_RX_SCALE_NEUT);

    kk_ppm_set_channel(kk_rx_ch_to_index(cfg->ch_lr), g_kk_ht.ppm_lr);
    kk_ppm_set_channel(kk_rx_ch_to_index(cfg->ch_ud), g_kk_ht.ppm_ud);
    kk_ppm_commit();
}
