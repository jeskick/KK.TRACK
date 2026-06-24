#include "kk/rx_profile.h"
#include "kk/rc_out.h"
#include "kk/imu_mount.h"
#include "kk/link_config.h"
#include "kk/tx_track_cfg.h"

#include "nvs.h"
#include <math.h>

kk_rx_profile_t kk_rx_profile_defaults(void)
{
    kk_rx_profile_t p = {
        .rc_proto = KK_RC_PROTO_PPM,
        .ch_lr = 6,
        .ch_ud = 7,
        .offset_lr = -280,
        .offset_ud = 0,
        .scale_lr = KK_RX_SCALE_NEUT,
        .scale_ud = KK_RX_SCALE_NEUT,
        .yaw_servo_deg = KK_RX_YAW_SERVO_270,
        .rev_lr = false,
        .rev_ud = false,
        .mount_horiz = 0,
        .mount_lr = 0,
        .mount_fb = 0,
        .gest_roll_deg = KK_GEST_ROLL_DEG_DEFAULT,
        .gest_swing_ms = KK_GEST_SWING_MS_DEFAULT,
        .gest_center_en = true,
        .track_decouple_en = true,
        .track_motion_en = false,
        .track_decouple_str_x100 = KK_TRACK_DEC_STR_DEFAULT,
        .track_decouple_dom_x10 = KK_TRACK_DEC_DOM_DEFAULT,
    };
    return p;
}

uint8_t kk_rx_ch_to_index(uint8_t ch)
{
    if (ch < KK_RX_CH_MIN || ch > KK_RX_CH_MAX) {
        return KK_RX_PPM_CH_BASE;
    }
    return (uint8_t)(ch - 1);
}

float kk_rx_scale_to_mult(uint8_t scale)
{
    if (scale > 100) {
        scale = 100;
    }
    if (scale == 0) {
        scale = 1;
    }
    /* 1→50% … 50→100% … 100→150% */
    return KK_RX_SCALE_MIN_MULT + (float)scale / 100.0f;
}

static void kk_rx_clamp_scale(uint8_t *scale)
{
    if (!scale) {
        return;
    }
    if (*scale < 1) {
        *scale = 1;
    }
    if (*scale > 100) {
        *scale = 100;
    }
}

int16_t kk_rx_clamp_offset(int16_t v)
{
    if (v < KK_RX_OFFSET_MIN) {
        return KK_RX_OFFSET_MIN;
    }
    if (v > KK_RX_OFFSET_MAX) {
        return KK_RX_OFFSET_MAX;
    }
    return v;
}

uint8_t kk_rx_clamp_ch(uint8_t ch)
{
    if (ch < KK_RX_CH_MIN) {
        return KK_RX_CH_MIN;
    }
    if (ch > KK_RX_CH_MAX) {
        return KK_RX_CH_MAX;
    }
    return ch;
}

uint16_t kk_rx_sanitize_yaw_servo_deg(uint16_t deg)
{
    return deg == KK_RX_YAW_SERVO_270 ? KK_RX_YAW_SERVO_270 : KK_RX_YAW_SERVO_180;
}

kk_yaw_servo_t kk_rx_yaw_servo_params(uint16_t yaw_servo_deg)
{
    if (yaw_servo_deg == KK_RX_YAW_SERVO_270) {
        const kk_yaw_servo_t s = {
            .center_us = KK_RX_PPM_CENTER,
            .min_us = 500,
            .max_us = 2500,
            .us_per_deg = 1000.0f / 135.0f, /* 270° 专用，仅 LR 通道 */
        };
        return s;
    }
    const kk_yaw_servo_t s = {
        .center_us = KK_RX_PPM_CENTER,
        .min_us = KK_RX_STD_SERVO_MIN_US,
        .max_us = KK_RX_STD_SERVO_MAX_US,
        .us_per_deg = KK_RX_STD_SERVO_US_PER_DEG,
    };
    return s;
}

void kk_rx_profile_sanitize(kk_rx_profile_t *p)
{
    if (!p) {
        return;
    }
    p->rc_proto = kk_rc_out_sanitize_proto(p->rc_proto);
    p->ch_lr = kk_rx_clamp_ch(p->ch_lr);
    p->ch_ud = kk_rx_clamp_ch(p->ch_ud);
    if (p->ch_lr == p->ch_ud) {
        for (uint8_t c = KK_RX_CH_MIN; c <= KK_RX_CH_MAX; c++) {
            if (c != p->ch_lr) {
                p->ch_ud = c;
                break;
            }
        }
    }
    p->offset_lr = kk_rx_clamp_offset(p->offset_lr);
    p->offset_ud = kk_rx_clamp_offset(p->offset_ud);
    kk_rx_clamp_scale(&p->scale_lr);
    kk_rx_clamp_scale(&p->scale_ud);
    p->yaw_servo_deg = kk_rx_sanitize_yaw_servo_deg(p->yaw_servo_deg);
    p->mount_horiz &= 3U;
    p->mount_lr &= 3U;
    p->mount_fb &= 3U;
    kk_gesture_cfg_t g = {
        .roll_deg = p->gest_roll_deg,
        .swing_ms = p->gest_swing_ms,
        .center_en = p->gest_center_en,
    };
    kk_gesture_cfg_sanitize(&g);
    p->gest_roll_deg = g.roll_deg;
    p->gest_swing_ms = g.swing_ms;
    p->gest_center_en = g.center_en;
    kk_tx_track_cfg_t t = {
        .decouple_en = p->track_decouple_en,
        .motion_en = p->track_motion_en,
        .decouple_str_x100 = p->track_decouple_str_x100,
        .decouple_dom_x10 = p->track_decouple_dom_x10,
    };
    kk_tx_track_cfg_sanitize(&t);
    p->track_decouple_en = t.decouple_en;
    p->track_motion_en = t.motion_en;
    p->track_decouple_str_x100 = t.decouple_str_x100;
    p->track_decouple_dom_x10 = t.decouple_dom_x10;
}

void kk_rx_profile_gesture_to_cfg(const kk_rx_profile_t *p, kk_gesture_cfg_t *out)
{
    if (!p || !out) {
        return;
    }
    out->roll_deg = p->gest_roll_deg;
    out->swing_ms = p->gest_swing_ms;
    out->center_en = p->gest_center_en;
    kk_gesture_cfg_sanitize(out);
}

void kk_rx_profile_track_to_cfg(const kk_rx_profile_t *p, kk_tx_track_cfg_t *out)
{
    if (!p || !out) {
        return;
    }
    out->decouple_en = p->track_decouple_en;
    out->motion_en = p->track_motion_en;
    out->decouple_str_x100 = p->track_decouple_str_x100;
    out->decouple_dom_x10 = p->track_decouple_dom_x10;
    kk_tx_track_cfg_sanitize(out);
}

void kk_rx_profile_mount_to_imu(const kk_rx_profile_t *p, kk_imu_mount_t *out)
{
    if (!p || !out) {
        return;
    }
    out->rot_horiz = p->mount_horiz;
    out->rot_lr = p->mount_lr;
    out->rot_fb = p->mount_fb;
    kk_imu_mount_sanitize(out);
}

void kk_rx_profile_mount_from_imu(kk_rx_profile_t *p, const kk_imu_mount_t *m)
{
    if (!p || !m) {
        return;
    }
    p->mount_horiz = m->rot_horiz;
    p->mount_lr = m->rot_lr;
    p->mount_fb = m->rot_fb;
}

/** YAW LR 通道：由网页 yaw_servo_deg(180/270) 决定脉宽映射 */
uint16_t kk_rx_yaw_angle_to_us(float deg, int16_t offset, uint16_t yaw_servo_deg, uint8_t scale_lr)
{
    if (!isfinite(deg)) {
        deg = 0.0f;
    }
    const kk_yaw_servo_t srv = kk_rx_yaw_servo_params(yaw_servo_deg);
    const float mult = kk_rx_scale_to_mult(scale_lr);
    float us = (float)srv.center_us + (float)offset + deg * mult * srv.us_per_deg;
    if (us < (float)srv.min_us) {
        us = (float)srv.min_us;
    }
    if (us > (float)srv.max_us) {
        us = (float)srv.max_us;
    }
    return (uint16_t)(us + 0.5f);
}

/** 俯仰 UD 等标准 180° 通道；不受 yaw_servo_deg 影响 */
uint16_t kk_rx_angle_to_us(float deg, int16_t offset, uint8_t scale_ud)
{
    if (!isfinite(deg)) {
        deg = 0.0f;
    }
    const float mult = kk_rx_scale_to_mult(scale_ud);
    float us = (float)KK_RX_PPM_CENTER + (float)offset + deg * mult * KK_RX_STD_SERVO_US_PER_DEG;
    if (us < (float)KK_RX_STD_SERVO_MIN_US) {
        us = (float)KK_RX_STD_SERVO_MIN_US;
    }
    if (us > (float)KK_RX_STD_SERVO_MAX_US) {
        us = (float)KK_RX_STD_SERVO_MAX_US;
    }
    return (uint16_t)(us + 0.5f);
}

void kk_rx_profile_load(kk_rx_profile_t *out)
{
    if (!out) {
        return;
    }
    *out = kk_rx_profile_defaults();
    nvs_handle_t h;
    if (nvs_open(KK_PREFS_RX_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    uint8_t v8;
    int16_t v16;
    uint8_t u8;
    if (nvs_get_u8(h, "rc_proto", &v8) == ESP_OK) {
        out->rc_proto = v8;
    }
    if (nvs_get_u8(h, "ch_lr", &v8) == ESP_OK) {
        out->ch_lr = v8;
    }
    if (nvs_get_u8(h, "ch_ud", &v8) == ESP_OK) {
        out->ch_ud = v8;
    }
    if (nvs_get_i16(h, "off_lr", &v16) == ESP_OK) {
        out->offset_lr = v16;
    }
    if (nvs_get_i16(h, "off_ud", &v16) == ESP_OK) {
        out->offset_ud = v16;
    }
    if (nvs_get_u8(h, "scale", &v8) == ESP_OK) {
        out->scale_lr = v8;
    }
    if (nvs_get_u8(h, "scale_ud", &v8) == ESP_OK) {
        out->scale_ud = v8;
    } else {
        out->scale_ud = out->scale_lr;
    }
    uint16_t v16u;
    if (nvs_get_u16(h, "yaw_srv", &v16u) == ESP_OK) {
        out->yaw_servo_deg = v16u;
    }
    if (nvs_get_u8(h, "rev_lr", &u8) == ESP_OK) {
        out->rev_lr = u8 != 0;
    }
    if (nvs_get_u8(h, "rev_ud", &u8) == ESP_OK) {
        out->rev_ud = u8 != 0;
    }
    if (nvs_get_u8(h, "m_horiz", &v8) == ESP_OK) {
        out->mount_horiz = v8;
    }
    if (nvs_get_u8(h, "m_lr", &v8) == ESP_OK) {
        out->mount_lr = v8;
    }
    if (nvs_get_u8(h, "m_fb", &v8) == ESP_OK) {
        out->mount_fb = v8;
    }
    if (nvs_get_u8(h, "g_roll", &v8) == ESP_OK) {
        out->gest_roll_deg = v8;
    }
    uint16_t v16u2;
    if (nvs_get_u16(h, "g_ms", &v16u2) == ESP_OK) {
        out->gest_swing_ms = v16u2;
    }
    if (nvs_get_u8(h, "g_cent", &u8) == ESP_OK) {
        out->gest_center_en = u8 != 0;
    }
    if (nvs_get_u8(h, "trk_dec", &u8) == ESP_OK) {
        out->track_decouple_en = u8 != 0;
    }
    if (nvs_get_u8(h, "trk_mob", &u8) == ESP_OK) {
        out->track_motion_en = u8 != 0;
    }
    if (nvs_get_u8(h, "trk_str", &v8) == ESP_OK) {
        out->track_decouple_str_x100 = v8;
    }
    if (nvs_get_u8(h, "trk_dom", &v8) == ESP_OK) {
        out->track_decouple_dom_x10 = v8;
    }
    nvs_close(h);
    kk_rx_profile_sanitize(out);
}

void kk_rx_profile_save(const kk_rx_profile_t *cfg)
{
    if (!cfg) {
        return;
    }
    kk_rx_profile_t p = *cfg;
    kk_rx_profile_sanitize(&p);
    nvs_handle_t h;
    if (nvs_open(KK_PREFS_RX_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u8(h, "rc_proto", p.rc_proto);
    nvs_set_u8(h, "ch_lr", p.ch_lr);
    nvs_set_u8(h, "ch_ud", p.ch_ud);
    nvs_set_i16(h, "off_lr", p.offset_lr);
    nvs_set_i16(h, "off_ud", p.offset_ud);
    nvs_set_u8(h, "scale", p.scale_lr);
    nvs_set_u8(h, "scale_ud", p.scale_ud);
    nvs_set_u16(h, "yaw_srv", p.yaw_servo_deg);
    nvs_set_u8(h, "rev_lr", p.rev_lr ? 1 : 0);
    nvs_set_u8(h, "rev_ud", p.rev_ud ? 1 : 0);
    nvs_set_u8(h, "m_horiz", p.mount_horiz);
    nvs_set_u8(h, "m_lr", p.mount_lr);
    nvs_set_u8(h, "m_fb", p.mount_fb);
    nvs_set_u8(h, "g_roll", p.gest_roll_deg);
    nvs_set_u16(h, "g_ms", p.gest_swing_ms);
    nvs_set_u8(h, "g_cent", p.gest_center_en ? 1 : 0);
    nvs_set_u8(h, "trk_dec", p.track_decouple_en ? 1 : 0);
    nvs_set_u8(h, "trk_mob", p.track_motion_en ? 1 : 0);
    nvs_set_u8(h, "trk_str", p.track_decouple_str_x100);
    nvs_set_u8(h, "trk_dom", p.track_decouple_dom_x10);
    nvs_commit(h);
    nvs_close(h);
}

void kk_rx_profile_reset(kk_rx_profile_t *out)
{
    if (!out) {
        return;
    }
    *out = kk_rx_profile_defaults();
    kk_rx_profile_save(out);
}
