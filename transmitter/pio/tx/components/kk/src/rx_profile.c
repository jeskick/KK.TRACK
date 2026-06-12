#include "kk/rx_profile.h"
#include "kk/imu_mount.h"
#include "kk/link_config.h"

#include "nvs.h"
#include <math.h>

kk_rx_profile_t kk_rx_profile_defaults(void)
{
    kk_rx_profile_t p = {
        .ch_lr = 6,
        .ch_ud = 5,
        .offset_lr = 0,
        .offset_ud = 0,
        .scale_lr = KK_RX_SCALE_NEUT,
        .jitter_x10 = 5,
        .yaw_servo_deg = KK_RX_YAW_SERVO_180,
        .rev_lr = false,
        .rev_ud = false,
        .mount_horiz = 0,
        .mount_lr = 0,
        .mount_fb = 0,
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
    if (scale >= KK_RX_SCALE_NEUT) {
        return 1.0f + (float)(scale - KK_RX_SCALE_NEUT) / 10.0f;
    }
    float m = 1.0f - (float)(KK_RX_SCALE_NEUT - scale) / 10.0f;
    return m < 0.1f ? 0.1f : m;
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
            .us_per_deg = 1000.0f / 135.0f,
        };
        return s;
    }
    const kk_yaw_servo_t s = {
        .center_us = KK_RX_PPM_CENTER,
        .min_us = KK_RX_PPM_MIN,
        .max_us = KK_RX_PPM_MAX,
        .us_per_deg = 500.0f / 90.0f,
    };
    return s;
}

void kk_rx_profile_sanitize(kk_rx_profile_t *p)
{
    if (!p) {
        return;
    }
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
    if (p->scale_lr < 1) {
        p->scale_lr = 1;
    }
    if (p->scale_lr > 100) {
        p->scale_lr = 100;
    }
    if (p->jitter_x10 > 200) {
        p->jitter_x10 = 200;
    }
    p->yaw_servo_deg = kk_rx_sanitize_yaw_servo_deg(p->yaw_servo_deg);
    p->mount_horiz &= 3U;
    p->mount_lr &= 3U;
    p->mount_fb &= 3U;
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

uint16_t kk_rx_yaw_angle_to_us(float deg, int16_t offset, uint16_t yaw_servo_deg, uint8_t scale_lr)
{
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

uint16_t kk_rx_angle_to_us(float deg, int16_t offset, bool use_lr_scale, uint8_t scale_lr)
{
    float mult = use_lr_scale ? kk_rx_scale_to_mult(scale_lr) : 1.0f;
    float us = (float)KK_RX_PPM_CENTER + (float)offset + deg * mult * KK_RX_US_PER_DEG;
    if (us < (float)KK_RX_PPM_MIN) {
        us = (float)KK_RX_PPM_MIN;
    }
    if (us > (float)KK_RX_PPM_MAX) {
        us = (float)KK_RX_PPM_MAX;
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
    if (nvs_get_u8(h, "jitter", &v8) == ESP_OK) {
        out->jitter_x10 = v8;
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
    nvs_set_u8(h, "ch_lr", p.ch_lr);
    nvs_set_u8(h, "ch_ud", p.ch_ud);
    nvs_set_i16(h, "off_lr", p.offset_lr);
    nvs_set_i16(h, "off_ud", p.offset_ud);
    nvs_set_u8(h, "scale", p.scale_lr);
    nvs_set_u8(h, "jitter", p.jitter_x10);
    nvs_set_u16(h, "yaw_srv", p.yaw_servo_deg);
    nvs_set_u8(h, "rev_lr", p.rev_lr ? 1 : 0);
    nvs_set_u8(h, "rev_ud", p.rev_ud ? 1 : 0);
    nvs_set_u8(h, "m_horiz", p.mount_horiz);
    nvs_set_u8(h, "m_lr", p.mount_lr);
    nvs_set_u8(h, "m_fb", p.mount_fb);
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
