#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "kk/gesture_cfg.h"
#include "kk/imu_mount.h"
#include "kk/rc_out.h"
#include "kk/tx_track_cfg.h"

#define KK_RX_CH_MIN         5
#define KK_RX_CH_MAX         8
/* UI 的 CHn（1 起始）→ PPM 帧内 0 起始下标；CH5→4 … CH8→7 */
#define KK_RX_PPM_CH_BASE    (KK_RX_CH_MIN - 1)
#define KK_RX_SCALE_NEUT     50
#define KK_RX_SCALE_MAX_MULT 1.5f   /* UI 100 → 150% */
#define KK_RX_SCALE_MIN_MULT 0.5f   /* UI 1 → 50% */
#define KK_RX_PPM_CENTER     1500

/*
 * 标准 180° 舵机脉宽（俯仰 UD 等通道，固定，与网页 YAW 选项无关）：
 * 中位 1500µs，行程 1000–2000µs，±90° → 500µs/90°。
 */
#define KK_RX_STD_SERVO_MIN_US     1000
#define KK_RX_STD_SERVO_MAX_US     2000
#define KK_RX_STD_SERVO_US_PER_DEG (500.0f / 90.0f)

/* 别名：俯仰/标准通道与 180° YAW 模式共用同一物理规格 */
#define KK_RX_PPM_MIN        KK_RX_STD_SERVO_MIN_US
#define KK_RX_PPM_MAX        KK_RX_STD_SERVO_MAX_US

#define KK_RX_OFFSET_MIN     (-400)
#define KK_RX_OFFSET_MAX     400

/* 仅 YAW(LR) 通道：网页 180°/270° → kk_rx_yaw_servo_params() */
#define KK_RX_YAW_SERVO_180  180U
#define KK_RX_YAW_SERVO_270  270U

typedef struct {
    uint16_t center_us;
    uint16_t min_us;
    uint16_t max_us;
    float us_per_deg;
} kk_yaw_servo_t;

typedef struct {
    uint8_t rc_proto;
    uint8_t ch_lr;
    uint8_t ch_ud;
    int16_t offset_lr;
    int16_t offset_ud;
    uint8_t scale_lr;
    uint8_t scale_ud;
    uint16_t yaw_servo_deg;
    bool rev_lr;
    bool rev_ud;
    uint8_t mount_horiz;
    uint8_t mount_lr;
    uint8_t mount_fb;
    uint8_t gest_roll_deg;
    uint16_t gest_swing_ms;
    bool gest_center_en;
    bool track_decouple_en;
    bool track_motion_en;
    uint8_t track_decouple_str_x100;
    uint8_t track_decouple_dom_x10;
} kk_rx_profile_t;

kk_rx_profile_t kk_rx_profile_defaults(void);
uint8_t kk_rx_ch_to_index(uint8_t ch);
float kk_rx_scale_to_mult(uint8_t scale);
int16_t kk_rx_clamp_offset(int16_t v);
uint8_t kk_rx_clamp_ch(uint8_t ch);
void kk_rx_profile_sanitize(kk_rx_profile_t *p);
uint16_t kk_rx_sanitize_yaw_servo_deg(uint16_t deg);
kk_yaw_servo_t kk_rx_yaw_servo_params(uint16_t yaw_servo_deg);
uint16_t kk_rx_yaw_angle_to_us(float deg, int16_t offset, uint16_t yaw_servo_deg, uint8_t scale_lr);
/** 俯仰 UD：标准 180° 舵机（1000–2000µs）；scale_ud 同 LR 比例规则 */
uint16_t kk_rx_angle_to_us(float deg, int16_t offset, uint8_t scale_ud);
void kk_rx_profile_load(kk_rx_profile_t *out);
void kk_rx_profile_save(const kk_rx_profile_t *cfg);
void kk_rx_profile_reset(kk_rx_profile_t *out);
void kk_rx_profile_mount_to_imu(const kk_rx_profile_t *p, kk_imu_mount_t *out);
void kk_rx_profile_mount_from_imu(kk_rx_profile_t *p, const kk_imu_mount_t *m);
void kk_rx_profile_gesture_to_cfg(const kk_rx_profile_t *p, kk_gesture_cfg_t *out);
void kk_rx_profile_track_to_cfg(const kk_rx_profile_t *p, kk_tx_track_cfg_t *out);
