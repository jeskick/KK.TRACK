#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "kk/rx_profile.h"

typedef struct {
    float yaw_f;
    float pitch_f;
    uint16_t ppm_lr;
    uint16_t ppm_ud;
} kk_head_track_t;

extern kk_head_track_t g_kk_ht;

void kk_head_track_reset(void);
void kk_head_track_center(const kk_rx_profile_t *cfg);
void kk_head_track_offset_center(const kk_rx_profile_t *cfg);
void kk_head_track_apply(const kk_rx_profile_t *cfg);
void kk_head_track_poll(const kk_rx_profile_t *cfg, bool ble_connected, bool ppm_active,
                        uint32_t now_ms);
void kk_head_track_failsafe_reset(void);
bool kk_head_track_failsafe_active(void);
