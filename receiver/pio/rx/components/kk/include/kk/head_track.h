#pragma once

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
void kk_head_track_apply(const kk_rx_profile_t *cfg);
