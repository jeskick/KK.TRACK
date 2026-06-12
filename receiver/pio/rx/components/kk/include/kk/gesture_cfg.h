#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define KK_GEST_ROLL_DEG_DEFAULT   20U
#define KK_GEST_SWING_MS_DEFAULT   1500U
#define KK_GEST_SWING_MS_MIN       500U
#define KK_GEST_SWING_MS_MAX       2000U

typedef struct {
    uint8_t roll_deg;
    uint16_t swing_ms;
} kk_gesture_cfg_t;

kk_gesture_cfg_t kk_gesture_cfg_defaults(void);
void kk_gesture_cfg_sanitize(kk_gesture_cfg_t *cfg);
bool kk_gesture_cmd_parse(const char *line, size_t len, kk_gesture_cfg_t *out);
