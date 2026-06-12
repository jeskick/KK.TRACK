#include "kk/gesture_cfg.h"

#include <stdio.h>
#include <string.h>

static uint8_t kk_gesture_sanitize_roll(uint8_t deg)
{
    if (deg == 20U || deg == 30U || deg == 40U) {
        return deg;
    }
    if (deg <= 25U) {
        return 20U;
    }
    if (deg <= 35U) {
        return 30U;
    }
    return 40U;
}

kk_gesture_cfg_t kk_gesture_cfg_defaults(void)
{
    const kk_gesture_cfg_t cfg = {
        .roll_deg = KK_GEST_ROLL_DEG_DEFAULT,
        .swing_ms = KK_GEST_SWING_MS_DEFAULT,
    };
    return cfg;
}

void kk_gesture_cfg_sanitize(kk_gesture_cfg_t *cfg)
{
    if (!cfg) {
        return;
    }
    cfg->roll_deg = kk_gesture_sanitize_roll(cfg->roll_deg);
    if (cfg->swing_ms < KK_GEST_SWING_MS_MIN) {
        cfg->swing_ms = KK_GEST_SWING_MS_MIN;
    }
    if (cfg->swing_ms > KK_GEST_SWING_MS_MAX) {
        cfg->swing_ms = KK_GEST_SWING_MS_MAX;
    }
}

bool kk_gesture_cmd_parse(const char *line, size_t len, kk_gesture_cfg_t *out)
{
    if (!line || !out || len < 7U) {
        return false;
    }
    if (strncmp(line, "GES,", 4) != 0) {
        return false;
    }

    unsigned deg = 0;
    unsigned ms = 0;
    if (sscanf(line, "GES,%u,%u", &deg, &ms) != 2) {
        return false;
    }

    out->roll_deg = (uint8_t)deg;
    out->swing_ms = (uint16_t)ms;
    kk_gesture_cfg_sanitize(out);
    return true;
}
