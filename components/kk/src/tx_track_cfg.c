#include "kk/tx_track_cfg.h"

#include <stdio.h>
#include <string.h>

kk_tx_track_cfg_t kk_tx_track_cfg_defaults(void)
{
    const kk_tx_track_cfg_t cfg = {
        .decouple_en = true,
        .motion_en = true,
        .decouple_str_x100 = KK_TRACK_DEC_STR_DEFAULT,
        .decouple_dom_x10 = KK_TRACK_DEC_DOM_DEFAULT,
    };
    return cfg;
}

void kk_tx_track_cfg_sanitize(kk_tx_track_cfg_t *cfg)
{
    if (!cfg) {
        return;
    }
    if (cfg->decouple_str_x100 < KK_TRACK_DEC_STR_MIN) {
        cfg->decouple_str_x100 = KK_TRACK_DEC_STR_MIN;
    }
    if (cfg->decouple_str_x100 > KK_TRACK_DEC_STR_MAX) {
        cfg->decouple_str_x100 = KK_TRACK_DEC_STR_MAX;
    }
    if (cfg->decouple_dom_x10 < KK_TRACK_DEC_DOM_MIN) {
        cfg->decouple_dom_x10 = KK_TRACK_DEC_DOM_MIN;
    }
    if (cfg->decouple_dom_x10 > KK_TRACK_DEC_DOM_MAX) {
        cfg->decouple_dom_x10 = KK_TRACK_DEC_DOM_MAX;
    }
}

bool kk_track_cmd_parse(const char *line, size_t len, kk_tx_track_cfg_t *out)
{
    if (!line || !out || len < 7U) {
        return false;
    }
    if (strncmp(line, "TRK,", 4) != 0) {
        return false;
    }

    unsigned dec = 0;
    unsigned mob = 0;
    unsigned str = KK_TRACK_DEC_STR_DEFAULT;
    unsigned dom = KK_TRACK_DEC_DOM_DEFAULT;
    const int n = sscanf(line, "TRK,%u,%u,%u,%u", &dec, &mob, &str, &dom);

    out->decouple_en = dec != 0;
    out->motion_en = mob != 0;
    if (n >= 3) {
        out->decouple_str_x100 = (uint8_t)str;
    } else {
        out->decouple_str_x100 = KK_TRACK_DEC_STR_DEFAULT;
    }
    if (n >= 4) {
        out->decouple_dom_x10 = (uint8_t)dom;
    } else {
        out->decouple_dom_x10 = KK_TRACK_DEC_DOM_DEFAULT;
    }
    kk_tx_track_cfg_sanitize(out);
    return true;
}
