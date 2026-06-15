#include "tx_track.h"

#include "kk/tx_track_cfg.h"

#include "esp_log.h"

static const char *TAG = "kk.tx.track";
static kk_tx_track_cfg_t s_cfg;

void kk_tx_track_load(kk_tx_track_cfg_t *out)
{
    s_cfg = kk_tx_track_cfg_defaults();
    if (out) {
        *out = s_cfg;
    }
    ESP_LOGW(TAG, "track dec=%u mob=%u", s_cfg.decouple_en ? 1U : 0U, s_cfg.motion_en ? 1U : 0U);
}

void kk_tx_track_apply(const kk_tx_track_cfg_t *cfg)
{
    if (!cfg) {
        return;
    }
    s_cfg = *cfg;
    kk_tx_track_cfg_sanitize(&s_cfg);
    ESP_LOGW(TAG, "track apply dec=%u mob=%u str=%u dom=%u", s_cfg.decouple_en ? 1U : 0U,
             s_cfg.motion_en ? 1U : 0U, s_cfg.decouple_str_x100, s_cfg.decouple_dom_x10);
}

const kk_tx_track_cfg_t *kk_tx_track_get(void)
{
    return &s_cfg;
}
