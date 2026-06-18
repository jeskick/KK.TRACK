#include "tx_gesture.h"

#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "kk.tx.gesture";
static kk_gesture_cfg_t s_cfg;

void kk_tx_gesture_load(kk_gesture_cfg_t *out)
{
    kk_gesture_cfg_t cfg = kk_gesture_cfg_defaults();
    nvs_handle_t h;
    if (nvs_open("kk_tx", NVS_READONLY, &h) == ESP_OK) {
        uint8_t roll;
        uint16_t ms;
        if (nvs_get_u8(h, "g_roll", &roll) == ESP_OK) {
            cfg.roll_deg = roll;
        }
        if (nvs_get_u16(h, "g_ms", &ms) == ESP_OK) {
            cfg.swing_ms = ms;
        }
        nvs_close(h);
    }
    kk_gesture_cfg_sanitize(&cfg);
    s_cfg = cfg;
    if (out) {
        *out = cfg;
    }
    ESP_LOGW(TAG, "gesture roll=%u swing_ms=%u center=%u", cfg.roll_deg, cfg.swing_ms,
             cfg.center_en ? 1U : 0U);
}

void kk_tx_gesture_save(const kk_gesture_cfg_t *cfg)
{
    if (!cfg) {
        return;
    }
    kk_gesture_cfg_t c = *cfg;
    kk_gesture_cfg_sanitize(&c);
    s_cfg = c;

    nvs_handle_t h;
    if (nvs_open("kk_tx", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u8(h, "g_roll", c.roll_deg);
    nvs_set_u16(h, "g_ms", c.swing_ms);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGW(TAG, "gesture cfg roll=%u° swing=%ums center=%u", c.roll_deg, c.swing_ms,
             c.center_en ? 1U : 0U);
}

bool kk_tx_gesture_center_enabled(void)
{
    return s_cfg.center_en;
}

void kk_tx_gesture_apply(const kk_gesture_cfg_t *cfg)
{
    if (!cfg) {
        return;
    }
    kk_tx_gesture_save(cfg);
}

const kk_gesture_cfg_t *kk_tx_gesture_get(void)
{
    return &s_cfg;
}
