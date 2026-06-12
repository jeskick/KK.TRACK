#include "kk/repair.h"
#include "kk/link_config.h"
#include "kk/time.h"

#include "driver/gpio.h"
#include <string.h>

void kk_btn_repair_init(kk_btn_repair_t *s)
{
    if (!s) {
        return;
    }
    s->down_since = 0;
    s->armed = true;
}

void kk_btn_short_init(kk_btn_short_t *s)
{
    if (!s) {
        return;
    }
    s->down_since = 0;
    s->was_down = false;
}

bool kk_btn_repair_holding(int pin)
{
    return gpio_get_level(pin) == 0;
}

bool kk_btn_repair_poll(int pin, kk_btn_repair_t *s)
{
    if (!s) {
        return false;
    }
    if (gpio_get_level(pin) != 0) {
        s->down_since = 0;
        s->armed = true;
        return false;
    }
    if (!s->armed) {
        return false;
    }
    if (s->down_since == 0) {
        s->down_since = kk_millis();
    }
    if (kk_millis() - s->down_since < KK_REPAIR_HOLD_MS) {
        return false;
    }
    s->armed = false;
    return true;
}

bool kk_btn_short_press_poll(int pin, kk_btn_short_t *s)
{
    if (!s) {
        return false;
    }

    const bool down = gpio_get_level(pin) == 0;
    const uint32_t now = kk_millis();

    if (down && !s->was_down) {
        s->down_since = now;
        s->was_down = true;
        return false;
    }

    if (!down && s->was_down) {
        s->was_down = false;
        const uint32_t held = now - s->down_since;
        if (held >= KK_BTN_SHORT_MIN_MS && held < KK_BTN_SHORT_MAX_MS) {
            return true;
        }
    }

    return false;
}

kk_btn_event_t kk_btn_multifunc_poll(int pin, kk_btn_repair_t *repair, kk_btn_short_t *short_btn)
{
    if (kk_btn_repair_poll(pin, repair)) {
        return KK_BTN_EVT_REPAIR;
    }
    if (kk_btn_short_press_poll(pin, short_btn)) {
        return KK_BTN_EVT_SHORT;
    }
    return KK_BTN_EVT_NONE;
}

bool kk_repair_cmd_match(const uint8_t *data, size_t len)
{
    if (!data || len < 6) {
        return false;
    }
    return memcmp(data, KK_BLE_REPAIR_CMD, 6) == 0;
}

bool kk_center_cmd_match(const uint8_t *data, size_t len)
{
    if (!data || len < 6) {
        return false;
    }
    return memcmp(data, KK_BLE_CENTER_CMD, 6) == 0;
}
