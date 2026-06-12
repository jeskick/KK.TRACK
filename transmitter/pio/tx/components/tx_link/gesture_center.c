#include "gesture_center.h"
#include "tx_gesture.h"

#include "kk/link_config.h"

#include "esp_log.h"

static const char *TAG = "kk.gesture";

typedef enum {
    KK_GS_IDLE = 0,
    KK_GS_SWING,
    KK_GS_SETTLE,
} kk_gesture_state_t;

static kk_gesture_center_cb_t s_on_fire;
static kk_gesture_state_t s_state;
static uint32_t s_swing_start_ms;
static uint32_t s_neut_start_ms;
static uint32_t s_settle_start_ms;
static uint32_t s_cooldown_until_ms;
static bool s_seen_pos;
static bool s_seen_neg;

static void kk_gesture_to_idle(void)
{
    s_state = KK_GS_IDLE;
    s_seen_pos = false;
    s_seen_neg = false;
    s_neut_start_ms = 0;
    s_settle_start_ms = 0;
}

void kk_gesture_center_init(kk_gesture_center_cb_t on_fire)
{
    s_on_fire = on_fire;
    kk_gesture_to_idle();
    s_cooldown_until_ms = 0;
}

void kk_gesture_center_suppress(uint32_t now_ms)
{
    kk_gesture_to_idle();
    s_cooldown_until_ms = now_ms + KK_TX_ROLL_GESTURE_COOLDOWN_MS;
}

void kk_gesture_center_poll(float roll_deg, uint32_t now_ms)
{
    if (!s_on_fire) {
        return;
    }
    if (now_ms < s_cooldown_until_ms) {
        return;
    }

    const kk_gesture_cfg_t *gest = kk_tx_gesture_get();
    const float trig = (float)gest->roll_deg;
    const uint32_t swing_ms = gest->swing_ms;

    if (roll_deg >= trig) {
        s_seen_pos = true;
    }
    if (roll_deg <= -trig) {
        s_seen_neg = true;
    }

    const bool roll_neut =
        roll_deg <= KK_TX_ROLL_NEUT_DEG && roll_deg >= -KK_TX_ROLL_NEUT_DEG;

    switch (s_state) {
    case KK_GS_IDLE:
        if (roll_neut) {
            break;
        }
        if (roll_deg >= trig || roll_deg <= -trig) {
            s_state = KK_GS_SWING;
            s_swing_start_ms = now_ms;
            s_seen_pos = roll_deg >= trig;
            s_seen_neg = roll_deg <= -trig;
            ESP_LOGW(TAG, "swing start R=%d trig=%u", (int)roll_deg, gest->roll_deg);
        }
        break;

    case KK_GS_SWING:
        if ((now_ms - s_swing_start_ms) > swing_ms) {
            ESP_LOGW(TAG, "swing timeout pos=%d neg=%d win=%ums trig=%u",
                     (int)s_seen_pos, (int)s_seen_neg, (unsigned)swing_ms, gest->roll_deg);
            kk_gesture_to_idle();
            break;
        }
        if (s_seen_pos && s_seen_neg) {
            s_state = KK_GS_SETTLE;
            s_settle_start_ms = now_ms;
            s_neut_start_ms = roll_neut ? now_ms : 0;
            ESP_LOGW(TAG, "swing ok -> settle");
        }
        break;

    case KK_GS_SETTLE:
        if ((now_ms - s_settle_start_ms) > KK_TX_ROLL_SETTLE_TIMEOUT_MS) {
            ESP_LOGW(TAG, "settle timeout");
            kk_gesture_to_idle();
            break;
        }
        if (roll_neut) {
            if (s_neut_start_ms == 0) {
                s_neut_start_ms = now_ms;
            } else if ((now_ms - s_neut_start_ms) >= KK_TX_ROLL_SETTLE_MS) {
                ESP_LOGW(TAG, "gesture center fire R=%d", (int)roll_deg);
                kk_gesture_to_idle();
                s_cooldown_until_ms = now_ms + KK_TX_ROLL_GESTURE_COOLDOWN_MS;
                s_on_fire();
            }
        } else {
            s_neut_start_ms = 0;
        }
        break;

    default:
        kk_gesture_to_idle();
        break;
    }
}
