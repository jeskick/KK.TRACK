#include "gesture_center.h"
#include "tx_gesture.h"

#include "kk/link_config.h"

#include "esp_log.h"
#include <math.h>

static const char *TAG = "kk.gesture";

typedef enum {
    KK_GS_IDLE = 0,
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
    s_swing_start_ms = 0;
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

bool kk_gesture_center_is_active(void)
{
    /* 仅 SETTLE 等待回正；摆动检测期不冻结遥测 */
    return s_state == KK_GS_SETTLE;
}

void kk_gesture_center_poll(float roll_deg, float pitch_deg, float yaw_deg,
                            float gyro_roll_dps, float gyro_yaw_dps, uint32_t now_ms)
{
    (void)pitch_deg;
    (void)yaw_deg;
    (void)gyro_yaw_dps;

    if (!s_on_fire || !kk_tx_gesture_center_enabled()) {
        return;
    }

    const kk_gesture_cfg_t *gest = kk_tx_gesture_get();
    if (!gest) {
        return;
    }

    if (now_ms < s_cooldown_until_ms) {
        return;
    }

    const float trig = (float)gest->roll_deg;
    const uint32_t swing_ms = gest->swing_ms;

    const bool roll_neut =
        roll_deg <= KK_TX_ROLL_NEUT_DEG && roll_deg >= -KK_TX_ROLL_NEUT_DEG;
    const bool roll_vertical = fabsf(roll_deg) >= KK_TX_GESTURE_ROLL_ABORT_DEG;

    if (roll_vertical && s_state != KK_GS_IDLE) {
        ESP_LOGW(TAG, "gesture abort vertical roll R=%d", (int)roll_deg);
        kk_gesture_to_idle();
        return;
    }

    switch (s_state) {
    case KK_GS_IDLE:
        if (roll_vertical) {
            break;
        }
        if (s_swing_start_ms == 0) {
            if (roll_neut) {
                break;
            }
            if (fabsf(gyro_roll_dps) < KK_TX_GESTURE_GYRO_ROLL_DPS) {
                break;
            }
            if (roll_deg >= trig || roll_deg <= -trig) {
                s_swing_start_ms = now_ms;
                s_seen_pos = roll_deg >= trig;
                s_seen_neg = roll_deg <= -trig;
            }
            break;
        }

        if (roll_deg >= trig) {
            s_seen_pos = true;
        }
        if (roll_deg <= -trig) {
            s_seen_neg = true;
        }

        if (s_seen_pos && s_seen_neg) {
            ESP_LOGW(TAG, "gesture both sides -> settle");
            s_state = KK_GS_SETTLE;
            s_settle_start_ms = now_ms;
            s_neut_start_ms = roll_neut ? now_ms : 0;
            s_swing_start_ms = 0;
            break;
        }

        if ((now_ms - s_swing_start_ms) > swing_ms) {
            ESP_LOGW(TAG, "gesture abort one-sided (pos=%u neg=%u)", s_seen_pos ? 1U : 0U,
                     s_seen_neg ? 1U : 0U);
            kk_gesture_to_idle();
        }
        break;

    case KK_GS_SETTLE:
        if ((now_ms - s_settle_start_ms) > KK_TX_ROLL_SETTLE_TIMEOUT_MS) {
            ESP_LOGW(TAG, "gesture settle timeout");
            kk_gesture_to_idle();
            break;
        }
        if (roll_neut) {
            if (s_neut_start_ms == 0) {
                s_neut_start_ms = now_ms;
            } else if ((now_ms - s_neut_start_ms) >= KK_TX_ROLL_SETTLE_MS) {
                ESP_LOGW(TAG, "gesture center fire");
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
