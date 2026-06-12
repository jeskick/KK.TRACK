#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef void (*kk_gesture_center_cb_t)(void);

void kk_gesture_center_init(kk_gesture_center_cb_t on_fire);
void kk_gesture_center_poll(float roll_deg, uint32_t now_ms);
/** 物理/网页回中后：清状态并短时抑制手势 */
void kk_gesture_center_suppress(uint32_t now_ms);
