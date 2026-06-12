#pragma once

#include "kk/gesture_cfg.h"

void kk_tx_gesture_load(kk_gesture_cfg_t *out);
void kk_tx_gesture_save(const kk_gesture_cfg_t *cfg);
void kk_tx_gesture_apply(const kk_gesture_cfg_t *cfg);
const kk_gesture_cfg_t *kk_tx_gesture_get(void);
