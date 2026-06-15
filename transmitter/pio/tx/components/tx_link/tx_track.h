#pragma once

#include "kk/tx_track_cfg.h"

void kk_tx_track_load(kk_tx_track_cfg_t *out);
void kk_tx_track_apply(const kk_tx_track_cfg_t *cfg);
const kk_tx_track_cfg_t *kk_tx_track_get(void);
