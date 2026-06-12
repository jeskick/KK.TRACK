#pragma once

#include <stdbool.h>
#include <stdint.h>

uint32_t kk_millis(void);
void kk_delay_ms(uint32_t ms);

/** 周期性诊断：到期返回 true 并刷新 *last_ms，否则立即返回 false（无 I/O）。 */
static inline bool kk_diag_due(uint32_t *last_ms, uint32_t interval_ms)
{
    const uint32_t now = kk_millis();
    if ((uint32_t)(now - *last_ms) < interval_ms) {
        return false;
    }
    *last_ms = now;
    return true;
}
