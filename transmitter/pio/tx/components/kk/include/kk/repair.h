#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t down_since;
    bool armed;
} kk_btn_repair_t;

typedef struct {
    uint32_t down_since;
    bool was_down;
} kk_btn_short_t;

typedef enum {
    KK_BTN_EVT_NONE = 0,
    KK_BTN_EVT_SHORT,
    KK_BTN_EVT_REPAIR,
} kk_btn_event_t;

void kk_btn_repair_init(kk_btn_repair_t *s);
void kk_btn_short_init(kk_btn_short_t *s);
bool kk_btn_repair_holding(int pin);
bool kk_btn_repair_poll(int pin, kk_btn_repair_t *s);
bool kk_btn_short_press_poll(int pin, kk_btn_short_t *s);
kk_btn_event_t kk_btn_multifunc_poll(int pin, kk_btn_repair_t *repair, kk_btn_short_t *short_btn);
bool kk_repair_cmd_match(const uint8_t *data, size_t len);
bool kk_center_cmd_match(const uint8_t *data, size_t len);
