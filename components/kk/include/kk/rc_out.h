#pragma once

#include <stdbool.h>
#include <stdint.h>

#define KK_RC_CH_COUNT 8

typedef enum {
    KK_RC_PROTO_PPM = 0,
    KK_RC_PROTO_SBUS = 1,
    KK_RC_PROTO_CRSF = 2,
} kk_rc_proto_t;

void kk_rc_out_begin(kk_rc_proto_t proto);
void kk_rc_out_stop(void);
void kk_rc_out_commit(void);
void kk_rc_out_set_channel(uint8_t index, uint16_t pulse_us);
uint16_t kk_rc_out_get_channel(uint8_t index);
void kk_rc_out_fill_center(void);
kk_rc_proto_t kk_rc_out_get_proto(void);
bool kk_rc_out_is_running(void);
uint8_t kk_rc_out_sanitize_proto(uint8_t v);
