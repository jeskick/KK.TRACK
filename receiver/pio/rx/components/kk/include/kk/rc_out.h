#pragma once

#include "kk/rc_proto.h"

#include <stdbool.h>
#include <stdint.h>

#define KK_RC_CH_COUNT 8
#define KK_CRSF_ADDR_FC 0xC8 /* Betaflight/ELRS 飞控 CRSF 目的地址 */

void kk_rc_out_begin(kk_rc_proto_t proto);
void kk_rc_out_stop(void);
void kk_rc_out_commit(void);
void kk_rc_out_set_channel(uint8_t index, uint16_t pulse_us);
uint16_t kk_rc_out_get_channel(uint8_t index);
void kk_rc_out_fill_center(void);
void kk_rc_out_set_failsafe(bool active);
kk_rc_proto_t kk_rc_out_get_proto(void);
bool kk_rc_out_is_running(void);
uint8_t kk_rc_out_sanitize_proto(uint8_t v);
