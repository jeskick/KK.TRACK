#pragma once

#include <stdint.h>

typedef enum {
    KK_RC_PROTO_PPM = 0,
    KK_RC_PROTO_SBUS = 1,
    KK_RC_PROTO_CRSF = 2, /* 单向 TX：RC 通道帧，不解析飞控回传 */
} kk_rc_proto_t;

uint8_t kk_rc_proto_sanitize(uint8_t v);
