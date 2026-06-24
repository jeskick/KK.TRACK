#include "kk/rc_proto.h"

uint8_t kk_rc_proto_sanitize(uint8_t v)
{
    if (v > KK_RC_PROTO_CRSF) {
        return KK_RC_PROTO_PPM;
    }
    return v;
}
