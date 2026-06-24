#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "kk/ota_image.h"
#include "kk/rc_proto.h"
#include "kk/rx_profile.h"

int main(void)
{
    kk_ota_img_check_t chk;
    uint8_t byte = 0x00;

    kk_ota_img_check_reset(&chk);
    assert(kk_ota_img_check_feed(&chk, &byte, 1, NULL) == KK_OTA_IMG_REJECT);
    assert(chk.decided);

    kk_ota_img_check_reset(&chk);
    byte = 0xE9;
    assert(kk_ota_img_check_feed(&chk, &byte, 1, NULL) == KK_OTA_IMG_PENDING);

    assert(kk_rc_proto_sanitize(99) == KK_RC_PROTO_PPM);
    assert(kk_rc_proto_sanitize(KK_RC_PROTO_SBUS) == KK_RC_PROTO_SBUS);

    assert(kk_rx_sanitize_yaw_servo_deg(270) == KK_RX_YAW_SERVO_270);
    assert(kk_rx_sanitize_yaw_servo_deg(999) == KK_RX_YAW_SERVO_180);

    kk_rx_profile_t p = kk_rx_profile_defaults();
    p.rc_proto = 99;
    p.ch_lr = p.ch_ud;
    p.scale_lr = 0;
    kk_rx_profile_sanitize(&p);
    assert(p.rc_proto == KK_RC_PROTO_PPM);
    assert(p.ch_lr != p.ch_ud);
    assert(p.scale_lr >= 1);

    puts("OK");
    return 0;
}
