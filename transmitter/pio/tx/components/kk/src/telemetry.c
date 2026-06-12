#include "kk/telemetry.h"
#include "kk/board_rx.h"
#include "kk/time.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

kk_telemetry_t g_kk_tel;

void kk_tel_reset(void)
{
    memset(&g_kk_tel, 0, sizeof(g_kk_tel));
}

int kk_tel_format_pose(char *buf, size_t cap, float yaw_deg, float pitch_deg)
{
    if (!buf || cap < 16) {
        return 0;
    }
    const int n = snprintf(buf, cap, "yaw:%.1f,pitch:%.1f", yaw_deg, pitch_deg);
    if (n <= 0 || (size_t)n >= cap) {
        return 0;
    }
    return n;
}

static bool kk_tel_parse_key_float(const char *s, const char *key, float *out)
{
    if (!s || !key || !out) {
        return false;
    }
    const char *p = strstr(s, key);
    if (!p) {
        return false;
    }
    p += strlen(key);
    if (*p != ':') {
        return false;
    }
    *out = strtof(p + 1, NULL);
    return true;
}

void kk_tel_on_udp_payload(const char *buf)
{
    if (!buf) {
        return;
    }
    float v = 0.0f;
    if (kk_tel_parse_key_float(buf, "yaw", &v)) {
        g_kk_tel.yaw_deg = v;
    }
    if (kk_tel_parse_key_float(buf, "pitch", &v)) {
        g_kk_tel.pitch_deg = v;
    }
    if (kk_tel_parse_key_float(buf, "txv", &v)) {
        g_kk_tel.tx_voltage = v;
        g_kk_tel.tx_v_valid = true;
        g_kk_tel.last_txv_ms = kk_millis();
    }
    if (kk_tel_parse_key_float(buf, "rxv", &v)) {
        g_kk_tel.rx_voltage = v;
        g_kk_tel.rx_v_valid = true;
    }
    g_kk_tel.last_pkt_ms = kk_millis();
}

void kk_tel_poll_rx_voltage(void)
{
#if PIN_VBAT_ADC >= 0
    (void)0;
#else
    (void)0;
#endif
}

void kk_tel_poll_link(void)
{
    const uint32_t now = kk_millis();
    if (g_kk_tel.last_txv_ms > 0 && now - g_kk_tel.last_txv_ms > 8000) {
        g_kk_tel.tx_v_valid = false;
    }
}
