#include "kk/telemetry.h"
#include "kk/time.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

kk_telemetry_t g_kk_tel;

void kk_tel_reset(void)
{
    memset(&g_kk_tel, 0, sizeof(g_kk_tel));
}

int kk_tel_format_pose(char *buf, size_t cap, float yaw_deg, float pitch_deg, bool motion_paused)
{
    if (!buf || cap < 16) {
        return 0;
    }
    if (!isfinite(yaw_deg) || !isfinite(pitch_deg)) {
        return 0;
    }
    const int n = snprintf(buf, cap, "yaw:%.1f,pitch:%.1f,mob:%u", yaw_deg, pitch_deg,
                             motion_paused ? 1U : 0U);
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
    return isfinite(*out);
}

static bool kk_tel_parse_key_uint(const char *s, const char *key, unsigned *out)
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
    *out = (unsigned)strtoul(p + 1, NULL, 10);
    return true;
}

void kk_tel_on_udp_payload(const char *buf)
{
    if (!buf) {
        return;
    }
    bool pose_ok = false;
    float v = 0.0f;
    if (kk_tel_parse_key_float(buf, "yaw", &v)) {
        g_kk_tel.yaw_deg = v;
        pose_ok = true;
    }
    if (kk_tel_parse_key_float(buf, "pitch", &v)) {
        g_kk_tel.pitch_deg = v;
        pose_ok = true;
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
    unsigned mob = 0;
    if (kk_tel_parse_key_uint(buf, "mob", &mob)) {
        g_kk_tel.motion_paused = mob != 0;
    }
    if (pose_ok) {
        g_kk_tel.last_pkt_ms = kk_millis();
    }
}

void kk_tel_poll_rx_voltage(void)
{
    (void)0;
}

void kk_tel_poll_link(void)
{
    const uint32_t now = kk_millis();
    /* last_txv_ms 由 BLE 主机任务写入，可能偶尔领先本任务的 now；用有符号差判断，
     * 避免 now-last 在 uint32 下溢成天文数字而误判 TX 电压超时失效。 */
    if (g_kk_tel.last_txv_ms > 0 && (int32_t)(now - g_kk_tel.last_txv_ms) > 8000) {
        g_kk_tel.tx_v_valid = false;
    }
}
