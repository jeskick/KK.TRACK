#include "kk/rx_web.h"
#include "kk/rx_web_page.h"
#include "kk/head_track.h"
#include "kk/imu_mount.h"
#include "kk/gesture_cfg.h"
#include "kk/rx_profile.h"
#include "kk/telemetry.h"

#include "kk/time.h"

#include "esp_http_server.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "kk.web";

static httpd_handle_t s_server;
static kk_rx_profile_t *s_profile;
static uint32_t s_last_http_ms;
static kk_rx_web_mount_sync_cb_t s_mount_sync;
static kk_rx_web_gesture_sync_cb_t s_gesture_sync;

static void kk_rx_web_touch(void)
{
    s_last_http_ms = kk_millis();
}

uint32_t kk_rx_web_last_http_ms(void)
{
    return s_last_http_ms;
}

static void kk_rx_web_json_config(char *buf, size_t cap)
{
    if (!s_profile) {
        snprintf(buf, cap, "{}");
        return;
    }
    snprintf(buf, cap,
             "{\"ch_lr\":%u,\"ch_ud\":%u,\"offset_lr\":%d,\"offset_ud\":%d,"
             "\"scale_lr\":%u,\"jitter_x10\":%u,\"yaw_servo_deg\":%u,\"rev_lr\":%u,\"rev_ud\":%u,"
             "\"mount_horiz\":%u,\"mount_lr\":%u,\"mount_fb\":%u,"
             "\"gest_roll_deg\":%u,\"gest_swing_ms\":%u}",
             s_profile->ch_lr, s_profile->ch_ud, s_profile->offset_lr, s_profile->offset_ud,
             s_profile->scale_lr, s_profile->jitter_x10, s_profile->yaw_servo_deg,
             s_profile->rev_lr ? 1U : 0U, s_profile->rev_ud ? 1U : 0U,
             kk_imu_mount_steps_to_deg(s_profile->mount_horiz),
             kk_imu_mount_steps_to_deg(s_profile->mount_lr),
             kk_imu_mount_steps_to_deg(s_profile->mount_fb),
             s_profile->gest_roll_deg, s_profile->gest_swing_ms);
}

static esp_err_t root_get(httpd_req_t *req)
{
    kk_rx_web_touch();
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, (const char *)kk_rx_web_page, kk_rx_web_page_len);
}

static esp_err_t favicon_get(httpd_req_t *req)
{
    kk_rx_web_touch();
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_get(httpd_req_t *req)
{
    kk_rx_web_touch();
    char buf[200];
    uint8_t clr = s_profile ? s_profile->ch_lr : 6;
    uint8_t cud = s_profile ? s_profile->ch_ud : 5;
    snprintf(buf, sizeof(buf),
             "{\"ppm_lr\":%u,\"ppm_ud\":%u,\"ch_lr\":%u,\"ch_ud\":%u,\"yaw\":%.2f,\"pitch\":%.2f}",
             g_kk_ht.ppm_lr, g_kk_ht.ppm_ud, clr, cud, g_kk_ht.yaw_f, g_kk_ht.pitch_f);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t config_get(httpd_req_t *req)
{
    kk_rx_web_touch();
    char buf[420];
    kk_rx_web_json_config(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static int kk_web_arg_int(const char *body, const char *key, int def)
{
    char search[32];
    snprintf(search, sizeof(search), "%s=", key);
    const char *p = strstr(body, search);
    if (!p) {
        return def;
    }
    p += strlen(search);
    return atoi(p);
}

static esp_err_t save_post(httpd_req_t *req)
{
    kk_rx_web_touch();
    char body[512];
    int n = httpd_req_recv(req, body, sizeof(body) - 1);
    if (n <= 0) {
        ESP_LOGW(TAG, "[WEB] save recv fail");
        return httpd_resp_send(req, "{\"ok\":false}", HTTPD_RESP_USE_STRLEN);
    }
    body[n] = '\0';

    if (!s_profile) {
        return httpd_resp_send(req, "{\"ok\":false}", HTTPD_RESP_USE_STRLEN);
    }

    kk_rx_profile_t p = *s_profile;
    p.ch_lr = (uint8_t)kk_web_arg_int(body, "ch_lr", p.ch_lr);
    p.ch_ud = (uint8_t)kk_web_arg_int(body, "ch_ud", p.ch_ud);
    p.offset_lr = (int16_t)kk_web_arg_int(body, "offset_lr", p.offset_lr);
    p.offset_ud = (int16_t)kk_web_arg_int(body, "offset_ud", p.offset_ud);
    p.scale_lr = (uint8_t)kk_web_arg_int(body, "scale_lr", p.scale_lr);
    p.jitter_x10 = (uint8_t)kk_web_arg_int(body, "jitter_x10", p.jitter_x10);
    p.yaw_servo_deg = (uint16_t)kk_rx_sanitize_yaw_servo_deg(
        (uint16_t)kk_web_arg_int(body, "yaw_servo_deg", (int)p.yaw_servo_deg));
    p.rev_lr = kk_web_arg_int(body, "rev_lr", 0) != 0;
    p.rev_ud = kk_web_arg_int(body, "rev_ud", 0) != 0;
    p.mount_horiz = kk_imu_mount_deg_to_steps((uint16_t)kk_web_arg_int(body, "mount_horiz", 0));
    p.mount_lr = kk_imu_mount_deg_to_steps((uint16_t)kk_web_arg_int(body, "mount_lr", 0));
    p.mount_fb = kk_imu_mount_deg_to_steps((uint16_t)kk_web_arg_int(body, "mount_fb", 0));
    p.gest_roll_deg = (uint8_t)kk_web_arg_int(body, "gest_roll_deg", (int)p.gest_roll_deg);
    p.gest_swing_ms = (uint16_t)kk_web_arg_int(body, "gest_swing_ms", (int)p.gest_swing_ms);
    kk_rx_profile_sanitize(&p);
    *s_profile = p;
    kk_rx_profile_save(s_profile);
    kk_head_track_apply(s_profile);
    if (s_mount_sync) {
        kk_imu_mount_t m;
        kk_rx_profile_mount_to_imu(s_profile, &m);
        s_mount_sync(&m);
    }
    if (s_gesture_sync) {
        kk_gesture_cfg_t g;
        kk_rx_profile_gesture_to_cfg(s_profile, &g);
        s_gesture_sync(&g);
    }

    ESP_LOGW(TAG, "[WEB] save ch_lr=%u ch_ud=%u yaw_srv=%u off_lr=%d off_ud=%d scale=%u jit=%u rev=%u/%u gest=%u/%u",
             p.ch_lr, p.ch_ud, p.yaw_servo_deg, (int)p.offset_lr, (int)p.offset_ud,
             p.scale_lr, p.jitter_x10, p.rev_lr ? 1U : 0U, p.rev_ud ? 1U : 0U,
             p.gest_roll_deg, p.gest_swing_ms);

    char cfg[360];
    char out[480];
    kk_rx_web_json_config(cfg, sizeof(cfg));
    snprintf(out, sizeof(out), "{\"ok\":true,\"cfg\":%s}", cfg);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t reset_post(httpd_req_t *req)
{
    kk_rx_web_touch();
    if (!s_profile) {
        return httpd_resp_send(req, "{\"ok\":false}", HTTPD_RESP_USE_STRLEN);
    }
    kk_rx_profile_reset(s_profile);
    kk_head_track_apply(s_profile);
    if (s_mount_sync) {
        kk_imu_mount_t m;
        kk_rx_profile_mount_to_imu(s_profile, &m);
        s_mount_sync(&m);
    }
    if (s_gesture_sync) {
        kk_gesture_cfg_t g;
        kk_rx_profile_gesture_to_cfg(s_profile, &g);
        s_gesture_sync(&g);
    }
    ESP_LOGW(TAG, "[WEB] reset defaults");
    char cfg[360];
    char out[480];
    kk_rx_web_json_config(cfg, sizeof(cfg));
    snprintf(out, sizeof(out), "{\"ok\":true,\"cfg\":%s}", cfg);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

void kk_rx_web_set_mount_sync(kk_rx_web_mount_sync_cb_t cb)
{
    s_mount_sync = cb;
}

void kk_rx_web_set_gesture_sync(kk_rx_web_gesture_sync_cb_t cb)
{
    s_gesture_sync = cb;
}

void kk_rx_web_begin(kk_rx_profile_t *profile)
{
    s_profile = profile;
    s_last_http_ms = 0;
    if (s_server) {
        return;
    }
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 10;
    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        s_server = NULL;
        return;
    }
    httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = root_get},
        {.uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_get},
        {.uri = "/api/config", .method = HTTP_GET, .handler = config_get},
        {.uri = "/api/save", .method = HTTP_POST, .handler = save_post},
        {.uri = "/api/reset", .method = HTTP_POST, .handler = reset_post},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_server, &uris[i]);
    }
}

void kk_rx_web_handle(void)
{
    (void)s_server;
}

void kk_rx_web_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
}
