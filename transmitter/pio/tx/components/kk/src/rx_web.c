#include "kk/rx_web.h"
#include "kk/rx_web_page.h"
#include "kk/head_track.h"
#include "kk/imu_mount.h"
#include "kk/gesture_cfg.h"
#include "kk/rx_profile.h"
#include "kk/rx_ota.h"
#include "kk/tx_track_cfg.h"
#include "kk/telemetry.h"
#include "kk/link_config.h"
#include "kk/fw_version.h"

#include "kk/time.h"

#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "kk.web";

static httpd_handle_t s_server;
static kk_rx_profile_t *s_profile;
static uint32_t s_last_http_ms;
static kk_rx_web_mount_sync_cb_t s_mount_sync;
static kk_rx_web_gesture_sync_cb_t s_gesture_sync;
static kk_rx_web_track_sync_cb_t s_track_sync;
static kk_rx_web_saved_cb_t s_on_saved;
static kk_rx_web_ota_prepare_cb_t s_ota_prepare;
static volatile bool s_ota_http_slot;

void kk_rx_web_touch(void)
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
             "\"scale_lr\":%u,\"scale_ud\":%u,\"yaw_servo_deg\":%u,\"rev_lr\":%u,\"rev_ud\":%u,"
             "\"mount_horiz\":%u,\"mount_lr\":%u,\"mount_fb\":%u,"
             "\"gest_roll_deg\":%u,\"gest_swing_ms\":%u,"
             "\"track_decouple_en\":%u,\"track_motion_en\":%u,"
             "\"track_decouple_str_x100\":%u,\"track_decouple_dom_x10\":%u}",
             s_profile->ch_lr, s_profile->ch_ud, s_profile->offset_lr, s_profile->offset_ud,
             s_profile->scale_lr, s_profile->scale_ud, s_profile->yaw_servo_deg,
             s_profile->rev_lr ? 1U : 0U, s_profile->rev_ud ? 1U : 0U,
             kk_imu_mount_steps_to_deg(s_profile->mount_horiz),
             kk_imu_mount_steps_to_deg(s_profile->mount_lr),
             kk_imu_mount_steps_to_deg(s_profile->mount_fb),
             s_profile->gest_roll_deg, s_profile->gest_swing_ms,
             s_profile->track_decouple_en ? 1U : 0U, s_profile->track_motion_en ? 1U : 0U,
             s_profile->track_decouple_str_x100, s_profile->track_decouple_dom_x10);
}

static esp_err_t root_get(httpd_req_t *req)
{
    kk_rx_web_touch();
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
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
    char buf[400];
    uint8_t clr = s_profile ? s_profile->ch_lr : 6;
    uint8_t cud = s_profile ? s_profile->ch_ud : 7;
    uint8_t trk_str = s_profile ? s_profile->track_decouple_str_x100 : KK_TRACK_DEC_STR_DEFAULT;
    uint8_t trk_dom = s_profile ? s_profile->track_decouple_dom_x10 : KK_TRACK_DEC_DOM_DEFAULT;
    uint8_t trk_dec = s_profile && s_profile->track_decouple_en ? 1U : 0U;
    uint8_t trk_mob = s_profile && s_profile->track_motion_en ? 1U : 0U;
    snprintf(buf, sizeof(buf),
             "{\"ppm_lr\":%u,\"ppm_ud\":%u,\"ch_lr\":%u,\"ch_ud\":%u,\"yaw\":%.2f,\"pitch\":%.2f,"
             "\"track_decouple_en\":%u,\"track_motion_en\":%u,"
             "\"track_decouple_str_x100\":%u,\"track_decouple_dom_x10\":%u,"
             "\"rx_ver\":\"%s\",\"tx_ver\":\"%s\",\"ota_max\":%u,\"ota_ready\":%u}",
             g_kk_ht.ppm_lr, g_kk_ht.ppm_ud, clr, cud, g_kk_ht.yaw_f, g_kk_ht.pitch_f, trk_dec,
             trk_mob, trk_str, trk_dom, kk_fw_local_version(), kk_fw_tx_version(),
             (unsigned)kk_rx_ota_max_image_bytes(),
             kk_rx_ota_max_image_bytes() > 0U ? 1U : 0U);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static void kk_rx_web_ota_json(char *buf, size_t cap)
{
    const kk_ota_status_t *st = kk_rx_ota_status();
    const char *target = "idle";
    if (st->phase == KK_OTA_RX_LOCAL) {
        target = "rx";
    } else if (st->phase == KK_OTA_TX_RELAY) {
        target = "tx";
    }
    snprintf(buf, cap,
             "{\"active\":%u,\"target\":\"%s\",\"pct\":%u,\"tx_pct\":%u,"
             "\"written\":%u,\"total\":%u,\"err\":%d,\"msg\":\"%s\"}",
             kk_rx_ota_is_active() ? 1U : 0U, target, st->pct, st->tx_pct,
             (unsigned)st->written, (unsigned)st->total, st->err, st->msg);
}

static esp_err_t ota_status_get(httpd_req_t *req)
{
    kk_rx_web_touch();
    char buf[256];
    kk_rx_web_ota_json(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ota_stream_run(httpd_req_t *req, bool target_tx, char *out, size_t out_cap)
{
    kk_rx_web_touch();
    if (kk_rx_ota_is_active()) {
        snprintf(out, out_cap, "{\"ok\":false,\"err\":\"busy\"}");
        return ESP_OK;
    }

    const size_t total = req->content_len;
    const size_t max_bytes = target_tx ? (size_t)KK_OTA_TX_IMAGE_MAX : kk_rx_ota_max_image_bytes();
    if (total == 0 || total > max_bytes) {
        snprintf(out, out_cap, "{\"ok\":false,\"err\":\"size\",\"max\":%u}",
                 (unsigned)max_bytes);
        return ESP_OK;
    }

    if (s_ota_prepare) {
        s_ota_prepare();
    }

    esp_err_t err = target_tx ? kk_rx_ota_tx_begin(total) : kk_rx_ota_local_begin(total);
    if (err != ESP_OK) {
        const kk_ota_status_t *st = kk_rx_ota_status();
        snprintf(out, out_cap, "{\"ok\":false,\"err\":\"begin\",\"code\":%d,\"msg\":\"%s\"}",
                 (int)err, st && st->msg[0] ? st->msg : "begin");
        return ESP_OK;
    }

    uint8_t raw[target_tx ? KK_OTA_TX_HTTP_BUF : KK_OTA_HTTP_BUF];
    size_t got = 0;
    const uint32_t relay_t0 = target_tx ? kk_millis() : 0;
    while (got < total) {
        if (target_tx && kk_millis() - relay_t0 > KK_OTA_TX_RELAY_TOTAL_MS) {
            kk_rx_ota_tx_abort();
            snprintf(out, out_cap, "{\"ok\":false,\"err\":\"timeout\",\"msg\":\"tx relay\"}");
            return ESP_OK;
        }
        const size_t want = total - got;
        const size_t chunk = want > sizeof(raw) ? sizeof(raw) : want;
        const int n = httpd_req_recv(req, (char *)raw, chunk);
        if (n <= 0) {
            if (target_tx) {
                kk_rx_ota_tx_abort();
            } else {
                kk_rx_ota_local_abort();
            }
            snprintf(out, out_cap, "{\"ok\":false,\"err\":\"recv\"}");
            return ESP_OK;
        }
        kk_rx_web_touch();
        err = target_tx ? kk_rx_ota_tx_write(raw, (size_t)n) : kk_rx_ota_local_write(raw, (size_t)n);
        if (err != ESP_OK) {
            if (target_tx) {
                kk_rx_ota_tx_abort();
            } else {
                kk_rx_ota_local_abort();
            }
            snprintf(out, out_cap, "{\"ok\":false,\"err\":\"write\",\"code\":%d}", (int)err);
            return ESP_OK;
        }
        got += (size_t)n;
    }

    err = target_tx ? kk_rx_ota_tx_finish() : kk_rx_ota_local_finish();
    if (err != ESP_OK) {
        snprintf(out, out_cap, "{\"ok\":false,\"err\":\"finish\",\"code\":%d}", (int)err);
        return ESP_OK;
    }
    snprintf(out, out_cap, "{\"ok\":true}");
    return ESP_OK;
}

typedef struct {
    httpd_req_t *req;
    bool target_tx;
    char out[160];
} ota_http_job_t;

static void ota_http_task(void *arg)
{
    ota_http_job_t *job = (ota_http_job_t *)arg;
    (void)ota_stream_run(job->req, job->target_tx, job->out, sizeof(job->out));
    httpd_resp_set_type(job->req, "application/json");
    httpd_resp_send(job->req, job->out, HTTPD_RESP_USE_STRLEN);
    httpd_req_async_handler_complete(job->req);
    s_ota_http_slot = false;
    free(job);
    vTaskDelete(NULL);
}

static esp_err_t ota_stream_post(httpd_req_t *req, bool target_tx)
{
    if (s_ota_http_slot || kk_rx_ota_is_active()) {
        return httpd_resp_send(req, "{\"ok\":false,\"err\":\"busy\"}", HTTPD_RESP_USE_STRLEN);
    }
    s_ota_http_slot = true;

    ota_http_job_t *job = (ota_http_job_t *)calloc(1, sizeof(*job));
    if (!job) {
        s_ota_http_slot = false;
        return httpd_resp_send(req, "{\"ok\":false,\"err\":\"nomem\"}", HTTPD_RESP_USE_STRLEN);
    }
    job->target_tx = target_tx;

    esp_err_t err = httpd_req_async_handler_begin(req, &job->req);
    if (err != ESP_OK) {
        s_ota_http_slot = false;
        free(job);
        return httpd_resp_send(req, "{\"ok\":false,\"err\":\"async\"}", HTTPD_RESP_USE_STRLEN);
    }

    if (xTaskCreate(ota_http_task, "ota_http", 16384, job, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
        s_ota_http_slot = false;
        httpd_req_async_handler_complete(job->req);
        free(job);
        return httpd_resp_send(req, "{\"ok\":false,\"err\":\"nomem\"}", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t ota_rx_post(httpd_req_t *req)
{
    return ota_stream_post(req, false);
}

static esp_err_t ota_tx_post(httpd_req_t *req)
{
    return ota_stream_post(req, true);
}

static esp_err_t config_get(httpd_req_t *req)
{
    kk_rx_web_touch();
    char buf[520];
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

static bool kk_web_read_body(httpd_req_t *req, char *body, size_t cap)
{
    if (!body || cap < 8) {
        return false;
    }

    size_t got = 0;
    const size_t want = req->content_len;

    if (want > 0) {
        while (got < want && got < cap - 1) {
            const int n = httpd_req_recv(req, body + got, (cap - 1) - got);
            if (n <= 0) {
                ESP_LOGW(TAG, "[WEB] recv err want=%u got=%u", (unsigned)want, (unsigned)got);
                return false;
            }
            got += (size_t)n;
        }
    } else {
        const int n = httpd_req_recv(req, body, cap - 1);
        if (n <= 0) {
            return false;
        }
        got = (size_t)n;
    }

    body[got] = '\0';
    return got > 0;
}

static esp_err_t save_post(httpd_req_t *req)
{
    kk_rx_web_touch();
    if (kk_rx_ota_is_active()) {
        return httpd_resp_send(req, "{\"ok\":false,\"err\":\"ota\"}", HTTPD_RESP_USE_STRLEN);
    }
    char body[512];
    if (!kk_web_read_body(req, body, sizeof(body))) {
        ESP_LOGW(TAG, "[WEB] save recv fail len=%d", (int)req->content_len);
        return httpd_resp_send(req, "{\"ok\":false}", HTTPD_RESP_USE_STRLEN);
    }

    if (!s_profile) {
        return httpd_resp_send(req, "{\"ok\":false}", HTTPD_RESP_USE_STRLEN);
    }

    kk_rx_profile_t p = *s_profile;
    p.ch_lr = (uint8_t)kk_web_arg_int(body, "ch_lr", p.ch_lr);
    p.ch_ud = (uint8_t)kk_web_arg_int(body, "ch_ud", p.ch_ud);
    p.offset_lr = (int16_t)kk_web_arg_int(body, "offset_lr", p.offset_lr);
    p.offset_ud = (int16_t)kk_web_arg_int(body, "offset_ud", p.offset_ud);
    p.scale_lr = (uint8_t)kk_web_arg_int(body, "scale_lr", p.scale_lr);
    p.scale_ud = (uint8_t)kk_web_arg_int(body, "scale_ud", p.scale_ud);
    p.yaw_servo_deg = (uint16_t)kk_rx_sanitize_yaw_servo_deg(
        (uint16_t)kk_web_arg_int(body, "yaw_servo_deg", (int)p.yaw_servo_deg));
    p.rev_lr = kk_web_arg_int(body, "rev_lr", 0) != 0;
    p.rev_ud = kk_web_arg_int(body, "rev_ud", 0) != 0;
    p.mount_horiz = kk_imu_mount_deg_to_steps((uint16_t)kk_web_arg_int(body, "mount_horiz", 0));
    p.mount_lr = kk_imu_mount_deg_to_steps((uint16_t)kk_web_arg_int(body, "mount_lr", 0));
    p.mount_fb = kk_imu_mount_deg_to_steps((uint16_t)kk_web_arg_int(body, "mount_fb", 0));
    p.gest_roll_deg = (uint8_t)kk_web_arg_int(body, "gest_roll_deg", (int)p.gest_roll_deg);
    p.gest_swing_ms = (uint16_t)kk_web_arg_int(body, "gest_swing_ms", (int)p.gest_swing_ms);
    p.gest_center_en = true;
    p.track_decouple_en = kk_web_arg_int(body, "track_decouple_en", p.track_decouple_en ? 1 : 0) != 0;
    p.track_motion_en = kk_web_arg_int(body, "track_motion_en", p.track_motion_en ? 1 : 0) != 0;
    p.track_decouple_str_x100 =
        (uint8_t)kk_web_arg_int(body, "track_decouple_str_x100", (int)p.track_decouple_str_x100);
    p.track_decouple_dom_x10 =
        (uint8_t)kk_web_arg_int(body, "track_decouple_dom_x10", (int)p.track_decouple_dom_x10);
    kk_rx_profile_sanitize(&p);
    *s_profile = p;
    kk_rx_profile_save(s_profile);
    kk_head_track_snap_telemetry();
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
    if (s_track_sync) {
        kk_tx_track_cfg_t t;
        kk_rx_profile_track_to_cfg(s_profile, &t);
        s_track_sync(&t);
    }
    if (s_on_saved) {
        s_on_saved();
    }

    ESP_LOGW(TAG, "[WEB] save ch_lr=%u ch_ud=%u yaw_srv=%u off_lr=%d off_ud=%d scale=%u/%u rev=%u/%u gest=%u/%u trk=%u/%u str=%u dom=%u",
             p.ch_lr, p.ch_ud, p.yaw_servo_deg, (int)p.offset_lr, (int)p.offset_ud,
             p.scale_lr, p.scale_ud, p.rev_lr ? 1U : 0U, p.rev_ud ? 1U : 0U,
             p.gest_roll_deg, p.gest_swing_ms,
             p.track_decouple_en ? 1U : 0U, p.track_motion_en ? 1U : 0U,
             p.track_decouple_str_x100, p.track_decouple_dom_x10);

    char cfg[480];
    char out[520];
    kk_rx_web_json_config(cfg, sizeof(cfg));
    snprintf(out, sizeof(out), "{\"ok\":true,\"cfg\":%s}", cfg);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t reset_post(httpd_req_t *req)
{
    kk_rx_web_touch();
    if (kk_rx_ota_is_active()) {
        return httpd_resp_send(req, "{\"ok\":false,\"err\":\"ota\"}", HTTPD_RESP_USE_STRLEN);
    }
    if (!s_profile) {
        return httpd_resp_send(req, "{\"ok\":false}", HTTPD_RESP_USE_STRLEN);
    }
    kk_rx_profile_reset(s_profile);
    kk_head_track_snap_telemetry();
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
    if (s_track_sync) {
        kk_tx_track_cfg_t t;
        kk_rx_profile_track_to_cfg(s_profile, &t);
        s_track_sync(&t);
    }
    if (s_on_saved) {
        s_on_saved();
    }
    ESP_LOGW(TAG, "[WEB] reset defaults");
    char cfg[480];
    char out[520];
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
    cfg.max_uri_handlers = 12;
    cfg.max_open_sockets = 4;
    cfg.stack_size = 8192;
    cfg.recv_wait_timeout = 120;
    cfg.send_wait_timeout = 30;
    cfg.lru_purge_enable = true;
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
        {.uri = "/api/ota/status", .method = HTTP_GET, .handler = ota_status_get},
        {.uri = "/api/ota/rx", .method = HTTP_POST, .handler = ota_rx_post},
        {.uri = "/api/ota/tx", .method = HTTP_POST, .handler = ota_tx_post},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_server, &uris[i]);
    }
}

void kk_rx_web_set_track_sync(kk_rx_web_track_sync_cb_t cb)
{
    s_track_sync = cb;
}

void kk_rx_web_set_on_saved(kk_rx_web_saved_cb_t cb)
{
    s_on_saved = cb;
}

void kk_rx_web_set_ota_prepare(kk_rx_web_ota_prepare_cb_t cb)
{
    s_ota_prepare = cb;
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
