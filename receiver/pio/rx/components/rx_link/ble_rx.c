#include "ble_rx.h"

#include "kk/gesture_cfg.h"
#include "kk/tx_track_cfg.h"
#include "kk/imu_mount.h"
#include "kk/link_config.h"
#include "kk/repair.h"
#include "kk/storage.h"
#include "kk/telemetry.h"
#include "kk/fw_version.h"
#include "kk/rx_web.h"
#include "kk/time.h"
#include "kk/rx_ota.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_att.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "host/ble_sm.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "kk.ble.rx";

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_link_val_handle;
static uint16_t s_ota_val_handle;
static bool s_connected;
static volatile bool s_tx_ota_rdy;
static volatile bool s_tx_ota_err;
static volatile bool s_tx_ota_done;
static volatile bool s_tx_ota_fail;
static uint16_t s_conn_mtu = 23;
static SemaphoreHandle_t s_ota_relay_req;
static SemaphoreHandle_t s_ota_relay_done;
static SemaphoreHandle_t s_ota_begin_done;
static SemaphoreHandle_t s_ota_finish_done;
static uint8_t s_ota_relay_buf[KK_OTA_TX_HTTP_BUF];
static size_t s_ota_relay_len;
static size_t s_ota_relay_off;
static bool s_ota_relay_busy;
static esp_err_t s_ota_relay_err;
static bool s_ota_begin_active;
static bool s_ota_begin_started;
static uint32_t s_ota_begin_deadline;
static esp_err_t s_ota_begin_err;
static size_t s_ota_begin_size;
static bool s_ota_finish_active;
static uint32_t s_ota_finish_sent_ms;
static esp_err_t s_ota_finish_err;
static kk_ble_rx_event_cb_t s_on_connect;
static kk_ble_rx_event_cb_t s_on_disconnect;
static kk_ble_rx_event_cb_t s_on_repair_peer;
static kk_ble_rx_event_cb_t s_on_center_peer;
static kk_ble_rx_event_cb_t s_on_ready;
static kk_ble_rx_event_cb_t s_on_telemetry;

#define KK_BLE_RX_F_REPAIR      (1u << 0)
#define KK_BLE_RX_F_CENTER      (1u << 1)
#define KK_BLE_RX_F_CONNECT     (1u << 2)
#define KK_BLE_RX_F_DISCONNECT  (1u << 3)
#define KK_BLE_RX_F_SAVE_PEER   (1u << 4)
#define KK_BLE_RX_F_ADV_RESTART (1u << 5)
#define KK_BLE_RX_F_TEL         (1u << 6)
#define KK_BLE_RX_F_MOUNT       (1u << 7)
#define KK_BLE_RX_F_GESTURE     (1u << 8)
#define KK_BLE_RX_F_TRACK       (1u << 9)

static portMUX_TYPE s_pending_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_pending_flags;
static char s_pending_peer_mac[24];
/* 配置同步缓存：网页(httpd 任务)只缓存+置标志，实际 BLE notify 在主循环 poll 发 */
static kk_imu_mount_t s_pending_mount;
static kk_gesture_cfg_t s_pending_gesture;
static kk_tx_track_cfg_t s_pending_track;

static esp_err_t kk_ble_rx_ota_send_one_chunk(void);
static bool kk_ble_rx_notify_link(const char *msg);
static void kk_ble_rx_send_mount_now(const kk_imu_mount_t *mount);
static void kk_ble_rx_send_gesture_now(const kk_gesture_cfg_t *cfg);
static void kk_ble_rx_send_track_now(const kk_tx_track_cfg_t *cfg);

static void kk_ble_rx_ota_poll_tasks(void)
{
    if (s_ota_begin_active) {
        kk_rx_web_touch();
        if (!s_ota_begin_started) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "OTA,START,%u", (unsigned)s_ota_begin_size);
            if (!kk_ble_rx_notify_link(cmd)) {
                s_ota_begin_active = false;
                s_ota_begin_err = ESP_FAIL;
                xSemaphoreGive(s_ota_begin_done);
            } else {
                s_ota_begin_started = true;
                ESP_LOGW(TAG, "TX OTA START sent size=%u", (unsigned)s_ota_begin_size);
            }
        } else if (s_tx_ota_rdy) {
            s_ota_begin_active = false;
            s_ota_begin_err = ESP_OK;
            ESP_LOGW(TAG, "TX OTA ready size=%u", (unsigned)s_ota_begin_size);
            xSemaphoreGive(s_ota_begin_done);
        } else if (s_tx_ota_err) {
            s_ota_begin_active = false;
            s_ota_begin_err = ESP_FAIL;
            ESP_LOGW(TAG, "TX OTA begin rejected");
            xSemaphoreGive(s_ota_begin_done);
        } else if (kk_millis() >= s_ota_begin_deadline) {
            kk_ble_rx_notify_link("OTA,ABORT");
            s_ota_begin_active = false;
            s_ota_begin_err = ESP_ERR_TIMEOUT;
            ESP_LOGW(TAG, "TX OTA ready timeout");
            xSemaphoreGive(s_ota_begin_done);
        }
    }

    if (s_ota_relay_req && xSemaphoreTake(s_ota_relay_req, 0) == pdTRUE) {
        s_ota_relay_busy = true;
        s_ota_relay_off = 0;
        s_ota_relay_err = ESP_OK;
    }

    if (s_ota_relay_busy) {
        kk_rx_web_touch();
        if (s_tx_ota_fail) {
            s_ota_relay_busy = false;
            s_ota_relay_err = ESP_FAIL;
            ESP_LOGW(TAG, "TX OTA FAIL -> stop relay");
            xSemaphoreGive(s_ota_relay_done);
        } else {
            esp_err_t err = kk_ble_rx_ota_send_one_chunk();
            if (err != ESP_OK) {
                s_ota_relay_busy = false;
                s_ota_relay_err = err;
                xSemaphoreGive(s_ota_relay_done);
            } else if (s_ota_relay_off >= s_ota_relay_len) {
                s_ota_relay_busy = false;
                s_ota_relay_err = ESP_OK;
                xSemaphoreGive(s_ota_relay_done);
            }
        }
    }

    if (s_ota_finish_active) {
        kk_rx_web_touch();
        if (s_ota_finish_sent_ms == 0) {
            if (kk_ble_rx_notify_link("OTA,END")) {
                s_ota_finish_sent_ms = kk_millis();
                ESP_LOGW(TAG, "TX OTA END sent");
            } else {
                s_ota_finish_active = false;
                s_ota_finish_err = ESP_FAIL;
                xSemaphoreGive(s_ota_finish_done);
            }
        } else if (s_tx_ota_done) {
            s_ota_finish_active = false;
            s_ota_finish_err = ESP_OK;
            ESP_LOGW(TAG, "TX OTA DONE ack");
            xSemaphoreGive(s_ota_finish_done);
        } else if (s_tx_ota_fail) {
            s_ota_finish_active = false;
            s_ota_finish_err = ESP_FAIL;
            ESP_LOGW(TAG, "TX OTA FAIL ack");
            xSemaphoreGive(s_ota_finish_done);
        } else if (!kk_ble_rx_is_connected() && s_ota_finish_sent_ms != 0) {
            s_ota_finish_active = false;
            s_ota_finish_err = ESP_FAIL;
            ESP_LOGW(TAG, "TX OTA disconnect without DONE");
            xSemaphoreGive(s_ota_finish_done);
        } else if (kk_millis() - s_ota_finish_sent_ms >= KK_OTA_TX_FINISH_MS) {
            s_ota_finish_active = false;
            s_ota_finish_err = ESP_ERR_TIMEOUT;
            ESP_LOGW(TAG, "TX OTA finish timeout");
            kk_ble_rx_notify_link("OTA,ABORT");
            xSemaphoreGive(s_ota_finish_done);
        }
    }
}

static void kk_ble_rx_flag(uint32_t bit)
{
    portENTER_CRITICAL(&s_pending_mux);
    s_pending_flags |= bit;
    portEXIT_CRITICAL(&s_pending_mux);
}

void kk_ble_rx_poll(void)
{
    uint32_t flags;
    char peer[24];
    kk_imu_mount_t mount;
    kk_gesture_cfg_t gesture;
    kk_tx_track_cfg_t track;

    portENTER_CRITICAL(&s_pending_mux);
    flags = s_pending_flags;
    s_pending_flags = 0;
    strncpy(peer, s_pending_peer_mac, sizeof(peer) - 1);
    peer[sizeof(peer) - 1] = '\0';
    mount = s_pending_mount;
    gesture = s_pending_gesture;
    track = s_pending_track;
    portEXIT_CRITICAL(&s_pending_mux);

    if (flags & KK_BLE_RX_F_DISCONNECT) {
        if (s_on_disconnect) {
            s_on_disconnect();
        }
    }
    if (flags & KK_BLE_RX_F_ADV_RESTART) {
        kk_ble_rx_start_adv();
    }
    if (flags & KK_BLE_RX_F_CONNECT) {
        if (s_on_connect) {
            s_on_connect();
        }
    }
    if (flags & KK_BLE_RX_F_SAVE_PEER) {
        if (peer[0] != '\0') {
            kk_storage_save_peer_mac(peer);
            ESP_LOGW(TAG, "paired saved %s (deferred)", peer);
        }
    }
    if (flags & KK_BLE_RX_F_REPAIR) {
        if (s_on_repair_peer) {
            s_on_repair_peer();
        }
    }
    if (flags & KK_BLE_RX_F_CENTER) {
        if (s_on_center_peer) {
            s_on_center_peer();
        }
    }
    if (flags & KK_BLE_RX_F_TEL) {
        if (s_on_telemetry && !kk_rx_ota_is_active()) {
            s_on_telemetry();
        }
    }
    if (flags & KK_BLE_RX_F_MOUNT) {
        kk_ble_rx_send_mount_now(&mount);
    }
    if (flags & KK_BLE_RX_F_GESTURE) {
        kk_ble_rx_send_gesture_now(&gesture);
    }
    if (flags & KK_BLE_RX_F_TRACK) {
        kk_ble_rx_send_track_now(&track);
    }
    kk_ble_rx_ota_poll_tasks();
}

static bool kk_ble_rx_adv_visible(void)
{
    char mac[24];
    return !kk_storage_load_paired(mac, sizeof(mac));
}

static bool kk_ble_rx_peer_mac(uint16_t conn_handle, char *mac, size_t cap)
{
    struct ble_gap_conn_desc desc;
    if (!mac || cap < 18 || ble_gap_conn_find(conn_handle, &desc) != 0) {
        return false;
    }
    const ble_addr_t *peer = &desc.peer_ota_addr;
    snprintf(mac, cap, "%02X:%02X:%02X:%02X:%02X:%02X",
             peer->val[5], peer->val[4], peer->val[3],
             peer->val[2], peer->val[1], peer->val[0]);
    kk_mac_normalize(mac);
    return true;
}

static bool kk_ble_rx_accept_peer(uint16_t conn_handle)
{
    char peer[24];
    if (!kk_ble_rx_peer_mac(conn_handle, peer, sizeof(peer))) {
        return false;
    }

    char stored[24];
    const bool was_paired = kk_storage_load_paired(stored, sizeof(stored));
    if (was_paired && strcmp(peer, stored) != 0) {
        ESP_LOGW(TAG, "reject peer %s (bound %s)", peer, stored);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return false;
    }
    return true;
}

static int kk_ble_rx_gap_event(struct ble_gap_event *event, void *arg);
static int kk_ble_rx_gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(KK_BLE_SVC_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(KK_BLE_CH_LINK_UUID16),
                .access_cb = kk_ble_rx_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_link_val_handle,
            },
            {
                .uuid = BLE_UUID16_DECLARE(KK_BLE_CH_OTA_UUID16),
                .access_cb = kk_ble_rx_gatt_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_ota_val_handle,
            },
            {0},
        },
    },
    {0},
};

static void kk_ble_rx_register_gatt(void)
{
    /* 必须在 nimble_port_freertos_init 之前注册；sync_cb 里太晚，ble_gatts_start 已跑完 */
    ESP_ERROR_CHECK(ble_gatts_count_cfg(s_gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(s_gatt_svcs));
}

static void kk_ble_rx_on_sync(void)
{
    ble_svc_gap_device_name_set(KK_BLE_NAME_RX);
    kk_ble_rx_start_adv();
    if (s_on_ready) {
        s_on_ready();
    }
}

static void kk_ble_rx_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static int kk_ble_rx_gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        return os_mbuf_append(ctxt->om, "rx", 2) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        char buf[96];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= sizeof(buf)) {
            len = sizeof(buf) - 1;
        }
        os_mbuf_copydata(ctxt->om, 0, len, buf);
        buf[len] = '\0';
        if (kk_rx_ota_is_active()) {
            if (strncmp(buf, "OTA,RDY", 7) == 0) {
                if (s_ota_begin_active) {
                    s_tx_ota_rdy = true;
                }
            } else if (strncmp(buf, "OTA,ERR", 7) == 0) {
                s_tx_ota_err = true;
            } else if (strncmp(buf, "OTA,DONE", 8) == 0) {
                s_tx_ota_done = true;
            } else if (strncmp(buf, "OTA,FAIL", 8) == 0) {
                s_tx_ota_fail = true;
            } else if (strncmp(buf, "OTA,PRG,", 8) == 0) {
                unsigned pct = 0;
                if (sscanf(buf, "OTA,PRG,%u", &pct) == 1) {
                    kk_rx_ota_tx_remote_pct((uint8_t)pct);
                }
            }
            return 0;
        }
        if (kk_repair_cmd_match((const uint8_t *)buf, len)) {
            kk_ble_rx_flag(KK_BLE_RX_F_REPAIR);
            return 0;
        }
        if (kk_center_cmd_match((const uint8_t *)buf, len)) {
            kk_ble_rx_flag(KK_BLE_RX_F_CENTER);
            return 0;
        }
        if (strncmp(buf, "OTA,RDY", 7) == 0) {
            if (s_ota_begin_active) {
                s_tx_ota_rdy = true;
            }
            return 0;
        }
        if (strncmp(buf, "VER,", 4) == 0) {
            kk_fw_set_tx_version(buf + 4);
            return 0;
        }
        if (len > 0) {
            char stored[24];
            if (!kk_storage_load_paired(stored, sizeof(stored))) {
                char peer[24];
                if (kk_ble_rx_peer_mac(conn_handle, peer, sizeof(peer))) {
                    portENTER_CRITICAL(&s_pending_mux);
                    strncpy(s_pending_peer_mac, peer, sizeof(s_pending_peer_mac) - 1);
                    s_pending_peer_mac[sizeof(s_pending_peer_mac) - 1] = '\0';
                    s_pending_flags |= KK_BLE_RX_F_SAVE_PEER;
                    portEXIT_CRITICAL(&s_pending_mux);
                }
            }
            kk_tel_on_udp_payload(buf);
            kk_ble_rx_flag(KK_BLE_RX_F_TEL);
        }
        return 0;
    }
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

void kk_ble_rx_start_adv(void)
{
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};
    const bool visible = kk_ble_rx_adv_visible();
    ble_uuid16_t svc_uuid = BLE_UUID16_INIT(KK_BLE_SVC_UUID16);

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    if (visible) {
        fields.name = (const uint8_t *)KK_BLE_NAME_RX;
        fields.name_len = strlen(KK_BLE_NAME_RX);
        fields.name_is_complete = 1;
        fields.uuids16 = &svc_uuid;
        fields.num_uuids16 = 1;
        fields.uuids16_is_complete = 1;
    }

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGW(TAG, "adv fields rc=%d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params,
                           kk_ble_rx_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "adv start rc=%d", rc);
        return;
    }
    if (visible) {
        ESP_LOGW(TAG, "advertising %s", KK_BLE_NAME_RX);
    } else {
        ESP_LOGW(TAG, "advertising (hidden)");
    }
}

static int kk_ble_rx_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
            ble_gap_adv_stop();
            if (!kk_ble_rx_accept_peer(event->connect.conn_handle)) {
                kk_ble_rx_flag(KK_BLE_RX_F_ADV_RESTART);
                return 0;
            }
            ESP_LOGW(TAG, "connected");
            kk_ble_rx_flag(KK_BLE_RX_F_CONNECT);
        } else {
            kk_ble_rx_flag(KK_BLE_RX_F_ADV_RESTART);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_connected = false;
        s_conn_mtu = 23;
        kk_fw_clear_tx_version();
        ESP_LOGI(TAG, "disconnected");
        kk_ble_rx_flag(KK_BLE_RX_F_DISCONNECT);
        kk_ble_rx_flag(KK_BLE_RX_F_ADV_RESTART);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        kk_ble_rx_flag(KK_BLE_RX_F_ADV_RESTART);
        return 0;

    case BLE_GAP_EVENT_MTU:
        s_conn_mtu = event->mtu.value;
        ESP_LOGW(TAG, "MTU %u", (unsigned)s_conn_mtu);
        return 0;

    default:
        return 0;
    }
}

void kk_ble_rx_init(void)
{
    if (!s_ota_relay_req) {
        s_ota_relay_req = xSemaphoreCreateBinary();
        s_ota_relay_done = xSemaphoreCreateBinary();
        s_ota_begin_done = xSemaphoreCreateBinary();
        s_ota_finish_done = xSemaphoreCreateBinary();
    }
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sync_cb = kk_ble_rx_on_sync;
    ESP_ERROR_CHECK(nimble_port_init());
    ble_att_set_preferred_mtu(517);
    ble_svc_gap_init();
    ble_svc_gatt_init();
    kk_ble_rx_register_gatt();
    nimble_port_freertos_init(kk_ble_rx_host_task);
}

bool kk_ble_rx_is_connected(void)
{
    return s_connected;
}

void kk_ble_rx_disconnect_peer(bool notify_peer)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    if (notify_peer) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(KK_BLE_REPAIR_CMD, 6);
        if (om) {
            ble_gatts_notify_custom(s_conn_handle, s_link_val_handle, om);
            kk_delay_ms(80);
        }
    }
    ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    kk_delay_ms(50);
}

void kk_ble_rx_send_center(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_link_val_handle == 0) {
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(KK_BLE_CENTER_CMD, 6);
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_link_val_handle, om);
    }
}

static void kk_ble_rx_send_mount_now(const kk_imu_mount_t *mount)
{
    if (!mount || s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_link_val_handle == 0) {
        return;
    }
    static const char *deg_lut[4] = {"0", "90", "180", "270"};
    char buf[20];
    snprintf(buf, sizeof(buf), "MNT,%s,%s,%s",
             deg_lut[mount->rot_horiz & 3U],
             deg_lut[mount->rot_lr & 3U],
             deg_lut[mount->rot_fb & 3U]);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, strlen(buf));
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_link_val_handle, om);
        ESP_LOGW(TAG, "notify TX %s", buf);
    }
}

static void kk_ble_rx_send_gesture_now(const kk_gesture_cfg_t *cfg)
{
    if (!cfg || s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_link_val_handle == 0) {
        return;
    }
    char buf[20];
    snprintf(buf, sizeof(buf), "GES,%u,%u,%u", cfg->roll_deg, cfg->swing_ms,
             cfg->center_en ? 1U : 0U);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, strlen(buf));
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_link_val_handle, om);
        ESP_LOGW(TAG, "notify TX %s", buf);
    }
}

static void kk_ble_rx_send_track_now(const kk_tx_track_cfg_t *cfg)
{
    if (!cfg || s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_link_val_handle == 0) {
        return;
    }
    char buf[24];
    snprintf(buf, sizeof(buf), "TRK,%u,%u,%u,%u", cfg->decouple_en ? 1U : 0U, cfg->motion_en ? 1U : 0U,
             cfg->decouple_str_x100, cfg->decouple_dom_x10);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, strlen(buf));
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_link_val_handle, om);
        ESP_LOGW(TAG, "notify TX %s", buf);
    }
}

/* 公开 API：可能从 httpd 任务调用，故只缓存配置并置标志，
 * 真正的 ble_gatts_notify 延迟到主循环 kk_ble_rx_poll() 执行，避免跨任务冲突。 */
void kk_ble_rx_send_mount(const kk_imu_mount_t *mount)
{
    if (!mount) {
        return;
    }
    portENTER_CRITICAL(&s_pending_mux);
    s_pending_mount = *mount;
    s_pending_flags |= KK_BLE_RX_F_MOUNT;
    portEXIT_CRITICAL(&s_pending_mux);
}

void kk_ble_rx_send_gesture(const kk_gesture_cfg_t *cfg)
{
    if (!cfg) {
        return;
    }
    portENTER_CRITICAL(&s_pending_mux);
    s_pending_gesture = *cfg;
    s_pending_flags |= KK_BLE_RX_F_GESTURE;
    portEXIT_CRITICAL(&s_pending_mux);
}

void kk_ble_rx_send_track(const kk_tx_track_cfg_t *cfg)
{
    if (!cfg) {
        return;
    }
    portENTER_CRITICAL(&s_pending_mux);
    s_pending_track = *cfg;
    s_pending_flags |= KK_BLE_RX_F_TRACK;
    portEXIT_CRITICAL(&s_pending_mux);
}

static bool kk_ble_rx_notify_link(const char *msg)
{
    if (!msg || s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_link_val_handle == 0) {
        return false;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
    if (!om) {
        return false;
    }
    int rc = ble_gatts_notify_custom(s_conn_handle, s_link_val_handle, om);
    if (rc != 0) {
        os_mbuf_free_chain(om);
        return false;
    }
    return true;
}

esp_err_t kk_ble_rx_ota_tx_begin(size_t size)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_link_val_handle == 0 || s_ota_val_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_ota_begin_done) {
        return ESP_ERR_INVALID_STATE;
    }
    s_tx_ota_rdy = false;
    s_tx_ota_err = false;
    s_tx_ota_done = false;
    s_tx_ota_fail = false;
    s_ota_begin_size = size;
    s_ota_begin_started = false;
    s_ota_begin_err = ESP_ERR_TIMEOUT;
    s_ota_begin_deadline = kk_millis() + KK_OTA_TX_RDY_MS;
    s_ota_begin_active = true;
    return ESP_ERR_NOT_FINISHED;
}

esp_err_t kk_ble_rx_ota_tx_begin_poll(void)
{
    if (!s_ota_begin_active) {
        return s_ota_begin_err;
    }
    if (xSemaphoreTake(s_ota_begin_done, pdMS_TO_TICKS(50)) == pdTRUE) {
        return s_ota_begin_err;
    }
    return ESP_ERR_NOT_FINISHED;
}

static size_t kk_ble_ota_chunk_cap(void)
{
    uint16_t mtu = s_conn_mtu;
    if (mtu < 23U) {
        mtu = 23U;
    }
    size_t cap = (size_t)mtu - 3U;
    if (cap > KK_OTA_BLE_CHUNK_MAX) {
        cap = KK_OTA_BLE_CHUNK_MAX;
    }
    /* MTU 未协商完前保守分片，避免 notify 被拒 */
    if (mtu < 128U && cap > 20U) {
        cap = 20U;
    }
    if (cap < KK_OTA_BLE_CHUNK_MIN) {
        cap = KK_OTA_BLE_CHUNK_MIN;
    }
    return cap;
}

static esp_err_t kk_ble_rx_ota_notify_chunk(const uint8_t *data, size_t len)
{
    if (s_ota_val_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        return ESP_ERR_NO_MEM;
    }
    int rc = ble_gatts_notify_custom(s_conn_handle, s_ota_val_handle, om);
    if (rc == 0) {
        return ESP_OK;
    }
    os_mbuf_free_chain(om);
    ESP_LOGW(TAG, "OTA notify rc=%d len=%u mtu=%u", rc, (unsigned)len, (unsigned)s_conn_mtu);
    return ESP_FAIL;
}

static esp_err_t kk_ble_rx_ota_send_one_chunk(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_ota_val_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    const size_t max_chunk = kk_ble_ota_chunk_cap();
    size_t try_chunk = s_ota_relay_len - s_ota_relay_off;
    if (try_chunk > max_chunk) {
        try_chunk = max_chunk;
    }
    for (int attempt = 0; attempt < 4; attempt++) {
        esp_err_t err = kk_ble_rx_ota_notify_chunk(s_ota_relay_buf + s_ota_relay_off, try_chunk);
        if (err == ESP_OK) {
            s_ota_relay_off += try_chunk;
            vTaskDelay(pdMS_TO_TICKS(KK_OTA_BLE_PACE_MS));
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5U * (uint32_t)(attempt + 1U)));
        if (try_chunk > KK_OTA_BLE_CHUNK_MIN) {
            try_chunk /= 2U;
        }
    }
    ESP_LOGW(TAG, "OTA chunk fail off=%u", (unsigned)s_ota_relay_off);
    return ESP_FAIL;
}

esp_err_t kk_ble_rx_ota_tx_send(const uint8_t *data, size_t len)
{
    /* 在 main loop (kk_ble_rx_poll) 里发 BLE，避免 httpd 任务与 NimBLE 冲突 */
    if (!data || len == 0 || len > sizeof(s_ota_relay_buf)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ota_relay_req || !s_ota_relay_done) {
        return ESP_ERR_INVALID_STATE;
    }
    memcpy(s_ota_relay_buf, data, len);
    s_ota_relay_len = len;
    xSemaphoreGive(s_ota_relay_req);
    const uint32_t t0 = kk_millis();
    while (xSemaphoreTake(s_ota_relay_done, pdMS_TO_TICKS(200)) != pdTRUE) {
        kk_rx_web_touch();
        if (kk_millis() - t0 > KK_OTA_TX_CHUNK_WAIT_MS) {
            kk_ble_rx_ota_tx_cancel_wait();
            return ESP_ERR_TIMEOUT;
        }
    }
    return s_ota_relay_err;
}

esp_err_t kk_ble_rx_ota_tx_finish(void)
{
    if (!s_ota_finish_done) {
        return ESP_ERR_INVALID_STATE;
    }
    s_tx_ota_done = false;
    s_tx_ota_fail = false;
    s_ota_finish_err = ESP_ERR_TIMEOUT;
    s_ota_finish_active = true;
    s_ota_finish_sent_ms = 0;
    const uint32_t deadline = kk_millis() + KK_OTA_TX_FINISH_MS + 5000UL;
    while (s_ota_finish_active && kk_millis() < deadline) {
        kk_rx_web_touch();
        if (xSemaphoreTake(s_ota_finish_done, pdMS_TO_TICKS(200)) == pdTRUE) {
            return s_ota_finish_err;
        }
    }
    s_ota_finish_active = false;
    kk_ble_rx_notify_link("OTA,ABORT");
    return ESP_ERR_TIMEOUT;
}

void kk_ble_rx_ota_tx_abort(void)
{
    kk_ble_rx_notify_link("OTA,ABORT");
    s_tx_ota_rdy = false;
    s_tx_ota_err = false;
    s_tx_ota_done = false;
    s_tx_ota_fail = false;
}

void kk_ble_rx_ota_tx_cancel_wait(void)
{
    s_ota_begin_active = false;
    s_ota_finish_active = false;
    s_tx_ota_rdy = false;
    s_tx_ota_err = false;
    s_tx_ota_done = false;
    s_tx_ota_fail = false;
    if (s_ota_relay_busy && s_ota_relay_done) {
        s_ota_relay_busy = false;
        s_ota_relay_err = ESP_ERR_INVALID_STATE;
        xSemaphoreGive(s_ota_relay_done);
    }
    if (s_ota_begin_done) {
        xSemaphoreGive(s_ota_begin_done);
    }
    if (s_ota_finish_done) {
        s_ota_finish_err = ESP_ERR_INVALID_STATE;
        xSemaphoreGive(s_ota_finish_done);
    }
}

bool kk_ble_rx_ota_tx_ready(void)
{
    return s_connected && s_ota_val_handle != 0;
}

void kk_ble_rx_set_on_connect(kk_ble_rx_event_cb_t cb) { s_on_connect = cb; }
void kk_ble_rx_set_on_disconnect(kk_ble_rx_event_cb_t cb) { s_on_disconnect = cb; }
void kk_ble_rx_set_on_repair_peer(kk_ble_rx_event_cb_t cb) { s_on_repair_peer = cb; }
void kk_ble_rx_set_on_center_peer(kk_ble_rx_event_cb_t cb) { s_on_center_peer = cb; }
void kk_ble_rx_set_on_telemetry(kk_ble_rx_event_cb_t cb) { s_on_telemetry = cb; }
void kk_ble_rx_set_on_ready(kk_ble_rx_event_cb_t cb) { s_on_ready = cb; }
