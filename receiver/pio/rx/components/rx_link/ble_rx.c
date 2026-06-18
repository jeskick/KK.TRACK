#include "ble_rx.h"

#include "kk/gesture_cfg.h"
#include "kk/tx_track_cfg.h"
#include "kk/imu_mount.h"
#include "kk/link_config.h"
#include "kk/repair.h"
#include "kk/storage.h"
#include "kk/telemetry.h"
#include "kk/time.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "host/ble_sm.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "kk.ble.rx";

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_link_val_handle;
static bool s_connected;
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

static portMUX_TYPE s_pending_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_pending_flags;
static char s_pending_peer_mac[24];

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

    portENTER_CRITICAL(&s_pending_mux);
    flags = s_pending_flags;
    s_pending_flags = 0;
    strncpy(peer, s_pending_peer_mac, sizeof(peer) - 1);
    peer[sizeof(peer) - 1] = '\0';
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
        if (s_on_telemetry) {
            s_on_telemetry();
        }
    }
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
        if (kk_repair_cmd_match((const uint8_t *)buf, len)) {
            kk_ble_rx_flag(KK_BLE_RX_F_REPAIR);
            return 0;
        }
        if (kk_center_cmd_match((const uint8_t *)buf, len)) {
            kk_ble_rx_flag(KK_BLE_RX_F_CENTER);
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
        ESP_LOGI(TAG, "disconnected");
        kk_ble_rx_flag(KK_BLE_RX_F_DISCONNECT);
        kk_ble_rx_flag(KK_BLE_RX_F_ADV_RESTART);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        kk_ble_rx_flag(KK_BLE_RX_F_ADV_RESTART);
        return 0;

    default:
        return 0;
    }
}

void kk_ble_rx_init(void)
{
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = kk_ble_rx_on_sync;
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

void kk_ble_rx_send_mount(const kk_imu_mount_t *mount)
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

void kk_ble_rx_send_gesture(const kk_gesture_cfg_t *cfg)
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

void kk_ble_rx_send_track(const kk_tx_track_cfg_t *cfg)
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

void kk_ble_rx_set_on_connect(kk_ble_rx_event_cb_t cb) { s_on_connect = cb; }
void kk_ble_rx_set_on_disconnect(kk_ble_rx_event_cb_t cb) { s_on_disconnect = cb; }
void kk_ble_rx_set_on_repair_peer(kk_ble_rx_event_cb_t cb) { s_on_repair_peer = cb; }
void kk_ble_rx_set_on_center_peer(kk_ble_rx_event_cb_t cb) { s_on_center_peer = cb; }
void kk_ble_rx_set_on_telemetry(kk_ble_rx_event_cb_t cb) { s_on_telemetry = cb; }
void kk_ble_rx_set_on_ready(kk_ble_rx_event_cb_t cb) { s_on_ready = cb; }
