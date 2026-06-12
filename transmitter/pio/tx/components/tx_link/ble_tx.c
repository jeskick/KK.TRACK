#include "ble_tx.h"
#include "imu_tx.h"
#include "tx_gesture.h"

#include "kk/gesture_cfg.h"
#include "kk/imu_mount.h"
#include "kk/link_config.h"
#include "kk/repair.h"
#include "kk/storage.h"
#include "kk/time.h"

#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_sm.h"
#include "services/gap/ble_svc_gap.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "kk.ble.tx";

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_link_val_handle;
static bool s_connected;
static bool s_gatt_ready;
static bool s_busy;
static ble_addr_t s_last_peer;
static bool s_have_peer;
static bool s_disc_complete;
static kk_ble_tx_event_cb_t s_on_disconnect;
static kk_ble_tx_event_cb_t s_on_repair_peer;
static kk_ble_tx_event_cb_t s_on_center_peer;
static bool s_host_ready;
static uint16_t s_disc_total;
static uint16_t s_disc_logged;

static int kk_ble_tx_gap_event(struct ble_gap_event *event, void *arg);
static int kk_ble_tx_on_disc(uint16_t conn_handle, const struct ble_gatt_error *error,
                             const struct ble_gatt_svc *svc, void *arg);
static int kk_ble_tx_on_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr, void *arg);
static int kk_ble_tx_on_dsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                            uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg);

static void kk_ble_tx_sm_open(void)
{
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
}

static void kk_ble_tx_on_sync(void)
{
    ble_svc_gap_device_name_set(KK_BLE_NAME_TX);
    s_host_ready = true;
    ESP_LOGW(TAG, "host ready");
}

static void kk_ble_tx_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void kk_ble_tx_gap_reset(void)
{
    ble_gap_disc_cancel();
    ble_gap_conn_cancel();
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_connected = false;
    s_gatt_ready = false;
    s_busy = false;
    s_have_peer = false;
    s_disc_complete = false;
    s_link_val_handle = 0;
    kk_delay_ms(500);
}

static bool kk_ble_tx_wait_connected(uint32_t timeout_ms)
{
    const uint32_t deadline = kk_millis() + timeout_ms;
    while (!s_connected && kk_millis() < deadline) {
        kk_delay_ms(50);
    }
    return s_connected;
}

static bool kk_ble_tx_wait_disc_complete(uint32_t timeout_ms)
{
    const uint32_t deadline = kk_millis() + timeout_ms;
    while (!s_disc_complete && kk_millis() < deadline) {
        kk_delay_ms(20);
    }
    return s_disc_complete;
}

static bool kk_ble_tx_wait_gatt_ready(uint32_t timeout_ms)
{
    const uint32_t deadline = kk_millis() + timeout_ms;
    while (s_connected && !s_gatt_ready && kk_millis() < deadline) {
        kk_delay_ms(20);
    }
    return s_gatt_ready && s_link_val_handle != 0;
}

static bool kk_ble_tx_parse_mac(const char *mac_str, ble_addr_t *out)
{
    unsigned int b[6];
    if (sscanf(mac_str, "%X:%X:%X:%X:%X:%X", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        return false;
    }
    out->type = BLE_ADDR_PUBLIC;
    for (int i = 0; i < 6; i++) {
        out->val[5 - i] = (uint8_t)b[i];
    }
    return true;
}

static void kk_ble_tx_disc_chrs(uint16_t conn_handle, const struct ble_gatt_svc *svc)
{
    ble_gattc_disc_all_chrs(conn_handle, svc->start_handle, svc->end_handle,
                            kk_ble_tx_on_chr, NULL);
}

static int kk_ble_tx_on_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    if (error->status == BLE_HS_EDONE) {
        s_busy = false;
        s_gatt_ready = true;
        ESP_LOGW(TAG, "GATT ready");
        if (s_link_val_handle != 0) {
            ESP_LOGW(TAG, "=== LINK OK ===");
        }
        return 0;
    }
    if (error->status != 0) {
        return 0;
    }
    if (ble_uuid_u16(&chr->uuid.u) == KK_BLE_CH_LINK_UUID16) {
        s_link_val_handle = chr->val_handle;
        ble_gattc_disc_all_dscs(conn_handle, chr->val_handle, chr->def_handle,
                                kk_ble_tx_on_dsc, NULL);
    }
    return 0;
}

static int kk_ble_tx_on_dsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                            uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)chr_val_handle;
    (void)arg;
    if (error->status == BLE_HS_EDONE) {
        return 0;
    }
    if (error->status != 0 || !dsc) {
        return 0;
    }
    if (ble_uuid_u16(&dsc->uuid.u) != BLE_GATT_DSC_CLT_CFG_UUID16) {
        return 0;
    }
    const uint8_t notify_en[2] = {1, 0};
    ble_gattc_write_flat(conn_handle, dsc->handle, notify_en, sizeof(notify_en), NULL, NULL);
    return 0;
}

static int kk_ble_tx_on_disc(uint16_t conn_handle, const struct ble_gatt_error *error,
                             const struct ble_gatt_svc *svc, void *arg)
{
    (void)arg;
    if (error->status == BLE_HS_EDONE) {
        s_busy = false;
        return 0;
    }
    if (error->status != 0) {
        return 0;
    }
    if (ble_uuid_u16(&svc->uuid.u) == KK_BLE_SVC_UUID16) {
        kk_ble_tx_disc_chrs(conn_handle, svc);
    }
    return 0;
}

static void kk_ble_tx_log_disc(const struct ble_gap_disc_desc *disc,
                               const struct ble_hs_adv_fields *fields)
{
    if (s_disc_logged >= 3) {
        return;
    }
    s_disc_logged++;

    char name[32] = {0};
    if (fields->name && fields->name_len > 0) {
        size_t n = fields->name_len;
        if (n >= sizeof(name)) {
            n = sizeof(name) - 1;
        }
        memcpy(name, fields->name, n);
    } else {
        strcpy(name, "(no name)");
    }

    ESP_LOGW(TAG, "seen #%u %02X:%02X:%02X:%02X:%02X:%02X evt=%u name=%s",
             (unsigned)s_disc_logged,
             disc->addr.val[5], disc->addr.val[4], disc->addr.val[3],
             disc->addr.val[2], disc->addr.val[1], disc->addr.val[0],
             (unsigned)disc->event_type, name);
}

static bool kk_ble_tx_adv_match(const struct ble_hs_adv_fields *fields)
{
    bool name_ok = fields->name && fields->name_len == strlen(KK_BLE_NAME_RX) &&
                   memcmp(fields->name, KK_BLE_NAME_RX, fields->name_len) == 0;
    if (name_ok) {
        return true;
    }
    if (!fields->uuids16) {
        return false;
    }
    for (int i = 0; i < fields->num_uuids16; i++) {
        if (ble_uuid_u16(&fields->uuids16[i].u) == KK_BLE_SVC_UUID16) {
            return true;
        }
    }
    return false;
}

static int kk_ble_tx_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        s_disc_total++;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) != 0) {
            return 0;
        }
        kk_ble_tx_log_disc(&event->disc, &fields);
        if (s_have_peer || !kk_ble_tx_adv_match(&fields)) {
            return 0;
        }
        s_last_peer = event->disc.addr;
        s_have_peer = true;
        ESP_LOGW(TAG, "found %s", KK_BLE_NAME_RX);
        ble_gap_disc_cancel();
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_disc_complete = true;
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_busy = true;
            s_connected = true;
            s_gatt_ready = false;
            ESP_LOGW(TAG, "link up");
            ble_gattc_disc_all_svcs(s_conn_handle, kk_ble_tx_on_disc, NULL);
        } else {
            ESP_LOGW(TAG, "connect rc=%d", event->connect.status);
            s_busy = false;
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_connected = false;
        s_gatt_ready = false;
        s_busy = false;
        ESP_LOGW(TAG, "disconnected");
        if (s_on_disconnect) {
            s_on_disconnect();
        }
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint8_t buf[16];
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }
        os_mbuf_copydata(event->notify_rx.om, 0, len, buf);
        if (kk_repair_cmd_match(buf, len) && s_on_repair_peer) {
            s_on_repair_peer();
            return 0;
        }
        if (kk_center_cmd_match(buf, len) && s_on_center_peer) {
            s_on_center_peer();
            return 0;
        }
        char line[24];
        if (len >= sizeof(line)) {
            len = sizeof(line) - 1;
        }
        memcpy(line, buf, len);
        line[len] = '\0';
        kk_imu_mount_t mount;
        if (kk_mount_cmd_parse(line, len, &mount)) {
            kk_imu_tx_set_mount(&mount);
            ESP_LOGW(TAG, "mount from RX %s", line);
            return 0;
        }
        kk_gesture_cfg_t gesture;
        if (kk_gesture_cmd_parse(line, len, &gesture)) {
            kk_tx_gesture_apply(&gesture);
            ESP_LOGW(TAG, "gesture from RX %s", line);
        }
        return 0;
    }

    default:
        return 0;
    }
}

static bool kk_ble_tx_start_connect(void)
{
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &s_last_peer, 30000, NULL,
                             kk_ble_tx_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "connect start rc=%d", rc);
        return false;
    }
    return true;
}

void kk_ble_tx_init(void)
{
    kk_ble_tx_sm_open();
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = kk_ble_tx_on_sync;
    ble_svc_gap_init();
    nimble_port_freertos_init(kk_ble_tx_host_task);
}

bool kk_ble_tx_is_connected(void)
{
    return s_connected;
}

bool kk_ble_tx_is_link_ready(void)
{
    return s_connected && s_gatt_ready && s_link_val_handle != 0;
}

bool kk_ble_tx_send_telemetry(const char *payload)
{
    if (!kk_ble_tx_is_link_ready() || !payload) {
        return false;
    }
    const size_t len = strlen(payload);
    if (len == 0 || len > 80) {
        return false;
    }
    const int rc = ble_gattc_write_no_rsp_flat(s_conn_handle, s_link_val_handle,
                                               payload, len);
    return rc == 0;
}

bool kk_ble_tx_host_ready(void)
{
    return s_host_ready && ble_hs_synced();
}

static bool kk_ble_tx_finish_pair(bool save_mac)
{
    if (!kk_ble_tx_wait_gatt_ready(5000)) {
        ESP_LOGW(TAG, "GATT timeout");
        return false;
    }
    if (s_have_peer && save_mac) {
        char mac[24];
        snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                 s_last_peer.val[5], s_last_peer.val[4], s_last_peer.val[3],
                 s_last_peer.val[2], s_last_peer.val[1], s_last_peer.val[0]);
        kk_storage_save_peer_mac(mac);
        ESP_LOGW(TAG, "paired saved %s", mac);
    }
    return true;
}

static bool kk_ble_tx_run_connect_phase(void)
{
    /* Scan already cancelled in DISC handler; short settle before connect */
    kk_delay_ms(300);
    if (!kk_ble_tx_start_connect()) {
        return false;
    }
    if (!kk_ble_tx_wait_connected(5000)) {
        ESP_LOGW(TAG, "link timeout");
        return false;
    }
    return true;
}

bool kk_ble_tx_scan_pair(void)
{
    if (!kk_ble_tx_host_ready()) {
        return false;
    }
    kk_ble_tx_gap_reset();

    s_busy = true;
    s_disc_total = 0;
    s_disc_logged = 0;
    struct ble_gap_disc_params p = {0};
    p.passive = 0;
    p.itvl = 0x10;
    p.window = 0x10;
    p.filter_policy = BLE_HCI_SCAN_FILT_NO_WL;

    ESP_LOGW(TAG, "scan %s", KK_BLE_NAME_RX);
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, KK_BLE_SCAN_SEC * 1000, &p,
                          kk_ble_tx_gap_event, NULL);
    if (rc != 0) {
        kk_ble_tx_gap_reset();
        ESP_LOGW(TAG, "scan start rc=%d", rc);
        return false;
    }

    const uint32_t scan_deadline = kk_millis() + (KK_BLE_SCAN_SEC * 1000UL);
    while (!s_have_peer && !s_disc_complete && kk_millis() < scan_deadline) {
        kk_delay_ms(50);
    }
    if (!s_have_peer) {
        ble_gap_disc_cancel();
        kk_ble_tx_wait_disc_complete(500);
        kk_ble_tx_gap_reset();
        ESP_LOGW(TAG, "not found (no adv) total=%u target=%s",
                 (unsigned)s_disc_total, KK_BLE_NAME_RX);
        return false;
    }

    if (!kk_ble_tx_run_connect_phase()) {
        kk_ble_tx_gap_reset();
        ESP_LOGW(TAG, "not found (connect timeout)");
        return false;
    }

    return kk_ble_tx_finish_pair(true);
}

bool kk_ble_tx_connect_mac(const char *mac, bool save_mac)
{
    if (!kk_ble_tx_host_ready() || !mac) {
        return false;
    }

    ble_addr_t peer;
    char norm[24];
    strncpy(norm, mac, sizeof(norm) - 1);
    norm[sizeof(norm) - 1] = '\0';
    kk_mac_normalize(norm);
    if (!kk_ble_tx_parse_mac(norm, &peer)) {
        return false;
    }

    kk_ble_tx_gap_reset();
    s_busy = true;
    s_have_peer = true;
    s_last_peer = peer;

    ESP_LOGW(TAG, "connect %s", norm);
    if (!kk_ble_tx_start_connect()) {
        kk_ble_tx_gap_reset();
        return false;
    }
    if (!kk_ble_tx_wait_connected(5000)) {
        kk_ble_tx_gap_reset();
        ESP_LOGW(TAG, "link timeout");
        return false;
    }

    return kk_ble_tx_finish_pair(save_mac);
}

void kk_ble_tx_send_repair(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_link_val_handle == 0) {
        return;
    }
    ble_gattc_write_flat(s_conn_handle, s_link_val_handle, KK_BLE_REPAIR_CMD, 6, NULL, NULL);
    kk_delay_ms(80);
}

void kk_ble_tx_send_center(void)
{
    if (!kk_ble_tx_is_link_ready()) {
        return;
    }
    ble_gattc_write_flat(s_conn_handle, s_link_val_handle, KK_BLE_CENTER_CMD, 6, NULL, NULL);
}

void kk_ble_tx_disconnect(void)
{
    kk_ble_tx_gap_reset();
}

void kk_ble_tx_set_on_disconnect(kk_ble_tx_event_cb_t cb) { s_on_disconnect = cb; }
void kk_ble_tx_set_on_repair_peer(kk_ble_tx_event_cb_t cb) { s_on_repair_peer = cb; }
void kk_ble_tx_set_on_center_peer(kk_ble_tx_event_cb_t cb) { s_on_center_peer = cb; }
