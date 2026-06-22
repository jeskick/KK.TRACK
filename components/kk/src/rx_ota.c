#include "kk/rx_ota.h"
#include "kk/link_config.h"
#include "kk/time.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "kk.ota.rx";

static kk_ota_status_t s_st;
static const esp_partition_t *s_part;
static esp_ota_handle_t s_handle;
static size_t s_total;
static size_t s_written;
static bool s_open;
static kk_rx_ota_tx_ops_t s_tx_ops;
static uint8_t s_tx_log_pct;
static uint8_t s_rx_log_pct;

static void kk_ota_close_failed(void);

static void kk_ota_set(kk_ota_phase_t phase, uint8_t pct, int err, const char *msg)
{
    s_st.phase = phase;
    s_st.pct = pct;
    s_st.err = err;
    if (msg) {
        strncpy(s_st.msg, msg, sizeof(s_st.msg) - 1);
        s_st.msg[sizeof(s_st.msg) - 1] = '\0';
    } else {
        s_st.msg[0] = '\0';
    }
}

static void kk_ota_format_msg(void)
{
    if (s_st.phase == KK_OTA_TX_RELAY) {
        if (s_st.tx_pct > 0) {
            snprintf(s_st.msg, sizeof(s_st.msg), "relay %u%% tx %u%%", (unsigned)s_st.pct,
                     (unsigned)s_st.tx_pct);
        } else {
            snprintf(s_st.msg, sizeof(s_st.msg), "relay %u%%", (unsigned)s_st.pct);
        }
    } else if (s_st.phase == KK_OTA_RX_LOCAL) {
        snprintf(s_st.msg, sizeof(s_st.msg), "RX OTA %u%%", (unsigned)s_st.pct);
    }
}

void kk_rx_ota_init(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_part = NULL;
    s_handle = 0;
    s_total = 0;
    s_written = 0;
    s_open = false;
}

void kk_rx_ota_log_partitions(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (run) {
        ESP_LOGW(TAG, "running part=%s off=0x%x size=%u", run->label, (unsigned)run->address,
                 (unsigned)run->size);
    } else {
        ESP_LOGW(TAG, "running part=NONE (OTA unavailable)");
    }
    if (next) {
        ESP_LOGW(TAG, "OTA target=%s size=%u max_image=%u", next->label, (unsigned)next->size,
                 (unsigned)next->size);
    } else {
        ESP_LOGW(TAG, "OTA target=NONE — erase flash and USB reflash with OTA partition table");
    }
}

void kk_rx_ota_mark_boot_valid(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running) {
        return;
    }
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) != ESP_OK) {
        return;
    }
    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGW(TAG, "mark boot valid rc=%d", (int)err);
    }
}

const kk_ota_status_t *kk_rx_ota_status(void)
{
    return &s_st;
}

bool kk_rx_ota_is_active(void)
{
    return s_st.phase == KK_OTA_RX_LOCAL || s_st.phase == KK_OTA_TX_RELAY;
}

bool kk_rx_ota_is_tx_relay(void)
{
    return s_st.phase == KK_OTA_TX_RELAY;
}

size_t kk_rx_ota_max_image_bytes(void)
{
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    return part ? part->size : 0;
}

static esp_err_t kk_ota_open(size_t size, kk_ota_phase_t phase)
{
    if (kk_rx_ota_is_active()) {
        return ESP_ERR_INVALID_STATE;
    }
    s_part = esp_ota_get_next_update_partition(NULL);
    if (!s_part) {
        kk_ota_set(KK_OTA_ERR, 0, ESP_ERR_NOT_FOUND, "no ota part");
        return ESP_ERR_NOT_FOUND;
    }
    if (size == 0 || size > s_part->size) {
        kk_ota_set(KK_OTA_ERR, 0, ESP_ERR_INVALID_SIZE, "size");
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err = esp_ota_begin(s_part, size, &s_handle);
    if (err != ESP_OK) {
        kk_ota_set(KK_OTA_ERR, 0, (int)err, "ota begin");
        return err;
    }
    s_total = size;
    s_written = 0;
    s_open = true;
    s_rx_log_pct = 0;
    s_st.written = 0;
    s_st.total = (uint32_t)size;
    s_st.tx_pct = 0;
    kk_ota_set(phase, 0, 0, phase == KK_OTA_RX_LOCAL ? "RX OTA 0%" : "recv");
    return ESP_OK;
}

static esp_err_t kk_ota_write_chunk(const uint8_t *data, size_t len)
{
    if (!s_open || !data || len == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_written + len > s_total) {
        kk_ota_close_failed();
        kk_ota_set(KK_OTA_ERR, s_st.pct, ESP_ERR_INVALID_SIZE, "overflow");
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err = esp_ota_write(s_handle, data, len);
    if (err != ESP_OK) {
        kk_ota_set(KK_OTA_ERR, s_st.pct, (int)err, "write");
        return err;
    }
    s_written += len;
    s_st.written = (uint32_t)s_written;
    if (s_total > 0) {
        const uint32_t pct = (uint32_t)((s_written * 100UL) / s_total);
        s_st.pct = pct > 100U ? 100U : (uint8_t)pct;
        if (s_st.phase == KK_OTA_RX_LOCAL &&
            (s_st.pct >= s_rx_log_pct + 5U || (s_rx_log_pct == 0 && s_st.pct > 0))) {
            s_rx_log_pct = (s_st.pct / 5U) * 5U;
            ESP_LOGW(TAG, "RX OTA %u%%", (unsigned)s_st.pct);
            kk_ota_format_msg();
        }
    }
    return ESP_OK;
}

static void kk_ota_close_failed(void)
{
    if (s_open) {
        esp_ota_abort(s_handle);
    }
    s_open = false;
    s_handle = 0;
    s_part = NULL;
    s_total = 0;
    s_written = 0;
}

static void kk_ota_reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(KK_OTA_REBOOT_DELAY_MS));
    esp_restart();
}

static esp_err_t kk_ota_close_ok(const char *done_msg)
{
    if (!s_open) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = esp_ota_end(s_handle);
    s_open = false;
    s_handle = 0;
    if (err != ESP_OK) {
        kk_ota_set(KK_OTA_ERR, s_st.pct, (int)err, "ota end");
        return err;
    }
    err = esp_ota_set_boot_partition(s_part);
    if (err != ESP_OK) {
        kk_ota_set(KK_OTA_ERR, s_st.pct, (int)err, "boot part");
        return err;
    }
    kk_ota_set(KK_OTA_DONE, 100, 0, done_msg ? done_msg : "reboot");
    xTaskCreate(kk_ota_reboot_task, "ota_reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

esp_err_t kk_rx_ota_local_begin(size_t size)
{
    return kk_ota_open(size, KK_OTA_RX_LOCAL);
}

esp_err_t kk_rx_ota_local_write(const uint8_t *data, size_t len)
{
    return kk_ota_write_chunk(data, len);
}

esp_err_t kk_rx_ota_local_finish(void)
{
    ESP_LOGW(TAG, "local OTA done %u bytes", (unsigned)s_written);
    return kk_ota_close_ok("rx reboot");
}

void kk_rx_ota_local_abort(void)
{
    kk_ota_close_failed();
    kk_ota_set(KK_OTA_IDLE, 0, 0, NULL);
}

void kk_rx_ota_set_tx_ops(const kk_rx_ota_tx_ops_t *ops)
{
    if (ops) {
        s_tx_ops = *ops;
    } else {
        memset(&s_tx_ops, 0, sizeof(s_tx_ops));
    }
}

esp_err_t kk_rx_ota_tx_begin(size_t size)
{
    if (!s_tx_ops.ready || !s_tx_ops.begin) {
        kk_ota_set(KK_OTA_ERR, 0, ESP_ERR_INVALID_STATE, "no tx ops");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_tx_ops.ready()) {
        kk_ota_set(KK_OTA_ERR, 0, ESP_ERR_INVALID_STATE, "tx offline");
        return ESP_ERR_INVALID_STATE;
    }
    s_total = size;
    s_written = 0;
    s_open = true;
    s_part = NULL;
    s_handle = 0;
    s_st.written = 0;
    s_st.total = (uint32_t)size;
    s_st.tx_pct = 0;
    /* 在 BLE begin 握手前就标记活跃，让 WiFi idle 逻辑保持 AP */
    kk_ota_set(KK_OTA_TX_RELAY, 0, 0, "tx begin");
    s_tx_log_pct = 0;
    esp_err_t err = s_tx_ops.begin(size);
    if (err != ESP_OK) {
        s_open = false;
        s_total = 0;
        kk_ota_set(KK_OTA_ERR, 0, (int)err, "tx begin");
        return err;
    }
    kk_ota_set(KK_OTA_TX_RELAY, 0, 0, "tx relay");
    ESP_LOGW(TAG, "TX OTA relay start size=%u", (unsigned)size);
    return ESP_OK;
}

esp_err_t kk_rx_ota_tx_write(const uint8_t *data, size_t len)
{
    if (!s_tx_ops.write || s_st.phase != KK_OTA_TX_RELAY) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = s_tx_ops.write(data, len);
    if (err != ESP_OK) {
        kk_ota_set(KK_OTA_ERR, s_st.pct, (int)err, "tx chunk");
        return err;
    }
    s_written += len;
    s_st.written = (uint32_t)s_written;
    if (s_total > 0) {
        const uint32_t pct = (uint32_t)((s_written * 100UL) / s_total);
        s_st.pct = pct > 100U ? 100U : (uint8_t)pct;
        if (s_st.pct >= s_tx_log_pct + 5U || (s_tx_log_pct == 0 && s_st.pct >= 5U)) {
            s_tx_log_pct = (s_st.pct / 5U) * 5U;
            ESP_LOGW(TAG, "TX OTA relay %u%%", (unsigned)s_st.pct);
            kk_ota_format_msg();
        }
    }
    return ESP_OK;
}

void kk_rx_ota_tx_remote_pct(uint8_t pct)
{
    if (s_st.phase != KK_OTA_TX_RELAY) {
        return;
    }
    if (pct > 100U) {
        pct = 100U;
    }
    s_st.tx_pct = pct;
    kk_ota_format_msg();
}

esp_err_t kk_rx_ota_tx_finish(void)
{
    if (!s_tx_ops.finish || s_st.phase != KK_OTA_TX_RELAY) {
        return ESP_ERR_INVALID_STATE;
    }
    kk_ota_set(KK_OTA_TX_RELAY, s_st.pct, 0, "tx finalize");
    esp_err_t err = s_tx_ops.finish();
    s_open = false;
    if (err != ESP_OK) {
        kk_ota_set(KK_OTA_ERR, s_st.pct, (int)err, "tx finish");
        return err;
    }
    kk_ota_set(KK_OTA_DONE, 100, 0, "tx done");
    ESP_LOGW(TAG, "TX OTA relay done %u bytes (TX confirmed)", (unsigned)s_written);
    return ESP_OK;
}

void kk_rx_ota_tx_abort(void)
{
    if (s_tx_ops.abort) {
        s_tx_ops.abort();
    }
    s_open = false;
    s_total = 0;
    s_written = 0;
    s_tx_log_pct = 0;
    s_rx_log_pct = 0;
    s_st.written = 0;
    s_st.total = 0;
    s_st.tx_pct = 0;
    kk_ota_set(KK_OTA_IDLE, 0, 0, NULL);
}
