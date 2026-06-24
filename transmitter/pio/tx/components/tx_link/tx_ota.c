#include "tx_ota.h"

#include "kk/link_config.h"
#include "kk/ota_image.h"
#include "kk/time.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "kk.ota.tx";

/*
 * 安全不变量（防止变砖）：
 * 1. 控制命令只走 LINK 特征；固件 chunk 只走 OTA 特征（ble_tx.c）
 * 2. esp_ota_set_boot_partition 仅在 esp_ota_end() 成功后调用
 * 3. pending != total 时绝不 finish；abort 必调 esp_ota_abort()
 * 4. 目标分区必须是非运行中的 ota_0/ota_1
 * 5. NimBLE 回调只入队；flash 写入仅在主循环 poll 中执行
 */

#define KK_TX_OTA_RX_CAP       16384U
#define KK_TX_OTA_DRAIN_MAX    4096U

static portMUX_TYPE s_ota_mux = portMUX_INITIALIZER_UNLOCKED;

static const esp_partition_t *s_part;
static esp_ota_handle_t s_handle;
static size_t s_total;
static size_t s_written;
static size_t s_rx_len;
static uint8_t s_rx_buf[KK_TX_OTA_RX_CAP];
static bool s_open;
static bool s_active;
static volatile bool s_finish_req;
static volatile bool s_abort_req;
static volatile bool s_start_req;
static size_t s_start_size;
static uint32_t s_finish_start_ms;
static uint32_t s_stall_ms;
static size_t s_stall_last;
static uint8_t s_log_pct;
static kk_ota_img_check_t s_img;
static bool s_boot_confirmed;
static uint32_t s_boot_ms;
static kk_tx_ota_signal_fn s_signal_fn;

static void kk_tx_ota_reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(KK_OTA_REBOOT_DELAY_MS));
    esp_restart();
}

static size_t kk_tx_ota_pending_unsafe(void)
{
    return s_written + s_rx_len;
}

static bool kk_tx_ota_rx_push_unsafe(const uint8_t *data, size_t len)
{
    if (!data || len == 0 || s_rx_len + len > KK_TX_OTA_RX_CAP) {
        return false;
    }
    memcpy(s_rx_buf + s_rx_len, data, len);
    s_rx_len += len;
    return true;
}

static void kk_tx_ota_log_progress(void)
{
    if (s_total == 0) {
        return;
    }
    const uint32_t pct = (uint32_t)((s_written * 100UL) / s_total);
    const uint8_t pct8 = pct > 100U ? 100U : (uint8_t)pct;
    if (pct8 >= s_log_pct + 10U) {
        s_log_pct = (pct8 / 10U) * 10U;
        ESP_LOGW(TAG, "OTA recv %u%% (%u/%u)", (unsigned)s_log_pct, (unsigned)s_written,
                 (unsigned)s_total);
        if (s_signal_fn) {
            char prg[16];
            snprintf(prg, sizeof(prg), "OTA,PRG,%u", (unsigned)s_log_pct);
            s_signal_fn(prg);
        }
    }
}

static esp_err_t kk_tx_ota_drain_rx(bool flush_all)
{
    size_t drained = 0;
    const size_t drain_cap = flush_all ? KK_TX_OTA_RX_CAP : KK_TX_OTA_DRAIN_MAX;

    while (drained < drain_cap) {
        uint8_t tmp[512];
        size_t n;

        /* 出队与缓冲压实须原子完成，否则并发的 kk_tx_ota_on_chunk 入队会撞坏缓冲 */
        portENTER_CRITICAL(&s_ota_mux);
        if (s_rx_len == 0) {
            portEXIT_CRITICAL(&s_ota_mux);
            break;
        }
        n = s_rx_len;
        if (n > sizeof(tmp)) {
            n = sizeof(tmp);
        }
        memcpy(tmp, s_rx_buf, n);
        s_rx_len -= n;
        if (s_rx_len > 0) {
            memmove(s_rx_buf, s_rx_buf + n, s_rx_len);
        }
        portEXIT_CRITICAL(&s_ota_mux);

        /* 首包校验：必须是合法 ESP app 镜像且工程名为 TX，杜绝刷错固件变砖 */
        if (kk_ota_img_check_feed(&s_img, tmp, n, KK_OTA_PROJ_TX) == KK_OTA_IMG_REJECT) {
            ESP_LOGW(TAG, "OTA reject image (magic/project mismatch)");
            return ESP_ERR_INVALID_CRC;
        }
        if (s_written + n > s_total) {
            ESP_LOGW(TAG, "OTA drain overflow %u+%u > %u", (unsigned)s_written, (unsigned)n,
                     (unsigned)s_total);
            return ESP_ERR_INVALID_SIZE;
        }

        esp_err_t err = esp_ota_write(s_handle, tmp, n);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "OTA write fail @%u rc=%d", (unsigned)s_written, (int)err);
            return err;
        }
        s_written += n;
        drained += n;
        kk_tx_ota_log_progress();
    }
    return ESP_OK;
}

void kk_tx_ota_set_signal_fn(kk_tx_ota_signal_fn fn)
{
    s_signal_fn = fn;
}

void kk_tx_ota_init(void)
{
    s_part = NULL;
    s_handle = 0;
    s_total = 0;
    s_written = 0;
    s_rx_len = 0;
    s_open = false;
    s_active = false;
    s_finish_req = false;
    s_abort_req = false;
    s_start_req = false;
    s_start_size = 0;
    s_finish_start_ms = 0;
    s_stall_ms = 0;
    s_stall_last = 0;
    s_log_pct = 0;
    kk_ota_img_check_reset(&s_img);
    s_boot_confirmed = false;
    s_boot_ms = kk_millis();
}

void kk_tx_ota_log_partitions(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_partition_iterator_t it =
        esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (it != NULL) {
        const esp_partition_t *p = esp_partition_get(it);
        ESP_LOGW(TAG, "app part %s @0x%x size=%u%s", p->label, (unsigned)p->address,
                 (unsigned)p->size, (run && p->address == run->address) ? " RUN" : "");
        it = esp_partition_next(it);
    }
}

void kk_tx_ota_mark_boot_valid(void)
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

void kk_tx_ota_poll_boot_confirm(uint32_t now_ms)
{
    if (s_boot_confirmed) {
        return;
    }
    if (s_active) {
        return;
    }
    if (s_boot_ms == 0 || now_ms < s_boot_ms) {
        s_boot_ms = now_ms;
        return;
    }
    if ((now_ms - s_boot_ms) < KK_OTA_BOOT_CONFIRM_MS) {
        return;
    }
    kk_tx_ota_mark_boot_valid();
    s_boot_confirmed = true;
    ESP_LOGW(TAG, "boot confirm after %lums", (unsigned long)KK_OTA_BOOT_CONFIRM_MS);
}

bool kk_tx_ota_is_active(void)
{
    return s_active;
}

static bool kk_tx_ota_part_ok(const esp_partition_t *part)
{
    const esp_partition_t *running;

    if (!part) {
        return false;
    }
    if (part->subtype != ESP_PARTITION_SUBTYPE_APP_OTA_0 &&
        part->subtype != ESP_PARTITION_SUBTYPE_APP_OTA_1) {
        ESP_LOGW(TAG, "OTA bad part subtype %u", (unsigned)part->subtype);
        return false;
    }
    running = esp_ota_get_running_partition();
    if (running && part->address == running->address) {
        ESP_LOGW(TAG, "OTA refuse running part %s", part->label);
        return false;
    }
    return true;
}

esp_err_t kk_tx_ota_begin(size_t size)
{
    if (s_active) {
        ESP_LOGW(TAG, "OTA begin rejected (busy)");
        return ESP_ERR_INVALID_STATE;
    }
    if (size < KK_OTA_TX_IMAGE_MIN || size > KK_OTA_TX_IMAGE_MAX) {
        ESP_LOGW(TAG, "OTA bad size %u (min %u max %u)", (unsigned)size,
                 (unsigned)KK_OTA_TX_IMAGE_MIN, (unsigned)KK_OTA_TX_IMAGE_MAX);
        return ESP_ERR_INVALID_SIZE;
    }
    s_part = esp_ota_get_next_update_partition(NULL);
    if (!kk_tx_ota_part_ok(s_part)) {
        ESP_LOGW(TAG, "OTA no safe update partition");
        return ESP_ERR_NOT_FOUND;
    }
    if (size > s_part->size) {
        ESP_LOGW(TAG, "OTA size %u > part %u", (unsigned)size, (unsigned)s_part->size);
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err = esp_ota_begin(s_part, size, &s_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OTA esp_ota_begin rc=%d", (int)err);
        return err;
    }
    s_total = size;
    s_written = 0;
    s_rx_len = 0;
    s_open = true;
    s_active = true;
    s_finish_req = false;
    s_abort_req = false;
    s_start_req = false;
    s_start_size = 0;
    s_finish_start_ms = 0;
    s_stall_ms = 0;
    s_stall_last = 0;
    s_log_pct = 0;
    kk_ota_img_check_reset(&s_img);
    s_stall_ms = kk_millis();
    s_stall_last = 0;
    ESP_LOGW(TAG, "OTA begin size=%u -> %s", (unsigned)size, s_part->label);
    return ESP_OK;
}

esp_err_t kk_tx_ota_finish(void)
{
    if (!s_open || !s_active) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_rx_len != 0 || s_written != s_total) {
        ESP_LOGW(TAG, "OTA finish blocked pending=%u total=%u", (unsigned)s_written,
                 (unsigned)s_total);
        return ESP_ERR_INVALID_SIZE;
    }
    if (!kk_tx_ota_part_ok(s_part)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esp_ota_end(s_handle);
    s_open = false;
    s_handle = 0;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OTA esp_ota_end rc=%d written=%u (boot unchanged)", (int)err,
                 (unsigned)s_written);
        s_active = false;
        if (s_signal_fn) {
            s_signal_fn("OTA,FAIL");
        }
        return err;
    }
    err = esp_ota_set_boot_partition(s_part);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OTA set_boot rc=%d (boot unchanged)", (int)err);
        s_active = false;
        if (s_signal_fn) {
            s_signal_fn("OTA,FAIL");
        }
        return err;
    }
    ESP_LOGW(TAG, "OTA done %u bytes -> reboot", (unsigned)s_written);
    if (s_signal_fn) {
        s_signal_fn("OTA,DONE");
    }
    s_active = false;
    xTaskCreate(kk_tx_ota_reboot_task, "ota_reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

void kk_tx_ota_abort(void)
{
    const bool was_active = s_active;

    if (s_open) {
        esp_ota_abort(s_handle);
    }
    s_open = false;
    s_active = false;
    s_finish_req = false;
    s_abort_req = false;
    s_start_req = false;
    s_start_size = 0;
    s_finish_start_ms = 0;
    s_stall_ms = 0;
    s_stall_last = 0;
    s_total = 0;
    s_written = 0;
    s_rx_len = 0;
    s_log_pct = 0;
    kk_ota_img_check_reset(&s_img);
    s_part = NULL;
    s_handle = 0;
    if (was_active && s_signal_fn) {
        ESP_LOGW(TAG, "OTA abort (boot unchanged)");
        s_signal_fn("OTA,FAIL");
    }
}

void kk_tx_ota_request_abort(void)
{
    if (!s_active) {
        return;
    }
    s_abort_req = true;
}

void kk_tx_ota_poll(void)
{
    size_t pending;

    if (s_start_req) {
        s_start_req = false;
        const size_t size = s_start_size;
        s_start_size = 0;
        if (kk_tx_ota_begin(size) == ESP_OK) {
            if (s_signal_fn) {
                s_signal_fn("OTA,RDY");
            }
        } else if (s_signal_fn) {
            s_signal_fn("OTA,ERR");
        }
    }

    if (!s_active) {
        return;
    }

    if (s_abort_req) {
        s_abort_req = false;
        kk_tx_ota_abort();
        return;
    }

    if (kk_tx_ota_drain_rx(s_finish_req) != ESP_OK) {
        kk_tx_ota_abort();
        return;
    }

    portENTER_CRITICAL(&s_ota_mux);
    pending = kk_tx_ota_pending_unsafe();
    portEXIT_CRITICAL(&s_ota_mux);

    if (!s_finish_req) {
        if (pending < s_total) {
            if (pending != s_stall_last) {
                s_stall_last = pending;
                s_stall_ms = kk_millis();
            } else if (kk_millis() - s_stall_ms > KK_OTA_TX_STALL_MS) {
                ESP_LOGW(TAG, "OTA stall pending=%u total=%u", (unsigned)pending,
                         (unsigned)s_total);
                kk_tx_ota_abort();
            }
        }
        return;
    }

    if (pending != s_total) {
        if (s_finish_start_ms == 0) {
            s_finish_start_ms = kk_millis();
        }
        if (kk_millis() - s_finish_start_ms > KK_OTA_TX_FINISH_MS) {
            ESP_LOGW(TAG, "OTA short at END pending=%u total=%u", (unsigned)pending,
                     (unsigned)s_total);
            s_finish_req = false;
            kk_tx_ota_abort();
        }
        return;
    }

    s_finish_req = false;
    s_finish_start_ms = 0;
    if (kk_tx_ota_finish() != ESP_OK) {
        kk_tx_ota_abort();
    }
}

bool kk_tx_ota_link_cmd(const uint8_t *data, uint16_t len)
{
    if (!data || len < 7) {
        return false;
    }
    char line[48];
    if (len >= sizeof(line)) {
        len = (uint16_t)(sizeof(line) - 1);
    }
    memcpy(line, data, len);
    line[len] = '\0';

    unsigned size = 0;
    if (strncmp(line, "OTA,START,", 10) == 0) {
        if (sscanf(line, "OTA,START,%u", &size) != 1 || size < KK_OTA_TX_IMAGE_MIN ||
            size > KK_OTA_TX_IMAGE_MAX) {
            return false;
        }
        if (s_active || s_start_req) {
            return false;
        }
        s_start_size = (size_t)size;
        s_start_req = true;
        return true;
    }
    if (strncmp(line, "OTA,END", 7) == 0) {
        if (!s_open || !s_active) {
            ESP_LOGW(TAG, "OTA END ignored (no session)");
            return true;
        }
        s_finish_req = true;
        s_finish_start_ms = kk_millis();
        ESP_LOGW(TAG, "OTA END armed pending=%u total=%u", (unsigned)kk_tx_ota_pending_unsafe(),
                 (unsigned)s_total);
        return true;
    }
    if (strncmp(line, "OTA,ABORT", 9) == 0) {
        s_abort_req = true;
        return true;
    }
    return false;
}

bool kk_tx_ota_on_chunk(const uint8_t *data, uint16_t len)
{
    size_t pending;

    if (!data || len == 0) {
        return false;
    }
    if (!s_active || !s_open) {
        return false;
    }

    portENTER_CRITICAL(&s_ota_mux);
    pending = kk_tx_ota_pending_unsafe();
    if (pending + len > s_total) {
        portEXIT_CRITICAL(&s_ota_mux);
        ESP_LOGW(TAG, "OTA chunk overflow pending=%u+%u > %u", (unsigned)pending, (unsigned)len,
                 (unsigned)s_total);
        kk_tx_ota_abort();
        return false;
    }
    if (!kk_tx_ota_rx_push_unsafe(data, len)) {
        portEXIT_CRITICAL(&s_ota_mux);
        ESP_LOGW(TAG, "OTA rx buf full pending=%u", (unsigned)pending);
        kk_tx_ota_request_abort();
        return false;
    }
    portEXIT_CRITICAL(&s_ota_mux);
    return true;
}
