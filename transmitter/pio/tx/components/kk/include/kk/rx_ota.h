#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    KK_OTA_IDLE = 0,
    KK_OTA_RX_LOCAL,
    KK_OTA_TX_RELAY,
    KK_OTA_DONE,
    KK_OTA_ERR,
} kk_ota_phase_t;

typedef struct {
    kk_ota_phase_t phase;
    uint8_t pct;     /* RX 侧：本地写入或 BLE 中继进度 */
    uint8_t tx_pct;  /* TX 侧 flash 写入进度（经 OTA,PRG 回传） */
    uint32_t written;
    uint32_t total;
    int err;
    char msg[48];
} kk_ota_status_t;

void kk_rx_ota_init(void);
void kk_rx_ota_log_partitions(void);
void kk_rx_ota_mark_boot_valid(void);
void kk_rx_ota_poll_boot_confirm(uint32_t now_ms);
const kk_ota_status_t *kk_rx_ota_status(void);
bool kk_rx_ota_is_active(void);
bool kk_rx_ota_is_tx_relay(void);
size_t kk_rx_ota_max_image_bytes(void);

esp_err_t kk_rx_ota_local_begin(size_t size);
esp_err_t kk_rx_ota_local_write(const uint8_t *data, size_t len);
esp_err_t kk_rx_ota_local_finish(void);
void kk_rx_ota_local_abort(void);

typedef esp_err_t (*kk_rx_ota_tx_begin_fn)(size_t size);
typedef esp_err_t (*kk_rx_ota_tx_write_fn)(const uint8_t *data, size_t len);
typedef esp_err_t (*kk_rx_ota_tx_finish_fn)(void);
typedef void (*kk_rx_ota_tx_abort_fn)(void);
typedef bool (*kk_rx_ota_tx_ready_fn)(void);

typedef struct {
    kk_rx_ota_tx_ready_fn ready;
    kk_rx_ota_tx_begin_fn begin;
    kk_rx_ota_tx_write_fn write;
    kk_rx_ota_tx_finish_fn finish;
    kk_rx_ota_tx_abort_fn abort;
} kk_rx_ota_tx_ops_t;

void kk_rx_ota_set_tx_ops(const kk_rx_ota_tx_ops_t *ops);

esp_err_t kk_rx_ota_tx_begin(size_t size);
esp_err_t kk_rx_ota_tx_write(const uint8_t *data, size_t len);
esp_err_t kk_rx_ota_tx_finish(void);
void kk_rx_ota_tx_abort(void);
void kk_rx_ota_tx_remote_pct(uint8_t pct);
