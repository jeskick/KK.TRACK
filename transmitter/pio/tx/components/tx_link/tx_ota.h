#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*kk_tx_ota_signal_fn)(const char *msg);

void kk_tx_ota_init(void);
void kk_tx_ota_log_partitions(void);
void kk_tx_ota_mark_boot_valid(void);
void kk_tx_ota_poll_boot_confirm(uint32_t now_ms);
void kk_tx_ota_set_signal_fn(kk_tx_ota_signal_fn fn);
bool kk_tx_ota_is_active(void);

esp_err_t kk_tx_ota_begin(size_t size);
esp_err_t kk_tx_ota_finish(void);
void kk_tx_ota_abort(void);
void kk_tx_ota_request_abort(void);
void kk_tx_ota_poll(void);

bool kk_tx_ota_link_cmd(const uint8_t *data, uint16_t len);
bool kk_tx_ota_on_chunk(const uint8_t *data, uint16_t len);
