#pragma once

#include <stdbool.h>
#include <stddef.h>

void kk_mac_normalize(char *mac);
bool kk_storage_load_paired(char *peer_mac, size_t mac_cap);
void kk_storage_save_peer_mac(const char *mac);
void kk_storage_mark_paired(void);
bool kk_storage_is_paired(void);
void kk_storage_clear_peer(void);
