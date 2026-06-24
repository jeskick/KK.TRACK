#include "kk/storage.h"
#include "kk/link_config.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <ctype.h>
#include <string.h>

static const char *TAG = "kk.nvs";

void kk_mac_normalize(char *mac)
{
    if (!mac) {
        return;
    }
    size_t i = 0;
    while (mac[i] == ' ' || mac[i] == '\t') {
        i++;
    }
    if (i > 0) {
        memmove(mac, mac + i, strlen(mac + i) + 1);
    }
    size_t len = strlen(mac);
    while (len > 0 && (mac[len - 1] == ' ' || mac[len - 1] == '\t')) {
        mac[--len] = '\0';
    }
    for (i = 0; mac[i]; i++) {
        mac[i] = (char)toupper((unsigned char)mac[i]);
    }
}

bool kk_storage_load_paired(char *peer_mac, size_t mac_cap)
{
    if (!peer_mac || mac_cap < 18) {
        return false;
    }
    nvs_handle_t h;
    if (nvs_open(KK_PREFS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    uint8_t paired = 0;
    nvs_get_u8(h, "paired", &paired);
    size_t len = mac_cap;
    esp_err_t err = nvs_get_str(h, "peer_mac", peer_mac, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        peer_mac[0] = '\0';
        return false;
    }
    kk_mac_normalize(peer_mac);
    return paired && strlen(peer_mac) >= 17;
}

void kk_storage_save_peer_mac(const char *mac)
{
    if (!mac) {
        return;
    }
    char buf[24];
    strncpy(buf, mac, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    kk_mac_normalize(buf);

    nvs_handle_t h;
    esp_err_t err = nvs_open(KK_PREFS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "save_peer_mac nvs_open rc=%d (pairing NOT persisted)", (int)err);
        return;
    }
    esp_err_t e1 = nvs_set_u8(h, "paired", 1);
    esp_err_t e2 = nvs_set_str(h, "peer_mac", buf);
    esp_err_t e3 = nvs_commit(h);
    nvs_close(h);
    if (e1 != ESP_OK || e2 != ESP_OK || e3 != ESP_OK) {
        ESP_LOGW(TAG, "save_peer_mac failed set=%d/%d commit=%d (pairing NOT persisted)", (int)e1,
                 (int)e2, (int)e3);
    }
}

void kk_storage_mark_paired(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(KK_PREFS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mark_paired nvs_open rc=%d", (int)err);
        return;
    }
    nvs_set_u8(h, "paired", 1);
    err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mark_paired commit rc=%d", (int)err);
    }
}

bool kk_storage_is_paired(void)
{
    nvs_handle_t h;
    if (nvs_open(KK_PREFS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    uint8_t paired = 0;
    nvs_get_u8(h, "paired", &paired);
    nvs_close(h);
    return paired != 0;
}

void kk_storage_clear_peer(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(KK_PREFS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "clear_peer nvs_open rc=%d", (int)err);
        return;
    }
    nvs_erase_key(h, "paired");
    nvs_erase_key(h, "peer_mac");
    err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "clear_peer commit rc=%d", (int)err);
    }
}
