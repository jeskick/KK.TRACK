#include "tx_mount.h"

#include "kk/link_config.h"

#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "kk.tx.mount";
static kk_imu_mount_t s_mount;

void kk_tx_mount_load(kk_imu_mount_t *out)
{
    kk_imu_mount_t m = kk_imu_mount_defaults();
    nvs_handle_t h;
    if (nvs_open("kk_tx", NVS_READONLY, &h) == ESP_OK) {
        uint8_t v;
        if (nvs_get_u8(h, "m_h", &v) == ESP_OK) {
            m.rot_horiz = v;
        }
        if (nvs_get_u8(h, "m_l", &v) == ESP_OK) {
            m.rot_lr = v;
        }
        if (nvs_get_u8(h, "m_f", &v) == ESP_OK) {
            m.rot_fb = v;
        }
        nvs_close(h);
    }
    kk_imu_mount_sanitize(&m);
    s_mount = m;
    if (out) {
        *out = m;
    }
    ESP_LOGW(TAG, "mount horiz=%u lr=%u fb=%u deg %u/%u/%u",
             m.rot_horiz, m.rot_lr, m.rot_fb,
             kk_imu_mount_steps_to_deg(m.rot_horiz),
             kk_imu_mount_steps_to_deg(m.rot_lr),
             kk_imu_mount_steps_to_deg(m.rot_fb));
}

void kk_tx_mount_save(const kk_imu_mount_t *mount)
{
    if (!mount) {
        return;
    }
    kk_imu_mount_t m = *mount;
    kk_imu_mount_sanitize(&m);
    s_mount = m;

    nvs_handle_t h;
    if (nvs_open("kk_tx", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u8(h, "m_h", m.rot_horiz);
    nvs_set_u8(h, "m_l", m.rot_lr);
    nvs_set_u8(h, "m_f", m.rot_fb);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGW(TAG, "saved mount %u/%u/%u deg",
             kk_imu_mount_steps_to_deg(m.rot_horiz),
             kk_imu_mount_steps_to_deg(m.rot_lr),
             kk_imu_mount_steps_to_deg(m.rot_fb));
}

void kk_tx_mount_apply(const kk_imu_mount_t *mount)
{
    if (!mount) {
        return;
    }
    kk_tx_mount_save(mount);
}

const kk_imu_mount_t *kk_tx_mount_get(void)
{
    return &s_mount;
}
