#include "nvs.h"

int nvs_open(const char *name, int open_mode, nvs_handle_t *out_handle)
{
    (void)name;
    (void)open_mode;
    if (out_handle) {
        *out_handle = 0;
    }
    return ESP_ERR_NVS_NOT_FOUND;
}

void nvs_close(nvs_handle_t handle)
{
    (void)handle;
}

int nvs_get_u8(nvs_handle_t handle, const char *key, uint8_t *out_value)
{
    (void)handle;
    (void)key;
    (void)out_value;
    return ESP_ERR_NVS_NOT_FOUND;
}

int nvs_get_i16(nvs_handle_t handle, const char *key, int16_t *out_value)
{
    (void)handle;
    (void)key;
    (void)out_value;
    return ESP_ERR_NVS_NOT_FOUND;
}

int nvs_get_u16(nvs_handle_t handle, const char *key, uint16_t *out_value)
{
    (void)handle;
    (void)key;
    (void)out_value;
    return ESP_ERR_NVS_NOT_FOUND;
}

int nvs_set_u8(nvs_handle_t handle, const char *key, uint8_t value)
{
    (void)handle;
    (void)key;
    (void)value;
    return ESP_OK;
}

int nvs_set_i16(nvs_handle_t handle, const char *key, int16_t value)
{
    (void)handle;
    (void)key;
    (void)value;
    return ESP_OK;
}

int nvs_set_u16(nvs_handle_t handle, const char *key, uint16_t value)
{
    (void)handle;
    (void)key;
    (void)value;
    return ESP_OK;
}

int nvs_commit(nvs_handle_t handle)
{
    (void)handle;
    return ESP_OK;
}
