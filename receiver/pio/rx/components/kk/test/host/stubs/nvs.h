#pragma once

#include <stddef.h>
#include <stdint.h>

typedef uint32_t nvs_handle_t;

#define ESP_OK 0
#define ESP_ERR_NVS_NOT_FOUND 0x1102

int nvs_open(const char *name, int open_mode, nvs_handle_t *out_handle);
void nvs_close(nvs_handle_t handle);
int nvs_get_u8(nvs_handle_t handle, const char *key, uint8_t *out_value);
int nvs_get_i16(nvs_handle_t handle, const char *key, int16_t *out_value);
int nvs_get_u16(nvs_handle_t handle, const char *key, uint16_t *out_value);
int nvs_set_u8(nvs_handle_t handle, const char *key, uint8_t value);
int nvs_set_i16(nvs_handle_t handle, const char *key, int16_t value);
int nvs_set_u16(nvs_handle_t handle, const char *key, uint16_t value);
int nvs_commit(nvs_handle_t handle);
