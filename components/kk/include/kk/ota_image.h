#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OTA 镜像首部安全校验器（RX 本地 OTA 与 TX 中继 OTA 共用）。
 *
 * 仅检查镜像前导字节，可跨 chunk 边界累积，不写 flash：
 *   1. 首字节 = 0xE9（ESP-IDF app image 魔数）
 *   2. esp_app_desc.magic_word = 0xABCD5432（确认是合法 app 描述）
 *   3. esp_app_desc.project_name 与期望工程名一致（杜绝 RX/TX 固件刷反）
 *
 * 镜像布局偏移（固定）：
 *   [0]   esp_image_header_t（24B），[0]=0xE9
 *   [24]  首段 esp_image_segment_header_t（8B）
 *   [32]  esp_app_desc_t：magic_word(4) secure(4) reserv1(8) version[32] project_name[32]
 *         → magic_word @32，project_name @80
 */

#define KK_OTA_IMG_HDR_NEED 112U /* 校验所需最少前导字节（覆盖到 project_name 末尾） */

typedef enum {
    KK_OTA_IMG_PENDING = 0, /* 字节不足，继续喂 */
    KK_OTA_IMG_OK,          /* 首部合法且工程名匹配 */
    KK_OTA_IMG_REJECT,      /* 非法镜像或工程名不符 */
} kk_ota_img_result_t;

typedef struct {
    uint8_t hdr[KK_OTA_IMG_HDR_NEED];
    uint8_t len;
    bool decided;
    kk_ota_img_result_t result;
} kk_ota_img_check_t;

void kk_ota_img_check_reset(kk_ota_img_check_t *c);

/*
 * 按镜像字节顺序喂入前导数据。expect_project 为期望 CMake 工程名
 * （如 KK_OTA_PROJ_RX / KK_OTA_PROJ_TX）；传 NULL 则只校验镜像合法性，不比对工程名。
 * 一旦判定（OK / REJECT）即缓存结果，后续调用直接返回，不再消耗字节。
 */
kk_ota_img_result_t kk_ota_img_check_feed(kk_ota_img_check_t *c, const uint8_t *data, size_t len,
                                          const char *expect_project);

#ifdef __cplusplus
}
#endif
