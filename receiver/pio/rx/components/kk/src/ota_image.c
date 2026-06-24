#include "kk/ota_image.h"
#include "kk/link_config.h"

#include <string.h>

#define KK_OTA_APP_DESC_OFF  32U /* sizeof(esp_image_header_t)+sizeof(esp_image_segment_header_t) */
#define KK_OTA_PROJ_NAME_OFF 80U /* app_desc 起始 + magic(4)+secure(4)+reserv1(8)+version[32] */
#define KK_OTA_PROJ_NAME_MAX 32U

void kk_ota_img_check_reset(kk_ota_img_check_t *c)
{
    if (!c) {
        return;
    }
    c->len = 0;
    c->decided = false;
    c->result = KK_OTA_IMG_PENDING;
}

static kk_ota_img_result_t kk_ota_img_decide(kk_ota_img_check_t *c, const char *expect_project)
{
    /* 镜像首字节魔数 */
    if (c->hdr[0] != KK_OTA_IMAGE_MAGIC) {
        return KK_OTA_IMG_REJECT;
    }
    /* esp_app_desc.magic_word（小端） */
    const uint32_t desc_magic = (uint32_t)c->hdr[KK_OTA_APP_DESC_OFF] |
                                ((uint32_t)c->hdr[KK_OTA_APP_DESC_OFF + 1U] << 8) |
                                ((uint32_t)c->hdr[KK_OTA_APP_DESC_OFF + 2U] << 16) |
                                ((uint32_t)c->hdr[KK_OTA_APP_DESC_OFF + 3U] << 24);
    if (desc_magic != KK_OTA_APP_DESC_MAGIC) {
        return KK_OTA_IMG_REJECT;
    }
    /* project_name 比对 */
    if (expect_project && expect_project[0] != '\0') {
        char proj[KK_OTA_PROJ_NAME_MAX + 1U];
        memcpy(proj, &c->hdr[KK_OTA_PROJ_NAME_OFF], KK_OTA_PROJ_NAME_MAX);
        proj[KK_OTA_PROJ_NAME_MAX] = '\0';
        if (strncmp(proj, expect_project, KK_OTA_PROJ_NAME_MAX) != 0) {
            return KK_OTA_IMG_REJECT;
        }
    }
    return KK_OTA_IMG_OK;
}

kk_ota_img_result_t kk_ota_img_check_feed(kk_ota_img_check_t *c, const uint8_t *data, size_t len,
                                          const char *expect_project)
{
    if (!c) {
        return KK_OTA_IMG_REJECT;
    }
    if (c->decided) {
        return c->result;
    }
    if (!data || len == 0) {
        return KK_OTA_IMG_PENDING;
    }
    /* 首字节可立即否决，避免继续接收明显非法镜像 */
    if (c->len == 0 && data[0] != KK_OTA_IMAGE_MAGIC) {
        c->decided = true;
        c->result = KK_OTA_IMG_REJECT;
        return c->result;
    }
    size_t take = (size_t)(KK_OTA_IMG_HDR_NEED - c->len);
    if (take > len) {
        take = len;
    }
    memcpy(&c->hdr[c->len], data, take);
    c->len = (uint8_t)(c->len + take);
    if (c->len < KK_OTA_IMG_HDR_NEED) {
        return KK_OTA_IMG_PENDING;
    }
    c->result = kk_ota_img_decide(c, expect_project);
    c->decided = true;
    return c->result;
}
