#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define KK_TRACK_DEC_STR_DEFAULT   85U   /* 解耦抑制 10–95 % */
#define KK_TRACK_DEC_DOM_DEFAULT   45U   /* 检测阈值 35–65 %（陀螺占比） */
#define KK_TRACK_DEC_STR_MIN       10U
#define KK_TRACK_DEC_STR_MAX       95U
#define KK_TRACK_DEC_DOM_MIN       35U
#define KK_TRACK_DEC_DOM_MAX       65U

typedef struct {
    bool decouple_en;
    bool motion_en;
    uint8_t decouple_str_x100;
    uint8_t decouple_dom_x10;
} kk_tx_track_cfg_t;

kk_tx_track_cfg_t kk_tx_track_cfg_defaults(void);
void kk_tx_track_cfg_sanitize(kk_tx_track_cfg_t *cfg);
bool kk_track_cmd_parse(const char *line, size_t len, kk_tx_track_cfg_t *out);
