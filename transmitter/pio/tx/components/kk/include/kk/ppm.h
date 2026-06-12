#pragma once

#include <stdint.h>

#define KK_PPM_CH_COUNT    8
#define KK_PPM_GAP_US      300
#define KK_PPM_SYNC_MIN_US 3000
/* 输出脉宽硬限（覆盖 180° 与 270° 舵机） */
#define KK_PPM_OUT_MIN_US  500
#define KK_PPM_OUT_MAX_US  2500
/* 标准 RC PPM 整帧 20ms = 50Hz；8ch 时各脉宽变化由同步段吸收 */
#define KK_PPM_FRAME_US    20000
#define KK_PPM_RATE_HZ     50

void kk_ppm_begin(void);
void kk_ppm_stop(void);
void kk_ppm_commit(void);
void kk_ppm_set_channel(uint8_t index, uint16_t pulse_us);
uint16_t kk_ppm_get_channel(uint8_t index);
void kk_ppm_fill_center(void);
