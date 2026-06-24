#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef void (*kk_motion_rezero_cb_t)(void);

/* 移动检测调试快照（供 KK_DBG_MOTION 实时流分析每一拍的判据）*/
typedef struct {
    float lin_metric;   /* 线加速度度量(EMA+peak) */
    float lin_trans;    /* 扣除转头向心假加速度后的平移分量，与 KK_MOB_LIN_* 对照 */
    float grav_tilt;    /* 逻辑系重力相对基线倾角(deg)，站/坐/大姿态变化 */
    float g_pitch;      /* 逻辑系陀螺 dps */
    float g_yaw;
    float g_roll;
    int8_t stability;   /* BNO stability 分类(4=运动中 3/2=稳定/静置…) */
    bool head_only;     /* 单轴头部转动主导(纯转/纯点) -> 不算身体运动 */
    bool head_nod;      /* 纯点头(低加速度) -> 不算身体运动 */
    bool head_active;   /* 正在转头/点头(含双轴) -> 默认豁免，仅站/坐/大步平移放行 */
    bool body_motion;   /* 本拍判定为身体运动 */
    uint8_t state;      /* 0=RUN(跟随) 1=PAUSE(回中保持) */
    uint32_t trigger_ms; /* 身体运动累计计时，达到 KK_MOB_TRIGGER_MS 进入 PAUSE */
    uint32_t settle_ms;  /* 静止累计计时，达到 KK_MOB_SETTLE_MS 重新归零恢复 */
} kk_motion_dbg_t;

void kk_motion_detect_init(kk_motion_rezero_cb_t on_rezero);
void kk_motion_detect_reset(void);
void kk_motion_detect_set_enabled(bool enabled);
bool kk_motion_detect_is_enabled(void);
bool kk_motion_detect_is_paused(void);
uint32_t kk_motion_detect_trigger_ms(void);
void kk_motion_detect_get_dbg(kk_motion_dbg_t *out);

void kk_motion_detect_apply(float in_yaw, float in_pitch, float gyro_pitch_dps,
                            float gyro_yaw_dps, float gyro_roll_dps,
                            int8_t stability_class, float lin_accel_mps2,
                            const float grav_logic[3], uint32_t elapsed_ms,
                            float *out_yaw, float *out_pitch);
