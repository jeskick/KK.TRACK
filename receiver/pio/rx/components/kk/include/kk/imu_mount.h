#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * 已验证参考安装（水平放置，Pin1 圆点在平面右下角）：
 *   传感器通道[0]=绕芯片X转角 → 俯仰 Pitch（低头+，抬头-）
 *   传感器通道[1]=绕芯片Y转角 → 横滚 Roll（左侧向下+，左侧向上-）
 *   传感器通道[2]=绕芯片Z转角 → 偏航 Yaw（左转+，右转-）
 * 默认逻辑映射：YAW ← Z，PITCH ← X
 *
 * 安装补偿用四元数（90°/270° 时欧拉分量不能简单点积）。
 */

#define KK_IMU_AXIS_X          0
#define KK_IMU_AXIS_Y          1
#define KK_IMU_AXIS_Z          2

#define KK_IMU_REF_YAW_AXIS    KK_IMU_AXIS_Z
#define KK_IMU_REF_PITCH_AXIS  KK_IMU_AXIS_X

#define KK_IMU_ROT_STEPS       4

typedef struct {
    uint8_t rot_horiz;
    uint8_t rot_lr;
    uint8_t rot_fb;
} kk_imu_mount_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} kk_quat_t;

kk_imu_mount_t kk_imu_mount_defaults(void);
void kk_imu_mount_sanitize(kk_imu_mount_t *m);
uint8_t kk_imu_mount_deg_to_steps(uint16_t deg);
uint16_t kk_imu_mount_steps_to_deg(uint8_t steps);

/** BNO08x 报告顺序：I,J,K,real → 内部 w,x,y,z */
kk_quat_t kk_quat_from_bno(float i, float j, float k, float real);

/** q_rel = conj(q_zero) * q_now */
kk_quat_t kk_quat_rel(const kk_quat_t *q_now, const kk_quat_t *q_zero);

/** q_zero 为归零时姿态；输出逻辑 yaw(Z)/pitch(X)/roll(Y) 度 */
void kk_imu_mount_apply_quat(const kk_quat_t *q_now, const kk_quat_t *q_zero,
                             const kk_imu_mount_t *mount, float *yaw_deg,
                             float *pitch_deg, float *roll_deg);

/** 从相对四元数提取传感器欧拉（调试用，与驱动公式一致） */
void kk_imu_quat_to_sensor_rel(const kk_quat_t *q_rel, float sensor_rel[3]);

bool kk_mount_cmd_parse(const char *line, size_t len, kk_imu_mount_t *out);
