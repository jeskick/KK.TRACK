#include "kk/imu_mount.h"
#include "kk/link_config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define KK_RAD2DEG  (180.0f / (float)M_PI)

static float kk_imu_wrap_deg(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static kk_quat_t kk_quat_normalize(kk_quat_t q)
{
    const float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-8f) {
        kk_quat_t id = {1.0f, 0.0f, 0.0f, 0.0f};
        return id;
    }
    q.w /= n;
    q.x /= n;
    q.y /= n;
    q.z /= n;
    return q;
}

static kk_quat_t kk_quat_mul(kk_quat_t a, kk_quat_t b)
{
    kk_quat_t r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return kk_quat_normalize(r);
}

static kk_quat_t kk_quat_conj(kk_quat_t q)
{
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    return q;
}

static kk_quat_t kk_quat_from_axis_angle(float ax, float ay, float az, float deg)
{
    const float half = deg * (float)M_PI / 180.0f * 0.5f;
    const float s = sinf(half);
    const float c = cosf(half);
    kk_quat_t q = {c, ax * s, ay * s, az * s};
    return kk_quat_normalize(q);
}

static void kk_imu_euler_from_quat(kk_quat_t q, float *roll_x, float *pitch_y, float *yaw_z)
{
    q = kk_quat_normalize(q);

    const float t0 = 2.0f * (q.w * q.x + q.y * q.z);
    const float t1 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    const float roll = atan2f(t0, t1);

    float t2 = 2.0f * (q.w * q.y - q.z * q.x);
    if (t2 > 1.0f) {
        t2 = 1.0f;
    }
    if (t2 < -1.0f) {
        t2 = -1.0f;
    }
    const float pitch = asinf(t2);

    const float t3 = 2.0f * (q.w * q.z + q.x * q.y);
    const float t4 = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    const float yaw = atan2f(t3, t4);

    if (roll_x) {
        *roll_x = roll * KK_RAD2DEG;
    }
    if (pitch_y) {
        *pitch_y = pitch * KK_RAD2DEG;
    }
    if (yaw_z) {
        *yaw_z = yaw * KK_RAD2DEG;
    }
}

/* 旋转方向：水平 Z-CW，左右 X-CCW，前后 Y-CCW */
static kk_quat_t kk_imu_mount_quat(const kk_imu_mount_t *mount)
{
    kk_imu_mount_t m = mount ? *mount : kk_imu_mount_defaults();
    kk_imu_mount_sanitize(&m);

    const float hz = -(float)(m.rot_horiz & 3U) * 90.0f;
    const float lr = (float)(m.rot_lr & 3U) * 90.0f;
    const float fb = (float)(m.rot_fb & 3U) * 90.0f;

    kk_quat_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    q = kk_quat_mul(q, kk_quat_from_axis_angle(0.0f, 0.0f, 1.0f, hz));
    q = kk_quat_mul(q, kk_quat_from_axis_angle(1.0f, 0.0f, 0.0f, lr));
    q = kk_quat_mul(q, kk_quat_from_axis_angle(0.0f, 1.0f, 0.0f, fb));
    return q;
}

kk_imu_mount_t kk_imu_mount_defaults(void)
{
    kk_imu_mount_t m = {0, 0, 0};
    return m;
}

void kk_imu_mount_sanitize(kk_imu_mount_t *m)
{
    if (!m) {
        return;
    }
    m->rot_horiz &= 3U;
    m->rot_lr &= 3U;
    m->rot_fb &= 3U;
}

uint8_t kk_imu_mount_deg_to_steps(uint16_t deg)
{
    switch (deg) {
    case 90:
        return 1;
    case 180:
        return 2;
    case 270:
        return 3;
    default:
        return 0;
    }
}

uint16_t kk_imu_mount_steps_to_deg(uint8_t steps)
{
    static const uint16_t tbl[4] = {0, 90, 180, 270};
    return tbl[steps & 3U];
}

kk_quat_t kk_quat_from_bno(float i, float j, float k, float real)
{
    kk_quat_t q = {real, i, j, k};
    return kk_quat_normalize(q);
}

kk_quat_t kk_quat_rel(const kk_quat_t *q_now, const kk_quat_t *q_zero)
{
    if (!q_now || !q_zero) {
        kk_quat_t id = {1.0f, 0.0f, 0.0f, 0.0f};
        return id;
    }
    return kk_quat_mul(kk_quat_conj(*q_zero), *q_now);
}

void kk_imu_quat_to_sensor_rel(const kk_quat_t *q_rel, float sensor_rel[3])
{
    if (!q_rel || !sensor_rel) {
        return;
    }
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    kk_imu_euler_from_quat(*q_rel, &roll, &pitch, &yaw);
    sensor_rel[0] = kk_imu_wrap_deg(roll);
    sensor_rel[1] = kk_imu_wrap_deg(pitch);
    sensor_rel[2] = kk_imu_wrap_deg(yaw);
}

static kk_quat_t kk_quat_twist_about(kk_quat_t q, float ax, float ay, float az)
{
    const float d = q.x * ax + q.y * ay + q.z * az;
    kk_quat_t t = {q.w, ax * d, ay * d, az * d};
    const float len = sqrtf(t.w * t.w + t.x * t.x + t.y * t.y + t.z * t.z);
    if (len < 1e-8f) {
        kk_quat_t id = {1.0f, 0.0f, 0.0f, 0.0f};
        return id;
    }
    t.w /= len;
    t.x /= len;
    t.y /= len;
    t.z /= len;
    if (t.w < 0.0f) {
        t.w = -t.w;
        t.x = -t.x;
        t.y = -t.y;
        t.z = -t.z;
    }
    return t;
}

static float kk_quat_twist_deg(kk_quat_t twist, float ax, float ay, float az)
{
    const float s = sqrtf(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z);
    float ang = 2.0f * atan2f(s, twist.w) * KK_RAD2DEG;
    const float dot = twist.x * ax + twist.y * ay + twist.z * az;
    if (dot < 0.0f) {
        ang = -ang;
    }
    return kk_imu_wrap_deg(ang);
}

static float kk_imu_clamp_pose_deg(float deg)
{
    if (deg > KK_IMU_POSE_CLAMP_DEG) {
        return KK_IMU_POSE_CLAMP_DEG;
    }
    if (deg < -KK_IMU_POSE_CLAMP_DEG) {
        return -KK_IMU_POSE_CLAMP_DEG;
    }
    return deg;
}

void kk_imu_mount_apply_quat(const kk_quat_t *q_now, const kk_quat_t *q_zero,
                             const kk_imu_mount_t *mount, float *yaw_deg,
                             float *pitch_deg, float *roll_deg)
{
    if (!q_now || !q_zero) {
        return;
    }

    kk_quat_t q_rel = kk_quat_mul(kk_quat_conj(*q_zero), *q_now);
    const kk_quat_t q_mount = kk_imu_mount_quat(mount);
    const kk_quat_t q_inv = kk_quat_conj(q_mount);
    kk_quat_t q_logic = kk_quat_mul(kk_quat_mul(q_inv, q_rel), q_mount);

    float rot_x = 0.0f;
    float rot_y = 0.0f;
    float rot_z = 0.0f;
    kk_imu_euler_from_quat(q_logic, &rot_x, &rot_y, &rot_z);

    float yaw = rot_z;
    float pitch = rot_x;
    float roll = rot_y;

    /* |Roll| 大时欧拉 Pitch/Yaw 奇异（日志 P=600+ Y 冻结）；改用 twist 并限幅 */
    if (fabsf(roll) >= KK_IMU_GIMBAL_ROLL_DEG) {
        const kk_quat_t q_yaw_tw = kk_quat_twist_about(q_logic, 0.0f, 0.0f, 1.0f);
        const kk_quat_t q_pitch_tw = kk_quat_twist_about(q_logic, 1.0f, 0.0f, 0.0f);
        yaw = kk_quat_twist_deg(q_yaw_tw, 0.0f, 0.0f, 1.0f);
        pitch = kk_quat_twist_deg(q_pitch_tw, 1.0f, 0.0f, 0.0f);
    }

    yaw = kk_imu_clamp_pose_deg(yaw);
    pitch = kk_imu_clamp_pose_deg(pitch);
    roll = kk_imu_clamp_pose_deg(roll);

    if (yaw_deg) {
        *yaw_deg = kk_imu_wrap_deg(yaw);
    }
    if (pitch_deg) {
        *pitch_deg = kk_imu_wrap_deg(pitch);
    }
    if (roll_deg) {
        *roll_deg = kk_imu_wrap_deg(roll);
    }
}

static void kk_quat_rotate_vec(kk_quat_t q, float vx, float vy, float vz, float *ox,
                               float *oy, float *oz)
{
    const float tx = 2.0f * (q.y * vz - q.z * vy);
    const float ty = 2.0f * (q.z * vx - q.x * vz);
    const float tz = 2.0f * (q.x * vy - q.y * vx);
    if (ox) {
        *ox = vx + q.w * tx + (q.y * tz - q.z * ty);
    }
    if (oy) {
        *oy = vy + q.w * ty + (q.z * tx - q.x * tz);
    }
    if (oz) {
        *oz = vz + q.w * tz + (q.x * ty - q.y * tx);
    }
}

void kk_imu_mount_gyro_to_logic(const kk_imu_mount_t *mount, float gx_dps,
                                float gy_dps, float gz_dps, float *pitch_dps,
                                float *yaw_dps, float *roll_dps)
{
    const kk_quat_t q = kk_imu_mount_quat(mount);
    float lx = 0.0f;
    float ly = 0.0f;
    float lz = 0.0f;
    kk_quat_rotate_vec(q, gx_dps, gy_dps, gz_dps, &lx, &ly, &lz);
    if (pitch_dps) {
        *pitch_dps = lx;
    }
    if (yaw_dps) {
        *yaw_dps = lz;
    }
    if (roll_dps) {
        *roll_dps = ly;
    }
}

bool kk_mount_cmd_parse(const char *line, size_t len, kk_imu_mount_t *out)
{
    if (!line || !out || len < 7) {
        return false;
    }
    if (strncmp(line, "MNT,", 4) != 0) {
        return false;
    }

    unsigned h = 0;
    unsigned l = 0;
    unsigned f = 0;
    if (sscanf(line, "MNT,%u,%u,%u", &h, &l, &f) != 3) {
        return false;
    }

    out->rot_horiz = kk_imu_mount_deg_to_steps((uint16_t)h);
    out->rot_lr = kk_imu_mount_deg_to_steps((uint16_t)l);
    out->rot_fb = kk_imu_mount_deg_to_steps((uint16_t)f);
    kk_imu_mount_sanitize(out);
    return true;
}
