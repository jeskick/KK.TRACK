#include "imu_tx.h"
#include "imu_decouple.h"
#include "motion_detect.h"
#include "tx_mount.h"
#include "tx_track.h"

#include "kk/imu_mount.h"
#include "kk/link_config.h"
#include "kk/time.h"

#include "bno08x_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "kk.imu.tx";

static BNO08x s_imu;
static bool s_inited;
static bool s_has_pose;
static bool s_stability_on;
static bool s_lin_accel_on;
static float s_sensor_rel[3];
static float s_yaw_deg;
static float s_pitch_deg;
static float s_roll_deg;
static float s_yaw_raw;
static float s_pitch_raw;
static kk_quat_t s_zero_quat;
static kk_quat_t s_last_quat;
static bool s_zero_set;
static kk_imu_decouple_t s_decouple;
static uint32_t s_gyro_count;
static uint32_t s_stab_count;
static uint32_t s_lin_count;
static int8_t s_stability;
static float s_lin_accel_mps2;
static float s_gyro_pitch_dps;
static float s_gyro_yaw_dps;
static float s_gyro_roll_dps;
static uint32_t s_last_pose_ms;
static float s_pose_yaw;
static float s_pose_pitch;
static uint32_t s_last_motion_ms;
static uint32_t s_quat_count;
static uint8_t s_stall_streak;
static uint32_t s_next_recover_ms;

#ifndef KK_RAD2DEG_F
#define KK_RAD2DEG_F 57.295779513082320876798154814105f
#endif

#ifndef KK_TX_POSE_DT_MS
#define KK_TX_POSE_DT_MS (KK_TX_IMU_REPORT_US / 1000UL)
#endif

static void kk_imu_tx_on_motion_rezero(void)
{
    s_zero_set = false;
    kk_imu_decouple_reset(&s_decouple, 0.0f, 0.0f);
    s_yaw_deg = 0.0f;
    s_pitch_deg = 0.0f;
    s_pose_yaw = 0.0f;
    s_pose_pitch = 0.0f;
    ESP_LOGW(TAG, "motion settled -> IMU re-zero");
}

static void kk_imu_tx_enable_base_reports(void)
{
    BNO08x_enable_game_rotation_vector(&s_imu, KK_TX_IMU_REPORT_US);
    BNO08x_enable_gyro(&s_imu, KK_TX_IMU_REPORT_US);
}

static bool kk_imu_tx_try_recover(void)
{
    const bool mob = kk_motion_detect_is_enabled();

    ESP_LOGW(TAG, "recover: hard reset");
    kk_imu_tx_motion_sensors(false);

    if (!BNO08x_hard_reset(&s_imu)) {
        ESP_LOGE(TAG, "recover: hard reset FAIL");
        return false;
    }

    kk_imu_tx_enable_base_reports();
    kk_imu_tx_motion_sensors(mob);

    s_gyro_count = 0;
    s_stab_count = 0;
    s_lin_count = 0;
    s_quat_count = 0;
    s_stability = 0;
    s_lin_accel_mps2 = 0.0f;
    s_zero_set = false;
    s_has_pose = false;
    s_last_pose_ms = 0;
    s_last_motion_ms = 0;
    s_stall_streak = 0;
    kk_imu_decouple_reset(&s_decouple, 0.0f, 0.0f);
    kk_motion_detect_reset();
    ESP_LOGW(TAG, "recover: OK mob=%u", mob ? 1U : 0U);
    return true;
}

static void kk_imu_tx_refresh_gyro(void)
{
    const uint32_t n = BNO08x_get_gyro_update_count(&s_imu);
    if (n == s_gyro_count) {
        return;
    }
    s_gyro_count = n;

    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    uint8_t acc = 0;
    BNO08x_get_gyro_calibrated_velocity(&s_imu, &gx, &gy, &gz, &acc);
    (void)acc;

    const kk_imu_mount_t *mount = kk_tx_mount_get();
    kk_imu_mount_gyro_to_logic(mount, gx * KK_RAD2DEG_F, gy * KK_RAD2DEG_F, gz * KK_RAD2DEG_F,
                               &s_gyro_pitch_dps, &s_gyro_yaw_dps, &s_gyro_roll_dps);
}

static void kk_imu_tx_refresh_stability(void)
{
    if (!s_stability_on) {
        s_stability = 0;
        return;
    }
    const uint32_t n = BNO08x_get_stability_update_count(&s_imu);
    if (n == s_stab_count) {
        return;
    }
    s_stab_count = n;
    s_stability = BNO08x_get_stability_classifier(&s_imu);
}

static void kk_imu_tx_refresh_lin_accel(void)
{
    if (!s_lin_accel_on) {
        s_lin_accel_mps2 = 0.0f;
        return;
    }
    const uint32_t n = BNO08x_get_lin_accel_update_count(&s_imu);
    if (n == s_lin_count) {
        return;
    }
    s_lin_count = n;

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    uint8_t acc = 0;
    BNO08x_get_linear_accel(&s_imu, &ax, &ay, &az, &acc);
    (void)acc;
    s_lin_accel_mps2 = sqrtf(ax * ax + ay * ay + az * az);
}

static void kk_imu_tx_motion_step(uint32_t elapsed_ms)
{
    kk_motion_detect_apply(s_pose_yaw, s_pose_pitch, s_gyro_pitch_dps, s_gyro_yaw_dps,
                           s_gyro_roll_dps, s_stability, s_lin_accel_mps2, elapsed_ms,
                           &s_yaw_deg, &s_pitch_deg);
}

static void kk_imu_tx_update_mapped(const kk_quat_t *q_now)
{
    const kk_imu_mount_t *mount = kk_tx_mount_get();
    const kk_tx_track_cfg_t *track = kk_tx_track_get();

    kk_imu_mount_apply_quat(q_now, &s_zero_quat, mount, &s_yaw_raw, &s_pitch_raw, &s_roll_deg);

    float pose_yaw = s_yaw_raw;
    float pose_pitch = s_pitch_raw;
    kk_imu_decouple_apply(&s_decouple, s_yaw_raw, s_pitch_raw, s_roll_deg, s_gyro_pitch_dps,
                          s_gyro_yaw_dps, s_gyro_roll_dps, track, &pose_yaw, &pose_pitch);
    s_pose_yaw = pose_yaw;
    s_pose_pitch = pose_pitch;

    uint32_t elapsed_ms = KK_TX_POSE_DT_MS;
    const uint32_t now = kk_millis();
    if (s_last_pose_ms != 0 && now > s_last_pose_ms) {
        elapsed_ms = now - s_last_pose_ms;
        if (elapsed_ms > 100U) {
            elapsed_ms = 100U;
        }
    }
    s_last_pose_ms = now;
    s_last_motion_ms = now;

    kk_imu_tx_motion_step(elapsed_ms);

    if (s_zero_set) {
        const kk_quat_t q_rel = kk_quat_rel(q_now, &s_zero_quat);
        kk_imu_quat_to_sensor_rel(&q_rel, s_sensor_rel);
    }
}

bool kk_imu_tx_init(void)
{
    if (s_inited) {
        return true;
    }

    kk_tx_mount_load(NULL);
    kk_tx_track_load(NULL);
    kk_motion_detect_init(kk_imu_tx_on_motion_rezero);

    BNO08x_config_t cfg = DEFAULT_IMU_CONFIG;
    ESP_LOGW(TAG, "init SPI host=%d SCK=%d MISO=%d MOSI=%d CS=%d INT=%d RST=%d %luHz",
             (int)cfg.spi_peripheral, (int)cfg.io_sclk, (int)cfg.io_miso, (int)cfg.io_mosi,
             (int)cfg.io_cs, (int)cfg.io_int, (int)cfg.io_rst,
             (unsigned long)cfg.sclk_speed);

    BNO08x_init(&s_imu, &cfg);
    if (!BNO08x_initialize(&s_imu)) {
        ESP_LOGE(TAG, "BNO08x_initialize FAIL reason=%u",
                 (unsigned)BNO08x_get_reset_reason(&s_imu));
        return false;
    }

    BNO08x_enable_game_rotation_vector(&s_imu, KK_TX_IMU_REPORT_US);
    BNO08x_enable_gyro(&s_imu, KK_TX_IMU_REPORT_US);
    kk_imu_tx_motion_sensors(kk_tx_track_get()->motion_en);

    s_inited = true;
    s_has_pose = false;
    s_zero_set = false;
    s_gyro_count = 0;
    s_stab_count = 0;
    s_lin_count = 0;
    s_quat_count = 0;
    s_stall_streak = 0;
    s_next_recover_ms = 0;
    s_stability = 0;
    s_lin_accel_mps2 = 0.0f;
    s_last_pose_ms = 0;
    s_gyro_pitch_dps = 0.0f;
    s_gyro_yaw_dps = 0.0f;
    s_gyro_roll_dps = 0.0f;
    kk_imu_decouple_reset(&s_decouple, 0.0f, 0.0f);
    ESP_LOGW(TAG, "BNO08x OK GRV+gyro %luus", (unsigned long)KK_TX_IMU_REPORT_US);
    return true;
}

void kk_imu_tx_motion_sensors(bool enable)
{
    if (!s_inited) {
        s_stability_on = enable;
        s_lin_accel_on = enable;
        return;
    }
    if (enable) {
        if (!s_stability_on) {
            s_stability_on = true;
            BNO08x_enable_stability_classifier(&s_imu, KK_MOB_STABILITY_REPORT_US);
            ESP_LOGW(TAG, "stability classifier on");
        }
        vTaskDelay(pdMS_TO_TICKS(40));
        if (!s_lin_accel_on) {
            s_lin_accel_on = true;
            BNO08x_enable_linear_accelerometer(&s_imu, KK_MOB_LIN_ACCEL_REPORT_US);
            ESP_LOGW(TAG, "linear accel on");
        }
        return;
    }
    if (s_stability_on) {
        s_stability_on = false;
        BNO08x_disable_stability_classifier(&s_imu);
        s_stability = 0;
        s_stab_count = 0;
        ESP_LOGW(TAG, "stability classifier off");
    }
    if (s_lin_accel_on) {
        s_lin_accel_on = false;
        BNO08x_disable_linear_accelerometer(&s_imu);
        s_lin_accel_mps2 = 0.0f;
        s_lin_count = 0;
        ESP_LOGW(TAG, "linear accel off");
    }
}

bool kk_imu_tx_poll(void)
{
    if (!s_inited) {
        return false;
    }

    const uint32_t now = kk_millis();
    if (s_stall_streak >= KK_TX_IMU_STALL_FAIL_MAX) {
        if (now < s_next_recover_ms) {
            return false;
        }
        s_next_recover_ms = now + KK_TX_IMU_STALL_RECOVER_MS;
        (void)kk_imu_tx_try_recover();
        return false;
    }

    if (!BNO08x_data_available(&s_imu)) {
        s_stall_streak++;
        if (s_stall_streak >= KK_TX_IMU_STALL_FAIL_MAX) {
            s_has_pose = false;
            s_next_recover_ms = now + KK_TX_IMU_STALL_RECOVER_MS;
            ESP_LOGW(TAG, "stall streak=%u -> recover in %lums", s_stall_streak,
                     (unsigned long)KK_TX_IMU_STALL_RECOVER_MS);
        }
        return false;
    }
    s_stall_streak = 0;

    const uint32_t gyro_before = s_gyro_count;
    const uint32_t stab_before = s_stab_count;
    const uint32_t lin_before = s_lin_count;

    kk_imu_tx_refresh_gyro();
    kk_imu_tx_refresh_stability();
    kk_imu_tx_refresh_lin_accel();

    const bool sensors_new =
        s_gyro_count != gyro_before || s_stab_count != stab_before || s_lin_count != lin_before;

    const uint32_t quat_n = BNO08x_get_quat_update_count(&s_imu);
    if (quat_n != s_quat_count) {
        s_quat_count = quat_n;

        float qi = 0.0f;
        float qj = 0.0f;
        float qk = 0.0f;
        float qr = 0.0f;
        float rad_acc = 0.0f;
        uint8_t acc = 0;
        BNO08x_get_quat(&s_imu, &qi, &qj, &qk, &qr, &rad_acc, &acc);
        (void)rad_acc;
        (void)acc;
        if (!isfinite(qi) || !isfinite(qj) || !isfinite(qk) || !isfinite(qr)) {
            ESP_LOGW(TAG, "quat non-finite, skip");
            return false;
        }
        s_last_quat = kk_quat_from_bno(qi, qj, qk, qr);

        if (!s_zero_set) {
            s_zero_quat = s_last_quat;
            s_zero_set = true;
            kk_imu_decouple_reset(&s_decouple, 0.0f, 0.0f);
            kk_motion_detect_reset();
            ESP_LOGW(TAG, "zero quat w=%.3f x=%.3f y=%.3f z=%.3f", s_zero_quat.w, s_zero_quat.x,
                     s_zero_quat.y, s_zero_quat.z);
        }

        kk_imu_tx_update_mapped(&s_last_quat);
        s_has_pose = true;
        return true;
    }

    if (s_has_pose && kk_motion_detect_is_enabled() && sensors_new) {
        uint32_t elapsed_ms = KK_TX_POSE_DT_MS;
        if (s_last_motion_ms != 0 && now > s_last_motion_ms) {
            elapsed_ms = now - s_last_motion_ms;
            if (elapsed_ms > 100U) {
                elapsed_ms = 100U;
            }
        }
        s_last_motion_ms = now;
        kk_imu_tx_motion_step(elapsed_ms);
        return true;
    }

    return false;
}

bool kk_imu_tx_ready(void)
{
    return s_inited;
}

bool kk_imu_tx_has_pose(void)
{
    return s_has_pose;
}

bool kk_imu_tx_is_motion_paused(void)
{
    return kk_motion_detect_is_paused();
}

float kk_imu_tx_roll_deg(void)
{
    return s_roll_deg;
}

float kk_imu_tx_pitch_deg(void)
{
    return s_pitch_deg;
}

float kk_imu_tx_yaw_deg(void)
{
    return s_yaw_deg;
}

float kk_imu_tx_sensor_deg(uint8_t axis)
{
    if (axis > 2) {
        return 0.0f;
    }
    return s_sensor_rel[axis];
}

void kk_imu_tx_rezero(void)
{
    s_zero_set = false;
    kk_imu_decouple_reset(&s_decouple, 0.0f, 0.0f);
    kk_motion_detect_reset();
}

void kk_imu_tx_set_mount(const kk_imu_mount_t *mount)
{
    kk_tx_mount_apply(mount);
    if (s_has_pose) {
        kk_imu_mount_apply_quat(&s_last_quat, &s_zero_quat, kk_tx_mount_get(), &s_yaw_raw,
                                &s_pitch_raw, &s_roll_deg);
        kk_imu_decouple_reset(&s_decouple, s_yaw_raw, s_pitch_raw);
        kk_imu_tx_update_mapped(&s_last_quat);
    }
}

void kk_imu_tx_apply_track_cfg(const kk_tx_track_cfg_t *cfg)
{
    kk_tx_track_apply(cfg);
    if (cfg) {
        kk_motion_detect_set_enabled(cfg->motion_en);
        kk_imu_tx_motion_sensors(cfg->motion_en);
        ESP_LOGW(TAG, "track mob=%u (sensors %s)", cfg->motion_en ? 1U : 0U,
                 cfg->motion_en ? "on" : "off");
    } else {
        kk_motion_detect_set_enabled(false);
        kk_imu_tx_motion_sensors(false);
    }
}

bool kk_imu_tx_motion_enabled(void)
{
    return kk_motion_detect_is_enabled();
}

float kk_imu_tx_motion_lin_mps2(void)
{
    return s_lin_accel_mps2;
}

int8_t kk_imu_tx_motion_stability(void)
{
    return s_stability;
}

uint32_t kk_imu_tx_motion_trigger_ms(void)
{
    return kk_motion_detect_trigger_ms();
}
