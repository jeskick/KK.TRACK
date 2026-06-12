#include "imu_tx.h"
#include "tx_mount.h"

#include "kk/imu_mount.h"
#include "kk/link_config.h"

#include "bno08x_driver.h"
#include "esp_log.h"

static const char *TAG = "kk.imu.tx";

static BNO08x s_imu;
static bool s_inited;
static bool s_has_pose;
static float s_sensor_rel[3];
static float s_yaw_deg;
static float s_pitch_deg;
static float s_roll_deg;
static kk_quat_t s_zero_quat;
static kk_quat_t s_last_quat;
static bool s_zero_set;

static void kk_imu_tx_read_quat(kk_quat_t *out)
{
    float qi = 0.0f;
    float qj = 0.0f;
    float qk = 0.0f;
    float qr = 0.0f;
    float rad_acc = 0.0f;
    uint8_t acc = 0;
    /* driver always writes rad_accuracy/accuracy — do not pass NULL */
    BNO08x_get_quat(&s_imu, &qi, &qj, &qk, &qr, &rad_acc, &acc);
    (void)rad_acc;
    (void)acc;
    *out = kk_quat_from_bno(qi, qj, qk, qr);
}

static void kk_imu_tx_update_mapped(const kk_quat_t *q_now)
{
    const kk_imu_mount_t *mount = kk_tx_mount_get();

    kk_imu_mount_apply_quat(q_now, &s_zero_quat, mount, &s_yaw_deg, &s_pitch_deg,
                            &s_roll_deg);

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
    s_inited = true;
    s_has_pose = false;
    s_zero_set = false;
    ESP_LOGW(TAG, "BNO08x OK GRV %luus mount=quat YAW=Z PITCH=X",
             (unsigned long)KK_TX_IMU_REPORT_US);
    return true;
}

bool kk_imu_tx_poll(void)
{
    if (!s_inited) {
        return false;
    }
    if (!BNO08x_data_available(&s_imu)) {
        return false;
    }

    kk_imu_tx_read_quat(&s_last_quat);

    if (!s_zero_set) {
        s_zero_quat = s_last_quat;
        s_zero_set = true;
        ESP_LOGW(TAG, "zero quat w=%.3f x=%.3f y=%.3f z=%.3f", s_zero_quat.w, s_zero_quat.x,
                 s_zero_quat.y, s_zero_quat.z);
    }

    kk_imu_tx_update_mapped(&s_last_quat);
    s_has_pose = true;
    return true;
}

bool kk_imu_tx_ready(void)
{
    return s_inited;
}

bool kk_imu_tx_has_pose(void)
{
    return s_has_pose;
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
}

void kk_imu_tx_set_mount(const kk_imu_mount_t *mount)
{
    kk_tx_mount_apply(mount);
    if (s_has_pose) {
        kk_imu_tx_update_mapped(&s_last_quat);
    }
}
