/**
 * KK-track IMU test — ESP-IDF / pure C
 * Driver: davidliyutong/esp32_bno08x_driver (MIT)
 * Pins: components/kk/include/kk/board_tx.h — SCK=4 MISO=5 MOSI=6 CS=7 INT=3 RST=10
 */
#include <stdio.h>

#include "bno08x_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "KK.IMU";

#define PIN_LED_GREEN  19
#define REPORT_US      20000U

void app_main(void)
{
    gpio_reset_pin(PIN_LED_GREEN);
    gpio_set_direction(PIN_LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED_GREEN, 0);

    ESP_LOGI(TAG, "=== KK IMU Test (ESP-IDF) ===");
    ESP_LOGI(TAG, "driver: esp32_bno08x / Hillcrest SHTP over SPI");

    BNO08x imu;
    BNO08x_config_t cfg = DEFAULT_IMU_CONFIG;

    ESP_LOGI(TAG, "SPI host=%d SCK=%d MISO=%d MOSI=%d CS=%d INT=%d RST=%d WAKE=%d %luHz",
             (int)cfg.spi_peripheral, (int)cfg.io_sclk, (int)cfg.io_miso, (int)cfg.io_mosi,
             (int)cfg.io_cs, (int)cfg.io_int, (int)cfg.io_rst, (int)cfg.io_wake,
             (unsigned long)cfg.sclk_speed);

    BNO08x_init(&imu, &cfg);

    if (!BNO08x_initialize(&imu)) {
        ESP_LOGE(TAG, "BNO08x_initialize FAILED (reset reason=%u)",
                 (unsigned)BNO08x_get_reset_reason(&imu));
        return;
    }

    ESP_LOGI(TAG, "BNO08x init OK, reset reason=%u",
             (unsigned)BNO08x_get_reset_reason(&imu));

    BNO08x_enable_game_rotation_vector(&imu, REPORT_US);
    ESP_LOGI(TAG, "GAME_ROTATION_VECTOR enabled (%u us)", (unsigned)REPORT_US);

    gpio_set_level(PIN_LED_GREEN, 1);
    ESP_LOGI(TAG, "reading — move board");

    while (1) {
        if (BNO08x_data_available(&imu)) {
            const float r = BNO08x_get_quat_real(&imu);
            const float i = BNO08x_get_quat_I(&imu);
            const float j = BNO08x_get_quat_J(&imu);
            const float k = BNO08x_get_quat_K(&imu);
            ESP_LOGI(TAG, "[GRV] r=%.4f i=%.4f j=%.4f k=%.4f", r, i, j, k);
        }
    }
}
