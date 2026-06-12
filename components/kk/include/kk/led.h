#pragma once

#include <stdint.h>
#include <stdbool.h>

#define KK_LED_SLOW_MS       500
#define KK_LED_FAST_MS       120

#define KK_GCODE_NONE        0x0
#define KK_GCODE_NO_DATA     0x2
#define KK_GCODE_WIFI        0x3
#define KK_GCODE_IMU_INT     0x5
#define KK_GCODE_IMU_SPI     0xA
#define KK_GCODE_BLE         0xC

#define KK_LED_CODE_LONG     400
#define KK_LED_CODE_SHORT    150
#define KK_LED_CODE_GAP      120
#define KK_LED_CODE_PAUSE    1400

/* 1K 限流电阻：蓝尽量亮，绿降 40%（PWM 8bit） */
#define KK_LED_PWM_HZ        2000
#define KK_LED_PWM_MAX       255
#define KK_LED_BLUE_DUTY     KK_LED_PWM_MAX
#define KK_LED_GREEN_DUTY    153

typedef enum {
    KK_BLUE_SLOW = 0,
    KK_BLUE_SOLID,
    KK_BLUE_FAST,
} kk_blue_state_t;

typedef enum {
    KK_GREEN_SOLID = 0,
    KK_GREEN_FAST,
    KK_GREEN_CODE,
} kk_green_mode_t;

typedef struct {
    bool ble_connected;
    bool wifi_active;
    bool func_prepare;
    bool func_ok;
    uint8_t err_code;
} kk_led_in_t;

typedef struct {
    uint8_t bit_idx;
    bool on_phase;
    uint32_t phase_end;
} kk_led_code_player_t;

void kk_led_pins_init(int pin_blue, int pin_green);
void kk_led_boot_test(int pin_blue, int pin_green);
void kk_led_code_init(kk_led_code_player_t *p);
void kk_led_apply(int pin_blue, int pin_green, const kk_led_in_t *in,
                  kk_led_code_player_t *code_player);
