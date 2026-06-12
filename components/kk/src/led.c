#include "kk/led.h"
#include "kk/time.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    int pin;
    ledc_channel_t ch;
    uint8_t duty_on;
    bool ready;
} kk_led_pwm_t;

static kk_led_pwm_t s_blue;
static kk_led_pwm_t s_green;

static bool kk_led_phase(uint32_t period_ms)
{
    return (kk_millis() / period_ms) % 2;
}

static void kk_led_pwm_out(ledc_channel_t ch, uint8_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static bool kk_led_pwm_bind(kk_led_pwm_t *slot, int pin, ledc_channel_t ch, uint8_t duty_on)
{
    if (!slot || pin < 0) {
        return false;
    }

    const ledc_channel_config_t cfg = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ch,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };

    if (ledc_channel_config(&cfg) != ESP_OK) {
        slot->ready = false;
        return false;
    }

    slot->pin = pin;
    slot->ch = ch;
    slot->duty_on = duty_on;
    slot->ready = true;
    kk_led_pwm_out(ch, 0);
    return true;
}

static void kk_led_set(int pin, bool on)
{
    if (pin == s_blue.pin && s_blue.ready) {
        kk_led_pwm_out(s_blue.ch, on ? s_blue.duty_on : 0);
        return;
    }
    if (pin == s_green.pin && s_green.ready) {
        kk_led_pwm_out(s_green.ch, on ? s_green.duty_on : 0);
        return;
    }
    gpio_set_level(pin, on ? 1 : 0);
}

void kk_led_pins_init(int pin_blue, int pin_green)
{
    static bool s_timer_ready;

    if (!s_timer_ready) {
        const ledc_timer_config_t tcfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = KK_LED_PWM_HZ,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&tcfg);
        s_timer_ready = true;
    }

    s_blue.ready = false;
    s_green.ready = false;

    if (!kk_led_pwm_bind(&s_blue, pin_blue, LEDC_CHANNEL_0, KK_LED_BLUE_DUTY)) {
        gpio_reset_pin(pin_blue);
        gpio_set_direction(pin_blue, GPIO_MODE_OUTPUT);
        gpio_set_level(pin_blue, 0);
        s_blue.pin = pin_blue;
    }

    if (!kk_led_pwm_bind(&s_green, pin_green, LEDC_CHANNEL_1, KK_LED_GREEN_DUTY)) {
        gpio_reset_pin(pin_green);
        gpio_set_direction(pin_green, GPIO_MODE_OUTPUT);
        gpio_set_level(pin_green, 0);
        s_green.pin = pin_green;
    }
}

void kk_led_boot_test(int pin_blue, int pin_green)
{
    kk_led_set(pin_green, false);
    kk_led_set(pin_blue, true);
    vTaskDelay(pdMS_TO_TICKS(800));
    kk_led_set(pin_blue, false);
    kk_led_set(pin_green, true);
    vTaskDelay(pdMS_TO_TICKS(800));
    kk_led_set(pin_green, false);
}

void kk_led_code_init(kk_led_code_player_t *p)
{
    if (!p) {
        return;
    }
    p->bit_idx = 0;
    p->on_phase = true;
    p->phase_end = 0;
}

static kk_blue_state_t kk_led_blue_pick(const kk_led_in_t *in)
{
    if (!in || !in->ble_connected) {
        return KK_BLUE_SLOW;
    }
    if (in->wifi_active) {
        return KK_BLUE_FAST;
    }
    return KK_BLUE_SOLID;
}

static void kk_led_blue_apply(int pin, kk_blue_state_t st)
{
    switch (st) {
    case KK_BLUE_SLOW:
        kk_led_set(pin, kk_led_phase(KK_LED_SLOW_MS));
        break;
    case KK_BLUE_FAST:
        kk_led_set(pin, kk_led_phase(KK_LED_FAST_MS));
        break;
    default:
        kk_led_set(pin, true);
        break;
    }
}

static kk_green_mode_t kk_led_green_pick(const kk_led_in_t *in)
{
    if (!in) {
        return KK_GREEN_SOLID;
    }
    if ((in->err_code & 0x0F) != 0) {
        return KK_GREEN_CODE;
    }
    if (in->func_prepare && !in->func_ok) {
        return KK_GREEN_FAST;
    }
    return KK_GREEN_SOLID;
}

static void kk_led_green_code_apply(int pin, uint8_t code4, kk_led_code_player_t *p)
{
    if (!p) {
        kk_led_set(pin, false);
        return;
    }
    code4 &= 0x0F;
    const uint32_t now = kk_millis();

    if (p->phase_end != 0 && now < p->phase_end) {
        return;
    }

    if (p->bit_idx >= 4) {
        kk_led_set(pin, false);
        p->phase_end = now + KK_LED_CODE_PAUSE;
        p->bit_idx = 0;
        p->on_phase = true;
        return;
    }

    if (p->on_phase) {
        const uint8_t bit = (code4 >> (3 - p->bit_idx)) & 1;
        const uint32_t on_ms = bit ? KK_LED_CODE_LONG : KK_LED_CODE_SHORT;
        kk_led_set(pin, true);
        p->phase_end = now + on_ms;
        p->on_phase = false;
        return;
    }

    kk_led_set(pin, false);
    p->phase_end = now + KK_LED_CODE_GAP;
    p->on_phase = true;
    p->bit_idx++;
}

static void kk_led_green_apply(int pin, kk_green_mode_t mode, uint8_t code4,
                               kk_led_code_player_t *p)
{
    if (mode == KK_GREEN_CODE) {
        kk_led_green_code_apply(pin, code4, p);
        return;
    }
    if (p) {
        p->bit_idx = 0;
        p->on_phase = true;
        p->phase_end = 0;
    }
    if (mode == KK_GREEN_FAST) {
        kk_led_set(pin, kk_led_phase(KK_LED_FAST_MS));
        return;
    }
    kk_led_set(pin, true);
}

void kk_led_apply(int pin_blue, int pin_green, const kk_led_in_t *in,
                  kk_led_code_player_t *code_player)
{
    if (!in) {
        return;
    }
    kk_led_blue_apply(pin_blue, kk_led_blue_pick(in));
    const kk_green_mode_t gm = kk_led_green_pick(in);
    kk_led_green_apply(pin_green, gm, in->err_code, code_player);
}
