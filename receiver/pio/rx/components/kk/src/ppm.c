#include "kk/ppm.h"

#include "kk/board_rx.h"

#include "kk/rx_profile.h"



#include "driver/gpio.h"

#include "driver/gptimer.h"

#include "esp_attr.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include "freertos/portmacro.h"



static const char *TAG = "kk.ppm";



typedef enum {

    KK_PPM_ST_PULSE_HI = 0,

    KK_PPM_ST_GAP,

    KK_PPM_ST_SYNC,

} kk_ppm_st_t;



typedef struct {

    volatile uint16_t us[KK_PPM_CH_COUNT];

} kk_ppm_state_t;



static kk_ppm_state_t s_ppm;

static kk_ppm_state_t s_ppm_frame;

static portMUX_TYPE s_ppm_mux = portMUX_INITIALIZER_UNLOCKED;

static gptimer_handle_t s_timer;

static volatile uint8_t s_ch;

static volatile kk_ppm_st_t s_st;

static bool s_ppm_running;



static uint16_t IRAM_ATTR kk_ppm_sync_us_calc(void)

{

    uint32_t used = 0;

    for (int i = 0; i < KK_PPM_CH_COUNT; i++) {

        used += s_ppm_frame.us[i];

    }

    used += (uint32_t)(KK_PPM_CH_COUNT - 1) * KK_PPM_GAP_US;

    if (used >= KK_PPM_FRAME_US) {

        return KK_PPM_SYNC_MIN_US;

    }

    uint32_t sync = KK_PPM_FRAME_US - used;

    if (sync > 65535U) {

        sync = KK_PPM_SYNC_MIN_US;

    }

    if (sync < KK_PPM_SYNC_MIN_US) {

        sync = KK_PPM_SYNC_MIN_US;

    }

    return (uint16_t)sync;

}



static esp_err_t kk_ppm_schedule_us(uint32_t us)

{

    gptimer_alarm_config_t alarm = {

        .alarm_count = us,

        .reload_count = 0,

        .flags.auto_reload_on_alarm = false,

    };

    ESP_ERROR_CHECK(gptimer_set_raw_count(s_timer, 0));

    return gptimer_set_alarm_action(s_timer, &alarm);

}



static bool IRAM_ATTR kk_ppm_on_alarm(gptimer_handle_t timer,

                                      const gptimer_alarm_event_data_t *edata,

                                      void *user_ctx)

{

    (void)edata;

    (void)user_ctx;



    gptimer_alarm_config_t alarm = {

        .reload_count = 0,

        .flags.auto_reload_on_alarm = false,

    };



    switch (s_st) {

    case KK_PPM_ST_PULSE_HI:

        gpio_set_level(PIN_PPM, 0);

        if (s_ch < (KK_PPM_CH_COUNT - 1)) {

            s_st = KK_PPM_ST_GAP;

            alarm.alarm_count = KK_PPM_GAP_US;

        } else {

            s_st = KK_PPM_ST_SYNC;

            alarm.alarm_count = kk_ppm_sync_us_calc();

        }

        break;



    case KK_PPM_ST_GAP:

        s_ch++;

        gpio_set_level(PIN_PPM, 1);

        s_st = KK_PPM_ST_PULSE_HI;

        alarm.alarm_count = s_ppm_frame.us[s_ch];

        break;



    case KK_PPM_ST_SYNC:

    default:

        s_ch = 0;

        gpio_set_level(PIN_PPM, 1);

        s_st = KK_PPM_ST_PULSE_HI;

        alarm.alarm_count = s_ppm_frame.us[0];

        break;

    }



    gptimer_set_raw_count(timer, 0);

    gptimer_set_alarm_action(timer, &alarm);

    return false;

}



void kk_ppm_stop(void)

{

    if (!s_ppm_running || !s_timer) {

        return;

    }

    gptimer_stop(s_timer);

    gptimer_disable(s_timer);

    gpio_set_level(PIN_PPM, 0);

    s_ppm_running = false;

}



void kk_ppm_begin(void)

{

    if (s_ppm_running) {

        return;

    }



    gpio_reset_pin(PIN_PPM);

    gpio_set_direction(PIN_PPM, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_PPM, 0);



    for (int i = 0; i < KK_PPM_CH_COUNT; i++) {

        s_ppm.us[i] = KK_RX_PPM_CENTER;

        s_ppm_frame.us[i] = KK_RX_PPM_CENTER;

    }



    if (!s_timer) {

        gptimer_config_t tcfg = {

            .clk_src = GPTIMER_CLK_SRC_DEFAULT,

            .direction = GPTIMER_COUNT_UP,

            .resolution_hz = 1000000,

        };

        ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &s_timer));



        gptimer_event_callbacks_t cbs = {

            .on_alarm = kk_ppm_on_alarm,

        };

        ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer, &cbs, NULL));

    }



    s_ch = 0;

    s_st = KK_PPM_ST_PULSE_HI;

    gpio_set_level(PIN_PPM, 1);



    ESP_ERROR_CHECK(gptimer_enable(s_timer));

    ESP_ERROR_CHECK(kk_ppm_schedule_us(s_ppm_frame.us[0]));

    ESP_ERROR_CHECK(gptimer_start(s_timer));



    s_ppm_running = true;

    ESP_LOGW(TAG, "PPM backend=GPTimer %uch @%dHz frame=%uus sync=%uus gpio=%d",

             KK_PPM_CH_COUNT, KK_PPM_RATE_HZ, KK_PPM_FRAME_US,

             kk_ppm_sync_us_calc(), PIN_PPM);

}



void kk_ppm_set_channel(uint8_t index, uint16_t pulse_us)

{

    if (index >= KK_PPM_CH_COUNT) {

        return;

    }

    if (pulse_us < KK_PPM_OUT_MIN_US) {

        pulse_us = KK_PPM_OUT_MIN_US;

    }

    if (pulse_us > KK_PPM_OUT_MAX_US) {

        pulse_us = KK_PPM_OUT_MAX_US;

    }

    s_ppm.us[index] = pulse_us;

}



void kk_ppm_commit(void)

{

    portENTER_CRITICAL(&s_ppm_mux);

    for (int i = 0; i < KK_PPM_CH_COUNT; i++) {

        s_ppm_frame.us[i] = s_ppm.us[i];

    }

    portEXIT_CRITICAL(&s_ppm_mux);

}



uint16_t kk_ppm_get_channel(uint8_t index)

{

    if (index >= KK_PPM_CH_COUNT) {

        return KK_RX_PPM_CENTER;

    }

    return s_ppm.us[index];

}



void kk_ppm_fill_center(void)

{

    for (int i = 0; i < KK_PPM_CH_COUNT; i++) {

        s_ppm.us[i] = KK_RX_PPM_CENTER;

    }

    kk_ppm_commit();

}


