#include "kk/rc_out.h"

#include "kk/board_rx.h"
#include "kk/ppm.h"
#include "kk/rx_profile.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/portmacro.h"
#include <string.h>

static const char *TAG = "kk.rc_out";

static kk_rc_proto_t s_proto = KK_RC_PROTO_PPM;
static bool s_running;
static uint16_t s_ch_us[KK_RC_CH_COUNT];
static uint16_t s_ch_frame[KK_RC_CH_COUNT];
static bool s_failsafe;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static uart_port_t s_uart = UART_NUM_1;
static esp_timer_handle_t s_serial_timer;
static bool s_uart_on;

static uint16_t kk_rc_us_to_serial(uint16_t us)
{
    /* Betaflight/CRSF：992↔1500µs 严格对中，避免 SBUS 与 PPM 中位偏差 */
    float v = 992.0f + ((float)us - 1500.0f) / 0.625f;
    if (v < 172.0f) {
        v = 172.0f;
    }
    if (v > 1811.0f) {
        v = 1811.0f;
    }
    return (uint16_t)(v + 0.5f);
}

static void kk_rc_pack_11bit(const uint16_t vals[16], uint8_t out[22])
{
    memset(out, 0, 22);
    uint32_t bitpos = 0;
    for (int i = 0; i < 16; i++) {
        const uint32_t v = vals[i] & 0x7FFU;
        for (int b = 0; b < 11; b++) {
            if (v & (1U << b)) {
                out[bitpos / 8] |= (uint8_t)(1U << (bitpos % 8));
            }
            bitpos++;
        }
    }
}

static uint8_t kk_rc_crc8_dvb_s2(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (uint8_t)((crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1));
        }
    }
    return crc;
}

static void kk_rc_serial_channels(uint16_t vals[16])
{
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < KK_RC_CH_COUNT; i++) {
        vals[i] = kk_rc_us_to_serial(s_ch_frame[i]);
    }
    portEXIT_CRITICAL(&s_mux);
    for (int i = KK_RC_CH_COUNT; i < 16; i++) {
        vals[i] = 992;
    }
}

static void kk_rc_send_sbus(void)
{
    uint16_t vals[16];
    uint8_t packed[22];
    kk_rc_serial_channels(vals);
    kk_rc_pack_11bit(vals, packed);

    uint8_t frame[25];
    frame[0] = 0x0F;
    memcpy(&frame[1], packed, 22);
    frame[23] = s_failsafe ? 0x0CU : 0U; /* bit2 frame lost + bit3 failsafe */
    frame[24] = 0;
    uart_write_bytes(s_uart, (const char *)frame, sizeof(frame));
}

static void kk_rc_send_crsf(void)
{
    /* 单向输出：仅向飞控发送 RC_CHANNELS_PACKED(0x16)，不接收遥测/回传 */
    uint16_t vals[16];
    uint8_t packed[22];
    kk_rc_serial_channels(vals);
    kk_rc_pack_11bit(vals, packed);

    uint8_t frame[26];
    frame[0] = KK_CRSF_ADDR_FC;
    frame[1] = 24;
    frame[2] = 0x16;
    memcpy(&frame[3], packed, 22);
    frame[25] = kk_rc_crc8_dvb_s2(&frame[2], 23);
    uart_write_bytes(s_uart, (const char *)frame, sizeof(frame));
}

static void kk_rc_serial_timer_cb(void *arg)
{
    (void)arg;
    if (!s_running || s_proto == KK_RC_PROTO_PPM) {
        return;
    }
    if (s_proto == KK_RC_PROTO_SBUS) {
        kk_rc_send_sbus();
    } else {
        kk_rc_send_crsf();
    }
}

static void kk_rc_serial_stop(void)
{
    if (s_serial_timer) {
        esp_timer_stop(s_serial_timer);
        esp_timer_delete(s_serial_timer);
        s_serial_timer = NULL;
    }
    if (s_uart_on) {
        uart_driver_delete(s_uart);
        s_uart_on = false;
    }
}

static esp_err_t kk_rc_serial_begin(kk_rc_proto_t proto)
{
    const int baud = proto == KK_RC_PROTO_SBUS ? 100000 : 420000;
    const uart_parity_t parity = proto == KK_RC_PROTO_SBUS ? UART_PARITY_EVEN : UART_PARITY_DISABLE;
    const uart_stop_bits_t stop =
        proto == KK_RC_PROTO_SBUS ? UART_STOP_BITS_2 : UART_STOP_BITS_1;

    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = parity,
        .stop_bits = stop,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* ESP-IDF 要求 rx_buffer>0；TX-only 仍分配最小 RX 并 flush，不读回 */
    esp_err_t err = uart_driver_install(s_uart, 256, 256, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart install fail %d", (int)err);
        return err;
    }
    s_uart_on = true;
    ESP_ERROR_CHECK(uart_param_config(s_uart, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(s_uart, PIN_PPM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_line_inverse(s_uart, UART_SIGNAL_TXD_INV));
    uart_flush_input(s_uart);

    const int64_t period_us = proto == KK_RC_PROTO_SBUS ? 9000 : 4000;
    const esp_timer_create_args_t targs = {
        .callback = kk_rc_serial_timer_cb,
        .name = "rc_serial",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_serial_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_serial_timer, period_us));
    return ESP_OK;
}

uint8_t kk_rc_out_sanitize_proto(uint8_t v)
{
    if (v > KK_RC_PROTO_CRSF) {
        return KK_RC_PROTO_PPM;
    }
    return v;
}

kk_rc_proto_t kk_rc_out_get_proto(void)
{
    return s_proto;
}

bool kk_rc_out_is_running(void)
{
    return s_running;
}

void kk_rc_out_stop(void)
{
    if (!s_running) {
        return;
    }
    if (s_proto == KK_RC_PROTO_PPM) {
        kk_ppm_stop();
    } else {
        kk_rc_serial_stop();
    }
    s_running = false;
}

void kk_rc_out_begin(kk_rc_proto_t proto)
{
    proto = (kk_rc_proto_t)kk_rc_out_sanitize_proto((uint8_t)proto);
    if (s_running && s_proto == proto) {
        return;
    }
    kk_rc_out_stop();
    s_proto = proto;

    for (int i = 0; i < KK_RC_CH_COUNT; i++) {
        s_ch_us[i] = KK_RX_PPM_CENTER;
        s_ch_frame[i] = KK_RX_PPM_CENTER;
    }

    if (proto == KK_RC_PROTO_PPM) {
        kk_ppm_begin();
        ESP_LOGW(TAG, "PPM gpio=%d", PIN_PPM);
    } else {
        if (kk_rc_serial_begin(proto) != ESP_OK) {
            ESP_LOGE(TAG, "serial begin fail -> PPM fallback gpio=%d", PIN_PPM);
            s_proto = KK_RC_PROTO_PPM;
            kk_ppm_begin();
        } else {
            ESP_LOGW(TAG, "%s uart tx-only gpio=%d",
                     proto == KK_RC_PROTO_SBUS ? "SBUS" : "CRSF(TX only)", PIN_PPM);
        }
    }
    s_running = true;
}

void kk_rc_out_set_channel(uint8_t index, uint16_t pulse_us)
{
    if (index >= KK_RC_CH_COUNT) {
        return;
    }
    if (pulse_us < KK_PPM_OUT_MIN_US) {
        pulse_us = KK_PPM_OUT_MIN_US;
    }
    if (pulse_us > KK_PPM_OUT_MAX_US) {
        pulse_us = KK_PPM_OUT_MAX_US;
    }
    if (s_proto == KK_RC_PROTO_PPM) {
        kk_ppm_set_channel(index, pulse_us);
        return;
    }
    portENTER_CRITICAL(&s_mux);
    s_ch_us[index] = pulse_us;
    portEXIT_CRITICAL(&s_mux);
}

void kk_rc_out_commit(void)
{
    if (s_proto == KK_RC_PROTO_PPM) {
        kk_ppm_commit();
        return;
    }
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < KK_RC_CH_COUNT; i++) {
        s_ch_frame[i] = s_ch_us[i];
    }
    portEXIT_CRITICAL(&s_mux);
}

void kk_rc_out_set_failsafe(bool active)
{
    s_failsafe = active;
}

uint16_t kk_rc_out_get_channel(uint8_t index)
{
    if (index >= KK_RC_CH_COUNT) {
        return KK_RX_PPM_CENTER;
    }
    if (s_proto == KK_RC_PROTO_PPM) {
        return kk_ppm_get_channel(index);
    }
    return s_ch_us[index];
}

void kk_rc_out_fill_center(void)
{
    for (int i = 0; i < KK_RC_CH_COUNT; i++) {
        kk_rc_out_set_channel(i, KK_RX_PPM_CENTER);
    }
    kk_rc_out_commit();
}
