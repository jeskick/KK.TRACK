#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float rx_voltage;
    float tx_voltage;
    float yaw_deg;
    float pitch_deg;
    uint32_t last_pkt_ms;
    uint32_t last_txv_ms;
    bool rx_v_valid;
    bool tx_v_valid;
    bool motion_paused;
} kk_telemetry_t;

extern kk_telemetry_t g_kk_tel;

void kk_tel_reset(void);
int kk_tel_format_pose(char *buf, size_t cap, float yaw_deg, float pitch_deg, bool motion_paused);
void kk_tel_on_udp_payload(const char *buf);
void kk_tel_poll_rx_voltage(void);
void kk_tel_poll_link(void);
