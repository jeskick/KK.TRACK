#include "wifi_rx.h"
#include "ble_rx.h"

#include "kk/head_track.h"
#include "kk/link_config.h"
#include "kk/rx_profile.h"
#include "kk/rx_ota.h"
#include "kk/rx_web.h"

extern kk_rx_profile_t g_profile;
#include "kk/telemetry.h"
#include "kk/time.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include <string.h>

static const char *TAG = "kk.wifi.rx";

static bool s_stack_ready;
static bool s_wifi_on;
static bool s_ap_ready;
static bool s_udp_on;
static int s_udp_sock = -1;
static uint32_t s_boot_until;
static uint32_t s_sta_join_ms;
static uint32_t s_sta_left_ms;
static bool s_had_sta;

static void kk_wifi_rx_sta_check(void)
{
    if (kk_rx_ota_is_active()) {
        return;
    }
    wifi_sta_list_t sta;
    if (esp_wifi_ap_get_sta_list(&sta) != ESP_OK) {
        return;
    }
    if (s_had_sta && sta.num == 0) {
        ESP_LOGW(TAG, "STA left -> AP off");
        kk_wifi_rx_ap_stop();
    }
}

static void kk_wifi_rx_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;

    switch (id) {
    case WIFI_EVENT_AP_START:
        s_ap_ready = true;
        ESP_LOGW(TAG, "AP started");
        break;
    case WIFI_EVENT_AP_STOP:
        s_ap_ready = false;
        break;
    case WIFI_EVENT_AP_STACONNECTED: {
        const wifi_event_ap_staconnected_t *ev = (const wifi_event_ap_staconnected_t *)data;
        s_had_sta = true;
        s_sta_join_ms = kk_millis();
        ESP_LOGW(TAG, "STA joined aid=%u", (unsigned)ev->aid);
        break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED: {
        const wifi_event_ap_stadisconnected_t *ev = (const wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGW(TAG, "STA left aid=%u", (unsigned)ev->aid);
        kk_wifi_rx_sta_check();
        break;
    }
    default:
        break;
    }
}

static esp_err_t kk_wifi_rx_ensure_stack(void)
{
    if (s_stack_ready) {
        return ESP_OK;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, kk_wifi_rx_event, NULL, NULL));

    s_stack_ready = true;
    ESP_LOGW(TAG, "WiFi stack init");
    return ESP_OK;
}

void kk_wifi_rx_init(void)
{
    /* WiFi off until BLE link stable (PPM on) */
}

bool kk_wifi_rx_is_on(void)
{
    return s_wifi_on;
}

bool kk_wifi_rx_ap_ready(void)
{
    return s_ap_ready;
}

void kk_wifi_rx_ap_stop(void)
{
    /* OTA 进行中不主动关 AP；非 OTA 才执行正常关 AP 逻辑 */
    if (kk_rx_ota_is_active()) {
        return;
    }

    if (s_udp_sock >= 0) {
        close(s_udp_sock);
        s_udp_sock = -1;
    }
    s_udp_on = false;
    s_ap_ready = false;
    if (s_wifi_on) {
        esp_wifi_stop();
        s_wifi_on = false;
        ESP_LOGW(TAG, "AP off");
    }
    s_had_sta = false;
    s_sta_join_ms = 0;
    s_sta_left_ms = 0;
    s_boot_until = 0;
}

void kk_wifi_rx_ap_start(void)
{
    if (s_wifi_on) {
        return;
    }

    kk_wifi_rx_ensure_stack();

    wifi_config_t ap = {0};
    strncpy((char *)ap.ap.ssid, KK_WIFI_SSID, sizeof(ap.ap.ssid) - 1);
    ap.ap.ssid[sizeof(ap.ap.ssid) - 1] = '\0';
    ap.ap.ssid_len = (uint8_t)strlen(KK_WIFI_SSID);
    strncpy((char *)ap.ap.password, KK_WIFI_PASS, sizeof(ap.ap.password) - 1);
    ap.ap.channel = KK_WIFI_CHANNEL;
    ap.ap.max_connection = KK_WIFI_MAX_STA;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(KK_WIFI_TX_POWER_QDBM));
    s_wifi_on = true;
    s_boot_until = kk_millis() + KK_WIFI_BOOT_WAIT_MS;
    s_had_sta = false;
    s_sta_join_ms = 0;
    s_sta_left_ms = 0;
    ESP_LOGW(TAG, "AP starting ssid=%s ch=%d tx=%.1fdBm boot=%lus http_idle=%lus",
             KK_WIFI_SSID, KK_WIFI_CHANNEL,
             KK_WIFI_TX_POWER_QDBM * 0.25f,
             (unsigned long)(KK_WIFI_BOOT_WAIT_MS / 1000UL),
             (unsigned long)(KK_WIFI_IDLE_MS / 1000UL));
}

static bool kk_wifi_rx_have_sta(void)
{
    wifi_sta_list_t sta;
    if (esp_wifi_ap_get_sta_list(&sta) != ESP_OK) {
        return false;
    }
    return sta.num > 0;
}

static void kk_wifi_rx_idle_timeout(uint32_t now)
{
    uint32_t last_http = kk_rx_web_last_http_ms();
    uint32_t deadline;

    if (last_http > 0) {
        deadline = last_http + KK_WIFI_IDLE_MS;
    } else if (s_sta_join_ms > 0) {
        deadline = s_sta_join_ms + KK_WIFI_JOIN_DELAY_MS + KK_WIFI_IDLE_MS;
    } else {
        return;
    }

    if (now >= deadline) {
        ESP_LOGW(TAG, "HTTP idle %lus -> AP off",
                 (unsigned long)(KK_WIFI_IDLE_MS / 1000UL));
        kk_wifi_rx_ap_stop();
    }
}

void kk_wifi_rx_idle_poll(void)
{
    if (!s_wifi_on) {
        return;
    }

    if (kk_rx_ota_is_active()) {
        return; /* OTA 期间跳过所有关 AP 检查 */
    }

    if (!kk_ble_rx_is_connected()) {
        ESP_LOGW(TAG, "BLE down -> AP off");
        kk_wifi_rx_ap_stop();
        return;
    }

    const uint32_t now = kk_millis();
    const bool have_sta = kk_wifi_rx_have_sta();

    if (have_sta) {
        s_had_sta = true;
        s_sta_left_ms = 0;
    }

    if (!s_had_sta) {
        if (now >= s_boot_until) {
            ESP_LOGW(TAG, "no STA in %lus -> AP off",
                     (unsigned long)(KK_WIFI_BOOT_WAIT_MS / 1000UL));
            kk_wifi_rx_ap_stop();
        }
        return;
    }

    if (!have_sta) {
        if (s_sta_left_ms == 0) {
            s_sta_left_ms = now;
            ESP_LOGW(TAG, "STA left, grace %lus before AP off",
                     (unsigned long)(KK_WIFI_STA_GRACE_MS / 1000UL));
            return;
        }
        if (now - s_sta_left_ms < KK_WIFI_STA_GRACE_MS) {
            return;
        }
        ESP_LOGW(TAG, "STA grace expired -> AP off");
        kk_wifi_rx_ap_stop();
        return;
    }

    kk_wifi_rx_idle_timeout(now);
}

void kk_wifi_rx_udp_poll(void)
{
    if (!s_wifi_on) {
        return;
    }
    if (!s_udp_on) {
        s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (s_udp_sock < 0) {
            return;
        }
        struct sockaddr_in addr = {0};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(KK_UDP_PORT);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(s_udp_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
            close(s_udp_sock);
            s_udp_sock = -1;
            return;
        }
        s_udp_on = true;
    }

    char buf[96];
    struct sockaddr_in src;
    socklen_t slen = sizeof(src);
    int n = recvfrom(s_udp_sock, buf, sizeof(buf) - 1, MSG_DONTWAIT,
                     (struct sockaddr *)&src, &slen);
    if (n <= 0) {
        return;
    }
    buf[n] = '\0';
    kk_tel_on_udp_payload(buf);

    const char ack[] = "ACK";
    sendto(s_udp_sock, ack, sizeof(ack) - 1, 0, (struct sockaddr *)&src, slen);
}
