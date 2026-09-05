#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "app_wifi.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "app_wifi";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_events;
static int s_retry_count;
static bool s_started;

/*
 * Reconnect is driven by a one-shot esp_timer, not by sleeping inside the
 * handler. Event handlers run on the shared `sys_evt` task: blocking there
 * stalls every other event - including IP_EVENT_STA_GOT_IP, the very event
 * that would end the wait - and delays Wi-Fi driver callbacks.
 */
static esp_timer_handle_t s_reconnect_timer;

static void reconnect_timer_cb(void *arg)
{
    (void)arg;
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
        ESP_LOGW(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
    }
}

static void schedule_reconnect(void)
{
    /* Progressive back-off: 0.5 s, 1 s, 2 s ... capped at 30 s. The original
     * sketch spun on delay(500) forever, which never recovered from a wrong
     * password and blocked every sensor read while it tried. */
    int shift = s_retry_count > 6 ? 6 : s_retry_count;
    uint64_t delay_us = (uint64_t)(500 << shift) * 1000ULL;
    if (delay_us > 30ULL * 1000000ULL) {
        delay_us = 30ULL * 1000000ULL;
    }
    s_retry_count++;

    /* Stopping first makes the call idempotent if two disconnect events land
     * back to back; esp_timer_stop on an idle timer is a harmless no-op. */
    esp_timer_stop(s_reconnect_timer);
    esp_err_t err = esp_timer_start_once(s_reconnect_timer, delay_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cannot arm the reconnect timer: %s", esp_err_to_name(err));
    }
    ESP_LOGW(TAG, "retry %d scheduled in %llu ms", s_retry_count, delay_us / 1000);
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *ev = event_data;
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);

        ESP_LOGW(TAG, "disconnected, reason %d%s", ev ? ev->reason : -1,
                 (ev && ev->reason == WIFI_REASON_AUTH_FAIL)
                     ? " (authentication failed - check the password)" : "");

        if (s_retry_count + 1 >= CONFIG_IRRIG_WIFI_MAX_RETRY) {
            xEventGroupSetBits(s_wifi_events, WIFI_FAIL_BIT);
        }
        schedule_reconnect();
        return;
    }

    if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = event_data;
        ESP_LOGI(TAG, "connected, IP " IPSTR, IP2STR(&ev->ip_info.ip));
        s_retry_count = 0;
        esp_timer_stop(s_reconnect_timer);
        xEventGroupClearBits(s_wifi_events, WIFI_FAIL_BIT);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

esp_err_t app_wifi_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* Wi-Fi calibration data lives in NVS; a stale or full partition after
         * a firmware change makes esp_wifi_init() fail. */
        ESP_LOGW(TAG, "NVS partition needs erasing, reformatting");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_flash_init failed");

    s_wifi_events = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_wifi_events != NULL, ESP_ERR_NO_MEM, TAG,
                        "cannot allocate event group");

    const esp_timer_create_args_t timer_args = {
        .callback = reconnect_timer_cb,
        .name     = "wifi_reconnect",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_reconnect_timer), TAG,
                        "cannot create the reconnect timer");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "esp_wifi_init failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                            WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL),
                        TAG, "cannot register Wi-Fi handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                            IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL),
                        TAG, "cannot register IP handler");

    wifi_config_t sta_cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {.capable = true, .required = false},
        },
    };
    strlcpy((char *)sta_cfg.sta.ssid, CONFIG_IRRIG_WIFI_SSID, sizeof(sta_cfg.sta.ssid));
    strlcpy((char *)sta_cfg.sta.password, CONFIG_IRRIG_WIFI_PASSWORD,
            sizeof(sta_cfg.sta.password));

    /* An empty password means an open network; leaving the WPA2 threshold in
     * place would reject it during the scan. */
    if (strlen(CONFIG_IRRIG_WIFI_PASSWORD) == 0) {
        sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set_mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg), TAG, "set_config failed");

    /* Modem sleep saves power but adds latency to inbound MQTT traffic; keep
     * the default (min modem) and disable the sleep-through-DTIM aggressively
     * enough that keepalives are answered on time. */
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_MIN_MODEM), TAG, "set_ps failed");

    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");

    s_started = true;
    ESP_LOGI(TAG, "station started, SSID \"%s\"", CONFIG_IRRIG_WIFI_SSID);
    return ESP_OK;
}

esp_err_t app_wifi_wait_connected(uint32_t timeout_ms)
{
    if (s_wifi_events == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdFALSE, ticks);
    return (bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool app_wifi_is_connected(void)
{
    if (s_wifi_events == NULL) {
        return false;
    }
    return (xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT) != 0;
}

esp_err_t app_time_sync(uint32_t timeout_ms)
{
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    cfg.start = true;
    cfg.server_from_dhcp = true;
    cfg.renew_servers_after_new_IP = true;
    cfg.sync_cb = NULL;

    esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        /* Already initialised by an earlier call - just wait again. */
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "SNTP init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(timeout_ms));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SNTP did not sync within %" PRIu32 " ms; TLS certificate "
                      "dates cannot be validated yet", timeout_ms);
        return err;
    }

    time_t now = 0;
    struct tm tm_utc = {0};
    time(&now);
    gmtime_r(&now, &tm_utc);
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC", &tm_utc);
    ESP_LOGI(TAG, "clock synchronised: %s", buf);
    return ESP_OK;
}
