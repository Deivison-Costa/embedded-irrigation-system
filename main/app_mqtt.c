#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_mqtt.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"

static const char *TAG = "app_mqtt";

/* Embedded by EMBED_TXTFILES in main/CMakeLists.txt. */
extern const uint8_t mqtt_ca_pem_start[] asm("_binary_mqtt_ca_pem_start");
extern const uint8_t mqtt_ca_pem_end[]   asm("_binary_mqtt_ca_pem_end");

#define MQTT_TOPIC_MAX   128
#define MQTT_PAYLOAD_MAX 160

static esp_mqtt_client_handle_t s_client;
static volatile bool s_connected;
static char s_client_id[64];
static char s_lwt_topic[MQTT_TOPIC_MAX];

static void mqtt_event_handler(void *args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    const esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        ESP_LOGI(TAG, "connected to broker");
        /* Clear the last will now that we are online. */
        esp_mqtt_client_publish(s_client, s_lwt_topic, "online", 0,
                                CONFIG_IRRIG_MQTT_QOS, 1);
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "disconnected from broker");
        break;

    case MQTT_EVENT_ERROR:
        s_connected = false;
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "transport error: esp_tls=0x%x, tls_stack=0x%x, sock_errno=%d",
                     event->error_handle->esp_tls_last_esp_err,
                     event->error_handle->esp_tls_stack_err,
                     event->error_handle->esp_transport_sock_errno);
            if (event->error_handle->esp_tls_stack_err != 0) {
                ESP_LOGE(TAG, "TLS handshake failed. Check that main/certs/mqtt_ca.pem "
                              "holds the broker's CA chain and that the clock is set "
                              "(SNTP).");
            }
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            const char *why;
            switch (event->error_handle->connect_return_code) {
            case MQTT_CONNECTION_REFUSE_PROTOCOL:      why = "unacceptable protocol version"; break;
            case MQTT_CONNECTION_REFUSE_ID_REJECTED:   why = "client ID rejected"; break;
            case MQTT_CONNECTION_REFUSE_SERVER_UNAVAILABLE: why = "server unavailable"; break;
            case MQTT_CONNECTION_REFUSE_BAD_USERNAME:  why = "bad username or password"; break;
            case MQTT_CONNECTION_REFUSE_NOT_AUTHORIZED: why = "not authorized"; break;
            default:                                   why = "unknown"; break;
            }
            ESP_LOGE(TAG, "broker refused the connection: 0x%02x (%s)",
                     event->error_handle->connect_return_code, why);
        }
        break;

    default:
        break;
    }
}

esp_err_t app_mqtt_start(void)
{
    if (s_client != NULL) {
        return ESP_OK;
    }

    /* A fixed client ID ("ESP32Client" in the sketch) makes two boards kick
     * each other off the broker in a loop; append the MAC to keep it unique. */
    uint8_t mac[6] = {0};
    ESP_RETURN_ON_ERROR(esp_read_mac(mac, ESP_MAC_WIFI_STA), TAG, "cannot read MAC");
    snprintf(s_client_id, sizeof(s_client_id), "%s-%02x%02x%02x",
             CONFIG_IRRIG_MQTT_CLIENT_ID, mac[3], mac[4], mac[5]);
    snprintf(s_lwt_topic, sizeof(s_lwt_topic), "%s/status", CONFIG_IRRIG_MQTT_TOPIC_PREFIX);

    const bool use_tls = strncmp(CONFIG_IRRIG_MQTT_URI, "mqtts://", 8) == 0 ||
                         strncmp(CONFIG_IRRIG_MQTT_URI, "wss://", 6) == 0;

    const size_t ca_len = (size_t)(mqtt_ca_pem_end - mqtt_ca_pem_start);
    /* EMBED_TXTFILES appends a NUL terminator, so a file with only comments or
     * whitespace still yields a few bytes; require a real PEM block. */
    const bool ca_present = ca_len > 64 &&
        memmem(mqtt_ca_pem_start, ca_len, "-----BEGIN CERTIFICATE-----", 27) != NULL;

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = CONFIG_IRRIG_MQTT_URI,
        .credentials = {
            .client_id = s_client_id,
            .username  = (strlen(CONFIG_IRRIG_MQTT_USERNAME) > 0)
                             ? CONFIG_IRRIG_MQTT_USERNAME : NULL,
            .authentication.password = (strlen(CONFIG_IRRIG_MQTT_PASSWORD) > 0)
                             ? CONFIG_IRRIG_MQTT_PASSWORD : NULL,
        },
        .session = {
            .keepalive = 60,
            .last_will = {
                .topic  = s_lwt_topic,
                .msg    = "offline",
                .msg_len = 7,
                .qos    = CONFIG_IRRIG_MQTT_QOS,
                .retain = 1,
            },
        },
        .network = {
            .reconnect_timeout_ms = 5000,
            .timeout_ms           = 10000,
        },
    };

    if (use_tls) {
#if CONFIG_IRRIG_MQTT_SKIP_CERT_VERIFY
        cfg.broker.verification.skip_cert_common_name_check = true;
        cfg.broker.verification.certificate = NULL;
        ESP_LOGW(TAG, "TLS certificate verification is DISABLED - debug builds only");
#else
        if (!ca_present) {
            ESP_LOGE(TAG, "%s uses TLS but main/certs/mqtt_ca.pem contains no "
                          "certificate. Save the broker's CA chain there and rebuild.",
                     CONFIG_IRRIG_MQTT_URI);
            return ESP_ERR_INVALID_STATE;
        }
        cfg.broker.verification.certificate     = (const char *)mqtt_ca_pem_start;
        cfg.broker.verification.certificate_len = ca_len;
#endif
    } else {
        ESP_LOGW(TAG, "%s is a plaintext connection - credentials and readings "
                      "travel unencrypted", CONFIG_IRRIG_MQTT_URI);
    }

    /* EMQX Serverless (and most hosted brokers) refuse anonymous clients: the
     * connection is accepted at TLS level and then dropped with CONNACK 0x05,
     * which is easy to misread as a certificate problem. Say so up front. */
    if (use_tls && strlen(CONFIG_IRRIG_MQTT_USERNAME) == 0) {
        ESP_LOGW(TAG, "no MQTT username configured. Hosted brokers normally "
                      "reject anonymous clients with CONNACK 0x05 (not authorized); "
                      "set them under menuconfig -> Irrigation System Configuration -> MQTT");
    }

    s_client = esp_mqtt_client_init(&cfg);
    ESP_RETURN_ON_FALSE(s_client != NULL, ESP_FAIL, TAG, "esp_mqtt_client_init failed");

    ESP_RETURN_ON_ERROR(esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                                       mqtt_event_handler, NULL),
                        TAG, "cannot register MQTT handler");
    ESP_RETURN_ON_ERROR(esp_mqtt_client_start(s_client), TAG, "esp_mqtt_client_start failed");

    ESP_LOGI(TAG, "client \"%s\" started against %s (%s)",
             s_client_id, CONFIG_IRRIG_MQTT_URI, use_tls ? "TLS" : "plaintext");
    return ESP_OK;
}

bool app_mqtt_is_connected(void)
{
    return s_connected;
}

esp_err_t app_mqtt_publish(const char *subtopic, const char *payload, bool retain)
{
    if (s_client == NULL || subtopic == NULL || payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_connected) {
        /* Dropping is deliberate: queueing readings while offline would grow
         * without bound and deliver a burst of stale values on reconnect. */
        return ESP_ERR_INVALID_STATE;
    }

    char topic[MQTT_TOPIC_MAX];
    int n = snprintf(topic, sizeof(topic), "%s/%s",
                     CONFIG_IRRIG_MQTT_TOPIC_PREFIX, subtopic);
    if (n < 0 || n >= (int)sizeof(topic)) {
        return ESP_ERR_INVALID_SIZE;
    }

    int msg_id = esp_mqtt_client_publish(s_client, topic, payload, 0,
                                         CONFIG_IRRIG_MQTT_QOS, retain ? 1 : 0);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "publish to %s failed", topic);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t app_mqtt_publish_float(const char *subtopic, float value, int decimals)
{
    if (!isfinite(value)) {
        return ESP_ERR_INVALID_ARG;
    }
    char payload[MQTT_PAYLOAD_MAX];
    snprintf(payload, sizeof(payload), "%.*f", decimals, value);
    return app_mqtt_publish(subtopic, payload, false);
}

esp_err_t app_mqtt_publish_double(const char *subtopic, double value, int decimals)
{
    if (!isfinite(value)) {
        return ESP_ERR_INVALID_ARG;
    }
    char payload[MQTT_PAYLOAD_MAX];
    snprintf(payload, sizeof(payload), "%.*f", decimals, value);
    return app_mqtt_publish(subtopic, payload, false);
}

esp_err_t app_mqtt_publish_int(const char *subtopic, long value)
{
    char payload[MQTT_PAYLOAD_MAX];
    snprintf(payload, sizeof(payload), "%ld", value);
    return app_mqtt_publish(subtopic, payload, false);
}

void app_mqtt_publish_error(const char *sensor, const char *fmt, ...)
{
    char detail[MQTT_PAYLOAD_MAX];
    va_list args;
    va_start(args, fmt);
    vsnprintf(detail, sizeof(detail), fmt, args);
    va_end(args);

    ESP_LOGW(TAG, "sensor error [%s]: %s", sensor, detail);

    if (!s_connected) {
        return;
    }

    /* JSON keeps the diagnostic machine-readable on the broker side, unlike
     * the sketch's free-text "Error in sensor X: Y". */
    char payload[MQTT_PAYLOAD_MAX + 96];
    snprintf(payload, sizeof(payload),
             "{\"sensor\":\"%s\",\"error\":\"%s\",\"uptime_ms\":%lld}",
             sensor, detail, esp_timer_get_time() / 1000);
    app_mqtt_publish("errors", payload, false);
}
