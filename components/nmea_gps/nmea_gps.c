#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nmea_gps.h"
#include "nmea_parse.h"

static const char *TAG = "nmea_gps";

#define NMEA_MAX_SENTENCE_LEN 96   /* NMEA 0183 caps a sentence at 82 chars */
#define NMEA_MAX_FIELDS       24
#define NMEA_UART_BUF_SIZE    2048 /* ~2 s of 9600-baud traffic */
#define NMEA_READ_CHUNK       256

struct nmea_gps_s {
    uart_port_t       port;
    TaskHandle_t      task;
    SemaphoreHandle_t mutex;
    volatile bool     running;
    nmea_gps_fix_t    fix;

    /* Sentence assembly state, touched only by the reader task. */
    char   line[NMEA_MAX_SENTENCE_LEN];
    size_t line_len;
    bool   overrun;
};

static void nmea_process_sentence(struct nmea_gps_s *h, char *line, size_t len)
{
    if (!nmea_verify_checksum(line, len)) {
        /* O contador vive no snapshot compartilhado, entao tambem precisa do mutex. */
        if (xSemaphoreTake(h->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            h->fix.checksum_errors++;
            xSemaphoreGive(h->mutex);
        }
        return;
    }

    const int64_t now_ms = esp_timer_get_time() / 1000;

    if (xSemaphoreTake(h->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    if (nmea_parse_sentence(line, &h->fix, now_ms)) {
        h->fix.sentences_ok++;
    }
    xSemaphoreGive(h->mutex);
}

/* --- reader task -------------------------------------------------------- */

static void nmea_gps_task(void *arg)
{
    struct nmea_gps_s *h = arg;
    uint8_t chunk[NMEA_READ_CHUNK];

    /* The task blocks on the UART, so it must be able to report to the
     * watchdog; subscribing is harmless if the TWDT is disabled. */
    esp_task_wdt_add(NULL);

    while (h->running) {
        esp_task_wdt_reset();

        int n = uart_read_bytes(h->port, chunk, sizeof(chunk), pdMS_TO_TICKS(200));
        if (n <= 0) {
            continue;
        }

        for (int i = 0; i < n; i++) {
            char c = (char)chunk[i];

            if (c == '$') {
                /* A new sentence always restarts assembly - a truncated
                 * previous one is discarded rather than merged. */
                h->line_len = 0;
                h->overrun  = false;
            }

            if (c == '\r' || c == '\n') {
                if (h->line_len > 0 && !h->overrun) {
                    h->line[h->line_len] = '\0';
                    nmea_process_sentence(h, h->line, h->line_len);
                }
                h->line_len = 0;
                h->overrun  = false;
                continue;
            }

            if (h->line_len < sizeof(h->line) - 1) {
                h->line[h->line_len++] = c;
            } else {
                h->overrun = true; /* oversized sentence: drop it, keep syncing */
            }
        }
    }

    esp_task_wdt_delete(NULL);
    h->task = NULL;
    vTaskDelete(NULL);
}

/* --- public API --------------------------------------------------------- */

esp_err_t nmea_gps_create(const nmea_gps_config_t *cfg, nmea_gps_handle_t *out)
{
    if (cfg == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    struct nmea_gps_s *h = calloc(1, sizeof(*h));
    if (h == NULL) {
        return ESP_ERR_NO_MEM;
    }
    h->port = cfg->uart_port;

    h->mutex = xSemaphoreCreateMutex();
    if (h->mutex == NULL) {
        free(h);
        return ESP_ERR_NO_MEM;
    }

    uart_config_t uart_cfg = {
        .baud_rate  = cfg->baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(h->port, NMEA_UART_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        goto fail_sem;
    }
    if ((err = uart_param_config(h->port, &uart_cfg)) != ESP_OK) goto fail_uart;
    if ((err = uart_set_pin(h->port, cfg->tx_pin, cfg->rx_pin,
                            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) != ESP_OK) goto fail_uart;

    h->running = true;

    BaseType_t ok = xTaskCreatePinnedToCore(nmea_gps_task, "nmea_gps", 4096, h,
                                            (UBaseType_t)cfg->task_priority, &h->task,
                                            (BaseType_t)cfg->task_core);
    if (ok != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto fail_uart;
    }

    ESP_LOGI(TAG, "UART%d RX=%d @ %d baud, reader task started",
             (int)h->port, cfg->rx_pin, cfg->baud_rate);
    *out = h;
    return ESP_OK;

fail_uart:
    uart_driver_delete(h->port);
fail_sem:
    vSemaphoreDelete(h->mutex);
    free(h);
    return err;
}

void nmea_gps_delete(nmea_gps_handle_t h)
{
    if (h == NULL) {
        return;
    }
    h->running = false;
    /* Let the task observe the flag and exit its blocking read. */
    for (int i = 0; i < 20 && h->task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    uart_driver_delete(h->port);
    vSemaphoreDelete(h->mutex);
    free(h);
}

esp_err_t nmea_gps_get_fix(nmea_gps_handle_t h, nmea_gps_fix_t *out, uint32_t max_age_ms)
{
    if (h == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(h->mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *out = h->fix;
    xSemaphoreGive(h->mutex);

    if (out->valid && max_age_ms > 0) {
        int64_t age = (esp_timer_get_time() / 1000) - out->timestamp_ms;
        if (age > (int64_t)max_age_ms) {
            out->valid = false;
        }
    }
    return ESP_OK;
}
