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

/* --- parsing helpers ---------------------------------------------------- */

static bool nmea_verify_checksum(const char *sentence, size_t len)
{
    /* Format: $<payload>*<HH> - the checksum covers the payload only. */
    if (len < 4 || sentence[0] != '$') {
        return false;
    }
    const char *star = memchr(sentence, '*', len);
    if (star == NULL || (size_t)(star - sentence) + 3 > len) {
        return false;
    }

    uint8_t computed = 0;
    for (const char *p = sentence + 1; p < star; p++) {
        computed ^= (uint8_t)*p;
    }

    char hex[3] = {star[1], star[2], '\0'};
    char *end = NULL;
    unsigned long received = strtoul(hex, &end, 16);
    if (end != hex + 2) {
        return false;
    }
    return computed == (uint8_t)received;
}

/* Splits payload in place at commas. Returns the field count. */
static int nmea_split(char *payload, char *fields[], int max_fields)
{
    int count = 0;
    char *p = payload;
    fields[count++] = p;
    while (*p && count < max_fields) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
        p++;
    }
    return count;
}

/* "ddmm.mmmm" + hemisphere -> signed decimal degrees. */
static bool nmea_parse_coord(const char *value, const char *hemi, double *out)
{
    if (value == NULL || *value == '\0' || hemi == NULL || *hemi == '\0') {
        return false;
    }
    char *end = NULL;
    double raw = strtod(value, &end);
    if (end == value) {
        return false;
    }

    double degrees = (double)((int)(raw / 100.0));
    double minutes = raw - degrees * 100.0;
    if (minutes < 0.0 || minutes >= 60.0) {
        return false;
    }
    double result = degrees + minutes / 60.0;

    if (*hemi == 'S' || *hemi == 'W') {
        result = -result;
    } else if (*hemi != 'N' && *hemi != 'E') {
        return false;
    }
    *out = result;
    return true;
}

static bool nmea_parse_double(const char *value, double *out)
{
    if (value == NULL || *value == '\0') {
        return false;
    }
    char *end = NULL;
    double v = strtod(value, &end);
    if (end == value) {
        return false;
    }
    *out = v;
    return true;
}

/* "hhmmss.sss" */
static void nmea_parse_time(const char *value, nmea_gps_fix_t *fix)
{
    if (value == NULL || strlen(value) < 6) {
        return;
    }
    fix->hour   = (uint8_t)((value[0] - '0') * 10 + (value[1] - '0'));
    fix->minute = (uint8_t)((value[2] - '0') * 10 + (value[3] - '0'));
    fix->second = (uint8_t)((value[4] - '0') * 10 + (value[5] - '0'));
}

/* "ddmmyy" */
static void nmea_parse_date(const char *value, nmea_gps_fix_t *fix)
{
    if (value == NULL || strlen(value) < 6) {
        return;
    }
    fix->day   = (uint8_t)((value[0] - '0') * 10 + (value[1] - '0'));
    fix->month = (uint8_t)((value[2] - '0') * 10 + (value[3] - '0'));
    uint16_t yy = (uint16_t)((value[4] - '0') * 10 + (value[5] - '0'));
    fix->year  = (uint16_t)(2000 + yy); /* NMEA two-digit year, valid to 2099 */
}

/* --- sentence handlers -------------------------------------------------- */

static void nmea_handle_gga(char *fields[], int n, nmea_gps_fix_t *fix)
{
    if (n < 10) {
        return;
    }
    nmea_parse_time(fields[1], fix);

    uint8_t quality = (fields[6] && *fields[6]) ? (uint8_t)atoi(fields[6]) : 0;
    fix->fix_quality = quality;

    if (quality == 0) {
        fix->valid = false;
        return;
    }

    double lat, lon, alt, hdop;
    bool have_pos = nmea_parse_coord(fields[2], fields[3], &lat) &&
                    nmea_parse_coord(fields[4], fields[5], &lon);
    if (!have_pos) {
        fix->valid = false;
        return;
    }

    fix->latitude_deg  = lat;
    fix->longitude_deg = lon;
    fix->satellites    = (fields[7] && *fields[7]) ? (uint8_t)atoi(fields[7]) : 0;
    if (nmea_parse_double(fields[8], &hdop)) {
        fix->hdop = (float)hdop;
    }
    if (nmea_parse_double(fields[9], &alt)) {
        fix->altitude_m = alt;
    }
    fix->valid        = true;
    fix->timestamp_ms = esp_timer_get_time() / 1000;
}

static void nmea_handle_rmc(char *fields[], int n, nmea_gps_fix_t *fix)
{
    if (n < 10) {
        return;
    }
    nmea_parse_time(fields[1], fix);
    nmea_parse_date(fields[9], fix);

    /* Field 2: 'A' = active/valid, 'V' = void. */
    if (fields[2] == NULL || *fields[2] != 'A') {
        fix->valid = false;
        return;
    }

    double lat, lon, speed_knots, course;
    if (!nmea_parse_coord(fields[3], fields[4], &lat) ||
        !nmea_parse_coord(fields[5], fields[6], &lon)) {
        fix->valid = false;
        return;
    }

    fix->latitude_deg  = lat;
    fix->longitude_deg = lon;
    if (nmea_parse_double(fields[7], &speed_knots)) {
        fix->speed_kmh = speed_knots * 1.852;
    }
    if (nmea_parse_double(fields[8], &course)) {
        fix->course_deg = course;
    }
    fix->valid        = true;
    fix->timestamp_ms = esp_timer_get_time() / 1000;
}

static void nmea_process_sentence(struct nmea_gps_s *h, char *line, size_t len)
{
    if (!nmea_verify_checksum(line, len)) {
        /* Counter lives in the shared snapshot, so it needs the mutex too. */
        if (xSemaphoreTake(h->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            h->fix.checksum_errors++;
            xSemaphoreGive(h->mutex);
        }
        return;
    }

    /* Strip "$" and everything from "*" onwards. */
    char *payload = line + 1;
    char *star = strchr(payload, '*');
    if (star) {
        *star = '\0';
    }

    char *fields[NMEA_MAX_FIELDS];
    int n = nmea_split(payload, fields, NMEA_MAX_FIELDS);
    if (n < 1 || strlen(fields[0]) < 5) {
        return;
    }

    /* Talker id varies (GP, GN, GL, GA...); match on the 3-char sentence type. */
    const char *type = fields[0] + 2;

    if (xSemaphoreTake(h->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    if (strncmp(type, "GGA", 3) == 0) {
        nmea_handle_gga(fields, n, &h->fix);
        h->fix.sentences_ok++;
    } else if (strncmp(type, "RMC", 3) == 0) {
        nmea_handle_rmc(fields, n, &h->fix);
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
