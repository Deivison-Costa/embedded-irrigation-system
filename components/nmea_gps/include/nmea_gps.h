/*
 * NMEA 0183 GPS receiver - ESP-IDF native.
 *
 * Replaces TinyGPSPlus.
 *
 * A dedicated task drains the UART continuously. The original sketch only
 * called Serial1.read() in the short window between two delay(2000) calls, so
 * the 128-byte UART FIFO overflowed on every cycle and most sentences were lost
 * or truncated - which is why the fix so often "never became valid".
 *
 * Parsed state is published as an immutable snapshot guarded by a mutex, so the
 * consumer never observes a half-updated fix.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "nmea_parse.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* nmea_gps_fix_t vem de nmea_parse.h (puro, testavel no host). */

typedef struct {
    uart_port_t uart_port;
    int         rx_pin;
    int         tx_pin;      /* UART_PIN_NO_CHANGE if the module is receive-only */
    int         baud_rate;
    int         task_priority;
    int         task_core;   /* tskNO_AFFINITY or a core id */
} nmea_gps_config_t;

#define NMEA_GPS_CONFIG_DEFAULT()        \
    (nmea_gps_config_t){                 \
        .uart_port     = UART_NUM_1,     \
        .baud_rate     = 9600,           \
        .tx_pin        = UART_PIN_NO_CHANGE, \
        .task_priority = 5,              \
        .task_core     = 1,              \
    }

typedef struct nmea_gps_s *nmea_gps_handle_t;

esp_err_t nmea_gps_create(const nmea_gps_config_t *cfg, nmea_gps_handle_t *out);
void nmea_gps_delete(nmea_gps_handle_t h);

/**
 * Copy the most recent fix.
 *
 * @param max_age_ms  a fix older than this is reported with .valid = false
 *                    (0 disables the age check)
 */
esp_err_t nmea_gps_get_fix(nmea_gps_handle_t h, nmea_gps_fix_t *out, uint32_t max_age_ms);

#ifdef __cplusplus
}
#endif
