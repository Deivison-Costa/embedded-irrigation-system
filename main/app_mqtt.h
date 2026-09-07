#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the MQTT client.
 *
 * esp-mqtt reconnects on its own, so there is no equivalent of the sketch's
 * blocking reconnect() loop: publishing while offline fails fast and the
 * sampling loop keeps running.
 */
esp_err_t app_mqtt_start(void);

bool app_mqtt_is_connected(void);

/** Publish a NUL-terminated payload under "<prefix>/<subtopic>". */
esp_err_t app_mqtt_publish(const char *subtopic, const char *payload, bool retain);

/** Publish a float with the given number of decimals; NaN/Inf are rejected. */
esp_err_t app_mqtt_publish_float(const char *subtopic, float value, int decimals);

/** Publish a double with the given number of decimals; NaN/Inf are rejected. */
esp_err_t app_mqtt_publish_double(const char *subtopic, double value, int decimals);

esp_err_t app_mqtt_publish_int(const char *subtopic, long value);

/** Publish a diagnostic message under "<prefix>/errors". */
void app_mqtt_publish_error(const char *sensor, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

#ifdef __cplusplus
}
#endif
