/*
 * DHT11 / DHT22 (AM2302) temperature + humidity sensor - ESP-IDF native driver.
 *
 * Replaces the Adafruit DHT / DHT_U libraries.
 *
 * Timing is produced by bit-banging inside a critical section, so the read
 * blocks the *calling core* for up to ~6 ms. Run the owning task on APP_CPU
 * (core 1) to keep the Wi-Fi/LWIP stack on PRO_CPU (core 0) unaffected.
 */
#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DHT_TYPE_DHT11,
    DHT_TYPE_DHT22, /* also AM2302 / RHT03 */
} dht_type_t;

typedef struct dht_dev_s *dht_handle_t;

/**
 * Configure the data pin.
 *
 * The line needs a pull-up to VCC. An external 4.7k-10k resistor is strongly
 * recommended; the internal pull-up (~45k) is enabled as a fallback but is too
 * weak for cables longer than a few centimetres.
 */
esp_err_t dht_create(gpio_num_t pin, dht_type_t type, dht_handle_t *out);

void dht_delete(dht_handle_t dev);

/**
 * Read humidity and temperature.
 *
 * Enforces the datasheet minimum sampling interval (2 s for DHT22, 1 s for
 * DHT11) by returning the last successful sample if called sooner, so a caller
 * polling faster than the sensor allows gets stale-but-valid data instead of
 * the checksum errors the Arduino library reports as NaN.
 *
 * @param[out] humidity_pct    relative humidity, %RH (may be NULL)
 * @param[out] temperature_c   degrees Celsius (may be NULL)
 * @return ESP_OK, ESP_ERR_TIMEOUT (no response / broken wiring),
 *         ESP_ERR_INVALID_CRC (checksum mismatch), ESP_ERR_INVALID_RESPONSE
 *         (values outside the sensor's own physical range).
 */
esp_err_t dht_read(dht_handle_t dev, float *humidity_pct, float *temperature_c);

#ifdef __cplusplus
}
#endif
