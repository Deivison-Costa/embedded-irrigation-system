#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Bring up Wi-Fi in station mode and start the reconnect state machine.
 *
 * Returns as soon as the driver is started - it does not block waiting for an
 * association. Use app_wifi_wait_connected() when a caller needs the link.
 */
esp_err_t app_wifi_start(void);

/**
 * Block until the station has an IPv4 address.
 *
 * @param timeout_ms  0 waits forever
 */
esp_err_t app_wifi_wait_connected(uint32_t timeout_ms);

bool app_wifi_is_connected(void);

/**
 * Synchronise the system clock over SNTP.
 *
 * TLS certificate validity ("not before" / "not after") cannot be checked
 * against an unset clock: without this the first mqtts:// handshake fails with
 * ESP_ERR_MBEDTLS_SSL_HANDSHAKE_FAILED / X509 "certificate is not yet valid".
 */
esp_err_t app_time_sync(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
