#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Guard against the reset loop the board falls into when the 3.3 V rail cannot
 * sustain the RF power-up.
 *
 * Enabling the radio (esp_wifi_start -> phy_init -> RF calibration) is by far
 * the largest current step of the whole boot. On a marginal supply the rail
 * collapses right there, the brownout detector fires, and the board reboots
 * into the very same step ~700 ms later - forever, because every retry is the
 * failing one.
 *
 * Call this once, at the top of app_main(), BEFORE app_wifi_start(): it runs
 * while the radio is still off, which is the lowest-current and therefore
 * safest moment of the boot.
 *
 * It logs why the chip last reset, keeps a boot-stability counter in NVS
 * (RTC memory does not survive the power-on reset a real brownout produces, so
 * the counter has to live in flash), and once the counter shows a loop it
 * sleeps before returning, so the regulator cools down and the bulk capacitor
 * recharges instead of being hammered at 1.4 Hz.
 *
 * Also initialises NVS, which app_wifi_start() would otherwise do first.
 */
void app_power_boot_guard(void);

/**
 * Declare the current run healthy and clear the boot-stability counter.
 *
 * Call it once the board has stayed up long enough to prove it survived the RF
 * power-up. Writes to NVS only when the counter is not already zero.
 */
void app_power_mark_stable(void);

/** Consecutive boots that never reached app_power_mark_stable(). */
uint32_t app_power_unstable_boots(void);

/** esp_reset_reason() of this boot, as a short string ("brownout", ...). */
const char *app_power_reset_reason_str(void);

/** True when this boot follows a reset that points at the power supply. */
bool app_power_last_reset_was_power_related(void);

#ifdef __cplusplus
}
#endif
