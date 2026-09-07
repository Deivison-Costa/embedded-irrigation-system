#include <inttypes.h>

#include "app_power.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "app_power";

/* Its own namespace so erasing the counter never touches the Wi-Fi
 * calibration data that shares the nvs partition. */
#define POWER_NVS_NAMESPACE "irrig_power"
#define POWER_NVS_KEY       "unstable"

static uint32_t           s_unstable_boots;
static esp_reset_reason_t s_reset_reason = ESP_RST_UNKNOWN;

static const char *reset_reason_str(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON:  return "power-on or brownout below the POR threshold";
    case ESP_RST_EXT:      return "external reset pin";
    case ESP_RST_SW:       return "esp_restart()";
    case ESP_RST_PANIC:    return "exception or panic";
    case ESP_RST_INT_WDT:  return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT:      return "other watchdog";
    case ESP_RST_BROWNOUT: return "brownout detector";
    case ESP_RST_DEEPSLEEP:return "deep sleep wake-up";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "unknown";
    }
}

/*
 * ESP_RST_BROWNOUT only appears when the detector fires early enough for its
 * ISR to store the hint in RTC memory and finish a software reset. When the
 * rail keeps falling past the chip's power-on-reset threshold the RTC domain is
 * wiped too and the very same event is reported as ESP_RST_POWERON - which is
 * also what a normal power-up looks like. The boot counter is what tells the
 * two apart: a genuine power-up happens once, a collapsing rail happens again
 * and again.
 */
bool app_power_last_reset_was_power_related(void)
{
    return s_reset_reason == ESP_RST_BROWNOUT || s_reset_reason == ESP_RST_POWERON;
}

const char *app_power_reset_reason_str(void)
{
    return reset_reason_str(s_reset_reason);
}

uint32_t app_power_unstable_boots(void)
{
    return s_unstable_boots;
}

static esp_err_t power_nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erasing, reformatting");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

static esp_err_t power_counter_store(uint32_t value)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(POWER_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u32(nvs, POWER_NVS_KEY, value);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

/* 5 s, 10 s, 20 s ... capped. Doubling matters more than the exact numbers:
 * the point is to stop retrying at the rate the failure itself sets. */
static uint32_t backoff_seconds(uint32_t unstable_boots)
{
    const uint32_t over = unstable_boots - CONFIG_IRRIG_BOOT_LOOP_THRESHOLD;
    const uint32_t shift = over > 5 ? 5 : over; /* 5 << 5 = 160, then clamped */
    uint32_t seconds = 5u << shift;
    if (seconds > (uint32_t)CONFIG_IRRIG_BOOT_LOOP_BACKOFF_MAX_S) {
        seconds = (uint32_t)CONFIG_IRRIG_BOOT_LOOP_BACKOFF_MAX_S;
    }
    return seconds;
}

void app_power_boot_guard(void)
{
    s_reset_reason = esp_reset_reason();
    ESP_LOGI(TAG, "last reset: %s (esp_reset_reason=%d)",
             reset_reason_str(s_reset_reason), (int)s_reset_reason);

    esp_err_t err = power_nvs_init();
    if (err != ESP_OK) {
        /* Without NVS there is no counter, but the firmware must still boot. */
        ESP_LOGE(TAG, "NVS unavailable (%s); boot-loop guard disabled",
                 esp_err_to_name(err));
        return;
    }

    nvs_handle_t nvs;
    if (nvs_open(POWER_NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_u32(nvs, POWER_NVS_KEY, &s_unstable_boots) != ESP_OK) {
            s_unstable_boots = 0; /* first boot after a flash erase */
        }
        nvs_close(nvs);
    }

    /* Counted before the radio comes up: from here to app_power_mark_stable()
     * is exactly the window this firmware keeps dying in. */
    s_unstable_boots++;
    err = power_counter_store(s_unstable_boots);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot persist the boot counter: %s", esp_err_to_name(err));
    }

    if (s_unstable_boots < (uint32_t)CONFIG_IRRIG_BOOT_LOOP_THRESHOLD) {
        ESP_LOGI(TAG, "boot %" PRIu32 " since the last stable run", s_unstable_boots);
        return;
    }

    const uint32_t wait_s = backoff_seconds(s_unstable_boots);

    ESP_LOGE(TAG, "==================================================================");
    ESP_LOGE(TAG, "RESET LOOP: %" PRIu32 " boots in a row never reached %d s of uptime.",
             s_unstable_boots, CONFIG_IRRIG_BOOT_STABLE_S);
    ESP_LOGE(TAG, "Last reset: %s.", reset_reason_str(s_reset_reason));
    if (app_power_last_reset_was_power_related()) {
        ESP_LOGE(TAG, "This is the signature of a 3.3 V rail that cannot sustain the");
        ESP_LOGE(TAG, "RF power-up: the board dies at 'phy_init', which is the largest");
        ESP_LOGE(TAG, "current step of the boot (~400 mA peak), and never gets past it.");
        ESP_LOGE(TAG, "Check, in this order: the USB cable and port (a charger-only or");
        ESP_LOGE(TAG, "long thin cable drops volts under load), the 3.3 V regulator's");
        ESP_LOGE(TAG, "temperature, a >=470 uF bulk capacitor across 3V3/GND, and");
        ESP_LOGE(TAG, "whether the GPS/RS-485 modules are sharing the same rail.");
    }
    ESP_LOGE(TAG, "Backing off %" PRIu32 " s with the radio off before retrying.", wait_s);
    ESP_LOGE(TAG, "==================================================================");

    /* vTaskDelay, not a busy-wait: the idle tasks must keep feeding the task
     * watchdog, and an idle CPU is also the point - it draws less. */
    vTaskDelay(pdMS_TO_TICKS(wait_s * 1000));
}

void app_power_mark_stable(void)
{
    if (s_unstable_boots == 0) {
        return; /* already clear - do not spend a flash write on it */
    }
    esp_err_t err = power_counter_store(0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot clear the boot counter: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "stable for %d s; boot counter cleared (was %" PRIu32 ")",
             CONFIG_IRRIG_BOOT_STABLE_S, s_unstable_boots);
    s_unstable_boots = 0;
}
