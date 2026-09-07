#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "dht.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"

static const char *TAG = "dht";

/* Bus timing, microseconds. */
#define DHT_START_LOW_US_DHT22  1200  /* datasheet: at least 1 ms */
#define DHT_START_LOW_US_DHT11  20000 /* datasheet: at least 18 ms */
#define DHT_START_RELEASE_US    40    /* release, then the sensor answers */
#define DHT_EDGE_TIMEOUT_US     200   /* every edge is well under 100 us */
#define DHT_BIT_THRESHOLD_US    50    /* "0" high pulse ~27 us, "1" ~70 us */

struct dht_dev_s {
    uint32_t pin;           /* uint32_t: what gpio_ll_* takes */
    dht_type_t type;
    portMUX_TYPE lock;
    uint32_t cycles_per_us; /* cached: reading it needs a flash call */
    int64_t last_read_us;
    bool has_sample;
    float last_humidity;
    float last_temperature;
    uint32_t min_interval_us;
};

/*
 * Everything below runs with interrupts disabled on this core, so it must not
 * touch flash. That rules out gpio_set_level()/gpio_get_level() (ordinary
 * flash-resident functions - a cache miss mid-frame adds tens of microseconds
 * and corrupts the 27 us / 70 us bit discrimination) and esp_timer_get_time()
 * (also flash-resident). gpio_ll_* are always_inline register accesses and
 * esp_cpu_get_cycle_count() reads the CCOUNT register inline, so both are safe.
 */

__attribute__((always_inline))
static inline void dht_pin_low(uint32_t pin)
{
    /* The pin stays in INPUT_OUTPUT_OD mode for the whole frame: driving 0
     * pulls the line down, writing 1 releases it to the pull-up. No call to
     * gpio_set_direction() is needed - and it must not be made here, because
     * it takes the GPIO driver's own spinlock inside ours. */
    gpio_ll_set_level(&GPIO, pin, 0);
}

__attribute__((always_inline))
static inline void dht_pin_release(uint32_t pin)
{
    gpio_ll_set_level(&GPIO, pin, 1);
}

__attribute__((always_inline))
static inline int dht_pin_read(uint32_t pin)
{
    return gpio_ll_get_level(&GPIO, pin);
}

/* Busy-wait for `level`, returning the elapsed microseconds or -1 on timeout. */
static inline int32_t dht_wait_level(uint32_t pin, int level, uint32_t cycles_per_us,
                                     uint32_t timeout_us)
{
    const uint32_t start = esp_cpu_get_cycle_count();
    const uint32_t limit = timeout_us * cycles_per_us;

    while (dht_pin_read(pin) != level) {
        /* Unsigned subtraction, so a CCOUNT wrap (every ~18 s at 240 MHz)
         * still yields the correct elapsed count. */
        if ((esp_cpu_get_cycle_count() - start) > limit) {
            return -1;
        }
    }
    return (int32_t)((esp_cpu_get_cycle_count() - start) / cycles_per_us);
}

/*
 * Samples the 40-bit frame. The caller has already driven the start pulse and
 * holds the critical section; only the response is timed here, which keeps
 * interrupts disabled for ~5 ms regardless of sensor type. (Doing the start
 * pulse in here too would mean 20 ms of disabled interrupts for a DHT11.)
 */
static esp_err_t dht_sample_bits(struct dht_dev_s *h, uint8_t data[5])
{
    const uint32_t pin = h->pin;
    const uint32_t cpu = h->cycles_per_us;

    /* Release the line; the sensor starts answering 20-40 us later. */
    dht_pin_release(pin);

    /* Sensor response: ~80 us low, then ~80 us high, before the first bit. */
    if (dht_wait_level(pin, 0, cpu, DHT_EDGE_TIMEOUT_US) < 0) return ESP_ERR_TIMEOUT;
    if (dht_wait_level(pin, 1, cpu, DHT_EDGE_TIMEOUT_US) < 0) return ESP_ERR_TIMEOUT;
    if (dht_wait_level(pin, 0, cpu, DHT_EDGE_TIMEOUT_US) < 0) return ESP_ERR_TIMEOUT;

    /* 40 bits, MSB first. Each bit is ~50 us low followed by a high pulse
     * whose width carries the value. */
    for (int i = 0; i < 40; i++) {
        if (dht_wait_level(pin, 1, cpu, DHT_EDGE_TIMEOUT_US) < 0) return ESP_ERR_TIMEOUT;
        int32_t high_us = dht_wait_level(pin, 0, cpu, DHT_EDGE_TIMEOUT_US);
        if (high_us < 0) return ESP_ERR_TIMEOUT;

        data[i / 8] <<= 1;
        if (high_us > DHT_BIT_THRESHOLD_US) {
            data[i / 8] |= 1;
        }
    }
    return ESP_OK;
}

esp_err_t dht_create(gpio_num_t pin, dht_type_t type, dht_handle_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        /* On the classic ESP32 this rejects GPIO 34-39, which cannot drive the
         * start pulse at all - a wiring mistake that is otherwise silent. */
        ESP_LOGE(TAG, "GPIO %d cannot drive an output", (int)pin);
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    dht_handle_t h = calloc(1, sizeof(*h));
    if (h == NULL) {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        /* INPUT_OUTPUT_OD once, for good: the level can be driven and read
         * without ever switching direction mid-frame. */
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE, /* fallback; fit an external 4.7k */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        free(h);
        return err;
    }
    gpio_set_level(pin, 1); /* idle high (released) */

    h->pin  = (uint32_t)pin;
    h->type = type;
    h->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    /* Cached because the accessor is flash-resident and the sampling loop runs
     * with interrupts off. Valid as long as the CPU frequency is fixed - do
     * not enable dynamic frequency scaling (CONFIG_PM_ENABLE) without adding
     * an ESP_PM_CPU_FREQ_MAX lock around dht_read(). */
    h->cycles_per_us = esp_rom_get_cpu_ticks_per_us();
    if (h->cycles_per_us == 0) {
        h->cycles_per_us = 240; /* should not happen; keeps the maths sane */
    }

    h->min_interval_us = (type == DHT_TYPE_DHT11) ? 1000000 : 2000000;
    /* Backdate so the interval guard does not block the very first read. */
    h->last_read_us = esp_timer_get_time() - (int64_t)h->min_interval_us;

    vTaskDelay(pdMS_TO_TICKS(1100)); /* datasheet power-on stabilisation */

    ESP_LOGI(TAG, "initialised on GPIO %d (%s, %" PRIu32 " CPU cycles/us)",
             (int)pin, type == DHT_TYPE_DHT11 ? "DHT11" : "DHT22", h->cycles_per_us);
    *out = h;
    return ESP_OK;
}

void dht_delete(dht_handle_t h)
{
    if (h == NULL) {
        return;
    }
    gpio_reset_pin((gpio_num_t)h->pin);
    free(h);
}

esp_err_t dht_read(dht_handle_t h, float *humidity_pct, float *temperature_c)
{
    if (h == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t now = esp_timer_get_time();
    if ((uint64_t)(now - h->last_read_us) < h->min_interval_us) {
        if (!h->has_sample) {
            return ESP_ERR_INVALID_STATE;
        }
        if (humidity_pct)  *humidity_pct  = h->last_humidity;
        if (temperature_c) *temperature_c = h->last_temperature;
        return ESP_OK;
    }

    uint8_t data[5] = {0};

    /* The start pulse only has a minimum duration, so it needs no protection
     * and is driven with interrupts still enabled. */
    dht_pin_low(h->pin);
    esp_rom_delay_us(h->type == DHT_TYPE_DHT11 ? DHT_START_LOW_US_DHT11
                                               : DHT_START_LOW_US_DHT22);

    /* The response, by contrast, must be sampled without preemption or
     * interrupts on this core - one missed edge corrupts every bit that
     * follows. That is ~5 ms; the owning task is pinned to APP_CPU so the
     * Wi-Fi and LWIP tasks on PRO_CPU are unaffected. */
    portENTER_CRITICAL(&h->lock);
    esp_err_t err = dht_sample_bits(h, data);
    portEXIT_CRITICAL(&h->lock);

    dht_pin_release(h->pin); /* leave the bus idle-high between reads */

    h->last_read_us = esp_timer_get_time();

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no response on GPIO %" PRIu32 " (check wiring and pull-up)", h->pin);
        return err;
    }

    uint8_t checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4]) {
        ESP_LOGW(TAG, "checksum mismatch: computed 0x%02X, received 0x%02X",
                 checksum, data[4]);
        return ESP_ERR_INVALID_CRC;
    }

    float humidity, temperature;
    if (h->type == DHT_TYPE_DHT11) {
        humidity    = (float)data[0] + (float)data[1] * 0.1f;
        temperature = (float)(data[2] & 0x7F) + (float)data[3] * 0.1f;
        if (data[2] & 0x80) {
            temperature = -temperature;
        }
    } else {
        humidity = (float)(((uint16_t)data[0] << 8) | data[1]) * 0.1f;
        /* Bit 15 is a sign flag, not part of the magnitude - masking it off
         * before scaling is what makes sub-zero readings come out right. */
        uint16_t raw_t = ((uint16_t)(data[2] & 0x7F) << 8) | data[3];
        temperature = (float)raw_t * 0.1f;
        if (data[2] & 0x80) {
            temperature = -temperature;
        }
    }

    /* The checksum is a weak additive one; a frame can pass it and still be
     * corrupt, so reject anything outside the sensor's own specified range. */
    const float t_min = (h->type == DHT_TYPE_DHT11) ? 0.0f : -40.0f;
    const float t_max = (h->type == DHT_TYPE_DHT11) ? 50.0f : 80.0f;
    if (humidity < 0.0f || humidity > 100.0f || temperature < t_min || temperature > t_max) {
        ESP_LOGW(TAG, "frame out of sensor range: %.1f%%RH %.1fC", humidity, temperature);
        return ESP_ERR_INVALID_RESPONSE;
    }

    h->last_humidity    = humidity;
    h->last_temperature = temperature;
    h->has_sample       = true;

    if (humidity_pct)  *humidity_pct  = humidity;
    if (temperature_c) *temperature_c = temperature;
    return ESP_OK;
}
