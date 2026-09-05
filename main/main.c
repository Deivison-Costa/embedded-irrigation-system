/*
 * embedded-irrigation-system - ESP-IDF firmware
 *
 * Port of the original Arduino sketch (embedded-irrigation-system.ino).
 * Sensors: RS-485/Modbus anemometer, DHT22, BH1750, BMP280, LM393 comparator,
 * NMEA GPS. Readings are published to an MQTT broker over TLS.
 */

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_mqtt.h"
#include "app_wifi.h"
#include "bh1750.h"
#include "bmp280.h"
#include "dht.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modbus_rtu.h"
#include "nmea_gps.h"

static const char *TAG = "irrigation";

/* Anemometer register map (Modbus RTU holding registers). Register 0x0000
 * carries wind speed scaled by 10, as in the original sketch. */
#define ANEMOMETER_REG_WIND_SPEED 0x0000
#define ANEMOMETER_SPEED_SCALE    10.0f

/* Plausibility limits. A reading outside these is reported as an error rather
 * than published, so downstream consumers never see impossible values. */
#define WIND_SPEED_MIN_MS   0.0f
#define WIND_SPEED_MAX_MS   100.0f
#define PRESSURE_MIN_HPA    300.0f
#define PRESSURE_MAX_HPA    1100.0f
#define LUX_MAX             65535.0f

/* The sensor task runs on APP_CPU so the DHT22 critical section never stalls
 * the Wi-Fi/LWIP tasks pinned to PRO_CPU. */
#define SENSOR_TASK_CORE     1
#define SENSOR_TASK_PRIORITY 4
#define SENSOR_TASK_STACK    6144

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    modbus_rtu_handle_t     anemometer;
    dht_handle_t            dht;
    bh1750_handle_t         light;
    bmp280_handle_t         bmp;
    nmea_gps_handle_t       gps;
    bool                    lm393_ready;
} app_sensors_t;

static app_sensors_t s_sensors;

/* --- initialisation ----------------------------------------------------- */

static esp_err_t init_i2c_bus(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = CONFIG_IRRIG_I2C_SDA,
        .scl_io_num        = CONFIG_IRRIG_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, /* weak; keep the 4.7k pull-ups
                                               * on the sensor breakouts */
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_sensors.i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed on SDA=%d SCL=%d: %s",
                 CONFIG_IRRIG_I2C_SDA, CONFIG_IRRIG_I2C_SCL, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2C bus ready (SDA=%d, SCL=%d)",
             CONFIG_IRRIG_I2C_SDA, CONFIG_IRRIG_I2C_SCL);
    return ESP_OK;
}

static esp_err_t init_lm393(void)
{
    const gpio_num_t pin = (gpio_num_t)CONFIG_IRRIG_LM393_PIN;

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_INPUT,
        /* The comparator output is usually open-collector: with no pull-up the
         * pin floats and reads as noise. */
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LM393 GPIO %d config failed: %s", (int)pin, esp_err_to_name(err));
        return err;
    }

    /* On the classic ESP32 the input-only pins (34-39) are exactly the ones
     * without an internal pull-up, so this test catches both facts at once. */
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        ESP_LOGW(TAG, "GPIO %d is input-only and has no internal pull-up; "
                      "fit an external 10k pull-up or the reading will float",
                 (int)pin);
    }

    s_sensors.lm393_ready = true;
    ESP_LOGI(TAG, "LM393 on GPIO %d (active %s)", (int)pin,
             CONFIG_IRRIG_LM393_ACTIVE_LOW ? "low" : "high");
    return ESP_OK;
}

/* Each sensor is optional at runtime: one missing device must not stop the
 * others from reporting, so failures are logged and the handle left NULL. */
static void init_sensors(void)
{
    if (init_i2c_bus() == ESP_OK) {
        esp_err_t err = bh1750_create(s_sensors.i2c_bus, BH1750_ADDR_LO,
                                      BH1750_MODE_CONTINUOUS_HIGH_RES,
                                      &s_sensors.light);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "BH1750 unavailable: %s", esp_err_to_name(err));
        }

        bmp280_config_t bmp_cfg = BMP280_CONFIG_DEFAULT();
        err = bmp280_create(s_sensors.i2c_bus, BMP280_ADDR_LO, &bmp_cfg, &s_sensors.bmp);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "BMP280 unavailable at 0x76: %s "
                          "(try 0x77 if SDO is tied high)", esp_err_to_name(err));
        }
    }

    modbus_rtu_config_t mb_cfg = MODBUS_RTU_CONFIG_DEFAULT();
    mb_cfg.uart_port = UART_NUM_2;
    mb_cfg.tx_pin    = CONFIG_IRRIG_RS485_UART_TX;
    mb_cfg.rx_pin    = CONFIG_IRRIG_RS485_UART_RX;
    mb_cfg.rts_pin   = CONFIG_IRRIG_RS485_UART_RTS;
    mb_cfg.baud_rate = CONFIG_IRRIG_RS485_BAUD;
    esp_err_t err = modbus_rtu_create(&mb_cfg, &s_sensors.anemometer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RS-485 master unavailable: %s", esp_err_to_name(err));
    }

    err = dht_create(CONFIG_IRRIG_DHT_PIN, DHT_TYPE_DHT22, &s_sensors.dht);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DHT22 unavailable on GPIO %d: %s",
                 CONFIG_IRRIG_DHT_PIN, esp_err_to_name(err));
    }

    nmea_gps_config_t gps_cfg = NMEA_GPS_CONFIG_DEFAULT();
    gps_cfg.uart_port = UART_NUM_1;
    gps_cfg.rx_pin    = CONFIG_IRRIG_GPS_UART_RX;
    gps_cfg.tx_pin    = UART_PIN_NO_CHANGE;
    gps_cfg.baud_rate = CONFIG_IRRIG_GPS_BAUD;
    gps_cfg.task_core = SENSOR_TASK_CORE;
    err = nmea_gps_create(&gps_cfg, &s_sensors.gps);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPS unavailable: %s", esp_err_to_name(err));
    }

    init_lm393();
}

/* --- per-sensor sampling ------------------------------------------------ */

static void sample_anemometer(void)
{
    if (s_sensors.anemometer == NULL) {
        return;
    }

    uint16_t reg = 0;
    esp_err_t err = modbus_rtu_read_holding_registers(
        s_sensors.anemometer, CONFIG_IRRIG_ANEMOMETER_SLAVE_ADDR,
        ANEMOMETER_REG_WIND_SPEED, 1, &reg);

    if (err != ESP_OK) {
        app_mqtt_publish_error("windSpeed", "Modbus read failed: %s",
                               esp_err_to_name(err));
        return;
    }

    float wind_speed = (float)reg / ANEMOMETER_SPEED_SCALE;
    ESP_LOGI(TAG, "Wind speed: %.1f m/s (raw 0x%04X)", wind_speed, reg);

    if (wind_speed < WIND_SPEED_MIN_MS || wind_speed > WIND_SPEED_MAX_MS) {
        app_mqtt_publish_error("windSpeed", "out of range: %.1f", wind_speed);
        return;
    }
    app_mqtt_publish_float("windSpeed", wind_speed, 1);
}

static void sample_dht(void)
{
    if (s_sensors.dht == NULL) {
        return;
    }

    float humidity = NAN, temperature = NAN;
    esp_err_t err = dht_read(s_sensors.dht, &humidity, &temperature);
    if (err != ESP_OK) {
        app_mqtt_publish_error("dht22", "read failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "DHT22: %.1f C, %.1f %%RH", temperature, humidity);
    app_mqtt_publish_float("temperature", temperature, 1);
    app_mqtt_publish_float("humidity", humidity, 1);
}

static void sample_light(void)
{
    if (s_sensors.light == NULL) {
        return;
    }

    float lux = NAN;
    esp_err_t err = bh1750_read_lux(s_sensors.light, &lux);
    if (err != ESP_OK) {
        app_mqtt_publish_error("luminosity", "read failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Luminosity: %.1f lx", lux);

    /* The original compared a uint16_t against 0 and 65535 - both ends were
     * unreachable, so the check never fired. Compare in float instead. */
    if (lux < 0.0f || lux > LUX_MAX) {
        app_mqtt_publish_error("luminosity", "out of range: %.1f", lux);
        return;
    }
    app_mqtt_publish_float("luminosity", lux, 1);
}

static void sample_pressure(void)
{
    if (s_sensors.bmp == NULL) {
        return;
    }

    float pressure = NAN, temperature = NAN;
    esp_err_t err = bmp280_read(s_sensors.bmp, &temperature, &pressure);
    if (err != ESP_OK) {
        app_mqtt_publish_error("pressure", "read failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "BMP280: %.2f hPa, %.2f C", pressure, temperature);

    if (pressure < PRESSURE_MIN_HPA || pressure > PRESSURE_MAX_HPA) {
        app_mqtt_publish_error("pressure", "out of range: %.2f", pressure);
    } else {
        app_mqtt_publish_float("pressure", pressure, 2);
    }

    /* The sketch read this value and then discarded it; publishing it gives a
     * second temperature source to cross-check the DHT22 against. */
    if (temperature > -40.0f && temperature < 85.0f) {
        app_mqtt_publish_float("bmpTemperature", temperature, 2);
    }
}

static void sample_lm393(void)
{
    if (!s_sensors.lm393_ready) {
        return;
    }

    int level = gpio_get_level((gpio_num_t)CONFIG_IRRIG_LM393_PIN);
#if CONFIG_IRRIG_LM393_ACTIVE_LOW
    int detected = (level == 0);
#else
    int detected = (level == 1);
#endif

    ESP_LOGI(TAG, "LM393: raw=%d, detected=%d", level, detected);
    /* "lm393" keeps the original topic; "rain" carries the normalised meaning. */
    app_mqtt_publish_int("lm393", level);
    app_mqtt_publish_int("rain", detected);
}

static void sample_gps(void)
{
    if (s_sensors.gps == NULL) {
        return;
    }

    nmea_gps_fix_t fix;
    esp_err_t err = nmea_gps_get_fix(s_sensors.gps, &fix,
                                     CONFIG_IRRIG_GPS_MAX_FIX_AGE_MS);
    if (err != ESP_OK) {
        app_mqtt_publish_error("GPS", "snapshot failed: %s", esp_err_to_name(err));
        return;
    }

    if (!fix.valid) {
        app_mqtt_publish_error("GPS", "no fix (sentences=%" PRIu32 ", crc_err=%" PRIu32
                                      ", sats=%u)",
                               fix.sentences_ok, fix.checksum_errors,
                               (unsigned)fix.satellites);
        return;
    }

    ESP_LOGI(TAG, "GPS: %.6f, %.6f, %.2f m (sats=%u, hdop=%.1f)",
             fix.latitude_deg, fix.longitude_deg, fix.altitude_m,
             (unsigned)fix.satellites, fix.hdop);

    app_mqtt_publish_double("latitude", fix.latitude_deg, 6);
    app_mqtt_publish_double("longitude", fix.longitude_deg, 6);
    app_mqtt_publish_double("altitude", fix.altitude_m, 2);
    app_mqtt_publish_int("satellites", fix.satellites);
}

/* --- sampling task ------------------------------------------------------ */

static void sensor_task(void *arg)
{
    (void)arg;

    esp_task_wdt_add(NULL);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONFIG_IRRIG_SAMPLE_INTERVAL_MS);
    uint32_t cycle = 0;

    while (1) {
        esp_task_wdt_reset();

        bool online = app_mqtt_is_connected();
        if (!online) {
            /* Keep sampling anyway: the logs stay useful while the link is
             * down, and esp-mqtt reconnects in the background. */
            ESP_LOGW(TAG, "broker offline, readings are logged but not published");
        }

        sample_anemometer();
        esp_task_wdt_reset();

        sample_dht();
        esp_task_wdt_reset();

        sample_light();
        sample_pressure();
        sample_lm393();
        sample_gps();

        if ((++cycle % 30) == 0) {
            ESP_LOGI(TAG, "heap: %" PRIu32 " B free, %" PRIu32 " B min, uptime %lld s",
                     esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
                     esp_timer_get_time() / 1000000);
            app_mqtt_publish_int("uptime", (long)(esp_timer_get_time() / 1000000));
            app_mqtt_publish_int("freeHeap", (long)esp_get_free_heap_size());
        }

        /* Fixed cadence rather than "work + delay(2000)": the interval stays
         * constant no matter how long the reads took. If a cycle did overrun
         * the period (a pile-up of Modbus timeouts, say), resynchronise
         * instead of letting vTaskDelayUntil run several cycles back to back
         * with no delay to catch up. */
        if (xTaskDelayUntil(&last_wake, period) == pdFALSE) {
            ESP_LOGW(TAG, "sampling cycle overran the %d ms period",
                     CONFIG_IRRIG_SAMPLE_INTERVAL_MS);
            last_wake = xTaskGetTickCount();
        }
    }
}

/* --- entry point -------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "embedded-irrigation-system starting (IDF %s)", esp_get_idf_version());

    ESP_ERROR_CHECK(app_wifi_start());

    /* Wait for the link, but do not make it fatal: the sensors and the console
     * log still work offline, and Wi-Fi keeps retrying in the background. */
    if (app_wifi_wait_connected(30000) != ESP_OK) {
        ESP_LOGW(TAG, "no Wi-Fi yet; continuing and retrying in the background");
    } else {
        /* Only meaningful once there is a route; needed before any TLS. */
        app_time_sync(15000);
    }

    esp_err_t err = app_mqtt_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT client not started: %s", esp_err_to_name(err));
    }

    init_sensors();

    BaseType_t ok = xTaskCreatePinnedToCore(sensor_task, "sensors", SENSOR_TASK_STACK,
                                            NULL, SENSOR_TASK_PRIORITY, NULL,
                                            SENSOR_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "cannot create the sensor task; restarting");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    ESP_LOGI(TAG, "initialisation complete");
    /* app_main returns; its task is deleted and the sensor task keeps running. */
}
