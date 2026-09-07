/*
 * BMP280 / BME280 pressure + temperature sensor - ESP-IDF native driver.
 *
 * Replaces Adafruit_BMP280 + Adafruit_Sensor. Implements the full Bosch
 * compensation formulas from the datasheet (rev. 1.23, section 3.11.3).
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SDO low -> 0x76 (the address the original sketch used), SDO high -> 0x77. */
#define BMP280_ADDR_LO 0x76
#define BMP280_ADDR_HI 0x77

typedef enum {
    BMP280_OSRS_SKIP = 0,
    BMP280_OSRS_X1   = 1,
    BMP280_OSRS_X2   = 2,
    BMP280_OSRS_X4   = 3,
    BMP280_OSRS_X8   = 4,
    BMP280_OSRS_X16  = 5,
} bmp280_oversampling_t;

typedef enum {
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_X2  = 1,
    BMP280_FILTER_X4  = 2,
    BMP280_FILTER_X8  = 3,
    BMP280_FILTER_X16 = 4,
} bmp280_filter_t;

typedef enum {
    BMP280_STANDBY_0M5  = 0,
    BMP280_STANDBY_62M5 = 1,
    BMP280_STANDBY_125M = 2,
    BMP280_STANDBY_250M = 3,
    BMP280_STANDBY_500M = 4,
    BMP280_STANDBY_1000M = 5,
    BMP280_STANDBY_2000M = 6,
    BMP280_STANDBY_4000M = 7,
} bmp280_standby_t;

typedef struct {
    bmp280_oversampling_t osrs_t;
    bmp280_oversampling_t osrs_p;
    bmp280_filter_t       filter;
    bmp280_standby_t      standby;
} bmp280_config_t;

/* Weather-station preset: low noise, ~1 Hz refresh, filter smooths gusts. */
#define BMP280_CONFIG_DEFAULT()            \
    (bmp280_config_t){                     \
        .osrs_t  = BMP280_OSRS_X2,         \
        .osrs_p  = BMP280_OSRS_X16,        \
        .filter  = BMP280_FILTER_X16,      \
        .standby = BMP280_STANDBY_500M,    \
    }

typedef struct bmp280_dev_s *bmp280_handle_t;

esp_err_t bmp280_create(i2c_master_bus_handle_t bus, uint8_t addr,
                        const bmp280_config_t *cfg, bmp280_handle_t *out);

void bmp280_delete(bmp280_handle_t dev);

/**
 * Read compensated temperature and pressure in one burst.
 *
 * Reading both registers in a single transaction is required: the temperature
 * compensation term t_fine feeds the pressure formula, so a split read can mix
 * samples from two different conversions.
 *
 * @param[out] temperature_c  degrees Celsius (may be NULL)
 * @param[out] pressure_hpa   hectopascal / millibar (may be NULL)
 */
esp_err_t bmp280_read(bmp280_handle_t dev, float *temperature_c, float *pressure_hpa);

#ifdef __cplusplus
}
#endif
