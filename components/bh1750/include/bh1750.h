/*
 * BH1750 ambient light sensor - ESP-IDF native driver.
 *
 * Replaces the Arduino "BH1750" library. Uses the i2c_master driver
 * (driver/i2c_master.h) introduced in ESP-IDF v5.2; the legacy driver/i2c.h
 * API is deprecated and emits build warnings on v5.5.
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ADDR pin low (default on most breakout boards) -> 0x23, ADDR high -> 0x5C. */
#define BH1750_ADDR_LO 0x23
#define BH1750_ADDR_HI 0x5C

typedef enum {
    BH1750_MODE_CONTINUOUS_HIGH_RES  = 0x10, /* 1 lx,   ~120 ms */
    BH1750_MODE_CONTINUOUS_HIGH_RES2 = 0x11, /* 0.5 lx, ~120 ms */
    BH1750_MODE_CONTINUOUS_LOW_RES   = 0x13, /* 4 lx,   ~16 ms  */
    BH1750_MODE_ONETIME_HIGH_RES     = 0x20,
    BH1750_MODE_ONETIME_HIGH_RES2    = 0x21,
    BH1750_MODE_ONETIME_LOW_RES      = 0x23,
} bh1750_mode_t;

typedef struct bh1750_dev_s *bh1750_handle_t;

/**
 * Probe and initialise the sensor on an existing I2C master bus.
 *
 * @param bus       bus created with i2c_new_master_bus()
 * @param addr      BH1750_ADDR_LO or BH1750_ADDR_HI
 * @param mode      measurement mode
 * @param[out] out  device handle
 */
esp_err_t bh1750_create(i2c_master_bus_handle_t bus, uint8_t addr,
                        bh1750_mode_t mode, bh1750_handle_t *out);

void bh1750_delete(bh1750_handle_t dev);

/**
 * Read illuminance.
 *
 * @param[out] lux  illuminance in lux (0 .. ~54612 lx in high-res mode)
 * @return ESP_OK, or an I2C error. Never returns a sentinel value inside
 *         *lux - unlike the Arduino library, which returns -1/-2 as a float
 *         and silently becomes 65535 when the caller stores it in a uint16_t.
 */
esp_err_t bh1750_read_lux(bh1750_handle_t dev, float *lux);

#ifdef __cplusplus
}
#endif
