#include <stdlib.h>
#include <string.h>

#include "bmp280.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bmp280";

#define BMP280_REG_CALIB    0x88
#define BMP280_REG_ID       0xD0
#define BMP280_REG_RESET    0xE0
#define BMP280_REG_STATUS   0xF3
#define BMP280_REG_CTRL     0xF4
#define BMP280_REG_CONFIG   0xF5
#define BMP280_REG_PRESS    0xF7

#define BMP280_CHIP_ID      0x58 /* BMP280 */
#define BME280_CHIP_ID      0x60 /* BME280 - register-compatible for T and P */
#define BMP280_RESET_VALUE  0xB6

#define BMP280_I2C_TIMEOUT_MS 1000

struct bmp280_dev_s {
    i2c_master_dev_handle_t dev;
    /* Bosch calibration words, names taken verbatim from the datasheet. */
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  chip_id;
};

static esp_err_t bmp280_read_regs(bmp280_handle_t h, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(h->dev, &reg, 1, buf, len, BMP280_I2C_TIMEOUT_MS);
}

static esp_err_t bmp280_write_reg(bmp280_handle_t h, uint8_t reg, uint8_t val)
{
    uint8_t payload[2] = {reg, val};
    return i2c_master_transmit(h->dev, payload, sizeof(payload), BMP280_I2C_TIMEOUT_MS);
}

static esp_err_t bmp280_read_calibration(bmp280_handle_t h)
{
    uint8_t c[24];
    esp_err_t err = bmp280_read_regs(h, BMP280_REG_CALIB, c, sizeof(c));
    if (err != ESP_OK) {
        return err;
    }

    h->dig_T1 = (uint16_t)(c[1]  << 8 | c[0]);
    h->dig_T2 = (int16_t) (c[3]  << 8 | c[2]);
    h->dig_T3 = (int16_t) (c[5]  << 8 | c[4]);
    h->dig_P1 = (uint16_t)(c[7]  << 8 | c[6]);
    h->dig_P2 = (int16_t) (c[9]  << 8 | c[8]);
    h->dig_P3 = (int16_t) (c[11] << 8 | c[10]);
    h->dig_P4 = (int16_t) (c[13] << 8 | c[12]);
    h->dig_P5 = (int16_t) (c[15] << 8 | c[14]);
    h->dig_P6 = (int16_t) (c[17] << 8 | c[16]);
    h->dig_P7 = (int16_t) (c[19] << 8 | c[18]);
    h->dig_P8 = (int16_t) (c[21] << 8 | c[20]);
    h->dig_P9 = (int16_t) (c[23] << 8 | c[22]);

    /* An all-zero or all-0xFF calibration block means the read did not really
     * reach the sensor; the compensation would silently produce garbage. */
    if (h->dig_T1 == 0 || h->dig_P1 == 0 ||
        (h->dig_T1 == 0xFFFF && h->dig_P1 == 0xFFFF)) {
        ESP_LOGE(TAG, "implausible calibration data (T1=%u P1=%u)",
                 (unsigned)h->dig_T1, (unsigned)h->dig_P1);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t bmp280_create(i2c_master_bus_handle_t bus, uint8_t addr,
                        const bmp280_config_t *cfg, bmp280_handle_t *out)
{
    if (bus == NULL || cfg == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    esp_err_t err = i2c_master_probe(bus, addr, BMP280_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no device ACKed at 0x%02X: %s", addr, esp_err_to_name(err));
        return err;
    }

    bmp280_handle_t h = calloc(1, sizeof(*h));
    if (h == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000,
    };
    err = i2c_master_bus_add_device(bus, &dev_cfg, &h->dev);
    if (err != ESP_OK) {
        free(h);
        return err;
    }

    if ((err = bmp280_read_regs(h, BMP280_REG_ID, &h->chip_id, 1)) != ESP_OK) goto fail;
    if (h->chip_id != BMP280_CHIP_ID && h->chip_id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "unexpected chip id 0x%02X at 0x%02X (expected 0x58/0x60)",
                 h->chip_id, addr);
        err = ESP_ERR_NOT_FOUND;
        goto fail;
    }

    if ((err = bmp280_write_reg(h, BMP280_REG_RESET, BMP280_RESET_VALUE)) != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(10)); /* datasheet start-up time: 2 ms */

    /* Wait for the NVM copy that follows a reset to finish (status bit 0). */
    for (int i = 0; i < 20; i++) {
        uint8_t status;
        if ((err = bmp280_read_regs(h, BMP280_REG_STATUS, &status, 1)) != ESP_OK) goto fail;
        if ((status & 0x01) == 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if ((err = bmp280_read_calibration(h)) != ESP_OK) goto fail;

    uint8_t config = (uint8_t)((cfg->standby << 5) | (cfg->filter << 2));
    if ((err = bmp280_write_reg(h, BMP280_REG_CONFIG, config)) != ESP_OK) goto fail;

    /* ctrl_meas last: writing it starts normal mode (bits 1:0 = 0b11). */
    uint8_t ctrl = (uint8_t)((cfg->osrs_t << 5) | (cfg->osrs_p << 2) | 0x03);
    if ((err = bmp280_write_reg(h, BMP280_REG_CTRL, ctrl)) != ESP_OK) goto fail;

    /* First conversion in normal mode; worst case ~43 ms at x16 oversampling. */
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "initialised at 0x%02X, chip id 0x%02X", addr, h->chip_id);
    *out = h;
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
    i2c_master_bus_rm_device(h->dev);
    free(h);
    return err;
}

void bmp280_delete(bmp280_handle_t h)
{
    if (h == NULL) {
        return;
    }
    bmp280_write_reg(h, BMP280_REG_CTRL, 0x00); /* sleep mode */
    i2c_master_bus_rm_device(h->dev);
    free(h);
}

/* Datasheet 3.11.3, "Compensation formula in 32 bit fixed point".
 * Returns temperature in 0.01 degC and stores the shared t_fine term. */
static int32_t bmp280_compensate_temperature(bmp280_handle_t h, int32_t adc_T, int32_t *t_fine)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)h->dig_T1 << 1))) * ((int32_t)h->dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)h->dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)h->dig_T1))) >> 12) *
                    ((int32_t)h->dig_T3)) >> 14;
    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8;
}

/* Returns pressure in Q24.8 pascal (i.e. Pa * 256). */
static uint32_t bmp280_compensate_pressure(bmp280_handle_t h, int32_t adc_P, int32_t t_fine)
{
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)h->dig_P6;
    var2 = var2 + ((var1 * (int64_t)h->dig_P5) << 17);
    var2 = var2 + (((int64_t)h->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)h->dig_P3) >> 8) + ((var1 * (int64_t)h->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)h->dig_P1) >> 33;

    if (var1 == 0) {
        return 0; /* datasheet: avoid division by zero */
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)h->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)h->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)h->dig_P7) << 4);

    return (uint32_t)p;
}

esp_err_t bmp280_read(bmp280_handle_t h, float *temperature_c, float *pressure_hpa)
{
    if (h == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Burst-read 0xF7..0xFC so pressure and temperature come from the same
     * conversion; the sensor shadows these registers during a burst. */
    uint8_t raw[6];
    esp_err_t err = bmp280_read_regs(h, BMP280_REG_PRESS, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int32_t adc_P = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4));

    /* 0x80000 is the reset value: the channel is disabled or not yet converted. */
    if (adc_T == 0x80000 || adc_P == 0x80000) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t t_fine = 0;
    int32_t temp_centi = bmp280_compensate_temperature(h, adc_T, &t_fine);
    uint32_t press_q24_8 = bmp280_compensate_pressure(h, adc_P, t_fine);

    if (press_q24_8 == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (temperature_c) {
        *temperature_c = (float)temp_centi / 100.0f;
    }
    if (pressure_hpa) {
        /* Q24.8 Pa -> Pa -> hPa */
        *pressure_hpa = ((float)press_q24_8 / 256.0f) / 100.0f;
    }
    return ESP_OK;
}
