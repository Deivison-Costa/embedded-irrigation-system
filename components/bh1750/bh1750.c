#include <stdlib.h>
#include <string.h>

#include "bh1750.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bh1750";

#define BH1750_CMD_POWER_DOWN 0x00
#define BH1750_CMD_POWER_ON   0x01
#define BH1750_CMD_RESET      0x07

#define BH1750_I2C_TIMEOUT_MS 1000

struct bh1750_dev_s {
    i2c_master_dev_handle_t dev;
    bh1750_mode_t mode;
    uint32_t conversion_ms;
    bool one_time;
};

static esp_err_t bh1750_write_cmd(bh1750_handle_t h, uint8_t cmd)
{
    return i2c_master_transmit(h->dev, &cmd, 1, BH1750_I2C_TIMEOUT_MS);
}

esp_err_t bh1750_create(i2c_master_bus_handle_t bus, uint8_t addr,
                        bh1750_mode_t mode, bh1750_handle_t *out)
{
    if (bus == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    esp_err_t err = i2c_master_probe(bus, addr, BH1750_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no device ACKed at 0x%02X: %s", addr, esp_err_to_name(err));
        return err;
    }

    bh1750_handle_t h = calloc(1, sizeof(*h));
    if (h == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000, /* BH1750 is specified up to 400 kHz;
                                    * 100 kHz is safe on long sensor cables. */
    };
    err = i2c_master_bus_add_device(bus, &cfg, &h->dev);
    if (err != ESP_OK) {
        free(h);
        return err;
    }

    h->mode          = mode;
    h->one_time      = (mode & 0x20) != 0;
    h->conversion_ms = ((mode & 0x03) == 0x03) ? 24 : 180; /* datasheet max + margin */

    if ((err = bh1750_write_cmd(h, BH1750_CMD_POWER_ON)) != ESP_OK) goto fail;
    if ((err = bh1750_write_cmd(h, BH1750_CMD_RESET)) != ESP_OK) goto fail;
    if ((err = bh1750_write_cmd(h, (uint8_t)mode)) != ESP_OK) goto fail;

    /* First continuous conversion must complete before the first read. */
    vTaskDelay(pdMS_TO_TICKS(h->conversion_ms));

    ESP_LOGI(TAG, "initialised at 0x%02X, mode 0x%02X", addr, (unsigned)mode);
    *out = h;
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
    i2c_master_bus_rm_device(h->dev);
    free(h);
    return err;
}

void bh1750_delete(bh1750_handle_t h)
{
    if (h == NULL) {
        return;
    }
    bh1750_write_cmd(h, BH1750_CMD_POWER_DOWN);
    i2c_master_bus_rm_device(h->dev);
    free(h);
}

esp_err_t bh1750_read_lux(bh1750_handle_t h, float *lux)
{
    if (h == NULL || lux == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* One-time modes power down after each conversion and must be re-armed. */
    if (h->one_time) {
        esp_err_t err = bh1750_write_cmd(h, (uint8_t)h->mode);
        if (err != ESP_OK) {
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(h->conversion_ms));
    }

    uint8_t raw[2] = {0};
    esp_err_t err = i2c_master_receive(h->dev, raw, sizeof(raw), BH1750_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t counts = ((uint16_t)raw[0] << 8) | raw[1];

    /* Datasheet: lx = counts / 1.2 ; H-resolution mode 2 doubles the counts. */
    float value = (float)counts / 1.2f;
    if (h->mode == BH1750_MODE_CONTINUOUS_HIGH_RES2 ||
        h->mode == BH1750_MODE_ONETIME_HIGH_RES2) {
        value /= 2.0f;
    }

    *lux = value;
    return ESP_OK;
}
