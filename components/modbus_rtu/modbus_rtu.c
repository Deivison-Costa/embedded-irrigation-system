#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modbus_rtu.h"

static const char *TAG = "modbus_rtu";

#define MODBUS_FC_READ_HOLDING   0x03
#define MODBUS_EXCEPTION_FLAG    0x80
#define MODBUS_MAX_ADU_LEN       256
#define MODBUS_UART_BUF_SIZE     512 /* driver minimum is 128; 512 fits any ADU */

struct modbus_rtu_s {
    uart_port_t port;
    uint32_t    response_timeout_ms;
    uint32_t    inter_frame_ms;
    uint8_t     last_exception;
    bool        driver_installed;
};

/* Standard Modbus CRC-16 (polynomial 0xA001, initial value 0xFFFF). */
static uint16_t modbus_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

esp_err_t modbus_rtu_create(const modbus_rtu_config_t *cfg, modbus_rtu_handle_t *out)
{
    if (cfg == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    /* Guards a division by zero below, and catches the DE/RE-on-the-UART-pin
     * miswiring that the original sketch shipped with. */
    if (cfg->baud_rate <= 0) {
        ESP_LOGE(TAG, "invalid baud rate %d", cfg->baud_rate);
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->rts_pin == cfg->tx_pin || cfg->rts_pin == cfg->rx_pin ||
        cfg->tx_pin == cfg->rx_pin) {
        ESP_LOGE(TAG, "TX=%d, RX=%d and RTS(DE/RE)=%d must be three distinct pins",
                 cfg->tx_pin, cfg->rx_pin, cfg->rts_pin);
        return ESP_ERR_INVALID_ARG;
    }

    modbus_rtu_handle_t h = calloc(1, sizeof(*h));
    if (h == NULL) {
        return ESP_ERR_NO_MEM;
    }
    h->port                = cfg->uart_port;
    h->response_timeout_ms = cfg->response_timeout_ms ? cfg->response_timeout_ms : 500;

    /* Modbus requires >= 3.5 character times of silence between frames.
     * One character is 1 start + 8 data + parity + stop bits. */
    uint32_t bits_per_char = 1u + 8u + (cfg->parity == UART_PARITY_DISABLE ? 0u : 1u) +
                             (cfg->stop_bits == UART_STOP_BITS_2 ? 2u : 1u);
    uint32_t silence_us = (35 * bits_per_char * 1000000UL) / (10UL * (uint32_t)cfg->baud_rate);
    /* Above 19200 baud the spec fixes the gap at 1.75 ms; below it the computed
     * value dominates. Round up to whole FreeRTOS ticks, minimum 2 ms. */
    h->inter_frame_ms = (silence_us + 999) / 1000;
    if (h->inter_frame_ms < 2) {
        h->inter_frame_ms = 2;
    }

    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = cfg->parity,
        .stop_bits = cfg->stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(h->port, MODBUS_UART_BUF_SIZE,
                                        MODBUS_UART_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        free(h);
        return err;
    }
    h->driver_installed = true;

    if ((err = uart_param_config(h->port, &uart_cfg)) != ESP_OK) goto fail;
    if ((err = uart_set_pin(h->port, cfg->tx_pin, cfg->rx_pin,
                            cfg->rts_pin, UART_PIN_NO_CHANGE)) != ESP_OK) goto fail;

    /* Hardware-timed DE/RE. The peripheral raises RTS before the first start
     * bit and drops it after the last stop bit has been shifted out. */
    if ((err = uart_set_mode(h->port, UART_MODE_RS485_HALF_DUPLEX)) != ESP_OK) goto fail;

    /* The RS-485 driver echoes its own transmission on RX; the peripheral's
     * collision detection tolerates this, but the read timeout must still be
     * short enough to detect a silent bus. */
    if ((err = uart_set_rx_timeout(h->port, 3)) != ESP_OK) goto fail;

    ESP_LOGI(TAG, "UART%d RS-485 half-duplex: TX=%d RX=%d RTS(DE/RE)=%d @ %d baud, "
                  "inter-frame gap %" PRIu32 " ms",
             (int)h->port, cfg->tx_pin, cfg->rx_pin, cfg->rts_pin, cfg->baud_rate,
             h->inter_frame_ms);

    *out = h;
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
    uart_driver_delete(h->port);
    free(h);
    return err;
}

void modbus_rtu_delete(modbus_rtu_handle_t h)
{
    if (h == NULL) {
        return;
    }
    if (h->driver_installed) {
        uart_driver_delete(h->port);
    }
    free(h);
}

uint8_t modbus_rtu_last_exception(modbus_rtu_handle_t h)
{
    return h ? h->last_exception : 0;
}

static const char *modbus_exception_str(uint8_t code)
{
    switch (code) {
    case 0x01: return "illegal function";
    case 0x02: return "illegal data address";
    case 0x03: return "illegal data value";
    case 0x04: return "slave device failure";
    case 0x05: return "acknowledge";
    case 0x06: return "slave device busy";
    case 0x08: return "memory parity error";
    case 0x0A: return "gateway path unavailable";
    case 0x0B: return "gateway target failed to respond";
    default:   return "unknown exception";
    }
}

esp_err_t modbus_rtu_read_holding_registers(modbus_rtu_handle_t h, uint8_t slave_addr,
                                            uint16_t start_reg, uint16_t count,
                                            uint16_t *regs)
{
    if (h == NULL || regs == NULL || count == 0 || count > 125 ||
        slave_addr < 1 || slave_addr > 247) {
        return ESP_ERR_INVALID_ARG;
    }

    h->last_exception = 0;

    uint8_t req[8];
    req[0] = slave_addr;
    req[1] = MODBUS_FC_READ_HOLDING;
    req[2] = (uint8_t)(start_reg >> 8);
    req[3] = (uint8_t)(start_reg & 0xFF);
    req[4] = (uint8_t)(count >> 8);
    req[5] = (uint8_t)(count & 0xFF);
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF); /* CRC is transmitted low byte first */
    req[7] = (uint8_t)(crc >> 8);

    /* Discard anything left from a previous partial or late reply, otherwise it
     * is mistaken for the answer to this request. */
    uart_flush_input(h->port);

    int written = uart_write_bytes(h->port, req, sizeof(req));
    if (written != (int)sizeof(req)) {
        return ESP_FAIL;
    }
    /* Block until the frame has physically left the wire so the response
     * timeout measures the slave's turnaround, not our own transmission. */
    esp_err_t err = uart_wait_tx_done(h->port, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        return err;
    }

    const size_t expected_len = 5 + (size_t)count * 2; /* addr fc bc data crc_lo crc_hi */
    uint8_t resp[MODBUS_MAX_ADU_LEN];
    size_t received = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(h->response_timeout_ms);

    while (received < expected_len) {
        TickType_t now = xTaskGetTickCount();
        if (now >= deadline) {
            break;
        }
        int n = uart_read_bytes(h->port, resp + received, expected_len - received,
                                deadline - now);
        if (n <= 0) {
            break;
        }
        received += (size_t)n;

        /* An exception reply is only 5 bytes long and would otherwise stall
         * here waiting for data that will never arrive. */
        if (received >= 2 && (resp[1] & MODBUS_EXCEPTION_FLAG)) {
            while (received < 5) {
                now = xTaskGetTickCount();
                if (now >= deadline) break;
                n = uart_read_bytes(h->port, resp + received, 5 - received, deadline - now);
                if (n <= 0) break;
                received += (size_t)n;
            }
            break;
        }
    }

    /* Enforce the silent interval before the next request may start. */
    vTaskDelay(pdMS_TO_TICKS(h->inter_frame_ms));

    if (received == 0) {
        ESP_LOGW(TAG, "no response from slave %u", (unsigned)slave_addr);
        return ESP_ERR_TIMEOUT;
    }

    if (received >= 5 && (resp[1] & MODBUS_EXCEPTION_FLAG)) {
        if (modbus_crc16(resp, 3) != (uint16_t)(resp[3] | (resp[4] << 8))) {
            return ESP_ERR_INVALID_CRC;
        }
        h->last_exception = resp[2];
        ESP_LOGW(TAG, "slave %u exception 0x%02X (%s)", (unsigned)slave_addr,
                 resp[2], modbus_exception_str(resp[2]));
        return ESP_FAIL;
    }

    if (received < expected_len) {
        ESP_LOGW(TAG, "short frame: %u of %u bytes", (unsigned)received,
                 (unsigned)expected_len);
        return ESP_ERR_TIMEOUT;
    }

    if (resp[0] != slave_addr || resp[1] != MODBUS_FC_READ_HOLDING ||
        resp[2] != (uint8_t)(count * 2)) {
        ESP_LOGW(TAG, "malformed frame: addr=0x%02X fc=0x%02X bc=0x%02X",
                 resp[0], resp[1], resp[2]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint16_t computed = modbus_crc16(resp, expected_len - 2);
    uint16_t received_crc = (uint16_t)(resp[expected_len - 2] |
                                       (resp[expected_len - 1] << 8));
    if (computed != received_crc) {
        ESP_LOGW(TAG, "CRC mismatch: computed 0x%04X, received 0x%04X",
                 computed, received_crc);
        return ESP_ERR_INVALID_CRC;
    }

    for (uint16_t i = 0; i < count; i++) {
        regs[i] = (uint16_t)((resp[3 + i * 2] << 8) | resp[4 + i * 2]);
    }
    return ESP_OK;
}
