/*
 * Minimal Modbus RTU master over RS-485 - ESP-IDF native.
 *
 * Replaces the Arduino "ModbusMaster" library.
 *
 * Direction control (MAX485 DE/RE_NEG) is handled by the UART peripheral in
 * UART_MODE_RS485_HALF_DUPLEX: hardware asserts RTS for exactly the duration
 * of the transmission. This removes the software preTransmission/
 * postTransmission callbacks entirely, and with them the race where the driver
 * is released before the last stop bit has left the shift register.
 *
 * Wire DE and RE_NEG together to the single RTS pin (the standard MAX485
 * half-duplex wiring).
 */
#pragma once

#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* rts_pin: modulo de direcao automatica, sem controle de DE/RE por software. */
#define MODBUS_RTU_NO_RTS (-1)

typedef struct {
    uart_port_t uart_port;
    int         tx_pin;      /* MAX485 DI  */
    int         rx_pin;      /* MAX485 RO  */
    /* MAX485 DE + RE_NEG ligados juntos. Use MODBUS_RTU_NO_RTS quando o modulo
     * for de direcao automatica (so VCC/GND/A/B/RXD/TXD): nesse caso o
     * transceptor comuta sozinho e o driver usa UART comum em vez do modo
     * RS-485 half-duplex, cuja deteccao de colisao atrapalha esses modulos. */
    int         rts_pin;
    int         baud_rate;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uint32_t    response_timeout_ms;
} modbus_rtu_config_t;

#define MODBUS_RTU_CONFIG_DEFAULT()          \
    (modbus_rtu_config_t){                   \
        .uart_port           = UART_NUM_2,   \
        .rts_pin             = MODBUS_RTU_NO_RTS, \
        .baud_rate           = 4800,         \
        .parity              = UART_PARITY_DISABLE, \
        .stop_bits           = UART_STOP_BITS_1,    \
        .response_timeout_ms = 500,          \
    }

typedef struct modbus_rtu_s *modbus_rtu_handle_t;

esp_err_t modbus_rtu_create(const modbus_rtu_config_t *cfg, modbus_rtu_handle_t *out);
void modbus_rtu_delete(modbus_rtu_handle_t h);

/**
 * Function code 0x03 - Read Holding Registers.
 *
 * @param slave_addr  1..247
 * @param start_reg   first register address
 * @param count       number of 16-bit registers, 1..125
 * @param[out] regs   caller-provided array of at least `count` entries
 *
 * @return ESP_OK on a valid, CRC-checked response.
 *         ESP_ERR_TIMEOUT        - no reply within response_timeout_ms
 *         ESP_ERR_INVALID_CRC    - CRC mismatch (noise, wrong baud, no termination)
 *         ESP_ERR_INVALID_RESPONSE - malformed frame or wrong slave/function echo
 *         ESP_FAIL               - slave returned a Modbus exception (logged)
 */
esp_err_t modbus_rtu_read_holding_registers(modbus_rtu_handle_t h, uint8_t slave_addr,
                                            uint16_t start_reg, uint16_t count,
                                            uint16_t *regs);

/** Last Modbus exception code received, or 0 if none. */
uint8_t modbus_rtu_last_exception(modbus_rtu_handle_t h);

#ifdef __cplusplus
}
#endif
