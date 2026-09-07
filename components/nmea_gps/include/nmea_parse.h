/*
 * Parser NMEA 0183 puro - sem dependencia de ESP-IDF ou FreeRTOS.
 *
 * Separado de nmea_gps.c de proposito: assim os testes de host (test/) compilam
 * este arquivo diretamente e exercitam o codigo que roda na placa, em vez de
 * uma copia que pode divergir com o tempo.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     valid;          /* true quando o receptor reporta fix utilizavel */
    double   latitude_deg;   /* positivo ao norte */
    double   longitude_deg;  /* positivo a leste  */
    double   altitude_m;     /* metros acima do nivel medio do mar */
    double   speed_kmh;
    double   course_deg;
    uint8_t  satellites;
    float    hdop;
    uint8_t  fix_quality;    /* campo 6 da GGA: 0 nenhum, 1 GPS, 2 DGPS, ... */
    /* UTC da ultima RMC; ano 0 significa "ainda nao recebido". */
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
    /* Milissegundos desde o boot quando o fix foi atualizado. */
    int64_t  timestamp_ms;
    /* Diagnostico. */
    uint32_t sentences_ok;
    uint32_t checksum_errors;
} nmea_gps_fix_t;

/** Confere o checksum XOR entre '$' e '*'. */
bool nmea_verify_checksum(const char *sentence, size_t len);

/**
 * Interpreta uma sentenca ja validada por checksum e atualiza o fix.
 *
 * @param sentence  modificada no lugar (os campos sao separados nas virgulas)
 * @param now_ms    relogio do chamador, gravado em fix->timestamp_ms
 * @return true se a sentenca era de um tipo tratado (GGA ou RMC)
 */
bool nmea_parse_sentence(char *sentence, nmea_gps_fix_t *fix, int64_t now_ms);

#ifdef __cplusplus
}
#endif
