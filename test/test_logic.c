/* Host-side verification of the algorithms that do not need hardware.
 * The functions are copied verbatim from the firmware sources. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

/* O parser NMEA nao e copiado: nmea_parse.c do firmware e compilado junto,
 * entao o teste exercita exatamente o codigo que roda na placa. */
#include "nmea_parse.h"

static int failures = 0;
#define CHECK(cond, fmt, ...) do { \
    if (!(cond)) { printf("  FAIL: " fmt "\n", ##__VA_ARGS__); failures++; } \
    else         { printf("  ok:   " fmt "\n", ##__VA_ARGS__); } } while (0)

/* ---------- from components/modbus_rtu/modbus_rtu.c ---------- */
static uint16_t modbus_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) crc = (uint16_t)((crc >> 1) ^ 0xA001);
            else              crc >>= 1;
        }
    }
    return crc;
}

/* ---------- from components/bmp280/bmp280.c ---------- */
typedef struct {
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} cal_t;

static int32_t compensate_T(const cal_t *h, int32_t adc_T, int32_t *t_fine)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)h->dig_T1 << 1))) * ((int32_t)h->dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)h->dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)h->dig_T1))) >> 12) * ((int32_t)h->dig_T3)) >> 14;
    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8;
}

static uint32_t compensate_P(const cal_t *h, int32_t adc_P, int32_t t_fine)
{
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)h->dig_P6;
    var2 = var2 + ((var1 * (int64_t)h->dig_P5) << 17);
    var2 = var2 + (((int64_t)h->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)h->dig_P3) >> 8) + ((var1 * (int64_t)h->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)h->dig_P1) >> 33;
    if (var1 == 0) return 0;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)h->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)h->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)h->dig_P7) << 4);
    return (uint32_t)p;
}

int main(void)
{
    printf("== Modbus CRC-16 ==\n");
    /* Modbus application protocol spec, read-holding-registers example:
     * slave 0x11, FC 0x03, start 0x006B, count 0x0003 -> CRC 0x8776 (lo 0x76 hi 0x87) */
    uint8_t f1[] = {0x11, 0x03, 0x00, 0x6B, 0x00, 0x03};
    CHECK(modbus_crc16(f1, 6) == 0x8776, "spec frame 11 03 006B 0003 -> 0x%04X (want 0x8776)",
          modbus_crc16(f1, 6));

    /* CRC-16/MODBUS catalogue check value - the authoritative test. */
    CHECK(modbus_crc16((const uint8_t *)"123456789", 9) == 0x4B37,
          "CRC(\"123456789\") = 0x%04X (catalogue check value 0x4B37)",
          modbus_crc16((const uint8_t *)"123456789", 9));

    /* Published vector 01 04 02 FF FF; note the usual quotation "B8 80" is the
     * wire order (low byte first), i.e. the integer 0x80B8. */
    uint8_t f2[] = {0x01, 0x04, 0x02, 0xFF, 0xFF};
    uint16_t c2 = modbus_crc16(f2, 5);
    CHECK((c2 & 0xFF) == 0xB8 && (c2 >> 8) == 0x80,
          "frame 01 04 02 FFFF -> wire bytes %02X %02X (want B8 80)", c2 & 0xFF, c2 >> 8);

    /* The exact request this firmware sends: slave 1, FC3, reg 0, count 1 */
    uint8_t f3[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
    uint16_t c3 = modbus_crc16(f3, 6);
    printf("  info: anemometer request CRC = 0x%04X (wire bytes %02X %02X)\n",
           c3, c3 & 0xFF, c3 >> 8);
    /* Self-check: appending the CRC must make the whole frame check to zero. */
    uint8_t f3full[8]; memcpy(f3full, f3, 6);
    f3full[6] = c3 & 0xFF; f3full[7] = c3 >> 8;
    CHECK(modbus_crc16(f3full, 8) == 0, "CRC over frame+CRC is 0x%04X (want 0)",
          modbus_crc16(f3full, 8));

    printf("\n== BMP280 compensation (Bosch datasheet worked example) ==\n");
    cal_t cal = {
        .dig_T1 = 27504, .dig_T2 = 26435, .dig_T3 = -1000,
        .dig_P1 = 36477, .dig_P2 = -10685, .dig_P3 = 3024, .dig_P4 = 2855,
        .dig_P5 = 140, .dig_P6 = -7, .dig_P7 = 15500, .dig_P8 = -14600, .dig_P9 = 6000,
    };
    int32_t t_fine = 0;
    int32_t T = compensate_T(&cal, 519888, &t_fine);
    uint32_t P = compensate_P(&cal, 415148, t_fine);
    CHECK(t_fine == 128422, "t_fine = %d (want 128422)", t_fine);
    CHECK(T == 2508, "temperature = %d centi-degC = %.2f C (want 2508 / 25.08)", T, T / 100.0);
    double pa = P / 256.0;
    CHECK(fabs(pa - 100653.27) < 0.5, "pressure = %.2f Pa = %.2f hPa (want ~100653.27 Pa)",
          pa, pa / 100.0);
    /* And the exact conversion the driver reports: */
    printf("  info: driver would publish %.2f hPa\n", (P / 256.0f) / 100.0f);

    printf("\n== NMEA checksum (codigo real, nao copia) ==\n");
    const char *gga = "$GPGGA,143012,1855.2340,S,04816.8900,W,1,09,0.8,862.4,M,-6.2,M,,*5E";
    const char *rmc = "$GPRMC,143012,A,1855.2340,S,04816.8900,W,003.7,221.5,050926,020.3,W*61";
    CHECK(nmea_verify_checksum(gga, strlen(gga)), "GGA valida aceita");
    CHECK(nmea_verify_checksum(rmc, strlen(rmc)), "RMC valida aceita");
    CHECK(!nmea_verify_checksum("$GPGGA,143012,1855.2340,S,04816.8900,W,1,09,0.8,862.4,M,-6.2,M,,*5F", 68),
          "checksum corrompido rejeitado");
    CHECK(!nmea_verify_checksum("$GPGGA,1", 8), "sentenca sem '*' rejeitada");
    CHECK(!nmea_verify_checksum("GPGGA,1*47", 10), "sentenca sem '$' rejeitada");
    CHECK(!nmea_verify_checksum("$GPGGA,1*4", 10), "checksum truncado rejeitado");
    CHECK(!nmea_verify_checksum("$GPGGA,1*ZZ", 11), "checksum nao-hexadecimal rejeitado");

    printf("\n== GGA completa -> fix (sem precisar de satelite) ==\n");
    nmea_gps_fix_t fix;
    char buf[128];

    memset(&fix, 0, sizeof(fix));
    strcpy(buf, gga);
    CHECK(nmea_parse_sentence(buf, &fix, 12345), "GGA reconhecida");
    CHECK(fix.valid, "fix marcado valido");
    CHECK(fabs(fix.latitude_deg + 18.920567) < 1e-5,
          "latitude = %.6f (esperado -18.920567, hemisferio sul)", fix.latitude_deg);
    CHECK(fabs(fix.longitude_deg + 48.281500) < 1e-5,
          "longitude = %.6f (esperado -48.281500, oeste)", fix.longitude_deg);
    CHECK(fabs(fix.altitude_m - 862.4) < 1e-6, "altitude = %.1f m", fix.altitude_m);
    CHECK(fix.satellites == 9, "satelites = %u", (unsigned)fix.satellites);
    CHECK(fabs(fix.hdop - 0.8f) < 1e-5, "hdop = %.1f", (double)fix.hdop);
    CHECK(fix.fix_quality == 1, "fix_quality = %u", (unsigned)fix.fix_quality);
    CHECK(fix.hour == 14 && fix.minute == 30 && fix.second == 12,
          "hora UTC = %02u:%02u:%02u", fix.hour, fix.minute, fix.second);
    CHECK(fix.timestamp_ms == 12345, "timestamp propagado = %lld", (long long)fix.timestamp_ms);

    printf("\n== RMC completa -> fix ==\n");
    memset(&fix, 0, sizeof(fix));
    strcpy(buf, rmc);
    CHECK(nmea_parse_sentence(buf, &fix, 999), "RMC reconhecida");
    CHECK(fix.valid, "fix marcado valido");
    CHECK(fabs(fix.latitude_deg + 18.920567) < 1e-5, "latitude = %.6f", fix.latitude_deg);
    CHECK(fabs(fix.speed_kmh - 3.7 * 1.852) < 1e-6,
          "velocidade = %.4f km/h (3.7 nos)", fix.speed_kmh);
    CHECK(fabs(fix.course_deg - 221.5) < 1e-6, "rumo = %.1f graus", fix.course_deg);
    CHECK(fix.day == 5 && fix.month == 9 && fix.year == 2026,
          "data = %04u-%02u-%02u", fix.year, fix.month, fix.day);

    printf("\n== sem fix: nao pode reportar posicao ==\n");
    memset(&fix, 0, sizeof(fix));
    strcpy(buf, "$GPGGA,143012,,,,,0,00,,,M,,M,,*63");
    nmea_parse_sentence(buf, &fix, 1);
    CHECK(!fix.valid, "GGA com fix_quality=0 nao vira fix valido");
    CHECK(fix.latitude_deg == 0.0, "latitude intacta em %.1f", fix.latitude_deg);

    memset(&fix, 0, sizeof(fix));
    strcpy(buf, "$GPRMC,143012,V,,,,,,,050926,,*3C");
    nmea_parse_sentence(buf, &fix, 1);
    CHECK(!fix.valid, "RMC com status 'V' (void) nao vira fix valido");

    printf("\n== fix valido nao pode ser apagado por sentenca sem fix ==\n");
    memset(&fix, 0, sizeof(fix));
    strcpy(buf, gga);
    nmea_parse_sentence(buf, &fix, 100);
    double lat_bom = fix.latitude_deg;
    strcpy(buf, "$GPGSV,3,1,11,01,05,040,20,03,25,120,35,06,70,200,42,11,45,300,38*74");
    CHECK(!nmea_parse_sentence(buf, &fix, 200), "GSV ignorada (tipo nao tratado)");
    CHECK(fix.valid && fix.latitude_deg == lat_bom,
          "fix anterior preservado apos sentenca ignorada");

    printf("\n== talker GNSS multi-constelacao (GN em vez de GP) ==\n");
    memset(&fix, 0, sizeof(fix));
    strcpy(buf, "$GNGGA,143013,1855.2340,S,04816.8900,W,2,11,0.6,862.9,M,-6.2,M,,*48");
    CHECK(nmea_parse_sentence(buf, &fix, 1), "GNGGA reconhecida");
    CHECK(fix.valid && fix.fix_quality == 2, "fix DGPS (quality=2) aceito");
    CHECK(fix.satellites == 11, "satelites = %u", (unsigned)fix.satellites);

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED",
           failures, failures == 1 ? "" : "s");
    return failures != 0;
}
