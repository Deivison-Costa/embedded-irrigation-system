/* Host-side verification of the algorithms that do not need hardware.
 * The functions are copied verbatim from the firmware sources. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

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

/* ---------- from components/nmea_gps/nmea_gps.c ---------- */
static bool nmea_verify_checksum(const char *sentence, size_t len)
{
    if (len < 4 || sentence[0] != '$') return false;
    const char *star = memchr(sentence, '*', len);
    if (star == NULL || (size_t)(star - sentence) + 3 > len) return false;
    uint8_t computed = 0;
    for (const char *p = sentence + 1; p < star; p++) computed ^= (uint8_t)*p;
    char hex[3] = {star[1], star[2], '\0'};
    char *end = NULL;
    unsigned long received = strtoul(hex, &end, 16);
    if (end != hex + 2) return false;
    return computed == (uint8_t)received;
}

static bool nmea_parse_coord(const char *value, const char *hemi, double *out)
{
    if (value == NULL || *value == '\0' || hemi == NULL || *hemi == '\0') return false;
    char *end = NULL;
    double raw = strtod(value, &end);
    if (end == value) return false;
    double degrees = (double)((int)(raw / 100.0));
    double minutes = raw - degrees * 100.0;
    if (minutes < 0.0 || minutes >= 60.0) return false;
    double result = degrees + minutes / 60.0;
    if (*hemi == 'S' || *hemi == 'W') result = -result;
    else if (*hemi != 'N' && *hemi != 'E') return false;
    *out = result;
    return true;
}

static int nmea_split(char *payload, char *fields[], int max_fields)
{
    int count = 0;
    char *p = payload;
    fields[count++] = p;
    while (*p && count < max_fields) {
        if (*p == ',') { *p = '\0'; fields[count++] = p + 1; }
        p++;
    }
    return count;
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

    printf("\n== NMEA checksum ==\n");
    const char *good_gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    const char *good_rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    const char *bad      = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48";
    CHECK(nmea_verify_checksum(good_gga, strlen(good_gga)), "valid GGA accepted");
    CHECK(nmea_verify_checksum(good_rmc, strlen(good_rmc)), "valid RMC accepted");
    CHECK(!nmea_verify_checksum(bad, strlen(bad)), "corrupted checksum rejected");
    CHECK(!nmea_verify_checksum("$GPGGA,1", 8), "sentence with no '*' rejected");
    CHECK(!nmea_verify_checksum("GPGGA,1*47", 10), "sentence with no '$' rejected");
    CHECK(!nmea_verify_checksum("$GPGGA,1*4", 10), "truncated checksum rejected");
    CHECK(!nmea_verify_checksum("$GPGGA,1*ZZ", 11), "non-hex checksum rejected");

    printf("\n== NMEA coordinate parsing ==\n");
    double v;
    CHECK(nmea_parse_coord("4807.038", "N", &v) && fabs(v - 48.1173) < 1e-4,
          "4807.038 N -> %.6f (want 48.117300)", v);
    CHECK(nmea_parse_coord("01131.000", "E", &v) && fabs(v - 11.516667) < 1e-5,
          "01131.000 E -> %.6f (want 11.516667)", v);
    CHECK(nmea_parse_coord("4916.45", "S", &v) && fabs(v + 49.274167) < 1e-5,
          "4916.45 S -> %.6f (want -49.274167)", v);
    CHECK(nmea_parse_coord("12311.12", "W", &v) && fabs(v + 123.185333) < 1e-5,
          "12311.12 W -> %.6f (want -123.185333)", v);
    /* Brazilian coordinates - the actual deployment region */
    CHECK(nmea_parse_coord("2233.7620", "S", &v) && fabs(v + 22.562700) < 1e-5,
          "2233.7620 S -> %.6f (want -22.562700)", v);
    CHECK(!nmea_parse_coord("", "N", &v), "empty coordinate rejected");
    CHECK(!nmea_parse_coord("4807.038", "", &v), "empty hemisphere rejected");
    CHECK(!nmea_parse_coord("4807.038", "X", &v), "bogus hemisphere rejected");
    CHECK(!nmea_parse_coord("4870.000", "N", &v), "minutes >= 60 rejected");

    printf("\n== NMEA field splitting (empty fields must survive) ==\n");
    char payload[] = "GPGGA,123519,,,,,0,00,,,M,,M,,";
    char *fields[24];
    int n = nmea_split(payload, fields, 24);
    CHECK(n == 15, "field count = %d (want 15)", n);
    CHECK(strcmp(fields[0], "GPGGA") == 0, "field 0 = \"%s\"", fields[0]);
    CHECK(fields[2][0] == '\0', "empty latitude field is empty, not skipped");
    CHECK(strcmp(fields[6], "0") == 0, "fix quality field = \"%s\" (want \"0\")", fields[6]);

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED",
           failures, failures == 1 ? "" : "s");
    return failures != 0;
}
