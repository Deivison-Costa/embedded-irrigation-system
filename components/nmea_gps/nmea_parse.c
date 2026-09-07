#include <stdlib.h>
#include <string.h>

#include "nmea_parse.h"

#define NMEA_MAX_FIELDS 24

/* --- parsing helpers ---------------------------------------------------- */

bool nmea_verify_checksum(const char *sentence, size_t len)
{
    /* Format: $<payload>*<HH> - the checksum covers the payload only. */
    if (len < 4 || sentence[0] != '$') {
        return false;
    }
    const char *star = memchr(sentence, '*', len);
    if (star == NULL || (size_t)(star - sentence) + 3 > len) {
        return false;
    }

    uint8_t computed = 0;
    for (const char *p = sentence + 1; p < star; p++) {
        computed ^= (uint8_t)*p;
    }

    char hex[3] = {star[1], star[2], '\0'};
    char *end = NULL;
    unsigned long received = strtoul(hex, &end, 16);
    if (end != hex + 2) {
        return false;
    }
    return computed == (uint8_t)received;
}

/* Splits payload in place at commas. Returns the field count. */
static int nmea_split(char *payload, char *fields[], int max_fields)
{
    int count = 0;
    char *p = payload;
    fields[count++] = p;
    while (*p && count < max_fields) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
        p++;
    }
    return count;
}

/* "ddmm.mmmm" + hemisphere -> signed decimal degrees. */
static bool nmea_parse_coord(const char *value, const char *hemi, double *out)
{
    if (value == NULL || *value == '\0' || hemi == NULL || *hemi == '\0') {
        return false;
    }
    char *end = NULL;
    double raw = strtod(value, &end);
    if (end == value) {
        return false;
    }

    double degrees = (double)((int)(raw / 100.0));
    double minutes = raw - degrees * 100.0;
    if (minutes < 0.0 || minutes >= 60.0) {
        return false;
    }
    double result = degrees + minutes / 60.0;

    if (*hemi == 'S' || *hemi == 'W') {
        result = -result;
    } else if (*hemi != 'N' && *hemi != 'E') {
        return false;
    }
    *out = result;
    return true;
}

static bool nmea_parse_double(const char *value, double *out)
{
    if (value == NULL || *value == '\0') {
        return false;
    }
    char *end = NULL;
    double v = strtod(value, &end);
    if (end == value) {
        return false;
    }
    *out = v;
    return true;
}

/* "hhmmss.sss" */
static void nmea_parse_time(const char *value, nmea_gps_fix_t *fix)
{
    if (value == NULL || strlen(value) < 6) {
        return;
    }
    fix->hour   = (uint8_t)((value[0] - '0') * 10 + (value[1] - '0'));
    fix->minute = (uint8_t)((value[2] - '0') * 10 + (value[3] - '0'));
    fix->second = (uint8_t)((value[4] - '0') * 10 + (value[5] - '0'));
}

/* "ddmmyy" */
static void nmea_parse_date(const char *value, nmea_gps_fix_t *fix)
{
    if (value == NULL || strlen(value) < 6) {
        return;
    }
    fix->day   = (uint8_t)((value[0] - '0') * 10 + (value[1] - '0'));
    fix->month = (uint8_t)((value[2] - '0') * 10 + (value[3] - '0'));
    uint16_t yy = (uint16_t)((value[4] - '0') * 10 + (value[5] - '0'));
    fix->year  = (uint16_t)(2000 + yy); /* NMEA two-digit year, valid to 2099 */
}

/* --- sentence handlers -------------------------------------------------- */

static void nmea_handle_gga(char *fields[], int n, nmea_gps_fix_t *fix, int64_t now_ms)
{
    if (n < 10) {
        return;
    }
    nmea_parse_time(fields[1], fix);

    uint8_t quality = (fields[6] && *fields[6]) ? (uint8_t)atoi(fields[6]) : 0;
    fix->fix_quality = quality;

    if (quality == 0) {
        fix->valid = false;
        return;
    }

    double lat, lon, alt, hdop;
    bool have_pos = nmea_parse_coord(fields[2], fields[3], &lat) &&
                    nmea_parse_coord(fields[4], fields[5], &lon);
    if (!have_pos) {
        fix->valid = false;
        return;
    }

    fix->latitude_deg  = lat;
    fix->longitude_deg = lon;
    fix->satellites    = (fields[7] && *fields[7]) ? (uint8_t)atoi(fields[7]) : 0;
    if (nmea_parse_double(fields[8], &hdop)) {
        fix->hdop = (float)hdop;
    }
    if (nmea_parse_double(fields[9], &alt)) {
        fix->altitude_m = alt;
    }
    fix->valid        = true;
    fix->timestamp_ms = now_ms;
}

static void nmea_handle_rmc(char *fields[], int n, nmea_gps_fix_t *fix, int64_t now_ms)
{
    if (n < 10) {
        return;
    }
    nmea_parse_time(fields[1], fix);
    nmea_parse_date(fields[9], fix);

    /* Field 2: 'A' = active/valid, 'V' = void. */
    if (fields[2] == NULL || *fields[2] != 'A') {
        fix->valid = false;
        return;
    }

    double lat, lon, speed_knots, course;
    if (!nmea_parse_coord(fields[3], fields[4], &lat) ||
        !nmea_parse_coord(fields[5], fields[6], &lon)) {
        fix->valid = false;
        return;
    }

    fix->latitude_deg  = lat;
    fix->longitude_deg = lon;
    if (nmea_parse_double(fields[7], &speed_knots)) {
        fix->speed_kmh = speed_knots * 1.852;
    }
    if (nmea_parse_double(fields[8], &course)) {
        fix->course_deg = course;
    }
    fix->valid        = true;
    fix->timestamp_ms = now_ms;
}


bool nmea_parse_sentence(char *sentence, nmea_gps_fix_t *fix, int64_t now_ms)
{
    if (sentence == NULL || fix == NULL) {
        return false;
    }

    /* Remove o "$" inicial e tudo a partir do "*". */
    char *payload = sentence + 1;
    char *star = strchr(payload, '*');
    if (star) {
        *star = '\0';
    }

    char *fields[NMEA_MAX_FIELDS];
    int n = nmea_split(payload, fields, NMEA_MAX_FIELDS);
    if (n < 1 || strlen(fields[0]) < 5) {
        return false;
    }

    /* O talker varia (GP, GN, GL, GA...); casar pelos 3 caracteres do tipo. */
    const char *type = fields[0] + 2;

    if (strncmp(type, "GGA", 3) == 0) {
        nmea_handle_gga(fields, n, fix, now_ms);
        return true;
    }
    if (strncmp(type, "RMC", 3) == 0) {
        nmea_handle_rmc(fields, n, fix, now_ms);
        return true;
    }
    return false;
}
