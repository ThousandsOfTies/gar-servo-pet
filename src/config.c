#define _POSIX_C_SOURCE 200809L

#include "config.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CSV_MAX_FIELDS 16
#define CSV_FIELD_MAX 512

struct csv_row {
    char value[CSV_MAX_FIELDS][CSV_FIELD_MAX];
    size_t count;
};

static void set_error(char *error, size_t error_size, const char *format, ...)
{
    va_list arguments;

    if (error == NULL || error_size == 0U) {
        return;
    }
    va_start(arguments, format);
    (void)vsnprintf(error, error_size, format, arguments);
    va_end(arguments);
}

static void trim(char *text)
{
    char *start = text;
    char *end;

    while (*start == ' ' || *start == '\t') {
        ++start;
    }
    if (start != text) {
        (void)memmove(text, start, strlen(start) + 1U);
    }

    end = text + strlen(text);
    while (end > text && (end[-1] == ' ' || end[-1] == '\t' ||
                           end[-1] == '\r' || end[-1] == '\n')) {
        --end;
    }
    *end = '\0';
}

static int parse_csv_row(const char *line, struct csv_row *row, char *error,
                         size_t error_size)
{
    const char *cursor = line;

    row->count = 0U;
    for (;;) {
        char *output;
        size_t length = 0U;
        int quoted = 0;

        if (row->count == CSV_MAX_FIELDS) {
            set_error(error, error_size, "too many CSV fields (maximum %u)",
                      CSV_MAX_FIELDS);
            return -1;
        }
        output = row->value[row->count];
        if (*cursor == '"') {
            quoted = 1;
            ++cursor;
        }

        while (*cursor != '\0') {
            char character = *cursor;

            if (quoted) {
                if (character == '"') {
                    if (cursor[1] == '"') {
                        character = '"';
                        cursor += 2;
                    } else {
                        ++cursor;
                        quoted = 0;
                        while (*cursor == ' ' || *cursor == '\t') {
                            ++cursor;
                        }
                        if (*cursor != ',' && *cursor != '\0' &&
                            *cursor != '\r' && *cursor != '\n') {
                            set_error(error, error_size,
                                      "unexpected text after quoted CSV field");
                            return -1;
                        }
                        break;
                    }
                } else {
                    ++cursor;
                }
            } else {
                if (character == ',' || character == '\r' ||
                    character == '\n') {
                    break;
                }
                ++cursor;
            }

            if (length + 1U >= CSV_FIELD_MAX) {
                set_error(error, error_size,
                          "CSV field is longer than %u bytes",
                          CSV_FIELD_MAX - 1U);
                return -1;
            }
            output[length++] = character;
        }
        if (quoted) {
            set_error(error, error_size, "unterminated quoted CSV field");
            return -1;
        }
        output[length] = '\0';
        trim(output);
        ++row->count;

        while (*cursor == '\r' || *cursor == '\n') {
            ++cursor;
        }
        if (*cursor == '\0') {
            return 0;
        }
        if (*cursor != ',') {
            set_error(error, error_size, "malformed CSV separator");
            return -1;
        }
        ++cursor;
        if (*cursor == '\0') {
            if (row->count == CSV_MAX_FIELDS) {
                set_error(error, error_size,
                          "too many CSV fields (maximum %u)", CSV_MAX_FIELDS);
                return -1;
            }
            row->value[row->count][0] = '\0';
            ++row->count;
            return 0;
        }
    }
}

static int is_ignored_line(const char *line)
{
    while (*line == ' ' || *line == '\t' || *line == '\r' ||
           *line == '\n') {
        ++line;
    }
    return *line == '\0' || *line == '#';
}

static int header_index(const struct csv_row *header, const char *name)
{
    size_t index;

    for (index = 0U; index < header->count; ++index) {
        if (strcmp(header->value[index], name) == 0) {
            return (int)index;
        }
    }
    return -1;
}

static int require_header(const struct csv_row *header, const char *name,
                          const char *path, char *error, size_t error_size)
{
    int index = header_index(header, name);

    if (index < 0) {
        set_error(error, error_size, "%s: missing required column '%s'", path,
                  name);
    }
    return index;
}

static int copy_field(char *destination, size_t destination_size,
                      const char *source, const char *field_name,
                      const char *path, size_t line_number, char *error,
                      size_t error_size)
{
    size_t length = strlen(source);

    if (length == 0U) {
        set_error(error, error_size, "%s:%zu: '%s' must not be empty", path,
                  line_number, field_name);
        return -1;
    }
    if (length >= destination_size) {
        set_error(error, error_size, "%s:%zu: '%s' is too long", path,
                  line_number, field_name);
        return -1;
    }
    (void)memcpy(destination, source, length + 1U);
    return 0;
}

static int parse_u32(const char *text, uint32_t minimum, uint32_t maximum,
                     uint32_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    if (text == NULL || *text == '\0' || *text == '-') {
        return -1;
    }
    errno = 0;
    parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed < minimum ||
        parsed > maximum) {
        return -1;
    }
    *value = (uint32_t)parsed;
    return 0;
}

static int read_header(FILE *file, const char *path, struct csv_row *header,
                       char **line, size_t *capacity, size_t *line_number,
                       char *error, size_t error_size)
{
    ssize_t length;

    while ((length = getline(line, capacity, file)) >= 0) {
        (void)length;
        ++*line_number;
        if (is_ignored_line(*line)) {
            continue;
        }
        if (parse_csv_row(*line, header, error, error_size) != 0) {
            char detail[256];
            (void)snprintf(detail, sizeof(detail), "%s", error);
            set_error(error, error_size, "%s:%zu: %s", path, *line_number,
                      detail);
            return -1;
        }
        if (header->count > 0U && strlen(header->value[0]) >= 3U &&
            (unsigned char)header->value[0][0] == 0xefU &&
            (unsigned char)header->value[0][1] == 0xbbU &&
            (unsigned char)header->value[0][2] == 0xbfU) {
            (void)memmove(header->value[0], header->value[0] + 3,
                          strlen(header->value[0] + 3) + 1U);
        }
        return 0;
    }
    if (ferror(file) != 0) {
        set_error(error, error_size, "%s: read failed: %s", path,
                  strerror(errno));
    } else {
        set_error(error, error_size, "%s: CSV file has no header", path);
    }
    return -1;
}

int gar_load_i2c_config(const char *path, struct gar_i2c_config *config,
                        char *error, size_t error_size)
{
    FILE *file = NULL;
    char *line = NULL;
    size_t capacity = 0U;
    size_t line_number = 0U;
    struct csv_row header;
    struct csv_row row;
    int name_column;
    int device_column;
    int address_column;
    int driver_column;
    int found = 0;
    int result = -1;

    if (path == NULL || config == NULL) {
        set_error(error, error_size, "invalid I2C configuration arguments");
        return -1;
    }
    (void)memset(config, 0, sizeof(*config));
    file = fopen(path, "r");
    if (file == NULL) {
        set_error(error, error_size, "%s: cannot open: %s", path,
                  strerror(errno));
        return -1;
    }
    if (read_header(file, path, &header, &line, &capacity, &line_number,
                    error, error_size) != 0) {
        goto done;
    }
    name_column = require_header(&header, "name", path, error, error_size);
    device_column = require_header(&header, "dev", path, error, error_size);
    address_column =
        require_header(&header, "address", path, error, error_size);
    driver_column = require_header(&header, "driver", path, error, error_size);
    if (name_column < 0 || device_column < 0 || address_column < 0 ||
        driver_column < 0) {
        goto done;
    }

    while (getline(&line, &capacity, file) >= 0) {
        uint32_t address;

        ++line_number;
        if (is_ignored_line(line)) {
            continue;
        }
        if (parse_csv_row(line, &row, error, error_size) != 0) {
            char detail[256];
            (void)snprintf(detail, sizeof(detail), "%s", error);
            set_error(error, error_size, "%s:%zu: %s", path, line_number,
                      detail);
            goto done;
        }
        if (row.count != header.count) {
            set_error(error, error_size,
                      "%s:%zu: expected %zu columns, found %zu", path,
                      line_number, header.count, row.count);
            goto done;
        }
        if (strcmp(row.value[driver_column], "pca9685") != 0) {
            continue;
        }
        if (found != 0) {
            set_error(error, error_size,
                      "%s:%zu: more than one pca9685 row is ambiguous", path,
                      line_number);
            goto done;
        }
        if (parse_u32(row.value[address_column], 0x03U, 0x77U, &address) != 0) {
            set_error(error, error_size,
                      "%s:%zu: address must be a usable 7-bit I2C address",
                      path, line_number);
            goto done;
        }
        if (copy_field(config->name, sizeof(config->name),
                       row.value[name_column], "name", path, line_number,
                       error, error_size) != 0 ||
            copy_field(config->device, sizeof(config->device),
                       row.value[device_column], "dev", path, line_number,
                       error, error_size) != 0) {
            goto done;
        }
        config->address = (uint8_t)address;
        found = 1;
    }
    if (ferror(file) != 0) {
        set_error(error, error_size, "%s: read failed: %s", path,
                  strerror(errno));
        goto done;
    }
    if (found == 0) {
        set_error(error, error_size, "%s: no driver=pca9685 row found", path);
        goto done;
    }
    result = 0;

done:
    free(line);
    (void)fclose(file);
    return result;
}

static int duplicate_servo(const struct gar_servo_config *config,
                           const char *name, uint8_t channel)
{
    size_t index;

    for (index = 0U; index < config->count; ++index) {
        if (strcmp(config->servos[index].name, name) == 0 ||
            config->servos[index].channel == channel) {
            return 1;
        }
    }
    return 0;
}

int gar_load_servo_config(const char *path, struct gar_servo_config *config,
                          char *error, size_t error_size)
{
    static const char *const required[] = {
        "name",       "channel",    "role",
        "direction",  "min_us",     "neutral_us",
        "max_us",     "max_rate_us_per_s", "calibrated",
    };
    FILE *file = NULL;
    char *line = NULL;
    size_t capacity = 0U;
    size_t line_number = 0U;
    struct csv_row header;
    struct csv_row row;
    int column[sizeof(required) / sizeof(required[0])];
    size_t index;
    int result = -1;

    if (path == NULL || config == NULL) {
        set_error(error, error_size, "invalid servo configuration arguments");
        return -1;
    }
    (void)memset(config, 0, sizeof(*config));
    file = fopen(path, "r");
    if (file == NULL) {
        set_error(error, error_size, "%s: cannot open: %s", path,
                  strerror(errno));
        return -1;
    }
    if (read_header(file, path, &header, &line, &capacity, &line_number,
                    error, error_size) != 0) {
        goto done;
    }
    for (index = 0U; index < sizeof(required) / sizeof(required[0]); ++index) {
        column[index] =
            require_header(&header, required[index], path, error, error_size);
        if (column[index] < 0) {
            goto done;
        }
    }

    while (getline(&line, &capacity, file) >= 0) {
        struct gar_servo_calibration *servo;
        uint32_t channel;
        uint32_t calibrated;
        uint32_t direction_value = 0U;

        ++line_number;
        if (is_ignored_line(line)) {
            continue;
        }
        if (parse_csv_row(line, &row, error, error_size) != 0) {
            char detail[256];
            (void)snprintf(detail, sizeof(detail), "%s", error);
            set_error(error, error_size, "%s:%zu: %s", path, line_number,
                      detail);
            goto done;
        }
        if (row.count != header.count) {
            set_error(error, error_size,
                      "%s:%zu: expected %zu columns, found %zu", path,
                      line_number, header.count, row.count);
            goto done;
        }
        if (config->count == GAR_MAX_SERVOS) {
            set_error(error, error_size,
                      "%s:%zu: more than %u servo rows are not supported",
                      path, line_number, GAR_MAX_SERVOS);
            goto done;
        }
        if (parse_u32(row.value[column[1]], 0U, 15U, &channel) != 0) {
            set_error(error, error_size,
                      "%s:%zu: channel must be an integer from 0 to 15", path,
                      line_number);
            goto done;
        }
        if (parse_u32(row.value[column[8]], 0U, 1U, &calibrated) != 0) {
            set_error(error, error_size,
                      "%s:%zu: calibrated must be 0 or 1", path, line_number);
            goto done;
        }
        if (row.value[column[0]][0] == '\0') {
            set_error(error, error_size, "%s:%zu: name must not be empty",
                      path, line_number);
            goto done;
        }
        if (duplicate_servo(config, row.value[column[0]], (uint8_t)channel)) {
            set_error(error, error_size,
                      "%s:%zu: servo name and channel must both be unique",
                      path, line_number);
            goto done;
        }

        servo = &config->servos[config->count];
        if (copy_field(servo->name, sizeof(servo->name), row.value[column[0]],
                       "name", path, line_number, error, error_size) != 0) {
            goto done;
        }
        if (strlen(row.value[column[2]]) >= sizeof(servo->role)) {
            set_error(error, error_size, "%s:%zu: role is too long", path,
                      line_number);
            goto done;
        }
        (void)memcpy(servo->role, row.value[column[2]],
                     strlen(row.value[column[2]]) + 1U);
        servo->channel = (uint8_t)channel;
        servo->calibrated = calibrated != 0U;

        if (servo->calibrated) {
            if (parse_u32(row.value[column[3]], 1U, 1U,
                          &direction_value) == 0) {
                servo->direction = 1;
            } else if (strcmp(row.value[column[3]], "-1") == 0) {
                servo->direction = -1;
            } else {
                set_error(error, error_size,
                          "%s:%zu: direction must be -1 or 1", path,
                          line_number);
                goto done;
            }
            if (parse_u32(row.value[column[4]], GAR_SERVO_MIN_PULSE_US,
                          GAR_SERVO_MAX_PULSE_US,
                          &servo->min_us) != 0 ||
                parse_u32(row.value[column[5]], GAR_SERVO_MIN_PULSE_US,
                          GAR_SERVO_MAX_PULSE_US,
                          &servo->neutral_us) != 0 ||
                parse_u32(row.value[column[6]], GAR_SERVO_MIN_PULSE_US,
                          GAR_SERVO_MAX_PULSE_US,
                          &servo->max_us) != 0 ||
                parse_u32(row.value[column[7]], 1U,
                          GAR_SERVO_MAX_RATE_US_PER_S,
                          &servo->max_rate_us_per_s) != 0) {
                set_error(error, error_size,
                          "%s:%zu: calibrated pulses must be %u..%u us and max_rate_us_per_s must be 1..%u",
                          path, line_number, GAR_SERVO_MIN_PULSE_US,
                          GAR_SERVO_MAX_PULSE_US,
                          GAR_SERVO_MAX_RATE_US_PER_S);
                goto done;
            }
            if (!(servo->min_us < servo->neutral_us &&
                  servo->neutral_us < servo->max_us)) {
                set_error(error, error_size,
                          "%s:%zu: require min_us < neutral_us < max_us",
                          path, line_number);
                goto done;
            }
        }
        ++config->count;
    }
    if (ferror(file) != 0) {
        set_error(error, error_size, "%s: read failed: %s", path,
                  strerror(errno));
        goto done;
    }
    if (config->count == 0U) {
        set_error(error, error_size, "%s: no servo rows found", path);
        goto done;
    }
    result = 0;

done:
    free(line);
    (void)fclose(file);
    return result;
}

const struct gar_servo_calibration *
gar_find_servo(const struct gar_servo_config *config, const char *name)
{
    size_t index;

    if (config == NULL || name == NULL) {
        return NULL;
    }
    for (index = 0U; index < config->count; ++index) {
        if (strcmp(config->servos[index].name, name) == 0) {
            return &config->servos[index];
        }
    }
    return NULL;
}
