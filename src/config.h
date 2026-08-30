#ifndef GAR_SERVO_PET_CONFIG_H
#define GAR_SERVO_PET_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GAR_CONFIG_PATH_MAX 512
#define GAR_CONFIG_NAME_MAX 64
#define GAR_CONFIG_ROLE_MAX 96
#define GAR_MAX_SERVOS 16
#define GAR_SERVO_MIN_PULSE_US 500U
#define GAR_SERVO_MAX_PULSE_US 2500U
#define GAR_SERVO_MAX_RATE_US_PER_S 5000U

struct gar_i2c_config {
    char name[GAR_CONFIG_NAME_MAX];
    char device[GAR_CONFIG_PATH_MAX];
    uint8_t address;
};

struct gar_servo_calibration {
    char name[GAR_CONFIG_NAME_MAX];
    char role[GAR_CONFIG_ROLE_MAX];
    uint8_t channel;
    int direction;
    uint32_t min_us;
    uint32_t neutral_us;
    uint32_t max_us;
    uint32_t max_rate_us_per_s;
    bool calibrated;
};

struct gar_servo_config {
    struct gar_servo_calibration servos[GAR_MAX_SERVOS];
    size_t count;
};

int gar_load_i2c_config(const char *path, struct gar_i2c_config *config,
                        char *error, size_t error_size);
int gar_load_servo_config(const char *path, struct gar_servo_config *config,
                          char *error, size_t error_size);

const struct gar_servo_calibration *
gar_find_servo(const struct gar_servo_config *config, const char *name);

#endif
