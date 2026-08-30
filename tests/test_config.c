#define _POSIX_C_SOURCE 200809L

#include "config.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int check(int condition, const char *message)
{
    if (!condition) {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        return -1;
    }
    return 0;
}

static int temporary_csv(const char *contents, char *path, size_t path_size)
{
    int descriptor;
    FILE *file;

    if (path_size < 32U) {
        return -1;
    }
    (void)snprintf(path, path_size, "/tmp/gar-servo-test-XXXXXX");
    descriptor = mkstemp(path);
    if (descriptor < 0) {
        return -1;
    }
    file = fdopen(descriptor, "w");
    if (file == NULL) {
        (void)close(descriptor);
        (void)unlink(path);
        return -1;
    }
    if (fputs(contents, file) == EOF || fclose(file) != 0) {
        (void)unlink(path);
        return -1;
    }
    return 0;
}

static int test_valid_files(void)
{
    static const char i2c_csv[] =
        "name,bus,dev,address,driver,sim,description\n"
        "other,0,/dev/i2c-0,0x20,other,none,ignored\n"
        "pwm,3,/dev/i2c-3,0x40,pca9685,pca9685,servo pwm\n";
    static const char servo_csv[] =
        "name,channel,role,direction,min_us,neutral_us,max_us,"
        "max_rate_us_per_s,calibrated\n"
        "front-left,0,left leg,1,1000,1500,2000,500,1\n"
        "front-right,1,right leg,-1,900,1450,2100,600,1\n"
        "rear-left,2,,,,,,,0\n";
    char i2c_path[64];
    char servo_path[64];
    char error[512];
    struct gar_i2c_config i2c;
    struct gar_servo_config servos;
    const struct gar_servo_calibration *front_right;
    int result = -1;

    if (temporary_csv(i2c_csv, i2c_path, sizeof(i2c_path)) != 0 ||
        temporary_csv(servo_csv, servo_path, sizeof(servo_path)) != 0) {
        return -1;
    }
    if (check(gar_load_i2c_config(i2c_path, &i2c, error, sizeof(error)) == 0,
              "valid i2c.csv") != 0 ||
        check(strcmp(i2c.device, "/dev/i2c-3") == 0 &&
                  i2c.address == 0x40U,
              "PCA9685 row selected") != 0 ||
        check(gar_load_servo_config(servo_path, &servos, error,
                                    sizeof(error)) == 0,
              "valid servo-calibration.csv") != 0 ||
        check(servos.count == 3U && !servos.servos[2].calibrated,
              "uncalibrated blank row accepted") != 0) {
        goto done;
    }
    front_right = gar_find_servo(&servos, "front-right");
    if (check(front_right != NULL && front_right->direction == -1 &&
                  front_right->neutral_us == 1450U,
              "calibration fields parsed") != 0) {
        goto done;
    }
    result = 0;

done:
    (void)unlink(i2c_path);
    (void)unlink(servo_path);
    return result;
}

static int test_invalid_calibration(void)
{
    static const char invalid_csv[] =
        "name,channel,role,direction,min_us,neutral_us,max_us,"
        "max_rate_us_per_s,calibrated\n"
        "servo-a,0,test,1,1500,1400,2000,500,1\n";
    static const char duplicate_csv[] =
        "name,channel,role,direction,min_us,neutral_us,max_us,"
        "max_rate_us_per_s,calibrated\n"
        "servo-a,0,test,1,1000,1500,2000,500,1\n"
        "servo-b,0,test,-1,1000,1500,2000,500,1\n";
    static const char unsafe_pulse_csv[] =
        "name,channel,role,direction,min_us,neutral_us,max_us,"
        "max_rate_us_per_s,calibrated\n"
        "servo-a,0,test,1,499,1500,2000,500,1\n";
    char path[64];
    char error[512];
    struct gar_servo_config servos;

    if (temporary_csv(invalid_csv, path, sizeof(path)) != 0) {
        return -1;
    }
    if (check(gar_load_servo_config(path, &servos, error, sizeof(error)) != 0,
              "invalid pulse ordering rejected") != 0) {
        (void)unlink(path);
        return -1;
    }
    (void)unlink(path);

    if (temporary_csv(duplicate_csv, path, sizeof(path)) != 0) {
        return -1;
    }
    if (check(gar_load_servo_config(path, &servos, error, sizeof(error)) != 0,
              "duplicate channel rejected") != 0) {
        (void)unlink(path);
        return -1;
    }
    (void)unlink(path);

    if (temporary_csv(unsafe_pulse_csv, path, sizeof(path)) != 0) {
        return -1;
    }
    if (check(gar_load_servo_config(path, &servos, error, sizeof(error)) != 0,
              "pulse outside the software envelope rejected") != 0) {
        (void)unlink(path);
        return -1;
    }
    (void)unlink(path);
    return 0;
}

int main(void)
{
    if (test_valid_files() != 0 || test_invalid_calibration() != 0) {
        return 1;
    }
    (void)printf("test_config: OK\n");
    return 0;
}
