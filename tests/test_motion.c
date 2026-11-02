#include "motion.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

struct fake_motion {
    uint32_t pulses[256];
    uint8_t channels[256];
    size_t pulse_count;
    size_t initialize_count;
    size_t off_count;
    uint32_t slept_ms;
    uint32_t stop_after_ms;
    bool stop_immediately;
};

static struct fake_motion *active_fake;

static int check(int condition, const char *message)
{
    if (!condition) {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        return -1;
    }
    return 0;
}

int pca9685_initialize(struct pca9685 *device, uint16_t frequency_hz)
{
    (void)device;
    if (frequency_hz != 50U) {
        errno = EINVAL;
        return -1;
    }
    ++active_fake->initialize_count;
    return 0;
}

int pca9685_set_pulse_us(struct pca9685 *device, uint8_t channel,
                         uint32_t pulse_us)
{
    (void)device;
    active_fake->channels[active_fake->pulse_count] = channel;
    active_fake->pulses[active_fake->pulse_count++] = pulse_us;
    return 0;
}

int pca9685_all_off(struct pca9685 *device)
{
    (void)device;
    ++active_fake->off_count;
    return 0;
}

static int fake_sleep(void *context, uint32_t milliseconds)
{
    struct fake_motion *fake = context;

    fake->slept_ms += milliseconds;
    return 0;
}

static bool fake_stop(void *context)
{
    struct fake_motion *fake = context;

    return fake->stop_immediately ||
           (fake->stop_after_ms != 0U &&
            fake->slept_ms >= fake->stop_after_ms);
}

static struct gar_motion_runtime runtime_for(struct fake_motion *fake)
{
    struct gar_motion_runtime runtime;

    runtime.context = fake;
    runtime.sleep_ms = fake_sleep;
    runtime.stop_requested = fake_stop;
    return runtime;
}

static struct gar_servo_calibration servo(int direction)
{
    struct gar_servo_calibration value;

    (void)memset(&value, 0, sizeof(value));
    (void)strcpy(value.name, "front-left");
    value.channel = 2U;
    value.direction = direction;
    value.min_us = 1000U;
    value.neutral_us = 1500U;
    value.max_us = 2000U;
    value.max_rate_us_per_s = 500U;
    value.calibrated = true;
    return value;
}

static int test_mapping(void)
{
    struct gar_servo_calibration normal = servo(1);
    struct gar_servo_calibration reversed = servo(-1);
    uint32_t pulse;

    if (check(gar_position_to_pulse(&normal, 1000, &pulse) == 0 &&
                  pulse == 2000U,
              "normal +1000 maps to max") != 0 ||
        check(gar_position_to_pulse(&normal, -1000, &pulse) == 0 &&
                  pulse == 1000U,
              "normal -1000 maps to min") != 0 ||
        check(gar_position_to_pulse(&reversed, 1000, &pulse) == 0 &&
                  pulse == 1000U,
              "reversed +1000 maps to min") != 0 ||
        check(gar_position_to_pulse(&reversed, -1000, &pulse) == 0 &&
                  pulse == 2000U,
              "reversed -1000 maps to max") != 0 ||
        check(gar_minimum_duration_ms(&normal, 1500U, 2000U) == 1000U,
              "rate limit duration") != 0) {
        return -1;
    }
    return 0;
}

static int test_move_and_off(void)
{
    struct fake_motion fake;
    struct gar_motion_runtime runtime;
    struct gar_servo_calibration calibration = servo(1);
    struct pca9685 controller;
    char error[256];

    (void)memset(&fake, 0, sizeof(fake));
    (void)memset(&controller, 0, sizeof(controller));
    active_fake = &fake;
    runtime = runtime_for(&fake);
    if (check(gar_motion_move(&controller, &calibration, 0, 1000, 1000U,
                              40U, &runtime, error, sizeof(error)) == 0,
              "move should succeed") != 0 ||
        check(fake.initialize_count == 1U, "move initializes once") != 0 ||
        check(fake.pulses[0] == 1500U,
              "move starts at calibrated neutral") != 0 ||
        check(fake.pulses[fake.pulse_count - 1U] == 2000U,
              "move reaches target") != 0 ||
        check(fake.slept_ms == 1040U, "duration plus hold is finite") != 0 ||
        check(fake.off_count == 1U, "move finishes FULL_OFF") != 0) {
        return -1;
    }
    return 0;
}

static int test_guardrails(void)
{
    struct fake_motion fake;
    struct gar_motion_runtime runtime;
    struct gar_servo_calibration calibration = servo(1);
    struct pca9685 controller;
    char error[256];

    (void)memset(&fake, 0, sizeof(fake));
    (void)memset(&controller, 0, sizeof(controller));
    active_fake = &fake;
    runtime = runtime_for(&fake);
    if (check(gar_motion_move(&controller, &calibration, 0, 1000, 999U, 0U,
                              &runtime, error, sizeof(error)) != 0,
              "too-fast move must fail") != 0 ||
        check(fake.initialize_count == 0U && fake.pulse_count == 0U,
              "rate rejection happens before I2C") != 0) {
        return -1;
    }

    calibration.calibrated = false;
    if (check(gar_motion_neutral(&controller, &calibration, 1U, 100U,
                                 &runtime, error, sizeof(error)) != 0,
              "uncalibrated neutral must fail") != 0 ||
        check(fake.initialize_count == 0U,
              "uncalibrated rejection happens before I2C") != 0) {
        return -1;
    }
    return 0;
}

static int test_interrupt_and_stagger(void)
{
    struct fake_motion fake;
    struct gar_motion_runtime runtime;
    struct gar_servo_calibration calibrations[2] = {servo(1), servo(-1)};
    struct pca9685 controller;
    char error[256];

    (void)strcpy(calibrations[1].name, "front-right");
    calibrations[1].channel = 3U;
    (void)memset(&fake, 0, sizeof(fake));
    (void)memset(&controller, 0, sizeof(controller));
    active_fake = &fake;
    runtime = runtime_for(&fake);
    if (check(gar_motion_neutral(&controller, calibrations, 2U, 100U,
                                 &runtime, error, sizeof(error)) == 0,
              "two-servo neutral should succeed") != 0 ||
        check(fake.pulse_count == 2U, "both neutral pulses sent") != 0 ||
        check(fake.slept_ms == 120U,
              "neutral includes 20 ms startup stagger") != 0 ||
        check(fake.off_count == 1U, "neutral finishes FULL_OFF") != 0) {
        return -1;
    }

    (void)memset(&fake, 0, sizeof(fake));
    active_fake = &fake;
    runtime = runtime_for(&fake);
    fake.stop_after_ms = 20U;
    if (check(gar_motion_move(&controller, &calibrations[0], 0, 1000, 1000U,
                              0U, &runtime, error, sizeof(error)) != 0,
              "interrupted move should fail") != 0 ||
        check(fake.off_count == 1U, "interrupt finishes FULL_OFF") != 0) {
        return -1;
    }
    return 0;
}

static int test_preexisting_stop_sends_no_i2c(void)
{
    struct fake_motion fake;
    struct gar_motion_runtime runtime;
    struct gar_servo_calibration calibration = servo(1);
    struct pca9685 controller;
    char error[256];

    (void)memset(&fake, 0, sizeof(fake));
    (void)memset(&controller, 0, sizeof(controller));
    fake.stop_immediately = true;
    active_fake = &fake;
    runtime = runtime_for(&fake);
    if (check(gar_motion_move(&controller, &calibration, 0, 250, 250U, 0U,
                              &runtime, error, sizeof(error)) != 0,
              "preexisting stop should reject move") != 0 ||
        check(fake.initialize_count == 0U && fake.pulse_count == 0U,
              "preexisting stop is checked before I2C") != 0) {
        return -1;
    }
    return 0;
}

static int test_calibration_deadman(void)
{
    struct fake_motion fake;
    struct gar_motion_runtime runtime;
    struct pca9685 controller;
    char error[256];

    (void)memset(&fake, 0, sizeof(fake));
    (void)memset(&controller, 0, sizeof(controller));
    active_fake = &fake;
    runtime = runtime_for(&fake);
    if (check(gar_motion_calibrate(&controller, 1U, 1500U, 100U, &runtime,
                                   error, sizeof(error)) == 0,
              "calibration pulse should succeed") != 0 ||
        check(fake.pulse_count == 1U && fake.pulses[0] == 1500U,
              "calibration pulse value") != 0 ||
        check(fake.slept_ms == 100U && fake.off_count == 1U,
              "calibration deadman turns output off") != 0 ||
        check(gar_motion_calibrate(&controller, 1U, 1500U,
                                   GAR_MAX_CALIBRATION_HOLD_MS + 1U, &runtime,
                                   error, sizeof(error)) != 0,
              "calibration hold maximum is enforced") != 0 ||
        check(gar_motion_calibrate(&controller, 1U,
                                   GAR_SERVO_MAX_PULSE_US + 1U, 100U,
                                   &runtime, error, sizeof(error)) != 0,
              "calibration pulse envelope is enforced") != 0) {
        return -1;
    }
    return 0;
}

static int test_motion_sequence(void)
{
    struct fake_motion fake;
    struct gar_motion_runtime runtime;
    struct gar_servo_calibration calibrations[2] = {servo(1), servo(-1)};
    struct pca9685 controller;
    const int32_t positions[] = {
        0, 0,
        250, -250,
        0, 0,
    };
    const int32_t too_fast[] = {
        0, 0,
        1000, -1000,
    };
    char error[256];

    (void)strcpy(calibrations[1].name, "front-right");
    calibrations[1].channel = 3U;
    (void)memset(&fake, 0, sizeof(fake));
    (void)memset(&controller, 0, sizeof(controller));
    active_fake = &fake;
    runtime = runtime_for(&fake);
    if (check(gar_motion_sequence(&controller, calibrations, 2U, positions,
                                  3U, 500U, 20U, &runtime, error,
                                  sizeof(error)) == 0,
              "three-frame sequence should succeed") != 0 ||
        check(fake.initialize_count == 1U,
              "sequence initializes once") != 0 ||
        check(fake.pulse_count == 102U,
              "sequence writes every 20 ms frame") != 0 ||
        check(fake.channels[0] == 2U && fake.channels[1] == 3U &&
                  fake.pulses[0] == 1500U && fake.pulses[1] == 1500U,
              "sequence starts at acknowledged neutral") != 0 ||
        check(fake.pulses[50] == 1625U && fake.pulses[51] == 1625U,
              "mirrored directions reach the same logical pose") != 0 ||
        check(fake.pulses[100] == 1500U && fake.pulses[101] == 1500U,
              "sequence returns to neutral") != 0 ||
        check(fake.slept_ms == 1060U,
              "sequence duration, holds, and stagger are finite") != 0 ||
        check(fake.off_count == 1U,
              "sequence finishes FULL_OFF") != 0) {
        return -1;
    }

    (void)memset(&fake, 0, sizeof(fake));
    active_fake = &fake;
    runtime = runtime_for(&fake);
    if (check(gar_motion_sequence(&controller, calibrations, 2U, too_fast,
                                  2U, 999U, 0U, &runtime, error,
                                  sizeof(error)) != 0,
              "sequence rate limit must reject a fast transition") != 0 ||
        check(fake.initialize_count == 0U && fake.pulse_count == 0U,
              "sequence rate rejection happens before I2C") != 0) {
        return -1;
    }
    (void)memset(&fake, 0, sizeof(fake));
    active_fake = &fake;
    runtime = runtime_for(&fake);
    fake.stop_after_ms = 80U;
    if (check(gar_motion_sequence(&controller, calibrations, 2U, positions,
                                  3U, 500U, 20U, &runtime, error,
                                  sizeof(error)) != 0,
              "stop interrupts an active multi-servo sequence") != 0 ||
        check(fake.slept_ms == 80U && fake.off_count == 1U,
              "interrupted sequence disables outputs without finishing gait") != 0) {
        return -1;
    }

    (void)memset(&fake, 0, sizeof(fake));
    active_fake = &fake;
    runtime = runtime_for(&fake);
    calibrations[1].calibrated = false;
    if (check(gar_motion_sequence(&controller, calibrations, 2U, positions,
                                  3U, 500U, 20U, &runtime, error,
                                  sizeof(error)) != 0,
              "uncalibrated leg rejects the entire sequence") != 0 ||
        check(fake.initialize_count == 0U && fake.pulse_count == 0U,
              "calibration rejection happens before writes") != 0) {
        return -1;
    }
    return 0;
}

int main(void)
{
    if (test_mapping() != 0 || test_move_and_off() != 0 ||
        test_guardrails() != 0 || test_interrupt_and_stagger() != 0 ||
        test_calibration_deadman() != 0 ||
        test_preexisting_stop_sends_no_i2c() != 0 ||
        test_motion_sequence() != 0) {
        return 1;
    }
    (void)printf("test_motion: OK\n");
    return 0;
}
