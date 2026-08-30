#include "motion.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define GAR_SERVO_PWM_HZ 50U
#define GAR_MOTION_STEP_MS 20U

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

static int stop_requested(const struct gar_motion_runtime *runtime)
{
    return runtime != NULL && runtime->stop_requested != NULL &&
           runtime->stop_requested(runtime->context);
}

static int sleep_interruptible(const struct gar_motion_runtime *runtime,
                               uint32_t milliseconds)
{
    uint32_t remaining = milliseconds;

    if (runtime == NULL || runtime->sleep_ms == NULL) {
        errno = EINVAL;
        return -1;
    }
    while (remaining > 0U) {
        uint32_t slice = remaining > GAR_MOTION_STEP_MS ? GAR_MOTION_STEP_MS
                                                        : remaining;

        if (stop_requested(runtime)) {
            errno = EINTR;
            return -1;
        }
        if (runtime->sleep_ms(runtime->context, slice) != 0) {
            return -1;
        }
        remaining -= slice;
    }
    if (stop_requested(runtime)) {
        errno = EINTR;
        return -1;
    }
    return 0;
}

static int finish_off(struct pca9685 *device, int operation_result,
                      const char *operation, char *error, size_t error_size)
{
    int operation_errno = errno;

    if (pca9685_all_off(device) != 0) {
        int off_errno = errno;

        if (operation_result == 0) {
            set_error(error, error_size,
                      "motion completed but fail-safe output disable failed: %s",
                      strerror(off_errno));
        } else {
            set_error(error, error_size,
                      "%s failed (%s), and fail-safe output disable also failed (%s)",
                      operation, strerror(operation_errno),
                      strerror(off_errno));
        }
        return -1;
    }
    if (operation_result != 0) {
        errno = operation_errno;
        if (operation_errno == EINTR) {
            set_error(error, error_size, "%s interrupted; outputs disabled",
                      operation);
        } else {
            set_error(error, error_size, "%s failed; outputs disabled: %s",
                      operation, strerror(operation_errno));
        }
        return -1;
    }
    return 0;
}

int gar_position_to_pulse(const struct gar_servo_calibration *servo,
                          int32_t position, uint32_t *pulse_us)
{
    int32_t physical_position;
    int64_t delta;
    int64_t pulse;

    if (servo == NULL || pulse_us == NULL || !servo->calibrated ||
        (servo->direction != 1 && servo->direction != -1) || position < -1000 ||
        position > 1000 || !(servo->min_us < servo->neutral_us &&
                             servo->neutral_us < servo->max_us)) {
        errno = EINVAL;
        return -1;
    }

    physical_position = position * servo->direction;
    if (physical_position >= 0) {
        delta = (int64_t)servo->max_us - servo->neutral_us;
    } else {
        delta = (int64_t)servo->neutral_us - servo->min_us;
    }
    pulse = (int64_t)servo->neutral_us +
            (delta * physical_position +
             (physical_position >= 0 ? 500 : -500)) /
                1000;
    if (pulse <= 0 || pulse > UINT32_MAX) {
        errno = ERANGE;
        return -1;
    }
    *pulse_us = (uint32_t)pulse;
    return 0;
}

uint32_t gar_minimum_duration_ms(const struct gar_servo_calibration *servo,
                                 uint32_t from_us, uint32_t to_us)
{
    uint64_t distance;
    uint64_t duration;

    if (servo == NULL || servo->max_rate_us_per_s == 0U) {
        return UINT32_MAX;
    }
    distance = from_us > to_us ? (uint64_t)from_us - to_us
                               : (uint64_t)to_us - from_us;
    duration =
        (distance * 1000U + servo->max_rate_us_per_s - 1U) /
        servo->max_rate_us_per_s;
    return duration > UINT32_MAX ? UINT32_MAX : (uint32_t)duration;
}

static int begin_motion(struct pca9685 *device, char *error, size_t error_size)
{
    if (pca9685_initialize(device, GAR_SERVO_PWM_HZ) != 0) {
        set_error(error, error_size,
                  "PCA9685 initialization failed; outputs remain disabled: %s",
                  strerror(errno));
        (void)pca9685_all_off(device);
        return -1;
    }
    return 0;
}

int gar_motion_calibrate(struct pca9685 *device, uint8_t channel,
                         uint32_t pulse_us, uint32_t hold_ms,
                         const struct gar_motion_runtime *runtime,
                         char *error, size_t error_size)
{
    int result = 0;

    if (device == NULL || runtime == NULL || channel >= PCA9685_CHANNEL_COUNT ||
        pulse_us < GAR_SERVO_MIN_PULSE_US ||
        pulse_us > GAR_SERVO_MAX_PULSE_US || hold_ms == 0U ||
        hold_ms > GAR_MAX_CALIBRATION_HOLD_MS) {
        errno = EINVAL;
        set_error(error, error_size, "invalid calibration motion arguments");
        return -1;
    }
    if (stop_requested(runtime)) {
        errno = EINTR;
        set_error(error, error_size,
                  "calibration motion interrupted before initialization");
        return -1;
    }
    if (begin_motion(device, error, error_size) != 0) {
        return -1;
    }
    if (stop_requested(runtime)) {
        errno = EINTR;
        result = -1;
    } else if (pca9685_set_pulse_us(device, channel, pulse_us) != 0 ||
        sleep_interruptible(runtime, hold_ms) != 0) {
        result = -1;
    }
    return finish_off(device, result, "calibration motion", error, error_size);
}

int gar_motion_neutral(struct pca9685 *device,
                       const struct gar_servo_calibration *servos,
                       size_t servo_count, uint32_t hold_ms,
                       const struct gar_motion_runtime *runtime,
                       char *error, size_t error_size)
{
    size_t index;
    int result = 0;

    if (device == NULL || servos == NULL || servo_count == 0U ||
        runtime == NULL || hold_ms == 0U ||
        hold_ms > GAR_MAX_NEUTRAL_HOLD_MS) {
        errno = EINVAL;
        set_error(error, error_size, "invalid neutral motion arguments");
        return -1;
    }
    for (index = 0U; index < servo_count; ++index) {
        if (!servos[index].calibrated) {
            errno = EPERM;
            set_error(error, error_size,
                      "servo '%s' is not calibrated; no motion was sent",
                      servos[index].name);
            return -1;
        }
    }
    if (stop_requested(runtime)) {
        errno = EINTR;
        set_error(error, error_size,
                  "neutral motion interrupted before initialization");
        return -1;
    }
    if (begin_motion(device, error, error_size) != 0) {
        return -1;
    }
    for (index = 0U; index < servo_count; ++index) {
        if (stop_requested(runtime)) {
            errno = EINTR;
            result = -1;
            break;
        }
        if (pca9685_set_pulse_us(device, servos[index].channel,
                                 servos[index].neutral_us) != 0) {
            result = -1;
            break;
        }
        /* Stagger starts to reduce simultaneous servo inrush current. */
        if (index + 1U < servo_count &&
            sleep_interruptible(runtime, GAR_MOTION_STEP_MS) != 0) {
            result = -1;
            break;
        }
    }
    if (result == 0 && sleep_interruptible(runtime, hold_ms) != 0) {
        result = -1;
    }
    return finish_off(device, result, "neutral motion", error, error_size);
}

static uint32_t interpolate_pulse(uint32_t from_us, uint32_t to_us,
                                  uint32_t elapsed_ms, uint32_t duration_ms)
{
    int64_t delta = (int64_t)to_us - from_us;
    int64_t numerator = delta * elapsed_ms;
    int64_t adjustment = numerator >= 0 ? (int64_t)duration_ms / 2
                                        : -(int64_t)duration_ms / 2;

    return (uint32_t)((int64_t)from_us +
                      (numerator + adjustment) / duration_ms);
}

int gar_motion_move(struct pca9685 *device,
                    const struct gar_servo_calibration *servo,
                    int32_t from_position, int32_t position,
                    uint32_t duration_ms, uint32_t hold_ms,
                    const struct gar_motion_runtime *runtime,
                    char *error, size_t error_size)
{
    uint32_t from_us;
    uint32_t target_us;
    uint32_t minimum_duration;
    uint32_t elapsed_ms = 0U;
    int result = 0;

    if (device == NULL || servo == NULL || runtime == NULL ||
        duration_ms == 0U || duration_ms > GAR_MAX_MOVE_DURATION_MS ||
        hold_ms > GAR_MAX_MOVE_HOLD_MS) {
        errno = EINVAL;
        set_error(error, error_size, "invalid move arguments");
        return -1;
    }
    if (!servo->calibrated) {
        errno = EPERM;
        set_error(error, error_size,
                  "servo '%s' is not calibrated; no motion was sent",
                  servo->name);
        return -1;
    }
    if (gar_position_to_pulse(servo, from_position, &from_us) != 0 ||
        gar_position_to_pulse(servo, position, &target_us) != 0) {
        set_error(error, error_size,
                  "start and target positions must be integers from -1000 to 1000");
        return -1;
    }
    minimum_duration = gar_minimum_duration_ms(servo, from_us, target_us);
    if (duration_ms < minimum_duration) {
        errno = ERANGE;
        set_error(error, error_size,
                  "duration %u ms exceeds '%s' rate limit; use at least %u ms",
                  duration_ms, servo->name, minimum_duration);
        return -1;
    }
    if (stop_requested(runtime)) {
        errno = EINTR;
        set_error(error, error_size,
                  "servo move interrupted before initialization");
        return -1;
    }
    if (begin_motion(device, error, error_size) != 0) {
        return -1;
    }
    if (stop_requested(runtime)) {
        errno = EINTR;
        result = -1;
    } else if (pca9685_set_pulse_us(device, servo->channel, from_us) != 0) {
        result = -1;
    }
    while (result == 0 && elapsed_ms < duration_ms) {
        uint32_t step_ms = duration_ms - elapsed_ms;
        uint32_t pulse_us;

        if (step_ms > GAR_MOTION_STEP_MS) {
            step_ms = GAR_MOTION_STEP_MS;
        }
        if (sleep_interruptible(runtime, step_ms) != 0) {
            result = -1;
            break;
        }
        elapsed_ms += step_ms;
        pulse_us =
            interpolate_pulse(from_us, target_us, elapsed_ms, duration_ms);
        if (pca9685_set_pulse_us(device, servo->channel, pulse_us) != 0) {
            result = -1;
        }
    }
    if (result == 0 && sleep_interruptible(runtime, hold_ms) != 0) {
        result = -1;
    }
    return finish_off(device, result, "servo move", error, error_size);
}

int gar_motion_sequence(struct pca9685 *device,
                        const struct gar_servo_calibration *servos,
                        size_t servo_count, const int32_t *positions,
                        size_t frame_count, uint32_t step_ms,
                        uint32_t hold_ms,
                        const struct gar_motion_runtime *runtime,
                        char *error, size_t error_size)
{
    uint32_t pulses[GAR_MAX_SEQUENCE_FRAMES][GAR_MAX_SERVOS];
    uint64_t total_ms;
    size_t frame;
    size_t servo_index;
    int result = 0;

    if (device == NULL || servos == NULL || positions == NULL ||
        runtime == NULL || servo_count == 0U || servo_count > GAR_MAX_SERVOS ||
        frame_count < 2U || frame_count > GAR_MAX_SEQUENCE_FRAMES ||
        step_ms == 0U || step_ms > GAR_MAX_SEQUENCE_STEP_MS ||
        hold_ms > GAR_MAX_SEQUENCE_HOLD_MS) {
        errno = EINVAL;
        set_error(error, error_size, "invalid motion sequence arguments");
        return -1;
    }
    total_ms = (uint64_t)(frame_count - 1U) *
               ((uint64_t)step_ms + hold_ms);
    if (total_ms > GAR_MAX_SEQUENCE_TOTAL_MS) {
        errno = ERANGE;
        set_error(error, error_size,
                  "motion sequence exceeds %u ms total limit",
                  GAR_MAX_SEQUENCE_TOTAL_MS);
        return -1;
    }

    for (servo_index = 0U; servo_index < servo_count; ++servo_index) {
        if (!servos[servo_index].calibrated) {
            errno = EPERM;
            set_error(error, error_size,
                      "servo '%s' is not calibrated; no motion was sent",
                      servos[servo_index].name);
            return -1;
        }
    }
    for (frame = 0U; frame < frame_count; ++frame) {
        for (servo_index = 0U; servo_index < servo_count; ++servo_index) {
            size_t position_index = frame * servo_count + servo_index;

            if (gar_position_to_pulse(&servos[servo_index],
                                      positions[position_index],
                                      &pulses[frame][servo_index]) != 0) {
                set_error(error, error_size,
                          "sequence positions must be integers from -1000 to 1000");
                return -1;
            }
            if (frame > 0U &&
                step_ms < gar_minimum_duration_ms(
                              &servos[servo_index],
                              pulses[frame - 1U][servo_index],
                              pulses[frame][servo_index])) {
                errno = ERANGE;
                set_error(error, error_size,
                          "sequence step %zu exceeds '%s' rate limit",
                          frame, servos[servo_index].name);
                return -1;
            }
        }
    }
    if (stop_requested(runtime)) {
        errno = EINTR;
        set_error(error, error_size,
                  "motion sequence interrupted before initialization");
        return -1;
    }
    if (begin_motion(device, error, error_size) != 0) {
        return -1;
    }

    for (servo_index = 0U; servo_index < servo_count; ++servo_index) {
        if (stop_requested(runtime)) {
            errno = EINTR;
            result = -1;
            break;
        }
        if (pca9685_set_pulse_us(device, servos[servo_index].channel,
                                 pulses[0][servo_index]) != 0) {
            result = -1;
            break;
        }
        if (servo_index + 1U < servo_count &&
            sleep_interruptible(runtime, GAR_MOTION_STEP_MS) != 0) {
            result = -1;
            break;
        }
    }

    for (frame = 1U; result == 0 && frame < frame_count; ++frame) {
        uint32_t elapsed_ms = 0U;

        while (result == 0 && elapsed_ms < step_ms) {
            uint32_t slice_ms = step_ms - elapsed_ms;

            if (slice_ms > GAR_MOTION_STEP_MS) {
                slice_ms = GAR_MOTION_STEP_MS;
            }
            if (sleep_interruptible(runtime, slice_ms) != 0) {
                result = -1;
                break;
            }
            elapsed_ms += slice_ms;
            for (servo_index = 0U; servo_index < servo_count;
                 ++servo_index) {
                uint32_t pulse_us;

                if (stop_requested(runtime)) {
                    errno = EINTR;
                    result = -1;
                    break;
                }
                pulse_us = interpolate_pulse(
                    pulses[frame - 1U][servo_index],
                    pulses[frame][servo_index], elapsed_ms, step_ms);
                if (pca9685_set_pulse_us(device,
                                         servos[servo_index].channel,
                                         pulse_us) != 0) {
                    result = -1;
                    break;
                }
            }
        }
        if (result == 0 && hold_ms > 0U &&
            sleep_interruptible(runtime, hold_ms) != 0) {
            result = -1;
        }
    }
    return finish_off(device, result, "motion sequence", error, error_size);
}
