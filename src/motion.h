#ifndef GAR_SERVO_PET_MOTION_H
#define GAR_SERVO_PET_MOTION_H

#include "config.h"
#include "pca9685.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GAR_MAX_CALIBRATION_HOLD_MS 10000U
#define GAR_MAX_NEUTRAL_HOLD_MS 30000U
#define GAR_MAX_MOVE_HOLD_MS 30000U
#define GAR_MAX_MOVE_DURATION_MS 60000U
#define GAR_MAX_SEQUENCE_FRAMES 12U
#define GAR_MAX_SEQUENCE_STEP_MS 5000U
#define GAR_MAX_SEQUENCE_HOLD_MS 1000U
#define GAR_MAX_SEQUENCE_TOTAL_MS 30000U

struct gar_motion_runtime {
    void *context;
    int (*sleep_ms)(void *context, uint32_t milliseconds);
    bool (*stop_requested)(void *context);
};

int gar_position_to_pulse(const struct gar_servo_calibration *servo,
                          int32_t position, uint32_t *pulse_us);
uint32_t gar_minimum_duration_ms(const struct gar_servo_calibration *servo,
                                 uint32_t from_us, uint32_t to_us);

int gar_motion_calibrate(struct pca9685 *device, uint8_t channel,
                         uint32_t pulse_us, uint32_t hold_ms,
                         const struct gar_motion_runtime *runtime,
                         char *error, size_t error_size);
int gar_motion_neutral(struct pca9685 *device,
                       const struct gar_servo_calibration *servos,
                       size_t servo_count, uint32_t hold_ms,
                       const struct gar_motion_runtime *runtime,
                       char *error, size_t error_size);
int gar_motion_move(struct pca9685 *device,
                    const struct gar_servo_calibration *servo,
                    int32_t from_position, int32_t position,
                    uint32_t duration_ms, uint32_t hold_ms,
                    const struct gar_motion_runtime *runtime,
                    char *error, size_t error_size);
int gar_motion_sequence(struct pca9685 *device,
                        const struct gar_servo_calibration *servos,
                        size_t servo_count, const int32_t *positions,
                        size_t frame_count, uint32_t step_ms,
                        uint32_t hold_ms,
                        const struct gar_motion_runtime *runtime,
                        char *error, size_t error_size);

#endif
