#define _POSIX_C_SOURCE 200809L

#include "config.h"
#include "motion.h"
#include "pca9685.h"

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define GAR_DEFAULT_STOP_REQUEST "/run/gar-servo-pet.stop"
#define GAR_EMERGENCY_OFF_REPEATS 6U
#define GAR_EMERGENCY_OFF_INTERVAL_MS 20U
#define GAR_EMERGENCY_LOCK_ATTEMPTS 100U
#define GAR_AIR_WALK_SERVO_COUNT 4U
#define GAR_AIR_WALK_MAX_AMPLITUDE 250U
#define GAR_AIR_WALK_MAX_CYCLES 5U
#define GAR_AIR_WALK_MIN_STEP_MS 250U
#define GAR_AIR_WALK_MAX_STEP_MS 2000U
#define GAR_AIR_WALK_MAX_PAUSE_MS 500U

struct cli_context {
    const char *i2c_config_path;
    const char *servo_config_path;
};

static volatile sig_atomic_t interrupted = 0;

static const char *stop_request_path(void)
{
    const char *configured = getenv("GAR_SERVO_STOP_FILE");

    return configured != NULL && *configured != '\0'
               ? configured
               : GAR_DEFAULT_STOP_REQUEST;
}

static int write_stop_request(void)
{
    const char *path = stop_request_path();
    int flags = O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC;
    int descriptor;
    char contents[64];
    int length;

#ifdef O_NOFOLLOW
    flags |= O_NOFOLLOW;
#endif
    descriptor = open(path, flags, 0644);
    if (descriptor < 0) {
        (void)fprintf(stderr, "error: cannot create stop request %s: %s\n",
                      path, strerror(errno));
        return -1;
    }
    length = snprintf(contents, sizeof(contents), "%ld\n", (long)getpid());
    if (length < 0 || (size_t)length >= sizeof(contents) ||
        write(descriptor, contents, (size_t)length) != length) {
        int saved_errno = errno == 0 ? EIO : errno;

        (void)close(descriptor);
        errno = saved_errno;
        (void)fprintf(stderr, "error: cannot write stop request %s: %s\n",
                      path, strerror(errno));
        return -1;
    }
    if (close(descriptor) != 0) {
        (void)fprintf(stderr, "error: cannot close stop request %s: %s\n",
                      path, strerror(errno));
        return -1;
    }
    return 0;
}

static int clear_stop_request(void)
{
    const char *path = stop_request_path();

    if (unlink(path) != 0 && errno != ENOENT) {
        (void)fprintf(stderr, "error: cannot clear stop request %s: %s\n",
                      path, strerror(errno));
        return -1;
    }
    return 0;
}

static bool stop_request_pending(void)
{
    if (access(stop_request_path(), F_OK) == 0) {
        return true;
    }
    return errno != ENOENT;
}

static void usage(FILE *stream)
{
    (void)fprintf(
        stream,
        "Usage:\n"
        "  gar-servoctl [CONFIG] validate\n"
        "  gar-servoctl [CONFIG] probe\n"
        "  gar-servoctl [CONFIG] off [all|SERVO]\n"
        "  gar-servoctl [CONFIG] calibrate --channel N --pulse-us N "
        "--confirm-pulse-us N --hold-ms N --ack-motion\n"
        "  gar-servoctl [CONFIG] neutral SERVO --hold-ms N\n"
        "  gar-servoctl [CONFIG] neutral all --hold-ms N --ack-all\n"
        "  gar-servoctl [CONFIG] move SERVO --from-position N --position N "
        "--duration-ms N --hold-ms N --ack-position-known\n"
        "  gar-servoctl [CONFIG] air-walk --amplitude N --cycles N "
        "--step-ms N --pause-ms N --ack-all --ack-neutral-known "
        "--body-lifted\n"
        "\n"
        "CONFIG:\n"
        "  --i2c-config PATH       i2c.csv (or GAR_I2C_CONFIG)\n"
        "  --servo-config PATH     servo-calibration.csv "
        "(or GAR_SERVO_CONFIG)\n"
        "\n"
        "Motion position is normalized to -1000..1000. Every motion command "
        "ends with outputs off.\n");
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
    parsed = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed < minimum ||
        parsed > maximum) {
        return -1;
    }
    *value = (uint32_t)parsed;
    return 0;
}

static int parse_i32(const char *text, int32_t minimum, int32_t maximum,
                     int32_t *value)
{
    char *end = NULL;
    long parsed;

    if (text == NULL || *text == '\0') {
        return -1;
    }
    errno = 0;
    parsed = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed < minimum ||
        parsed > maximum) {
        return -1;
    }
    *value = (int32_t)parsed;
    return 0;
}

static int require_paths(const struct cli_context *context, bool servo_needed)
{
    if (context->i2c_config_path == NULL ||
        (servo_needed && context->servo_config_path == NULL)) {
        (void)fprintf(stderr,
                      "error: configuration path missing; use --i2c-config%s "
                      "or the run wrapper/environment\n",
                      servo_needed ? " and --servo-config" : "");
        return -1;
    }
    return 0;
}

static int load_i2c(const struct cli_context *context,
                    struct gar_i2c_config *i2c)
{
    char error[512];

    if (require_paths(context, false) != 0) {
        return -1;
    }
    if (gar_load_i2c_config(context->i2c_config_path, i2c, error,
                            sizeof(error)) != 0) {
        (void)fprintf(stderr, "error: %s\n", error);
        return -1;
    }
    return 0;
}

static int load_all(const struct cli_context *context,
                    struct gar_i2c_config *i2c,
                    struct gar_servo_config *servos)
{
    char error[512];

    if (require_paths(context, true) != 0) {
        return -1;
    }
    if (gar_load_i2c_config(context->i2c_config_path, i2c, error,
                            sizeof(error)) != 0 ||
        gar_load_servo_config(context->servo_config_path, servos, error,
                              sizeof(error)) != 0) {
        (void)fprintf(stderr, "error: %s\n", error);
        return -1;
    }
    return 0;
}

static int open_controller(const struct gar_i2c_config *i2c,
                           struct pca9685_linux *linux_io,
                           struct pca9685 *controller)
{
    char error[512];

    linux_io->fd = -1;
    if (pca9685_linux_open(linux_io, i2c->device, i2c->address, error,
                           sizeof(error)) != 0) {
        (void)fprintf(stderr, "error: %s\n", error);
        return -1;
    }
    pca9685_linux_bind(linux_io, controller);
    return 0;
}

static int prepare_motion_controller(const struct gar_i2c_config *i2c,
                                     struct pca9685_linux *linux_io,
                                     struct pca9685 *controller)
{
    if (open_controller(i2c, linux_io, controller) != 0) {
        return -1;
    }
    if (stop_request_pending()) {
        (void)pca9685_all_off(controller);
        (void)fprintf(
            stderr,
            "error: emergency stop is latched; run 'off all' before starting motion\n");
        pca9685_linux_close(linux_io);
        return -1;
    }
    return 0;
}

static void signal_handler(int signal_number)
{
    (void)signal_number;
    interrupted = 1;
}

static int install_signal_handlers(void)
{
    struct sigaction action;

    (void)memset(&action, 0, sizeof(action));
    action.sa_handler = signal_handler;
    (void)sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) != 0 ||
        sigaction(SIGTERM, &action, NULL) != 0) {
        return -1;
    }
    return 0;
}

static int runtime_sleep_ms(void *context, uint32_t milliseconds)
{
    struct timespec request;
    struct timespec remaining;

    (void)context;
    request.tv_sec = (time_t)(milliseconds / 1000U);
    request.tv_nsec = (long)(milliseconds % 1000U) * 1000000L;
    while (nanosleep(&request, &remaining) != 0) {
        if (errno != EINTR || interrupted != 0) {
            return -1;
        }
        request = remaining;
    }
    return 0;
}

static bool runtime_stop_requested(void *context)
{
    (void)context;
    if (interrupted != 0) {
        return true;
    }
    return stop_request_pending();
}

static struct gar_motion_runtime motion_runtime(void)
{
    struct gar_motion_runtime runtime;

    runtime.context = NULL;
    runtime.sleep_ms = runtime_sleep_ms;
    runtime.stop_requested = runtime_stop_requested;
    return runtime;
}

static int command_validate(const struct cli_context *context)
{
    struct gar_i2c_config i2c;
    struct gar_servo_config servos;
    size_t index;
    size_t calibrated = 0U;

    if (load_all(context, &i2c, &servos) != 0) {
        return 1;
    }
    for (index = 0U; index < servos.count; ++index) {
        if (servos.servos[index].calibrated) {
            ++calibrated;
        }
    }
    (void)printf("configuration valid: controller=%s device=%s address=0x%02x "
                 "servos=%zu calibrated=%zu\n",
                 i2c.name, i2c.device, i2c.address, servos.count, calibrated);
    return 0;
}

static int command_probe(const struct cli_context *context)
{
    struct gar_i2c_config i2c;
    struct pca9685_linux linux_io;
    struct pca9685 controller;
    struct pca9685_probe probe;
    int result = 1;

    if (load_i2c(context, &i2c) != 0 ||
        open_controller(&i2c, &linux_io, &controller) != 0) {
        return 1;
    }
    if (pca9685_probe_readonly(&controller, &probe) != 0) {
        (void)fprintf(stderr, "error: PCA9685 read-only probe failed: %s\n",
                      strerror(errno));
    } else {
        (void)printf("PCA9685 present: device=%s address=0x%02x "
                     "MODE1=0x%02x MODE2=0x%02x PRE_SCALE=0x%02x\n",
                     i2c.device, i2c.address, probe.mode1, probe.mode2,
                     probe.prescale);
        result = 0;
    }
    pca9685_linux_close(&linux_io);
    return result;
}

static const struct gar_servo_calibration *
find_channel(const struct gar_servo_config *servos, uint8_t channel)
{
    size_t index;

    for (index = 0U; index < servos->count; ++index) {
        if (servos->servos[index].channel == channel) {
            return &servos->servos[index];
        }
    }
    return NULL;
}

static const struct gar_servo_calibration *
find_unique_role(const struct gar_servo_config *servos, const char *role)
{
    const struct gar_servo_calibration *match = NULL;
    size_t matches = 0U;
    size_t index;

    for (index = 0U; index < servos->count; ++index) {
        if (strcmp(servos->servos[index].role, role) == 0) {
            match = &servos->servos[index];
            ++matches;
        }
    }
    return matches == 1U ? match : NULL;
}

static int command_off(const struct cli_context *context, int argc,
                       char **argv)
{
    struct gar_i2c_config i2c;
    struct gar_servo_config servos;
    const struct gar_servo_calibration *servo = NULL;
    struct pca9685_linux linux_io;
    struct pca9685 controller;
    const char *target = argc == 0 ? "all" : argv[0];
    char open_error[512];
    bool stop_marker_written;
    uint32_t attempt;
    int result = 0;

    if (argc > 1) {
        (void)fprintf(stderr, "error: off accepts at most one target\n");
        return 2;
    }
    if (strcmp(target, "all") == 0) {
        if (load_i2c(context, &i2c) != 0) {
            return 1;
        }
    } else {
        if (load_all(context, &i2c, &servos) != 0) {
            return 1;
        }
        servo = gar_find_servo(&servos, target);
        if (servo == NULL) {
            (void)fprintf(stderr, "error: unknown servo '%s'\n", target);
            return 1;
        }
    }
    stop_marker_written = write_stop_request() == 0;
    linux_io.fd = -1;
    if (pca9685_linux_open(&linux_io, i2c.device, i2c.address, open_error,
                           sizeof(open_error)) == 0) {
        pca9685_linux_bind(&linux_io, &controller);
        result = servo == NULL
                     ? pca9685_all_off(&controller)
                     : pca9685_channel_off(&controller, servo->channel);
        if (result == 0 && clear_stop_request() != 0) {
            result = -1;
        }
        if (result == 0) {
            (void)printf("outputs off: %s\n", target);
        } else {
            (void)fprintf(stderr, "error: failed to disable outputs: %s\n",
                          strerror(errno));
        }
        pca9685_linux_close(&linux_io);
        return result == 0 ? 0 : 1;
    }
    if (errno != EWOULDBLOCK && errno != EAGAIN) {
        (void)fprintf(stderr, "error: %s\n", open_error);
        return 1;
    }

    if (pca9685_linux_open_unlocked(&linux_io, i2c.device, i2c.address,
                                    open_error, sizeof(open_error)) != 0) {
        (void)fprintf(stderr, "error: emergency output disable failed: %s\n",
                      open_error);
        return 1;
    }
    pca9685_linux_bind(&linux_io, &controller);
    for (attempt = 0U; attempt < GAR_EMERGENCY_OFF_REPEATS; ++attempt) {
        if (pca9685_all_off(&controller) != 0) {
            result = -1;
            break;
        }
        if (attempt + 1U < GAR_EMERGENCY_OFF_REPEATS &&
            runtime_sleep_ms(NULL, GAR_EMERGENCY_OFF_INTERVAL_MS) != 0) {
            result = -1;
            break;
        }
    }
    pca9685_linux_close(&linux_io);
    if (result != 0) {
        (void)fprintf(stderr, "error: emergency output disable failed: %s\n",
                      strerror(errno));
        return 1;
    }
    if (!stop_marker_written) {
        (void)fprintf(
            stderr,
            "error: outputs were disabled repeatedly, but the active motion cancellation marker could not be created\n");
    }

    for (attempt = 0U; attempt < GAR_EMERGENCY_LOCK_ATTEMPTS; ++attempt) {
        linux_io.fd = -1;
        if (pca9685_linux_open(&linux_io, i2c.device, i2c.address, open_error,
                               sizeof(open_error)) == 0) {
            break;
        }
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
            (void)fprintf(stderr,
                          "error: cannot finalize emergency stop: %s\n",
                          open_error);
            return 1;
        }
        if (attempt + 1U < GAR_EMERGENCY_LOCK_ATTEMPTS &&
            runtime_sleep_ms(NULL, GAR_EMERGENCY_OFF_INTERVAL_MS) != 0) {
            (void)fprintf(stderr,
                          "error: emergency stop interrupted; stop remains latched\n");
            return 1;
        }
    }
    if (linux_io.fd < 0) {
        (void)fprintf(
            stderr,
            "error: active controller did not stop within 2 seconds; stop remains latched\n");
        return 1;
    }
    pca9685_linux_bind(&linux_io, &controller);
    result = pca9685_all_off(&controller);
    if (result == 0 && clear_stop_request() != 0) {
        result = -1;
    }
    if (result == 0) {
        (void)printf("outputs off: all (preempted active controller)\n");
    } else {
        (void)fprintf(stderr, "error: failed to finalize emergency stop: %s\n",
                      strerror(errno));
    }
    pca9685_linux_close(&linux_io);
    return result == 0 ? 0 : 1;
}

static int command_calibrate(const struct cli_context *context, int argc,
                             char **argv)
{
    struct gar_i2c_config i2c;
    struct gar_servo_config servos;
    struct pca9685_linux linux_io;
    struct pca9685 controller;
    struct gar_motion_runtime runtime;
    uint32_t channel = UINT32_MAX;
    uint32_t pulse_us = 0U;
    uint32_t confirm_pulse_us = 0U;
    uint32_t hold_ms = 0U;
    bool acknowledged = false;
    int index;
    char error[512];
    int result;

    for (index = 0; index < argc; ++index) {
        if (strcmp(argv[index], "--ack-motion") == 0) {
            acknowledged = true;
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--channel") == 0) {
            if (parse_u32(argv[++index], 0U, 15U, &channel) != 0) {
                (void)fprintf(stderr, "error: invalid --channel\n");
                return 2;
            }
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--pulse-us") == 0) {
            if (parse_u32(argv[++index], GAR_SERVO_MIN_PULSE_US,
                          GAR_SERVO_MAX_PULSE_US, &pulse_us) != 0) {
                (void)fprintf(stderr,
                              "error: --pulse-us must be %u..%u\n",
                              GAR_SERVO_MIN_PULSE_US,
                              GAR_SERVO_MAX_PULSE_US);
                return 2;
            }
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--confirm-pulse-us") == 0) {
            if (parse_u32(argv[++index], GAR_SERVO_MIN_PULSE_US,
                          GAR_SERVO_MAX_PULSE_US,
                          &confirm_pulse_us) != 0) {
                (void)fprintf(stderr,
                              "error: --confirm-pulse-us must be %u..%u\n",
                              GAR_SERVO_MIN_PULSE_US,
                              GAR_SERVO_MAX_PULSE_US);
                return 2;
            }
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--hold-ms") == 0) {
            if (parse_u32(argv[++index], 1U,
                          GAR_MAX_CALIBRATION_HOLD_MS, &hold_ms) != 0) {
                (void)fprintf(stderr, "error: invalid --hold-ms\n");
                return 2;
            }
        } else {
            (void)fprintf(stderr, "error: unknown calibrate option '%s'\n",
                          argv[index]);
            return 2;
        }
    }
    if (!acknowledged || channel == UINT32_MAX || pulse_us == 0U ||
        confirm_pulse_us == 0U || hold_ms == 0U) {
        (void)fprintf(stderr,
                      "error: calibrate requires --channel, --pulse-us, "
                      "--confirm-pulse-us, --hold-ms and --ack-motion\n");
        return 2;
    }
    if (pulse_us != confirm_pulse_us) {
        (void)fprintf(stderr,
                      "error: --confirm-pulse-us must exactly match "
                      "--pulse-us; no motion was sent\n");
        return 2;
    }
    if (load_all(context, &i2c, &servos) != 0) {
        return 1;
    }
    if (find_channel(&servos, (uint8_t)channel) == NULL) {
        (void)fprintf(stderr,
                      "error: channel %u is not declared in servo configuration\n",
                      channel);
        return 1;
    }
    if (prepare_motion_controller(&i2c, &linux_io, &controller) != 0) {
        return 1;
    }
    runtime = motion_runtime();
    result = gar_motion_calibrate(&controller, (uint8_t)channel, pulse_us,
                                  hold_ms, &runtime, error, sizeof(error));
    if (result != 0) {
        (void)fprintf(stderr, "error: %s\n", error);
    } else {
        (void)printf("calibration pulse complete: channel=%u pulse_us=%u; "
                     "outputs off\n",
                     channel, pulse_us);
    }
    pca9685_linux_close(&linux_io);
    return result == 0 ? 0 : 1;
}

static int command_neutral(const struct cli_context *context, int argc,
                           char **argv)
{
    struct gar_i2c_config i2c;
    struct gar_servo_config servos;
    struct pca9685_linux linux_io;
    struct pca9685 controller;
    struct gar_motion_runtime runtime;
    const struct gar_servo_calibration *selected_servo = NULL;
    const char *target = NULL;
    bool target_set = false;
    bool acknowledged_all = false;
    uint32_t hold_ms = 0U;
    int index;
    char error[512];
    int result;

    for (index = 0; index < argc; ++index) {
        if (strcmp(argv[index], "--ack-all") == 0) {
            acknowledged_all = true;
        } else if (strcmp(argv[index], "--hold-ms") == 0 &&
                   index + 1 < argc) {
            if (parse_u32(argv[++index], 1U, GAR_MAX_NEUTRAL_HOLD_MS,
                          &hold_ms) != 0) {
                (void)fprintf(stderr, "error: invalid --hold-ms\n");
                return 2;
            }
        } else if (argv[index][0] != '-' && !target_set) {
            target = argv[index];
            target_set = true;
        } else {
            (void)fprintf(stderr, "error: invalid neutral argument '%s'\n",
                          argv[index]);
            return 2;
        }
    }
    if (!target_set || hold_ms == 0U) {
        (void)fprintf(stderr,
                      "error: neutral requires an explicit SERVO or all target and --hold-ms\n");
        return 2;
    }
    if (strcmp(target, "all") == 0 && !acknowledged_all) {
        (void)fprintf(stderr,
                      "error: neutral all requires --ack-all; no I2C access or motion was attempted\n");
        return 2;
    }
    if (strcmp(target, "all") != 0 && acknowledged_all) {
        (void)fprintf(stderr,
                      "error: --ack-all is valid only with neutral all\n");
        return 2;
    }
    if (load_all(context, &i2c, &servos) != 0) {
        return 1;
    }
    if (strcmp(target, "all") == 0) {
        size_t servo_index;

        for (servo_index = 0U; servo_index < servos.count; ++servo_index) {
            if (!servos.servos[servo_index].calibrated) {
                (void)fprintf(
                    stderr,
                    "error: servo '%s' is not calibrated; no I2C access or motion was attempted\n",
                    servos.servos[servo_index].name);
                return 1;
            }
        }
    } else {
        selected_servo = gar_find_servo(&servos, target);
        if (selected_servo == NULL) {
            (void)fprintf(stderr, "error: unknown servo '%s'\n", target);
            return 1;
        }
        if (!selected_servo->calibrated) {
            (void)fprintf(
                stderr,
                "error: servo '%s' is not calibrated; no I2C access or motion was attempted\n",
                target);
            return 1;
        }
    }
    if (prepare_motion_controller(&i2c, &linux_io, &controller) != 0) {
        return 1;
    }
    runtime = motion_runtime();
    if (strcmp(target, "all") == 0) {
        result = gar_motion_neutral(&controller, servos.servos, servos.count,
                                    hold_ms, &runtime, error, sizeof(error));
    } else {
        result = gar_motion_neutral(&controller, selected_servo, 1U, hold_ms,
                                    &runtime, error, sizeof(error));
    }
    if (result != 0) {
        (void)fprintf(stderr, "error: %s\n", error);
    } else {
        (void)printf("neutral complete: %s; outputs off\n", target);
    }
    pca9685_linux_close(&linux_io);
    return result == 0 ? 0 : 1;
}

static int command_move(const struct cli_context *context, int argc,
                        char **argv)
{
    struct gar_i2c_config i2c;
    struct gar_servo_config servos;
    struct pca9685_linux linux_io;
    struct pca9685 controller;
    struct gar_motion_runtime runtime;
    const struct gar_servo_calibration *servo;
    const char *name;
    int32_t from_position = INT32_MAX;
    int32_t position = INT32_MAX;
    uint32_t duration_ms = 0U;
    uint32_t hold_ms = UINT32_MAX;
    bool hold_set = false;
    bool acknowledged_position = false;
    int index;
    char error[512];
    int result;

    if (argc == 0 || argv[0][0] == '-') {
        (void)fprintf(stderr, "error: move requires a servo name\n");
        return 2;
    }
    name = argv[0];
    for (index = 1; index < argc; ++index) {
        if (strcmp(argv[index], "--ack-position-known") == 0) {
            acknowledged_position = true;
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--from-position") == 0) {
            if (parse_i32(argv[++index], -1000, 1000, &from_position) != 0) {
                (void)fprintf(stderr, "error: invalid --from-position\n");
                return 2;
            }
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--position") == 0) {
            if (parse_i32(argv[++index], -1000, 1000, &position) != 0) {
                (void)fprintf(stderr, "error: invalid --position\n");
                return 2;
            }
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--duration-ms") == 0) {
            if (parse_u32(argv[++index], 1U, GAR_MAX_MOVE_DURATION_MS,
                          &duration_ms) != 0) {
                (void)fprintf(stderr, "error: invalid --duration-ms\n");
                return 2;
            }
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--hold-ms") == 0) {
            if (parse_u32(argv[++index], 0U, GAR_MAX_MOVE_HOLD_MS,
                          &hold_ms) != 0) {
                (void)fprintf(stderr, "error: invalid --hold-ms\n");
                return 2;
            }
            hold_set = true;
        } else {
            (void)fprintf(stderr, "error: invalid move argument '%s'\n",
                          argv[index]);
            return 2;
        }
    }
    if (from_position == INT32_MAX || position == INT32_MAX ||
        duration_ms == 0U || !hold_set || !acknowledged_position) {
        (void)fprintf(stderr,
                      "error: move requires --from-position, --position, "
                      "--duration-ms, --hold-ms and --ack-position-known\n");
        return 2;
    }
    if (load_all(context, &i2c, &servos) != 0) {
        return 1;
    }
    servo = gar_find_servo(&servos, name);
    if (servo == NULL) {
        (void)fprintf(stderr, "error: unknown servo '%s'\n", name);
        return 1;
    }
    if (!servo->calibrated) {
        (void)fprintf(
            stderr,
            "error: servo '%s' is not calibrated; no I2C access or motion was attempted\n",
            name);
        return 1;
    }
    if (prepare_motion_controller(&i2c, &linux_io, &controller) != 0) {
        return 1;
    }
    runtime = motion_runtime();
    result = gar_motion_move(&controller, servo, from_position, position,
                             duration_ms, hold_ms, &runtime, error,
                             sizeof(error));
    if (result != 0) {
        (void)fprintf(stderr, "error: %s\n", error);
    } else {
        (void)printf("move complete: servo=%s from=%d position=%d; outputs off\n",
                     name, from_position, position);
    }
    pca9685_linux_close(&linux_io);
    return result == 0 ? 0 : 1;
}

static int command_air_walk(const struct cli_context *context, int argc,
                            char **argv)
{
    static const char *const roles[GAR_AIR_WALK_SERVO_COUNT] = {
        "front-left", "front-right", "rear-left", "rear-right",
    };
    struct gar_i2c_config i2c;
    struct gar_servo_config servos;
    struct gar_servo_calibration ordered[GAR_AIR_WALK_SERVO_COUNT];
    struct pca9685_linux linux_io;
    struct pca9685 controller;
    struct gar_motion_runtime runtime;
    int32_t positions[GAR_MAX_SEQUENCE_FRAMES * GAR_AIR_WALK_SERVO_COUNT];
    uint32_t amplitude = 0U;
    uint32_t cycles = 0U;
    uint32_t step_ms = 0U;
    uint32_t pause_ms = UINT32_MAX;
    bool acknowledged_all = false;
    bool acknowledged_neutral = false;
    bool body_lifted = false;
    size_t frame_count;
    size_t frame;
    size_t role_index;
    uint32_t cycle;
    int index;
    char error[512];
    int result;

    for (index = 0; index < argc; ++index) {
        if (strcmp(argv[index], "--ack-all") == 0) {
            acknowledged_all = true;
        } else if (strcmp(argv[index], "--ack-neutral-known") == 0) {
            acknowledged_neutral = true;
        } else if (strcmp(argv[index], "--body-lifted") == 0) {
            body_lifted = true;
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--amplitude") == 0) {
            if (parse_u32(argv[++index], 1U, GAR_AIR_WALK_MAX_AMPLITUDE,
                          &amplitude) != 0) {
                (void)fprintf(stderr, "error: --amplitude must be 1..%u\n",
                              GAR_AIR_WALK_MAX_AMPLITUDE);
                return 2;
            }
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--cycles") == 0) {
            if (parse_u32(argv[++index], 1U, GAR_AIR_WALK_MAX_CYCLES,
                          &cycles) != 0) {
                (void)fprintf(stderr, "error: --cycles must be 1..%u\n",
                              GAR_AIR_WALK_MAX_CYCLES);
                return 2;
            }
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--step-ms") == 0) {
            if (parse_u32(argv[++index], GAR_AIR_WALK_MIN_STEP_MS,
                          GAR_AIR_WALK_MAX_STEP_MS, &step_ms) != 0) {
                (void)fprintf(stderr,
                              "error: --step-ms must be %u..%u\n",
                              GAR_AIR_WALK_MIN_STEP_MS,
                              GAR_AIR_WALK_MAX_STEP_MS);
                return 2;
            }
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--pause-ms") == 0) {
            if (parse_u32(argv[++index], 0U, GAR_AIR_WALK_MAX_PAUSE_MS,
                          &pause_ms) != 0) {
                (void)fprintf(stderr, "error: --pause-ms must be 0..%u\n",
                              GAR_AIR_WALK_MAX_PAUSE_MS);
                return 2;
            }
        } else {
            (void)fprintf(stderr, "error: invalid air-walk argument '%s'\n",
                          argv[index]);
            return 2;
        }
    }
    if (amplitude == 0U || cycles == 0U || step_ms == 0U ||
        pause_ms == UINT32_MAX || !acknowledged_all ||
        !acknowledged_neutral || !body_lifted) {
        (void)fprintf(
            stderr,
            "error: air-walk requires --amplitude, --cycles, --step-ms, "
            "--pause-ms, --ack-all, --ack-neutral-known and --body-lifted; "
            "no I2C access or motion was attempted\n");
        return 2;
    }
    if (load_all(context, &i2c, &servos) != 0) {
        return 1;
    }
    if (servos.count != GAR_AIR_WALK_SERVO_COUNT) {
        (void)fprintf(
            stderr,
            "error: air-walk requires exactly four configured servos; no motion was attempted\n");
        return 1;
    }
    for (role_index = 0U; role_index < GAR_AIR_WALK_SERVO_COUNT;
         ++role_index) {
        const struct gar_servo_calibration *servo =
            find_unique_role(&servos, roles[role_index]);

        if (servo == NULL || !servo->calibrated) {
            (void)fprintf(
                stderr,
                "error: air-walk requires one calibrated '%s' servo; no motion was attempted\n",
                roles[role_index]);
            return 1;
        }
        ordered[role_index] = *servo;
    }

    frame_count = (size_t)cycles * 2U + 2U;
    (void)memset(positions, 0, sizeof(positions));
    for (cycle = 0U; cycle < cycles; ++cycle) {
        size_t first = ((size_t)cycle * 2U + 1U) *
                       GAR_AIR_WALK_SERVO_COUNT;
        size_t second = first + GAR_AIR_WALK_SERVO_COUNT;
        int32_t value = (int32_t)amplitude;

        positions[first + 0U] = value;
        positions[first + 1U] = -value;
        positions[first + 2U] = -value;
        positions[first + 3U] = value;
        positions[second + 0U] = -value;
        positions[second + 1U] = value;
        positions[second + 2U] = value;
        positions[second + 3U] = -value;
    }
    frame = frame_count - 1U;
    for (role_index = 0U; role_index < GAR_AIR_WALK_SERVO_COUNT;
         ++role_index) {
        positions[frame * GAR_AIR_WALK_SERVO_COUNT + role_index] = 0;
    }

    if (prepare_motion_controller(&i2c, &linux_io, &controller) != 0) {
        return 1;
    }
    runtime = motion_runtime();
    result = gar_motion_sequence(
        &controller, ordered, GAR_AIR_WALK_SERVO_COUNT, positions,
        frame_count, step_ms, pause_ms, &runtime, error, sizeof(error));
    if (result != 0) {
        (void)fprintf(stderr, "error: %s\n", error);
    } else {
        (void)printf(
            "air-walk complete: amplitude=%u cycles=%u; neutral reached and outputs off\n",
            amplitude, cycles);
    }
    pca9685_linux_close(&linux_io);
    return result == 0 ? 0 : 1;
}

int main(int argc, char **argv)
{
    struct cli_context context;
    const char *command;
    int index = 1;

    context.i2c_config_path = getenv("GAR_I2C_CONFIG");
    context.servo_config_path = getenv("GAR_SERVO_CONFIG");

    while (index < argc && strncmp(argv[index], "--", 2U) == 0) {
        if (strcmp(argv[index], "--help") == 0) {
            usage(stdout);
            return 0;
        }
        if (index + 1 < argc && strcmp(argv[index], "--i2c-config") == 0) {
            context.i2c_config_path = argv[index + 1];
            index += 2;
        } else if (index + 1 < argc &&
                   strcmp(argv[index], "--servo-config") == 0) {
            context.servo_config_path = argv[index + 1];
            index += 2;
        } else {
            (void)fprintf(stderr, "error: unknown global option '%s'\n",
                          argv[index]);
            usage(stderr);
            return 2;
        }
    }
    if (index >= argc) {
        usage(stderr);
        return 2;
    }
    command = argv[index++];
    if (install_signal_handlers() != 0) {
        (void)fprintf(stderr, "error: cannot install signal handlers: %s\n",
                      strerror(errno));
        return 1;
    }
    if (strcmp(command, "validate") == 0) {
        if (index != argc) {
            (void)fprintf(stderr, "error: validate accepts no arguments\n");
            return 2;
        }
        return command_validate(&context);
    }
    if (strcmp(command, "probe") == 0) {
        if (index != argc) {
            (void)fprintf(stderr, "error: probe accepts no arguments\n");
            return 2;
        }
        return command_probe(&context);
    }
    if (strcmp(command, "off") == 0) {
        return command_off(&context, argc - index, &argv[index]);
    }
    if (strcmp(command, "calibrate") == 0) {
        return command_calibrate(&context, argc - index, &argv[index]);
    }
    if (strcmp(command, "neutral") == 0) {
        return command_neutral(&context, argc - index, &argv[index]);
    }
    if (strcmp(command, "move") == 0) {
        return command_move(&context, argc - index, &argv[index]);
    }
    if (strcmp(command, "air-walk") == 0) {
        return command_air_walk(&context, argc - index, &argv[index]);
    }
    (void)fprintf(stderr, "error: unknown command '%s'\n", command);
    usage(stderr);
    return 2;
}
