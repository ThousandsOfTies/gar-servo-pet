#define _POSIX_C_SOURCE 200809L

#include "pca9685.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define PCA9685_MODE1 0x00U
#define PCA9685_MODE2 0x01U
#define PCA9685_LED0_ON_L 0x06U
#define PCA9685_PRESCALE 0xfeU
#define PCA9685_ALL_LED_OFF_H 0xfdU

#define PCA9685_MODE1_RESTART 0x80U
#define PCA9685_MODE1_AUTO_INCREMENT 0x20U
#define PCA9685_MODE1_SLEEP 0x10U
#define PCA9685_MODE2_OUTDRV 0x04U
#define PCA9685_FULL_OFF 0x10U

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

static int linux_read_reg(void *context, uint8_t reg, uint8_t *value)
{
    struct pca9685_linux *linux_io = context;
    struct i2c_msg messages[2];
    struct i2c_rdwr_ioctl_data transfer;
    uint8_t register_number = reg;

    messages[0].addr = linux_io->address;
    messages[0].flags = 0U;
    messages[0].len = 1U;
    messages[0].buf = &register_number;
    messages[1].addr = linux_io->address;
    messages[1].flags = I2C_M_RD;
    messages[1].len = 1U;
    messages[1].buf = value;
    transfer.msgs = messages;
    transfer.nmsgs = 2U;

    return ioctl(linux_io->fd, I2C_RDWR, &transfer) < 0 ? -1 : 0;
}

static int linux_write_regs(void *context, uint8_t start_reg,
                            const uint8_t *values, size_t value_count)
{
    struct pca9685_linux *linux_io = context;
    struct i2c_msg message;
    struct i2c_rdwr_ioctl_data transfer;
    uint8_t buffer[1U + 8U];

    if (value_count > sizeof(buffer) - 1U) {
        errno = EINVAL;
        return -1;
    }
    buffer[0] = start_reg;
    if (value_count > 0U) {
        (void)memcpy(&buffer[1], values, value_count);
    }
    message.addr = linux_io->address;
    message.flags = 0U;
    message.len = (uint16_t)(value_count + 1U);
    message.buf = buffer;
    transfer.msgs = &message;
    transfer.nmsgs = 1U;

    return ioctl(linux_io->fd, I2C_RDWR, &transfer) < 0 ? -1 : 0;
}

static int linux_write_reg(void *context, uint8_t reg, uint8_t value)
{
    return linux_write_regs(context, reg, &value, 1U);
}

static int linux_sleep_us(void *context, uint32_t microseconds)
{
    struct timespec request;
    struct timespec remaining;

    (void)context;
    request.tv_sec = (time_t)(microseconds / 1000000U);
    request.tv_nsec = (long)(microseconds % 1000000U) * 1000L;
    while (nanosleep(&request, &remaining) != 0) {
        if (errno != EINTR) {
            return -1;
        }
        request = remaining;
    }
    return 0;
}

static int linux_open(struct pca9685_linux *linux_io, const char *device,
                      uint8_t address, bool exclusive, char *error,
                      size_t error_size)
{
    if (linux_io == NULL || device == NULL || address > 0x7fU) {
        errno = EINVAL;
        set_error(error, error_size, "invalid PCA9685 Linux I2C arguments");
        return -1;
    }
    linux_io->fd = open(device, O_RDWR | O_CLOEXEC);
    if (linux_io->fd < 0) {
        set_error(error, error_size, "%s: cannot open: %s", device,
                  strerror(errno));
        return -1;
    }
    if (exclusive && flock(linux_io->fd, LOCK_EX | LOCK_NB) != 0) {
        int saved_errno = errno;

        set_error(error, error_size, "%s: controller is already in use: %s",
                  device, strerror(saved_errno));
        (void)close(linux_io->fd);
        linux_io->fd = -1;
        errno = saved_errno;
        return -1;
    }
    linux_io->address = address;
    return 0;
}

int pca9685_linux_open(struct pca9685_linux *linux_io, const char *device,
                       uint8_t address, char *error, size_t error_size)
{
    return linux_open(linux_io, device, address, true, error, error_size);
}

int pca9685_linux_open_unlocked(struct pca9685_linux *linux_io,
                                const char *device, uint8_t address,
                                char *error, size_t error_size)
{
    return linux_open(linux_io, device, address, false, error, error_size);
}

void pca9685_linux_close(struct pca9685_linux *linux_io)
{
    if (linux_io != NULL && linux_io->fd >= 0) {
        (void)close(linux_io->fd);
        linux_io->fd = -1;
    }
}

void pca9685_linux_bind(struct pca9685_linux *linux_io, struct pca9685 *device)
{
    (void)memset(device, 0, sizeof(*device));
    device->io.context = linux_io;
    device->io.read_reg = linux_read_reg;
    device->io.write_reg = linux_write_reg;
    device->io.write_regs = linux_write_regs;
    device->io.sleep_us = linux_sleep_us;
    device->oscillator_hz = PCA9685_DEFAULT_OSCILLATOR_HZ;
}

static int valid_io(const struct pca9685 *device)
{
    return device != NULL && device->io.read_reg != NULL &&
           device->io.write_reg != NULL && device->io.write_regs != NULL &&
           device->io.sleep_us != NULL;
}

int pca9685_all_off(struct pca9685 *device)
{
    if (!valid_io(device)) {
        errno = EINVAL;
        return -1;
    }
    return device->io.write_reg(device->io.context, PCA9685_ALL_LED_OFF_H,
                                PCA9685_FULL_OFF);
}

static int fail_off(struct pca9685 *device)
{
    int saved_errno = errno;

    (void)pca9685_all_off(device);
    errno = saved_errno;
    return -1;
}

uint8_t pca9685_frequency_to_prescale(uint32_t oscillator_hz,
                                      uint16_t frequency_hz)
{
    uint64_t denominator;
    uint64_t rounded_divisor;

    if (oscillator_hz == 0U || frequency_hz == 0U) {
        return 0U;
    }
    denominator = (uint64_t)4096U * frequency_hz;
    rounded_divisor = ((uint64_t)oscillator_hz + denominator / 2U) /
                      denominator;
    if (rounded_divisor <= 1U) {
        return 0U;
    }
    if (rounded_divisor > 256U) {
        return 255U;
    }
    return (uint8_t)(rounded_divisor - 1U);
}

uint16_t pca9685_pulse_us_to_count(uint32_t pulse_us, uint16_t frequency_hz)
{
    uint64_t numerator = (uint64_t)pulse_us * frequency_hz * 4096U;
    uint64_t count = (numerator + 500000U) / 1000000U;

    if (count > 4095U) {
        count = 4095U;
    }
    return (uint16_t)count;
}

int pca9685_initialize(struct pca9685 *device, uint16_t frequency_hz)
{
    uint8_t mode1;
    uint8_t sleep_mode;
    uint8_t wake_mode;
    uint8_t prescale;
    uint8_t channel;
    static const uint8_t channel_off[4] = {0U, 0U, 0U, PCA9685_FULL_OFF};

    if (!valid_io(device) || frequency_hz == 0U || frequency_hz > 1526U) {
        errno = EINVAL;
        return -1;
    }

    /* This is deliberately the first write: never re-clock active outputs. */
    if (pca9685_all_off(device) != 0) {
        return -1;
    }
    if (device->io.read_reg(device->io.context, PCA9685_MODE1, &mode1) != 0) {
        return fail_off(device);
    }
    sleep_mode = (uint8_t)((mode1 & (uint8_t)~PCA9685_MODE1_RESTART) |
                           PCA9685_MODE1_AUTO_INCREMENT |
                           PCA9685_MODE1_SLEEP);
    if (device->io.write_reg(device->io.context, PCA9685_MODE1, sleep_mode) !=
        0) {
        return fail_off(device);
    }

    /*
     * ALL_LED_OFF_H is a write-only broadcast register, not a separate global
     * latch. With auto-increment now enabled, initialize every channel's four
     * real registers while the oscillator is asleep. A later channel write
     * can then enable only that channel without touching ALL_LED_* again.
     */
    for (channel = 0U; channel < PCA9685_CHANNEL_COUNT; ++channel) {
        uint8_t channel_register =
            (uint8_t)(PCA9685_LED0_ON_L + channel * 4U);

        if (device->io.write_regs(device->io.context, channel_register,
                                  channel_off, sizeof(channel_off)) != 0) {
            return fail_off(device);
        }
    }

    prescale =
        pca9685_frequency_to_prescale(device->oscillator_hz, frequency_hz);
    if (device->io.write_reg(device->io.context, PCA9685_PRESCALE, prescale) !=
        0) {
        return fail_off(device);
    }
    /* Never preserve a stale INVRT/OCH/OUTNE setting for servo pulses. */
    if (device->io.write_reg(device->io.context, PCA9685_MODE2,
                             PCA9685_MODE2_OUTDRV) != 0) {
        return fail_off(device);
    }

    wake_mode = (uint8_t)(sleep_mode & (uint8_t)~PCA9685_MODE1_SLEEP);
    if (device->io.write_reg(device->io.context, PCA9685_MODE1, wake_mode) !=
        0) {
        return fail_off(device);
    }
    if (device->io.sleep_us(device->io.context, 500U) != 0) {
        return fail_off(device);
    }
    if (device->io.write_reg(device->io.context, PCA9685_MODE1,
                             (uint8_t)(wake_mode | PCA9685_MODE1_RESTART)) !=
        0) {
        return fail_off(device);
    }
    device->frequency_hz = frequency_hz;
    return 0;
}

int pca9685_probe_readonly(struct pca9685 *device, struct pca9685_probe *probe)
{
    if (!valid_io(device) || probe == NULL) {
        errno = EINVAL;
        return -1;
    }
    if (device->io.read_reg(device->io.context, PCA9685_MODE1,
                            &probe->mode1) != 0 ||
        device->io.read_reg(device->io.context, PCA9685_MODE2,
                            &probe->mode2) != 0 ||
        device->io.read_reg(device->io.context, PCA9685_PRESCALE,
                            &probe->prescale) != 0) {
        return -1;
    }
    return 0;
}

int pca9685_set_pulse_us(struct pca9685 *device, uint8_t channel,
                         uint32_t pulse_us)
{
    uint32_t period_us;
    uint16_t count;
    uint8_t values[4];
    uint8_t channel_register;

    if (!valid_io(device) || device->frequency_hz == 0U ||
        channel >= PCA9685_CHANNEL_COUNT) {
        errno = EINVAL;
        return -1;
    }
    period_us = 1000000U / device->frequency_hz;
    if (pulse_us == 0U || pulse_us >= period_us) {
        errno = ERANGE;
        return -1;
    }

    count = pca9685_pulse_us_to_count(pulse_us, device->frequency_hz);
    values[0] = 0U;
    values[1] = 0U;
    values[2] = (uint8_t)(count & 0xffU);
    values[3] = (uint8_t)((count >> 8U) & 0x0fU);
    channel_register = (uint8_t)(PCA9685_LED0_ON_L + channel * 4U);
    if (device->io.write_regs(device->io.context, channel_register, values,
                              sizeof(values)) != 0) {
        return fail_off(device);
    }
    return 0;
}

int pca9685_channel_off(struct pca9685 *device, uint8_t channel)
{
    uint8_t off_high;
    uint8_t off_register;

    if (!valid_io(device) || channel >= PCA9685_CHANNEL_COUNT) {
        errno = EINVAL;
        return -1;
    }
    off_register = (uint8_t)(PCA9685_LED0_ON_L + channel * 4U + 3U);
    if (device->io.read_reg(device->io.context, off_register, &off_high) != 0) {
        return fail_off(device);
    }
    off_high = (uint8_t)(off_high | PCA9685_FULL_OFF);
    if (device->io.write_reg(device->io.context, off_register, off_high) != 0) {
        return fail_off(device);
    }
    return 0;
}
