#include "pca9685.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

struct write_event {
    uint8_t reg;
    uint8_t values[8];
    size_t count;
};

struct fake_io {
    uint8_t registers[256];
    struct write_event writes[128];
    size_t write_count;
    size_t read_count;
    size_t operation_count;
    size_t fail_once_at;
};

static int check(int condition, const char *message)
{
    if (!condition) {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        return -1;
    }
    return 0;
}

static int should_fail(struct fake_io *fake)
{
    ++fake->operation_count;
    if (fake->fail_once_at != 0U &&
        fake->operation_count == fake->fail_once_at) {
        errno = EIO;
        return 1;
    }
    return 0;
}

static int fake_read(void *context, uint8_t reg, uint8_t *value)
{
    struct fake_io *fake = context;

    if (should_fail(fake)) {
        return -1;
    }
    if (reg >= 0xfaU && reg <= 0xfdU) {
        errno = EACCES;
        return -1;
    }
    ++fake->read_count;
    *value = fake->registers[reg];
    return 0;
}

static int fake_write_regs(void *context, uint8_t reg, const uint8_t *values,
                           size_t count)
{
    struct fake_io *fake = context;
    struct write_event *event;
    size_t index;

    if (should_fail(fake)) {
        return -1;
    }
    if (fake->write_count >= ARRAY_SIZE(fake->writes) || count > 8U) {
        errno = ENOSPC;
        return -1;
    }
    event = &fake->writes[fake->write_count++];
    event->reg = reg;
    event->count = count;
    (void)memcpy(event->values, values, count);
    if (reg == 0xfdU && count == 1U) {
        for (index = 0U; index < PCA9685_CHANNEL_COUNT; ++index) {
            fake->registers[0x09U + index * 4U] = values[0];
        }
        return 0;
    }
    for (index = 0U; index < count; ++index) {
        fake->registers[(uint8_t)(reg + index)] = values[index];
    }
    return 0;
}

static int fake_write(void *context, uint8_t reg, uint8_t value)
{
    return fake_write_regs(context, reg, &value, 1U);
}

static int fake_sleep(void *context, uint32_t microseconds)
{
    struct fake_io *fake = context;

    (void)microseconds;
    if (should_fail(fake)) {
        return -1;
    }
    return 0;
}

static void bind_fake(struct fake_io *fake, struct pca9685 *device)
{
    (void)memset(fake, 0, sizeof(*fake));
    (void)memset(device, 0, sizeof(*device));
    device->io.context = fake;
    device->io.read_reg = fake_read;
    device->io.write_reg = fake_write;
    device->io.write_regs = fake_write_regs;
    device->io.sleep_us = fake_sleep;
    device->oscillator_hz = PCA9685_DEFAULT_OSCILLATOR_HZ;
}

static int test_conversion(void)
{
    if (check(pca9685_frequency_to_prescale(25000000U, 50U) == 121U,
              "50 Hz prescale must be 121") != 0 ||
        check(pca9685_pulse_us_to_count(1500U, 50U) == 307U,
              "1500 us must round to 307 counts at 50 Hz") != 0) {
        return -1;
    }
    return 0;
}

static int test_initialize_and_pulse(void)
{
    struct fake_io fake;
    struct pca9685 device;
    size_t channel;

    bind_fake(&fake, &device);
    fake.registers[0x00] = 0x01U;
    fake.registers[0x01] = 0x18U;
    for (channel = 0U; channel < PCA9685_CHANNEL_COUNT; ++channel) {
        fake.registers[0x06U + channel * 4U] = 0xaaU;
        fake.registers[0x07U + channel * 4U] = 0x0bU;
        fake.registers[0x08U + channel * 4U] = 0xccU;
        fake.registers[0x09U + channel * 4U] = 0x0dU;
    }
    if (check(pca9685_initialize(&device, 50U) == 0,
              "initialize should succeed") != 0 ||
        check(fake.writes[0].reg == 0xfdU &&
                  fake.writes[0].values[0] == 0x10U,
              "global FULL_OFF must be the first write") != 0) {
        return -1;
    }
    for (channel = 0U; channel < PCA9685_CHANNEL_COUNT; ++channel) {
        uint8_t base = (uint8_t)(0x06U + channel * 4U);

        if (check(fake.registers[base] == 0U &&
                      fake.registers[base + 1U] == 0U &&
                      fake.registers[base + 2U] == 0U &&
                      fake.registers[base + 3U] == 0x10U,
                  "each stale channel must be reset to known FULL_OFF") != 0) {
            return -1;
        }
    }
    if (check(fake.registers[0xfe] == 121U, "prescale register") != 0 ||
        check(fake.registers[0x00] == 0xa1U,
              "MODE1 must finish at RESTART|AI|ALLCALL") != 0 ||
        check(fake.registers[0x01] == 0x04U,
              "MODE2 must deterministically clear INVRT and OCH") != 0 ||
        check(fake.registers[0x09] == 0x10U &&
                  fake.registers[0x45] == 0x10U,
              "initialize must leave every real channel disabled") != 0) {
        return -1;
    }
    if (check(pca9685_set_pulse_us(&device, 0U, 1500U) == 0,
              "set pulse should succeed") != 0 ||
        check(fake.registers[0x06] == 0U && fake.registers[0x07] == 0U &&
                  fake.registers[0x08] == 0x33U &&
                  fake.registers[0x09] == 0x01U,
              "channel 0 registers must contain count 307") != 0 ||
        check(fake.registers[0x0d] == 0x10U,
              "enabling channel 0 must not enable channel 1") != 0) {
        return -1;
    }
    if (check(pca9685_all_off(&device) == 0,
              "broadcast off should succeed") != 0 ||
        check(fake.registers[0x09] == 0x10U &&
                  fake.registers[0x0d] == 0x10U,
              "broadcast off must fill every channel OFF_H") != 0 ||
        check(pca9685_set_pulse_us(&device, 1U, 1600U) == 0,
              "channel 1 should enable after broadcast off") != 0 ||
        check(fake.registers[0x09] == 0x10U &&
                  (fake.registers[0x0d] & 0x10U) == 0U,
              "enabling channel 1 must leave stale channel 0 disabled") != 0) {
        return -1;
    }
    fake.registers[0x09] = 0x03U;
    if (check(pca9685_channel_off(&device, 0U) == 0,
              "channel off should succeed") != 0 ||
        check(fake.registers[0x09] == 0x13U,
              "channel off must preserve count bits with RMW") != 0) {
        return -1;
    }
    return 0;
}

static int test_probe_is_read_only(void)
{
    struct fake_io fake;
    struct pca9685 device;
    struct pca9685_probe probe;

    bind_fake(&fake, &device);
    fake.registers[0x00] = 0xa1U;
    fake.registers[0x01] = 0x04U;
    fake.registers[0xfe] = 0x79U;
    if (check(pca9685_probe_readonly(&device, &probe) == 0,
              "probe should succeed") != 0 ||
        check(fake.write_count == 0U, "probe must perform no writes") != 0 ||
        check(fake.read_count == 3U, "probe should read three registers") !=
            0 ||
        check(probe.mode1 == 0xa1U && probe.mode2 == 0x04U &&
                  probe.prescale == 0x79U,
              "probe values") != 0) {
        return -1;
    }
    return 0;
}

static int test_failure_attempts_fail_off(void)
{
    struct fake_io fake;
    struct pca9685 device;
    size_t index;
    int saw_final_off = 0;

    bind_fake(&fake, &device);
    /* First write succeeds; fail one channel-clearing write; cleanup succeeds. */
    fake.fail_once_at = 3U;
    if (check(pca9685_initialize(&device, 50U) != 0,
              "injected initialization failure") != 0) {
        return -1;
    }
    for (index = 1U; index < fake.write_count; ++index) {
        if (fake.writes[index].reg == 0xfdU &&
            fake.writes[index].values[0] == 0x10U) {
            saw_final_off = 1;
        }
    }
    return check(saw_final_off, "failure must attempt global FULL_OFF");
}

int main(void)
{
    if (test_conversion() != 0 || test_initialize_and_pulse() != 0 ||
        test_probe_is_read_only() != 0 ||
        test_failure_attempts_fail_off() != 0) {
        return 1;
    }
    (void)printf("test_pca9685: OK\n");
    return 0;
}
