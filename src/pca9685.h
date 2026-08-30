#ifndef GAR_SERVO_PET_PCA9685_H
#define GAR_SERVO_PET_PCA9685_H

#include <stddef.h>
#include <stdint.h>

#define PCA9685_CHANNEL_COUNT 16U
#define PCA9685_DEFAULT_OSCILLATOR_HZ 25000000U

struct pca9685_io {
    void *context;
    int (*read_reg)(void *context, uint8_t reg, uint8_t *value);
    int (*write_reg)(void *context, uint8_t reg, uint8_t value);
    int (*write_regs)(void *context, uint8_t start_reg, const uint8_t *values,
                      size_t value_count);
    int (*sleep_us)(void *context, uint32_t microseconds);
};

struct pca9685 {
    struct pca9685_io io;
    uint32_t oscillator_hz;
    uint16_t frequency_hz;
};

struct pca9685_linux {
    int fd;
    uint8_t address;
};

struct pca9685_probe {
    uint8_t mode1;
    uint8_t mode2;
    uint8_t prescale;
};

int pca9685_linux_open(struct pca9685_linux *linux_io, const char *device,
                       uint8_t address, char *error, size_t error_size);
int pca9685_linux_open_unlocked(struct pca9685_linux *linux_io,
                                const char *device, uint8_t address,
                                char *error, size_t error_size);
void pca9685_linux_close(struct pca9685_linux *linux_io);
void pca9685_linux_bind(struct pca9685_linux *linux_io, struct pca9685 *device);

int pca9685_probe_readonly(struct pca9685 *device, struct pca9685_probe *probe);
int pca9685_initialize(struct pca9685 *device, uint16_t frequency_hz);
int pca9685_all_off(struct pca9685 *device);
int pca9685_channel_off(struct pca9685 *device, uint8_t channel);
int pca9685_set_pulse_us(struct pca9685 *device, uint8_t channel,
                         uint32_t pulse_us);
uint16_t pca9685_pulse_us_to_count(uint32_t pulse_us, uint16_t frequency_hz);
uint8_t pca9685_frequency_to_prescale(uint32_t oscillator_hz,
                                      uint16_t frequency_hz);

#endif
