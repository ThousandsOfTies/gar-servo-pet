# gar-servo-pet

`gar-servoctl` is the dependency-minimal Linux control application for
GarServoPet's PCA9685 and four servos. It uses the Linux `I2C_RDWR` interface
directly; Python and `i2c-tools` are not runtime dependencies.

This repository contains reusable application and motion logic only. Board
wiring and measured mechanical calibration belong to the parent Product
repository under `GarServoPet/hardware`.

## Safety model

The controller is deliberately conservative:

- `validate` only parses files and performs no I2C access.
- `probe` only reads PCA9685 registers and performs no I2C writes.
- Before changing PCA9685 clock settings, the first write broadcasts
  `FULL_OFF`. All four registers of all 16 real channels are then reset to a
  known `FULL_OFF` state. The write-only `ALL_LED_*` registers are never read
  or used to re-enable output; writing one selected channel enables only that
  channel, so a stale register cannot restart another servo.
- `neutral` and `move` reject every servo whose `calibrated` field is not `1`.
- `calibrate` requires the requested pulse to be entered twice identically
  before configuration or I2C access begins.
- Calibration and stored motion are constrained to an absolute 500..2500 us
  software envelope; this only blocks gross input mistakes and is not a claim
  that every pulse in that range is mechanically safe.
- `move` requires an explicit normalized starting position and acknowledgement
  that the mechanism is actually there, then maps both endpoints through
  measured limits and enforces `max_rate_us_per_s` over that ramp.
- Every motion has a deadman timeout and ends in global `FULL_OFF`, including
  `SIGINT`, `SIGTERM`, and I2C error paths. The hard limits are 10 seconds for a
  calibration pulse, 30 seconds for neutral/move hold, and 60 seconds for a
  move.
- Multi-servo neutral requires both the literal `all` target and `--ack-all`.
  Starts are staggered by one 50 Hz frame, but this does not make an undersized
  servo power supply safe.
- `air-walk` is an unloaded four-arm commissioning sequence, not a validated
  ground gait. It requires exactly one calibrated `front-left`, `front-right`,
  `rear-left`, and `rear-right` role plus `--ack-all`,
  `--ack-neutral-known`, and `--body-lifted`. Its normalized amplitude is
  capped at 250, cycles at five, transition time at 250..2000 ms, pause time
  at 500 ms, and total runtime at 30 seconds. It returns to neutral and
  disables every output.
- A non-blocking advisory lock prevents two `gar-servoctl` processes from
  issuing motion on the same Linux I2C device at once. `off` first creates a
  `/run/gar-servo-pet.stop` cancellation marker. If a motion process owns the
  lock, `off` uses a safety-only unlocked path to broadcast `FULL_OFF` for six
  consecutive 20 ms frames; the owner checks the marker before every pulse and
  exits through its own fail-off path. The marker remains latched until `off`
  acquires the exclusive lock, writes a final `FULL_OFF`, and clears it; new
  motion refuses to start while it is latched. Thus a second terminal can stop
  a held or ramping command without waiting for its deadman timeout.

`FULL_OFF` removes the PWM signal and therefore holding torque. It does not
disconnect the external servo power supply.

Before any motion, power servo `V+` from a suitably rated external 5 V supply,
join that supply's ground to the board/PCA9685 ground, and confirm polarity.
Do not power four servos from a controller board's header 5 V pin. During
calibration, remove horns or lift the mechanism so an unexpected endpoint
cannot bind it.

## Configuration contract

The application never embeds a bus number, I2C address, channel assignment,
direction, pulse limit, or mechanical role. It reads two Product-owned CSV
files.

`i2c.csv` must contain the columns below. Exactly one row must have
`driver=pca9685`.

```csv
name,bus,dev,address,driver,sim,description
```

`servo-calibration.csv` must use this schema:

```csv
name,channel,role,direction,min_us,neutral_us,max_us,max_rate_us_per_s,calibrated
```

The `name` and `channel` fields are required and unique. For an uncalibrated
servo, leave the measured fields empty and set `calibrated=0`. Once measured,
set `direction` to `1` or `-1`, require
`min_us < neutral_us < max_us`, record a positive `max_rate_us_per_s`, and only
then set `calibrated=1`. Pulses must stay within 500..2500 us and the configured
rate is capped at 5000 us/s so a misplaced digit cannot disable the ramp guard.

Configuration paths can be passed explicitly:

```sh
gar-servoctl \
  --i2c-config /path/to/i2c.csv \
  --servo-config /path/to/servo-calibration.csv \
  validate
```

They can also be supplied as `GAR_I2C_CONFIG` and `GAR_SERVO_CONFIG`.
`GAR_SERVO_STOP_FILE` overrides the stop-marker path for a sandbox or test;
the root-run commissioning wrapper normally uses the `/run` default. A future
non-root service must provide a protected writable runtime directory and point
this variable into it before motion is enabled.

## Commands

```text
validate
probe
off [all|SERVO]
calibrate --channel N --pulse-us N --confirm-pulse-us N --hold-ms N --ack-motion
neutral SERVO --hold-ms N
neutral all --hold-ms N --ack-all
move SERVO --from-position -1000..1000 --position -1000..1000 \
  --duration-ms N --hold-ms N --ack-position-known
air-walk --amplitude 1..250 --cycles 1..5 --step-ms 250..2000 \
  --pause-ms 0..500 --ack-all --ack-neutral-known --body-lifted
```

Recommended commissioning order:

```sh
./run validate
./run probe
./run off all
./run calibrate --channel "$CHANNEL" --pulse-us "$TEST_PULSE_US" \
  --confirm-pulse-us "$TEST_PULSE_US" --hold-ms 500 --ack-motion
```

`--ack-motion` is mandatory because `calibrate` bypasses measured servo limits.
The pulse must also be independently entered in `--pulse-us` and
`--confirm-pulse-us`; a mismatch is rejected before the I2C device is opened.
The command still accepts only a declared channel, limits the pulse to the
500..2500 us software envelope, and turns all outputs off after the requested
hold time. Start at 1500 us with the horn removed or mechanism unloaded, then
approach endpoints in small increments; do not jump directly to 500 or 2500 us.

After recording measured values in the Product configuration:

```sh
./run neutral "$SERVO_NAME" --hold-ms 500
./run move "$SERVO_NAME" --from-position 0 --position 250 \
  --duration-ms 1000 --hold-ms 200 --ack-position-known
./run off all
```

A `move` first commands the declared `--from-position`, ramps to the target,
holds for the finite interval, and disables output. Because PWM is off between
processes, software cannot observe the physical starting angle. Use
`--ack-position-known` only when the unloaded mechanism was just placed at that
position (for example, immediately after a neutral command). A wrong starting
claim can still cause an abrupt jump; this first CLI intentionally does not
guess or persist physical state.

After all four installed arms have been calibrated, explicitly place them at
neutral and keep the body lifted before the first coordinated test:

```sh
./run neutral all --hold-ms 1000 --ack-all
./run air-walk --amplitude 100 --cycles 1 --step-ms 750 --pause-ms 150 \
  --ack-all --ack-neutral-known --body-lifted
./run off
```

The diagonal pairs alternate continuously, then ramp back to neutral before
PWM is disabled. Do not treat a successful lifted-body test as evidence that
the one-degree-of-freedom arms can walk safely or effectively on a surface.

## Build and test

Host build and deterministic tests:

```sh
make check
```

The tests use fake I2C and clock backends. They cover CSV validation, normalized
motion mapping, rate limits, deadman/fail-off behavior, first-write safety,
PCA9685 prescale, and the 1500 microsecond to 307-count conversion at 50 Hz.

Cross-build contract used by the Product build hook:

```sh
make target-build TARGET_CC=aarch64-linux-gnu-gcc
```

The result is `build/target/gar-servoctl`. A deployed application directory is
laid out as follows:

```text
run
gar-servoctl
hardware/
  i2c.csv
  servo-calibration.csv
```

The source-controlled `run` wrapper resolves this directory independent of the
caller's current working directory and supplies both CSV paths. They remain
overridable through the environment. With an explicit argument, `run` executes
one bounded CLI command as shown above. With no arguments, it first runs the
non-I2C `validate` command and then waits as a non-actuating service so a
physical Target lifecycle can supervise it safely. SIGTERM, SIGINT, or SIGHUP
ends that wait; no PWM output is enabled by the service mode.

## Repository boundary

```text
gar-servo-pet              application code and motion safety
GarServoPet/hardware       Product-specific wiring and measured calibration
gar-tools/targets/*        reusable Target Pack support
GaplessAgentRuntime        build, artifact, simulation, deployment contracts
```
