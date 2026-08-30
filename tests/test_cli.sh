#!/bin/sh
set -eu

binary=${1:?gar-servoctl path required}
unset GAR_I2C_CONFIG GAR_SERVO_CONFIG

run_rejected()
{
    set +e
    output=$("$binary" "$@" 2>&1)
    status=$?
    set -e
}

run_rejected calibrate --channel 0 --pulse-us 1500 --hold-ms 100 \
    --ack-motion
[ "$status" -eq 2 ]
case "$output" in
    *"requires --channel, --pulse-us, --confirm-pulse-us"*) ;;
    *)
        echo "FAIL: missing confirmation was not rejected by parser" >&2
        exit 1
        ;;
esac

run_rejected calibrate --channel 0 --pulse-us 1500 \
    --confirm-pulse-us 1501 --hold-ms 100 --ack-motion
[ "$status" -eq 2 ]
case "$output" in
    *"must exactly match --pulse-us; no motion was sent"*) ;;
    *)
        echo "FAIL: mismatched confirmation was not rejected before config/I2C" >&2
        exit 1
        ;;
esac

run_rejected neutral --hold-ms 100
[ "$status" -eq 2 ]
case "$output" in
    *"requires an explicit SERVO or all target"*) ;;
    *)
        echo "FAIL: neutral accepted an implicit all target" >&2
        exit 1
        ;;
esac

run_rejected neutral all --hold-ms 100
[ "$status" -eq 2 ]
case "$output" in
    *"neutral all requires --ack-all"*) ;;
    *)
        echo "FAIL: all-servo neutral did not require acknowledgement" >&2
        exit 1
        ;;
esac

run_rejected move servo-1 --from-position 0 --position 250 \
    --duration-ms 500 --hold-ms 0
[ "$status" -eq 2 ]
case "$output" in
    *"--ack-position-known"*) ;;
    *)
        echo "FAIL: move did not require start-position acknowledgement" >&2
        exit 1
        ;;
esac

run_rejected air-walk --amplitude 250 --cycles 1 --step-ms 500 \
    --pause-ms 100 --ack-all --ack-neutral-known
[ "$status" -eq 2 ]
case "$output" in
    *"air-walk requires"*"--body-lifted"*) ;;
    *)
        echo "FAIL: air-walk did not require every physical acknowledgement" >&2
        exit 1
        ;;
esac

run_rejected air-walk --amplitude 251 --cycles 1 --step-ms 500 \
    --pause-ms 100 --ack-all --ack-neutral-known --body-lifted
[ "$status" -eq 2 ]
case "$output" in
    *"--amplitude must be 1..250"*) ;;
    *)
        echo "FAIL: air-walk accepted an excessive amplitude" >&2
        exit 1
        ;;
esac

# Complete acknowledgements pass the parser and then fail because this
# isolated test intentionally supplies no Product configuration.
run_rejected air-walk --amplitude 100 --cycles 1 --step-ms 500 \
    --pause-ms 100 --ack-all --ack-neutral-known --body-lifted
[ "$status" -eq 1 ]
case "$output" in
    *"configuration path missing"*) ;;
    *)
        echo "FAIL: valid air-walk arguments did not reach configuration loading" >&2
        exit 1
        ;;
esac

run_rejected calibrate --channel 0 --pulse-us 499 \
    --confirm-pulse-us 499 --hold-ms 100 --ack-motion
[ "$status" -eq 2 ]
case "$output" in
    *"--pulse-us must be 500..2500"*) ;;
    *)
        echo "FAIL: out-of-envelope pulse was not rejected by parser" >&2
        exit 1
        ;;
esac

# Matching values pass the parser and then fail because this isolated test does
# not provide Product configuration. This distinguishes the acceptance path
# without opening an I2C device.
run_rejected calibrate --channel 0 --pulse-us 1500 \
    --confirm-pulse-us 1500 --hold-ms 100 --ack-motion
[ "$status" -eq 1 ]
case "$output" in
    *"configuration path missing"*) ;;
    *)
        echo "FAIL: matching confirmation did not pass the parser" >&2
        exit 1
        ;;
esac

echo "test_cli: OK"
