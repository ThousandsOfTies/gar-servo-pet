#!/bin/sh
set -eu

run=${1:?run wrapper path required}
test_root=$(mktemp -d /tmp/gar-servo-run-test.XXXXXX)
service_pid=""
cleanup() {
    if [ -n "$service_pid" ]; then
        kill "$service_pid" 2>/dev/null || true
        wait "$service_pid" 2>/dev/null || true
    fi
    rm -rf -- "$test_root"
}
trap cleanup EXIT HUP INT TERM

fake="$test_root/gar-servoctl"
calls="$test_root/calls"
cat >"$fake" <<'EOF'
#!/bin/sh
printf '%s\n' "$*" >>"$GAR_TEST_CALLS"
if [ "${GAR_TEST_VALIDATE_FAILURE:-0}" = 1 ] && [ "${1:-}" = validate ]; then
    exit 7
fi
EOF
chmod 0755 "$fake"

GAR_SERVOCTL_BIN="$fake" GAR_TEST_CALLS="$calls" "$run" &
service_pid=$!
attempt=0
while [ ! -s "$calls" ] && [ "$attempt" -lt 20 ]; do
    sleep 0.05
    attempt=$((attempt + 1))
done
[ "$(sed -n '1p' "$calls")" = validate ]
kill -TERM "$service_pid"
wait "$service_pid"
service_pid=""

GAR_SERVOCTL_BIN="$fake" GAR_TEST_CALLS="$calls" "$run" probe
[ "$(sed -n '2p' "$calls")" = probe ]

set +e
GAR_SERVOCTL_BIN="$fake" GAR_TEST_CALLS="$calls" GAR_TEST_VALIDATE_FAILURE=1 \
    "$run"
status=$?
set -e
[ "$status" -eq 7 ]

echo "test_run: OK"
