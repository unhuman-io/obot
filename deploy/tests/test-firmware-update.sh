#!/bin/sh
# firmware-update.sh against stand-ins for motor_util, dfu-util and systemctl: one fake
# board whose state (running firmware, bootloader or not, flash contents) lives in files.
set -eu
here="$(cd "$(dirname "$0")/.." && pwd)"
t="$(mktemp -d)"
trap 'rm -rf "$t"' EXIT
mkdir -p "$t/bin" "$t/state" "$t/pkg/motor_test" "$t/pkg2/motor_test"
export BOARD_DIR="$t/board" OBOT_FIRMWARE_STATE="$t/state" PATH="$t/bin:$PATH"
mkdir -p "$BOARD_DIR"
echo old_config > "$BOARD_DIR/config"; echo runtime > "$BOARD_DIR/mode"; : > "$BOARD_DIR/calls"

cat > "$t/bin/motor_util" <<'STUB'
#!/bin/sh
[ "$(cat "$BOARD_DIR/mode")" = runtime ] || exit 0          # in the bootloader: no board listed
case "$*" in
    *--list-names-only*) echo wl ;;
    *--list-serial-number-only*) echo 206C36793032 ;;
    *"--set-api config"*) echo "wl: $(cat "$BOARD_DIR/config")" ;;
    *"--set-api version"*) echo "wl: v1.2-3" ;;
esac
STUB
cat > "$t/bin/dfu-util" <<'STUB'
#!/bin/sh
echo "$*" >> "$BOARD_DIR/calls"
mode="$(cat "$BOARD_DIR/mode")"
case "$*" in
    -l) if [ "$mode" = runtime ]; then echo 'Found Runtime: [3293:0100] name="ST DFU mode", serial="206C36793032"'
        else echo 'Found DFU: [0483:df11] alt=0, name="@Internal Flash", serial="206C36793032"'; fi ;;
    -e*) echo dfu > "$BOARD_DIR/mode" ;;
    *:leave*) echo runtime > "$BOARD_DIR/mode" ;;
    *-U*) [ "$mode" = dfu ] || exit 1; f="$(echo "$*" | sed 's/.*-U \([^ ]*\).*/\1/')"; echo flash-image > "$f" ;;
    *-D*) [ "$mode" = dfu ] || exit 1
          [ -e "$BOARD_DIR/fail-download" ] && exit 1
          case "$*" in *_noparam.bin*) echo motor_test > "$BOARD_DIR/config" ;; esac
          echo "File downloaded successfully" ;;
esac
STUB
cat > "$t/bin/systemctl" <<'STUB'
#!/bin/sh
echo "systemctl $*" >> "$BOARD_DIR/calls"
[ "$1" = is-active ] && { [ -e "$BOARD_DIR/rtd-active" ]; exit $?; }
exit 0
STUB
chmod +x "$t/bin/"*

make_pkg() {   # $1: dir, $2: firmware bytes
    echo "$2" > "$1/motor_test/motor_test_noparam.bin"
    echo param > "$1/motor_test/motor_test_param.bin"
    : > "$1/motor_test/motor_test_calibration.bin"
    tar -C "$1" -czf "$1.tgz" motor_test
}
make_pkg "$t/pkg" firmware-one
make_pkg "$t/pkg2" firmware-two
fail() { echo "FAIL: $*" >&2; cat "$BOARD_DIR/calls" >&2; exit 1; }
run() { sh "$here/firmware-update.sh" "$@" > "$t/out" 2>&1; }

touch "$BOARD_DIR/rtd-active"
run wl "$t/pkg.tgz" || fail "first load failed: $(cat "$t/out")"
grep -q "loaded motor_test v1.2-3" "$t/out" || fail "no load reported"
grep -q "previous contents saved" "$t/out" || fail "no backup on the first write"
ls "$t/state"/206c36793032.backup-*.bin >/dev/null || fail "backup file missing"
[ "$(grep -c -- "-D" "$BOARD_DIR/calls")" = 2 ] || fail "expected firmware and parameters to be written (not the empty calibration)"
grep -q "0x08000000 -D" "$BOARD_DIR/calls" && grep -q "0x08060000 -D" "$BOARD_DIR/calls" || fail "wrong addresses"
[ "$(grep -n 'systemctl stop obot-rtd' "$BOARD_DIR/calls" | cut -d: -f1)" -lt "$(grep -n -- '-D' "$BOARD_DIR/calls" | head -1 | cut -d: -f1)" ] ||
    fail "the daemon was not stopped before writing"
grep -q "systemctl start obot-rtd" "$BOARD_DIR/calls" || fail "the daemon was not started again"

: > "$BOARD_DIR/calls"
run wl "$t/pkg.tgz" || fail "second run failed"
grep -q "up to date" "$t/out" || fail "second run should be a no-op: $(cat "$t/out")"
grep -q -- "-D\|-e\|systemctl stop" "$BOARD_DIR/calls" && fail "an up to date board was touched"

run 206C36793032 "$t/pkg.tgz" && grep -q "up to date" "$t/out" || fail "lookup by serial number"

: > "$BOARD_DIR/calls"
run wl "$t/pkg2.tgz" || fail "load of a changed package failed"
grep -q "loaded motor_test" "$t/out" || fail "a changed package must be loaded"
[ "$(ls "$t/state"/206c36793032.backup-*.bin | wc -l)" = 1 ] || fail "only the first write is backed up"

: > "$BOARD_DIR/calls"
run --force wl "$t/pkg2.tgz" && grep -q "loaded" "$t/out" || fail "--force must load"

# A failed write is an error, leaves no "loaded" record, and the next run recovers from the bootloader.
touch "$BOARD_DIR/fail-download"
run --force wl "$t/pkg.tgz" && fail "a failed write must be an error"
[ ! -e "$t/state/206c36793032.loaded" ] || fail "record kept after a failed write"
rm "$BOARD_DIR/fail-download"
run 206c36793032 "$t/pkg.tgz" || fail "recovery from the bootloader failed: $(cat "$t/out")"
grep -q "loaded motor_test" "$t/out" || fail "recovery did not load"

run nosuchboard "$t/pkg.tgz" && fail "an unknown board must be an error"
grep -q "no board with that name" "$t/out" || fail "unhelpful message for an unknown board"
echo "firmware-update: all checks passed"
