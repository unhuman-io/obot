#!/bin/sh
# Load obot-controller firmware onto a motor board over USB, unless it already runs it.
# Runs on the robot as root (deploy.sh --firmware BOARD=PACKAGE does that).
#
#   firmware-update.sh [--force] BOARD PACKAGE.tgz
#
#   BOARD    the board's name as motor_util lists it, or its serial number (the serial
#            number is the one to use for a board that still has another name)
#   PACKAGE  a package from the obot-controller build (<config>.tgz with
#            <config>_noparam.bin, <config>_param.bin, <config>_calibration.bin)
#
# "Up to date" means: the package's contents are the ones this script last loaded onto
# that board, and the board still identifies itself over USB with the version string it
# had right after that load (both recorded under /var/lib/obot/firmware). If the board
# also answers its text interface, the configuration name it reports has to match too.
# Before a board is written for the first time its whole flash is saved there.
# obot-rtd is stopped while a board is being written.
#
# Needs motor_util and dfu-util. Exit status: 0 up to date or loaded, 1 failed.
set -eu
force=0
[ "${1:-}" = "--force" ] && { force=1; shift; }
board="${1:?usage: firmware-update.sh [--force] BOARD PACKAGE.tgz}"
package="${2:?usage: firmware-update.sh [--force] BOARD PACKAGE.tgz}"
state="${OBOT_FIRMWARE_STATE:-/var/lib/obot/firmware}"
flash_size=0x80000          # STM32G474xE: 512 KiB from 0x08000000
say() { echo "firmware $board: $*"; }
die() { echo "firmware $board: $*" >&2; exit 1; }
lower() { echo "$1" | tr 'A-Z' 'a-z'; }

command -v motor_util >/dev/null || die "motor_util is not installed"
command -v dfu-util >/dev/null || die "dfu-util is not installed"
[ -f "$package" ] || die "no such package: $package"

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
tar -xzmf "$package" -C "$work"
config="$(ls "$work" | head -1)"
dir="$work/$config"
[ -f "$dir/${config}_noparam.bin" ] && [ -f "$dir/${config}_param.bin" ] ||
    die "$package does not look like an obot-controller package"
want="$(cat "$dir/${config}_noparam.bin" "$dir/${config}_param.bin" "$dir/${config}_calibration.bin" 2>/dev/null | sha256sum | cut -d' ' -f1)"

# The serial number as each tool spells it: firmware versions differ in letter case.
dfu_serial() {   # $1: Runtime or DFU; prints the exact serial string of our board in that mode
    dfu-util -l 2>/dev/null | grep "Found $1" | grep -io "serial=\"$serial\"" | head -1 | cut -d'"' -f2
}
wait_for() {     # $1: description, rest: command that succeeds when the condition holds
    what="$1"; shift
    i=0
    until "$@" >/dev/null 2>&1; do
        i=$((i + 1)); [ "$i" -gt 40 ] && die "timed out waiting for $what"
        sleep 0.5
    done
}

# Find the board: by name or by serial number among the running boards, or by serial
# number in the bootloader (a load that was interrupted leaves it there).
serial=""
names="$(motor_util --list-names-only 2>/dev/null || true)"
serials="$(motor_util --list-serial-number-only 2>/dev/null || true)"
n=1
for name in $names; do
    s="$(echo "$serials" | tr ' ' '\n' | sed -n "${n}p")"
    if [ "$name" = "$board" ] || [ "$(lower "$s")" = "$(lower "$board")" ]; then serial="$(lower "$s")"; fi
    n=$((n + 1))
done
running=1
if [ -z "$serial" ]; then
    serial="$(lower "$board")"
    [ -n "$(dfu_serial DFU)" ] || die "no board with that name or serial number is connected (motor_util lists: ${names:-none})"
    running=0
fi

# The firmware version string the board presents in its USB descriptors (always there,
# unlike answers from the text interface).
usb_version() {
    for dev in "${OBOT_SYSFS:-/sys/class/usbmisc}"/mtr* "${OBOT_SYSFS:-/sys/class/usbmisc}"/usbrt*; do
        [ -r "$dev/device/../serial" ] || continue
        if [ "$(lower "$(cat "$dev/device/../serial")")" = "$serial" ]; then
            cat "$dev/device/../configuration" 2>/dev/null
            return 0
        fi
    done
}

# One value from the board's text interface. Some boards answer it slowly or only now and
# then, so an empty answer is asked for again a few times. The version check is off: the
# question has to work on firmware of any age.
ask() {
    tries=0
    while [ "$tries" -lt 4 ]; do
        for spelling in "$serial" "$(echo "$serial" | tr 'a-z' 'A-Z')"; do
            answer="$(motor_util -c none --no-list -s "$spelling" --set-api "$1" 2>/dev/null | grep -v "^ *$" | tail -1 | sed 's/.*: *//')"
            case "$answer" in "" | *xception* | *what\(\)*) ;; *) echo "$answer"; return 0 ;; esac
        done
        tries=$((tries + 1))
        sleep 0.5
    done
    return 0
}

record="$state/$serial.loaded"
if [ "$running" = 1 ] && [ "$force" = 0 ]; then
    has="$(ask config)"
    if [ "$(sed -n 1p "$record" 2>/dev/null)" = "$want" ] && [ "$(sed -n 2p "$record" 2>/dev/null)" = "$(usb_version)" ] &&
        { [ -z "$has" ] || [ "$has" = "$config" ]; }; then
        say "up to date ($config)"
        exit 0
    fi
    say "runs '${has:-$(usb_version)}', loading '$config'"
else
    say "loading '$config'"
fi

mkdir -p "$state"
restart=0
if systemctl is-active --quiet obot-rtd 2>/dev/null; then
    systemctl stop obot-rtd
    restart=1
fi
# However this ends, the daemon that was running is started again.
trap 'rm -rf "$work"; [ "$restart" = 1 ] && systemctl start obot-rtd' EXIT

if [ "$running" = 1 ]; then
    exact="$(dfu_serial Runtime)"
    [ -n "$exact" ] || die "the board does not offer firmware update over USB"
    dfu-util -e -S "$exact" >/dev/null 2>&1 || true
fi
wait_for "the bootloader" sh -c "dfu-util -l 2>/dev/null | grep 'Found DFU' | grep -qi 'serial=\"$serial\"'"
exact="$(dfu_serial DFU)"

if ! ls "$state"/"$serial".backup-*.bin >/dev/null 2>&1; then
    backup="$state/$serial.backup-$(date +%Y%m%d-%H%M%S).bin"
    dfu-util -a0 -s "0x08000000:$flash_size" -U "$backup" -S "$exact" >/dev/null 2>&1 ||
        die "could not save the board's current flash; nothing was written"
    say "previous contents saved to $backup"
fi

download() {   # $1: address, $2: file
    [ -s "$2" ] || return 0
    dfu-util -a0 -s "$1" -D "$2" -S "$exact" 2>&1 | grep -q "File downloaded successfully" ||
        die "writing $(basename "$2") failed; the board is still in its bootloader, run this again"
}
rm -f "$record"
download 0x08000000 "$dir/${config}_noparam.bin"
download 0x08060000 "$dir/${config}_param.bin"
download 0x08070000 "$dir/${config}_calibration.bin"
# Leaving the bootloader is done by a zero length transfer with the "leave" modifier.
dfu-util -a0 -s 0x08000000:8:leave -U "$work/leave" -S "$exact" >/dev/null 2>&1 || true

wait_for "the board to restart" sh -c "motor_util --list-serial-number-only 2>/dev/null | tr 'A-Z' 'a-z' | grep -qw '$serial'"
now="$(ask config)"
[ -z "$now" ] || [ "$now" = "$config" ] || die "after loading, the board reports '$now' instead of '$config'"
printf '%s\n%s\n' "$want" "$(usb_version)" > "$record"
if [ -n "$now" ]; then
    say "loaded $config ($(usb_version))"
else
    say "loaded $config ($(usb_version)); the board's text interface did not answer, so the configuration name is unconfirmed"
fi
exit 0
