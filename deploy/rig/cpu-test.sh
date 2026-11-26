#!/bin/sh
# What the software costs in processor time, without installing anything: obot-rtd on
# simulated actuators and the page server with replayed cameras are started as the user this
# runs as, next to whatever else runs on the computer, measured (obot cpu-test), and stopped.
#
#   deploy/rig/cpu-test.sh BIN_DIR [options of obot cpu-test]
#
# BIN_DIR has obot-rtd and obot-navd for this computer. Needs curl or uv, and the internet
# the first time (Python and the packages are kept by uv after that).
set -eu
bin="$(cd "${1:?directory with obot-rtd and obot-navd}" && pwd)"
shift
repo="$(cd "$(dirname "$0")/../.." && pwd)"
port="${OBOT_RIG_PORT:-8091}"
shm="obot_rig_$$"
work="$(mktemp -d)"
rtd="" ui=""
cleanup() {
    [ -n "$ui" ] && kill "$ui" 2>/dev/null
    [ -n "$rtd" ] && kill "$rtd" 2>/dev/null
    wait 2>/dev/null || true
    rm -rf "$work" "/dev/shm/$shm"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

uv="$(command -v uv || echo "$HOME/.local/bin/uv")"
if [ ! -x "$uv" ]; then
    curl -LsSf https://astral.sh/uv/install.sh | sh >/dev/null 2>&1
    uv="$HOME/.local/bin/uv"
fi
"$uv" venv -q --python 3.12 "$work/venv"
"$uv" pip install -q -p "$work/venv" "$repo/python[ui]"

config="$repo/config/obot1_base.toml"
"$bin/obot-rtd" --config "$config" --no-rt --sim --shm-name "$shm" 2>"$work/rtd.log" &
rtd=$!
OBOT_NAVD="$bin/obot-navd" "$work/venv/bin/obot" --config "$config" --shm-name "$shm" ui --host 127.0.0.1 \
    --port "$port" --replay-cameras room >"$work/ui.log" 2>&1 &
ui=$!
n=0
until curl -fs -m 2 "http://127.0.0.1:$port/api/info" >/dev/null 2>&1; do
    n=$((n + 1))
    if [ "$n" -gt 60 ] || ! kill -0 "$ui" 2>/dev/null; then
        echo "the page server did not start:" >&2
        cat "$work/ui.log" "$work/rtd.log" >&2
        exit 2
    fi
    sleep 1
done
sleep 25   # the replayed cameras draw their pictures first, and that is not what is measured

status=0
"$work/venv/bin/obot" cpu-test --url "http://127.0.0.1:$port" "$@" || status=$?
if [ "$status" != 0 ]; then
    echo "--- the page server's log:" >&2
    tail -20 "$work/ui.log" >&2
fi
exit "$status"
