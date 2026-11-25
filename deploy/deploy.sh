#!/bin/sh
# Build, install and verify the obot software on an aarch64 robot over ssh.
#
#   deploy/deploy.sh [options] user@robot config/obot1.toml
#
#   --cpu N          CPU for the control thread (default: none)
#   --isolate        reserve that CPU on the kernel command line (isolcpus, nohz_full, rcu_nocbs)
#   --reboot         reboot if the kernel command line changed, then verify
#   --sim            run on simulated actuators (a board with nothing connected)
#   --force-config   replace the robot's existing /etc/obot/robot.toml
#   --no-tools       daemon only: no Python tools, no operator page
#   --firmware BOARD=PACKAGE.tgz
#                    load this obot-controller firmware package onto the motor board with
#                    that name or serial number, unless it already runs it (repeatable)
#   --jump HOST      reach the robot through this ssh host
#   --verify-only    change nothing, only run the checks
#
# Needs here: cmake, ninja, git, g++-aarch64-linux-gnu (the first build fetches libudev-zero). On the robot: ssh with a key and
# passwordless sudo. Running it again is safe. Exit status is that of the checks.
set -eu
cpu=-1 isolate=0 reboot=0 args="" force=0 tools=1 jump="" verify_only=0 firmware=""
while [ $# -gt 0 ]; do
    case "$1" in
        --cpu) cpu="$2"; shift 2 ;;
        --isolate) isolate=1; shift ;;
        --reboot) reboot=1; shift ;;
        --sim) args="--sim"; shift ;;
        --force-config) force=1; shift ;;
        --no-tools) tools=0; shift ;;
        --jump) jump="$2"; shift 2 ;;
        --firmware) firmware="$firmware $2"; shift 2 ;;
        --verify-only) verify_only=1; shift ;;
        -*) echo "unknown option $1" >&2; exit 2 ;;
        *) break ;;
    esac
done
target="${1:?usage: deploy.sh [options] user@host <robot.toml>}"
config="${2:?robot configuration, e.g. config/obot1.toml}"
repo="$(cd "$(dirname "$0")/.." && pwd)"
opts="-o BatchMode=yes -o LogLevel=ERROR -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10"
[ -n "$jump" ] && opts="$opts -J $jump"
env="OBOT_CPU=$cpu OBOT_ISOLATE=$isolate OBOT_TOOLS=$tools OBOT_FORCE_CONFIG=$force OBOT_ARGS=$args"
run() { ssh $opts "$target" "$@"; }

verify() {
    echo "== checks on $target"
    scp -q $opts "$repo/deploy/verify.sh" "$target:/tmp/obot-verify.sh"
    run "$env sh /tmp/obot-verify.sh"
}
if [ "$verify_only" = 1 ]; then verify; exit $?; fi

echo "== $target"
run 'tr -d "\0" < /proc/device-tree/model 2>/dev/null; echo; head -1 /etc/nv_tegra_release 2>/dev/null; uname -srm'

echo "== building static aarch64 programs"
build="${OBOT_DEPLOY_BUILD:-$HOME/.cache/obot-build/deploy-aarch64}"
cmake -S "$repo/rt" -B "$build" -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
    -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
    -DOBOT_RT_STATIC=ON -DOBOT_RT_BUILD_TESTS=OFF >/dev/null
cmake --build "$build" | tail -1

stage="$(mktemp -d)"
trap 'rm -rf "$stage"' EXIT
mkdir -p "$stage/obot/bin"
cp "$build/obot-rtd" "$build/obot-sim-node" "$build/obot-rt-latency" "$stage/obot/bin/"
cp "$repo"/deploy/remote-install.sh "$repo"/deploy/obot-rtd.service "$repo"/deploy/obot-ui.service "$stage/obot/"
if [ -f "$config" ]; then cp "$config" "$stage/obot/robot.toml"; else cp "$repo/$config" "$stage/obot/robot.toml"; fi
cp -r "$repo/description" "$stage/obot/description"
mkdir "$stage/obot/python" && cp -r "$repo/python/src" "$repo/python/pyproject.toml" "$repo/python/README.md" "$stage/obot/python/"
mkdir "$stage/obot/firmware"
cp "$repo/deploy/firmware-update.sh" "$stage/obot/"
for entry in $firmware; do
    board="${entry%%=*}"; package="${entry#*=}"
    [ "$board" != "$entry" ] && [ -f "$package" ] || { echo "--firmware $entry: expected BOARD=PACKAGE.tgz, and the package must exist" >&2; exit 2; }
    cp "$package" "$stage/obot/firmware/"
    echo "$board $(basename "$package")" >> "$stage/obot/firmware/manifest"
done
find "$stage" -name __pycache__ -o -name "*.egg-info" | xargs rm -rf
git -C "$repo" describe --always --dirty > "$stage/obot/VERSION" 2>/dev/null || true
tar -C "$stage" -czf "$stage/obot.tar.gz" obot

echo "== installing ($(cat "$stage/obot/VERSION" 2>/dev/null))"
scp -q $opts "$stage/obot.tar.gz" "$target:/tmp/obot.tar.gz"
out="$(run "rm -rf /tmp/obot && tar -C /tmp -xzmf /tmp/obot.tar.gz && sudo $env sh /tmp/obot/remote-install.sh && sudo cp /tmp/obot/VERSION /etc/obot/VERSION" | tee /dev/stderr)"

if echo "$out" | grep -q "REBOOT NEEDED"; then
    if [ "$reboot" = 1 ]; then
        echo "== rebooting $target"
        run "sudo systemctl reboot" || true
        sleep 20
        i=0
        until run true 2>/dev/null; do
            i=$((i + 1)); [ $i -gt 40 ] && { echo "did not come back after the reboot" >&2; exit 1; }
            sleep 5
        done
        sleep 15   # services
    else
        echo "== not rebooting (pass --reboot): the isolation check below will fail until then"
    fi
fi
verify
