#!/bin/sh
# Build the daemon for an aarch64 robot and install it over ssh.
#
#   deploy/deploy.sh user@robot config/obot1.toml [control cpu] [--sim]
#
# Needs on this machine: cmake, ninja, g++-aarch64-linux-gnu. On the robot: ssh access and
# sudo. With --sim the service runs on simulated actuators, which is the way to try a
# board that has nothing connected to it.
set -eu
target="${1:?usage: deploy.sh user@host <robot.toml> [control cpu] [--sim]}"
config="${2:?robot configuration, e.g. config/obot1.toml}"
cpu="${3:--1}"
extra="${4:-}"
repo="$(cd "$(dirname "$0")/.." && pwd)"

echo "== what is $target?"
ssh "$target" 'cat /proc/device-tree/model 2>/dev/null; echo; cat /etc/nv_tegra_release 2>/dev/null | head -1;
               uname -srm; nproc; python3 --version 2>&1; cat /sys/kernel/realtime 2>/dev/null || echo "not PREEMPT_RT"'

echo "== building static aarch64 programs"
build="${OBOT_DEPLOY_BUILD:-$HOME/.cache/obot-build/deploy-aarch64}"
cmake -S "$repo/rt" -B "$build" -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
    -DOBOT_RT_STATIC=ON -DOBOT_RT_BUILD_TESTS=OFF >/dev/null
cmake --build "$build"

stage="$(mktemp -d)"
trap 'rm -rf "$stage"' EXIT
mkdir -p "$stage/obot/bin" "$stage/obot/description"
cp "$build/obot-rtd" "$build/obot-sim-node" "$build/obot-rt-latency" "$stage/obot/bin/"
cp "$repo"/deploy/install.sh "$repo"/deploy/obot-rtd.service "$stage/obot/"
cp "$repo/$config" "$stage/obot/robot.toml" 2>/dev/null || cp "$config" "$stage/obot/robot.toml"
cp -r "$repo"/description/* "$stage/obot/description/"
tar -C "$stage" -czf "$stage/obot.tar.gz" obot

echo "== installing on $target"
scp -q "$stage/obot.tar.gz" "$target:/tmp/obot.tar.gz"
ssh -t "$target" "rm -rf /tmp/obot && tar -C /tmp -xzf /tmp/obot.tar.gz && sudo sh /tmp/obot/install.sh /tmp/obot/robot.toml '$cpu' '$extra' && sudo systemctl restart obot-rtd && sleep 2 && systemctl --no-pager status obot-rtd | head -12"
