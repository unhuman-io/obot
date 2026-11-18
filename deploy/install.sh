#!/bin/sh
# Install obot-rtd as a systemd service on an existing Linux system.
#   sudo ./install.sh <robot.toml> [control cpu]
set -eu
here="$(cd "$(dirname "$0")" && pwd)"
config="${1:?usage: install.sh <robot.toml> [control cpu]}"
cpu="${2:--1}"
[ "$(id -u)" = 0 ] || { echo "run as root" >&2; exit 1; }

install -d /usr/local/bin /etc/obot
install -m 0755 "$here"/bin/obot-rtd "$here"/bin/obot-sim-node "$here"/bin/obot-rt-latency /usr/local/bin/
if [ -e /etc/obot/robot.toml ]; then
    echo "keeping the existing /etc/obot/robot.toml"
else
    install -m 0644 "$config" /etc/obot/robot.toml
fi
echo "OBOT_RT_CPU=$cpu" > /etc/default/obot-rtd

getent group obot >/dev/null || groupadd --system obot
id obot >/dev/null 2>&1 || useradd --system --no-create-home --gid obot --shell /usr/sbin/nologin obot

install -m 0644 "$here"/obot-rtd.service /etc/systemd/system/obot-rtd.service
systemctl daemon-reload
systemctl enable obot-rtd.service
echo "installed. Check with: obot-rtd --check --config /etc/obot/robot.toml"
echo "start with:           systemctl start obot-rtd"
