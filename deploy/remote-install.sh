#!/bin/sh
# Runs on the robot as root (deploy.sh copies the bundle to /tmp/obot and calls this).
# Safe to run again: every step leaves the same result.
#   OBOT_CPU      CPU for the control thread, -1 for none
#   OBOT_ARGS     extra obot-rtd arguments, e.g. --sim
#   OBOT_ISOLATE  1: reserve OBOT_CPU on the kernel command line (takes effect after a reboot)
#   OBOT_FORCE_CONFIG  1: replace an existing /etc/obot/robot.toml
#   OBOT_TOOLS    1: install the Python tools and the operator page service
set -eu
here="$(cd "$(dirname "$0")" && pwd)"
user="${SUDO_USER:?run with sudo from the login user}"
home="$(getent passwd "$user" | cut -d: -f6)"
cpu="${OBOT_CPU:--1}"

echo "== programs"
install -d /usr/local/bin /etc/obot
systemctl stop obot-rtd 2>/dev/null || true
rm -f /dev/shm/obot*.lock   # lock files that earlier versions of the daemon left behind
install -m 0755 "$here"/bin/* /usr/local/bin/
rm -rf /etc/obot/description && cp -r "$here/description" /etc/obot/description
if [ ! -e /etc/obot/robot.toml ] || [ "${OBOT_FORCE_CONFIG:-0}" = 1 ]; then
    [ -e /etc/obot/robot.toml ] && cp /etc/obot/robot.toml /etc/obot/robot.toml.previous
    install -m 0644 "$here/robot.toml" /etc/obot/robot.toml
elif ! cmp -s "$here/robot.toml" /etc/obot/robot.toml; then
    install -m 0644 "$here/robot.toml" /etc/obot/robot.toml.new
    echo "kept the existing /etc/obot/robot.toml; the deployed one is robot.toml.new (--force-config replaces)"
fi
printf 'OBOT_RT_CPU=%s\nOBOT_RTD_ARGS=%s\n' "$cpu" "${OBOT_ARGS:-}" > /etc/default/obot-rtd
obot-rtd --check --config /etc/obot/robot.toml

getent group obot >/dev/null || groupadd --system obot
id obot >/dev/null 2>&1 || useradd --system --no-create-home --gid obot --shell /usr/sbin/nologin obot
usermod -aG obot "$user"
install -m 0644 "$here/obot-rtd.service" /etc/systemd/system/obot-rtd.service

if [ "${OBOT_TOOLS:-1}" = 1 ]; then
    echo "== python tools"
    # The OS's own Python may be too old (3.6 on L4T R32); uv brings one.
    sudo -u "$user" -H sh -c '
        set -e
        [ -x "$HOME/.local/bin/uv" ] || curl -LsSf https://astral.sh/uv/install.sh | sh >/dev/null 2>&1
        [ -x "$HOME/obot-venv/bin/python" ] || "$HOME/.local/bin/uv" venv -q --python 3.12 "$HOME/obot-venv"
        install="$HOME/.local/bin/uv pip install -q -p $HOME/obot-venv --reinstall-package obot '"$here"'/python[ui,log]"
        # A robot without internet access can still be updated from what uv has cached.
        $install 2>/dev/null || $install --offline
    '
    sed "s|OBOT_USER|$user|g; s|/home/$user|$home|g" "$here/obot-ui.service" > /etc/systemd/system/obot-ui.service
fi

reboot_needed=0
if [ "${OBOT_ISOLATE:-0}" = 1 ] && [ "$cpu" -ge 0 ]; then
    echo "== kernel command line"
    conf=/boot/extlinux/extlinux.conf
    want="isolcpus=$cpu nohz_full=$cpu rcu_nocbs=$cpu"
    if [ ! -f "$conf" ]; then
        echo "no $conf: add '$want' to the kernel command line by hand" >&2
    elif ! grep -q "isolcpus=$cpu " "$conf" && ! grep -q "isolcpus=$cpu\$" "$conf"; then
        [ -e "$conf.before-obot" ] || cp "$conf" "$conf.before-obot"
        # Drop any earlier isolation arguments, then add ours to every active APPEND line.
        sed -i -E '/^[[:space:]]*APPEND/ { s/ (isolcpus|nohz_full|rcu_nocbs)=[^ ]*//g; s/[[:space:]]*$/ '"$want"'/ }' "$conf"
        echo "set: $want (original kept as $conf.before-obot)"
    fi
    grep -q "isolcpus=$cpu" /proc/cmdline || reboot_needed=1
fi

install -m 0755 "$here/firmware-update.sh" /usr/local/bin/obot-firmware-update
if [ -s "$here/firmware/manifest" ]; then
    echo "== motor board firmware"
    # obot-rtd is stopped at this point, so nothing else is talking to the boards.
    # A board that could not be loaded is reported at the end; the services still start.
    while read -r board package; do
        sh "$here/firmware-update.sh" "$board" "$here/firmware/$package" || touch "$here/firmware/failed"
    done < "$here/firmware/manifest"
fi

systemctl daemon-reload
systemctl enable -q obot-rtd
systemctl restart obot-rtd
if [ "${OBOT_TOOLS:-1}" = 1 ]; then
    systemctl enable -q obot-ui
    systemctl restart obot-ui
fi
[ "$reboot_needed" = 1 ] && echo "REBOOT NEEDED for the CPU isolation to take effect"
if [ -e "$here/firmware/failed" ]; then
    echo "FIRMWARE: at least one motor board could not be loaded (see above)" >&2
    exit 1
fi
exit 0
