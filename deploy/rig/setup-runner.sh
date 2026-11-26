#!/bin/sh
# Makes this computer a GitHub Actions runner for one repository, for jobs that need a real
# robot computer (processor time, the GPU). Run on the rig as root, with a registration token
# for the repository on standard input:
#
#   gh api -X POST repos/OWNER/REPO/actions/runners/registration-token --jq .token |
#       ssh rig 'sudo sh -s -- OWNER/REPO' < deploy/rig/setup-runner.sh      (see docs/host.md)
#
# A runner executes what a workflow tells it to, so what matters is who can make a workflow
# run here and what a job can reach. This sets up:
#
#   - a user of its own for the runner (ghrunner): no sudo, no login, a home outside /home;
#   - a firewall rule for that user: nothing on the local network (other robots, their
#     operator pages, this computer's owner) can be reached from a job, only the internet
#     and this computer itself;
#   - the runner as a service that cannot see /home, cannot write outside its own directory,
#     and cannot gain privileges; it does not start without the firewall rule;
#   - a check before every job, on this computer and not in the repository: only pushes to,
#     and manual or scheduled runs in, this one repository. A pull request's job is refused
#     here whatever the repository's settings say;
#   - one label, obot-rig, and none of the usual ones: `runs-on: self-hosted` does not match.
#
# Not set up here, and needed as well: in the repository's Actions settings, approval for
# the workflows of all outside contributors, and a read-only default token.
# Running it again is safe: everything is written again; a runner that is registered stays so
# (and then no token is needed).
set -eu
repository="${1:?usage: setup-runner.sh OWNER/REPO [runner name] < registration-token}"
name="${2:-$(hostname)}"
version=2.338.2
sha256=670adb2f268fe21dcbb58b3db0e0aa9d5b3bddfe2c651481336e561ff86de945   # linux-arm64, from the release notes
user=ghrunner
home=/var/lib/ghrunner
read -r token || token=""
[ "$(uname -m)" = aarch64 ] || { echo "this is for an aarch64 computer" >&2; exit 2; }

id "$user" >/dev/null 2>&1 || useradd --system --create-home --home-dir "$home" --shell /usr/sbin/nologin "$user"
chmod 750 "$home"

# --- the firewall rule -----------------------------------------------------------------------
cat > /usr/local/sbin/obot-rig-firewall <<'FIREWALL'
#!/bin/sh
# Jobs of the CI runner (user ghrunner) reach the internet and this computer, nothing else.
set -eu
for tables in iptables ip6tables; do
    $tables -N OBOT_RIG 2>/dev/null || $tables -F OBOT_RIG
    $tables -C OUTPUT -m owner --uid-owner ghrunner -j OBOT_RIG 2>/dev/null ||
        $tables -I OUTPUT -m owner --uid-owner ghrunner -j OBOT_RIG
    $tables -A OBOT_RIG -o lo -j RETURN
done
for net in 10.0.0.0/8 172.16.0.0/12 192.168.0.0/16 169.254.0.0/16 100.64.0.0/10 224.0.0.0/4 255.255.255.255/32; do
    iptables -A OBOT_RIG -d "$net" -j REJECT
done
for net in fc00::/7 fe80::/10 ff00::/8; do
    ip6tables -A OBOT_RIG -d "$net" -j REJECT
done
FIREWALL
chmod 755 /usr/local/sbin/obot-rig-firewall
cat > /etc/systemd/system/obot-rig-firewall.service <<'UNIT'
[Unit]
Description=CI jobs on this computer cannot reach the local network
After=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/sbin/obot-rig-firewall

[Install]
WantedBy=multi-user.target
UNIT

# --- the check before every job, and the tidying after ------------------------------------------
mkdir -p /usr/local/lib/obot-rig
cat > /usr/local/lib/obot-rig/job-started.sh <<HOOK
#!/bin/bash
# Run by the runner before a job's first step; if this fails, the job does.
allowed="$repository"
HOOK
cat >> /usr/local/lib/obot-rig/job-started.sh <<'HOOK'
refuse() { echo "::error::this runner refuses the job: $1"; exit 1; }
[ "${GITHUB_REPOSITORY:-}" = "$allowed" ] || refuse "it works for $allowed only"
case "${GITHUB_EVENT_NAME:-}" in
    push | workflow_dispatch | schedule) ;;
    *) refuse "it runs pushes, manual and scheduled runs, not '${GITHUB_EVENT_NAME:-}' (a pull request's code is anybody's)" ;;
esac
case "${GITHUB_WORKFLOW_REF:-}" in
    "$allowed"/.github/workflows/*) ;;
    *) refuse "the workflow is not one of $allowed's own" ;;
esac
# Start from nothing: what an earlier job left running, and its files.
# (The runner runs this with -e: a pkill that finds nothing must not count as a failure.)
for name in obot-rtd obot-navd obot-sim-node; do pkill -KILL -u ghrunner -x "$name" || true; done
pkill -KILL -u ghrunner -f 'venv/bin/obot ' || true
if [ -n "${GITHUB_WORKSPACE:-}" ] && [ -d "$GITHUB_WORKSPACE" ]; then
    find "$GITHUB_WORKSPACE" -mindepth 1 -delete || true
fi
exit 0
HOOK
cat > /usr/local/lib/obot-rig/job-completed.sh <<'HOOK'
#!/bin/bash
# Run by the runner after a job, however it ended: nothing of it keeps running.
for name in obot-rtd obot-navd obot-sim-node; do pkill -KILL -u ghrunner -x "$name" || true; done
pkill -KILL -u ghrunner -f 'venv/bin/obot ' || true
exit 0
HOOK
chmod 755 /usr/local/lib/obot-rig/*.sh

# --- the runner ---------------------------------------------------------------------------------
systemctl stop obot-rig-runner 2>/dev/null || true
dir="$home/actions-runner"
if [ ! -x "$dir/bin/Runner.Listener" ] || [ "$(cat "$dir/.obot-version" 2>/dev/null)" != "$version" ]; then
    rm -rf "$dir" && mkdir -p "$dir"
    archive="/tmp/actions-runner-$version.tar.gz"
    curl -fsSL -o "$archive" "https://github.com/actions/runner/releases/download/v$version/actions-runner-linux-arm64-$version.tar.gz"
    echo "$sha256  $archive" | sha256sum -c - >/dev/null
    tar -C "$dir" -xzf "$archive" && rm -f "$archive"
    echo "$version" > "$dir/.obot-version"
    chown -R "$user:" "$dir"
fi
systemctl daemon-reload
systemctl enable -q --now obot-rig-firewall
cd "$dir"
if [ -f .runner ]; then
    echo "already registered: left as it is (to register again, remove $dir first)"
else
    [ -n "$token" ] || { echo "a registration token on standard input is needed" >&2; exit 2; }
    runuser -u "$user" -- ./config.sh --unattended --replace --url "https://github.com/$repository" \
        --token "$token" --name "$name" --labels obot-rig --no-default-labels --work _work
fi

cat > /etc/systemd/system/obot-rig-runner.service <<UNIT
[Unit]
Description=GitHub Actions runner for $repository (obot test rig)
After=network-online.target obot-rig-firewall.service
Requires=obot-rig-firewall.service

[Service]
User=$user
WorkingDirectory=$dir
ExecStart=$dir/run.sh
Environment=ACTIONS_RUNNER_HOOK_JOB_STARTED=/usr/local/lib/obot-rig/job-started.sh
Environment=ACTIONS_RUNNER_HOOK_JOB_COMPLETED=/usr/local/lib/obot-rig/job-completed.sh
Environment=HOME=$home
Restart=always
RestartSec=15
KillMode=control-group
Nice=5
TasksMax=1024
# What a job can touch: its own directory, a /tmp of its own, shared memory. Not /home.
NoNewPrivileges=yes
ProtectSystem=strict
ReadWritePaths=$home
ProtectHome=yes
PrivateTmp=yes
ProtectKernelTunables=yes
ProtectKernelModules=yes
ProtectControlGroups=yes

[Install]
WantedBy=multi-user.target
UNIT
systemctl daemon-reload
systemctl enable -q --now obot-rig-runner
sleep 5
systemctl is-active obot-rig-firewall obot-rig-runner
