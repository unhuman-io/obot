#!/bin/sh
# Runs on the robot after a deployment: every check prints ok or FAIL; exit status 1 if any failed.
#   OBOT_CPU, OBOT_ISOLATE, OBOT_TOOLS as for remote-install.sh
fail=0
check() { if eval "$2" >/dev/null 2>&1; then echo "ok    $1"; else echo "FAIL  $1"; fail=1; fi; }
cpu="${OBOT_CPU:--1}"
# The daemon's shared memory name, as configured.
shm="$(sed -n 's/^shm_name *= *"\(.*\)".*/\1/p' /etc/obot/robot.toml 2>/dev/null | head -1)"
shm="${shm:-obot}"

check "obot-rtd service is active" "systemctl is-active obot-rtd"
check "shared memory is published" "test -s /dev/shm/$shm"
check "daemon is realtime (SCHED_FIFO)" "ps -eLo cls,comm | grep -q '^ *FF *obot-rtd'"
check "no realtime setup warnings in the log" "! journalctl -u obot-rtd -b --no-pager | tail -40 | grep -q 'warning: could not'"
if [ "$cpu" -ge 0 ]; then
    check "control thread is on cpu $cpu" "ps -eLo psr,cls,comm | grep -q '^ *$cpu *FF *obot-rtd'"
fi
if [ "${OBOT_ISOLATE:-0}" = 1 ]; then
    check "cpu $cpu is isolated" "grep -qw $cpu /sys/devices/system/cpu/isolated"
fi
# Not a failure of the deployment, but it decides what the latency numbers below are worth.
if [ "$(cat /sys/kernel/realtime 2>/dev/null)" = 1 ]; then echo "ok    kernel is PREEMPT_RT"; else echo "WARN  kernel is not PREEMPT_RT ($(uname -r))"; fi
if [ "${OBOT_TOOLS:-1}" = 1 ]; then
    # Give the page a moment after a restart.
    for _ in 1 2 3 4 5 6 7 8 9 10; do curl -sf -m 2 http://127.0.0.1:8080/api/info >/dev/null && break; sleep 1; done
    check "obot-ui service is active" "systemctl is-active obot-ui"
    check "operator page answers" "curl -sf -m 3 http://127.0.0.1:8080/api/info | grep -q '\"robot\"'"
    check "robot is idle and healthy (obot status)" "sg obot -c '$HOME/obot-venv/bin/obot status'"
fi
# Ten seconds of the control loop: no overruns, and the worst wake-up within a millisecond.
if [ "${OBOT_TOOLS:-1}" = 1 ]; then
    report="$(sg obot -c "$HOME/obot-venv/bin/python - <<'PY'
import time
from obot import Robot
r = Robot('$shm'); first = r.state(); r.read(); worst = 0; end = time.time() + 10
while time.time() < end:
    time.sleep(0.5); s = r.read(); worst = max(worst, int(s['wake_latency_ns'].max()))
last = r.state()
print(int(last['cycle'] - first['cycle']), int(last['overruns'] - first['overruns']), worst // 1000)
PY
" 2>/dev/null)" || report=""
    echo "      10 s of control loop: cycles, overruns, worst wake-up (us): $report"
    set -- $report
    check "control loop ran at its rate (about 10000 cycles)" "test ${1:-0} -ge 9000"
    check "no overruns in 10 s" "test ${2:-1} -eq 0"
    check "worst wake-up under 1 ms" "test ${3:-9999} -lt 1000"
fi
exit $fail
