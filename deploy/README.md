# Running on an existing OS

For a robot that keeps its own Linux (for example the v1 robot's Jetson Nano with L4T R32)
instead of the image built in `os/`. CI packages these files with statically linked
programs as the artifact `obot-rt-aarch64-static`.

## From a development machine, over ssh

```sh
sudo apt install cmake ninja-build g++-aarch64-linux-gnu
deploy/deploy.sh --cpu 3 --isolate --reboot user@robot config/obot1.toml
deploy/deploy.sh --cpu 3 --isolate --reboot --sim --force-config user@spare config/obot1.toml
deploy/deploy.sh --jump user@gateway --sim user@behind-it config/obot1.toml
deploy/deploy.sh --verify-only --cpu 3 --isolate user@robot config/obot1.toml
```

One command builds statically linked programs, installs them with the robot description,
the Python tools and both services (`obot-rtd`, `obot-ui`), and then runs the checks in
`verify.sh` on the robot; its exit status is the result. Running it again is safe.

| option | |
|---|---|
| `--cpu N` | CPU for the control thread |
| `--isolate`, `--reboot` | put `isolcpus=N nohz_full=N rcu_nocbs=N` on the kernel command line (`/boot/extlinux/extlinux.conf`, original kept as `.before-obot`) and reboot if that changed |
| `--sim` | simulated actuators: for a board with nothing connected |
| `--force-config` | replace the robot's `/etc/obot/robot.toml` (otherwise an existing one is kept and the new one is left beside it as `robot.toml.new`) |
| `--jump HOST` | reach the robot through another ssh host |
| `--no-tools` | daemon only |
| `--verify-only` | change nothing, run the checks |

The robot needs ssh with a key and passwordless sudo for the login user, and internet
access the first time (the tools bring their own Python 3.12 through
[uv](https://docs.astral.sh/uv/), since L4T R32 has Python 3.6). The checks: services
active, realtime priority, the control thread on its CPU, that CPU isolated, the operator
page answering, and ten seconds of the control loop with no overruns and no wake-up later
than 1 ms. A kernel without PREEMPT_RT is reported as a warning.

Tried on two Jetson Nano developer kits: L4T R32.5.0 with a 4.9-rt kernel, and stock
L4T R32.4.4 (not realtime); on both the worst wake-up in the ten seconds was about 20 us
on an idle board.

`.github/workflows/deploy-test.yml` runs the same command from a self-hosted runner on
the robots' network (label `obot-lab`), against the board named in the repository
variable `DEPLOY_TEST_TARGET`.

## From the CI artifact

```sh
tar -xzf obot-rt-aarch64-static.tar.gz && cd obot-rt-aarch64
cp config/obot1.toml robot.toml && cp -r ../description . 2>/dev/null
sudo OBOT_CPU=3 OBOT_TOOLS=0 sh ./remote-install.sh
obot-rtd --check --config /etc/obot/robot.toml
sudo systemctl start obot-rtd && journalctl -u obot-rtd -f
```

`install.sh` copies the programs to `/usr/local/bin`, the description to
`/etc/obot/robot.toml` (kept if it already exists), creates the `obot` user and group and
installs `obot-rtd.service`. It does not change the kernel command line: for a quiet
control CPU add `isolcpus=<cpu> nohz_full=<cpu> rcu_nocbs=<cpu>` to the boot arguments
(`/boot/extlinux/extlinux.conf` on L4T) and use a PREEMPT_RT kernel.

USB boards need the `usb_rt` driver (`sudo modprobe usb_rt`; `motor_util` lists the board
names the description refers to) and the `obot` user needs access to the device nodes: the
driver's udev rule makes them world writable.

The Python tools need Python 3.10 or newer. Where the OS has none, run them from the
container in `docker/` with `--ipc=host`.
