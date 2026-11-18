# Running on an existing OS

For a robot that keeps its own Linux (for example the v1 robot's Jetson Nano with L4T R32)
instead of the image built in `os/`. CI packages these files with statically linked
programs as the artifact `obot-rt-aarch64-static`.

## From a development machine, over ssh

```sh
sudo apt install cmake ninja-build g++-aarch64-linux-gnu
deploy/deploy.sh user@robot config/obot1.toml 3          # control thread on CPU 3
deploy/deploy.sh user@jetson config/obot1.toml -1 --sim  # a board with nothing connected
```

The script prints what the board is (model, L4T release, kernel, Python), cross-compiles
statically linked programs, copies them with the robot description and model, and installs
and starts `obot-rtd.service`. With `--sim` the service runs on simulated actuators, so a
bare Jetson can be used to try the daemon, the operator page and deployment itself.

The operator tools need Python 3.10 or newer, which L4T R32 (Ubuntu 18.04) does not have.
[uv](https://docs.astral.sh/uv/) brings its own:

```sh
# on the robot, once; then log out and in for the group to take effect
curl -LsSf https://astral.sh/uv/install.sh | sh
sudo usermod -aG obot $USER
# from the development machine
scp -r python user@robot:obot-python
# on the robot
uv venv --python 3.12 ~/obot-venv && uv pip install -p ~/obot-venv ~/obot-python'[ui,log]'
~/obot-venv/bin/obot ui --test-camera        # http://<robot>:8080/ from any browser
```

## From the CI artifact

```sh
tar -xzf obot-rt-aarch64-static.tar.gz && cd obot-rt-aarch64
sudo ./install.sh config/obot1.toml 3      # robot description, CPU for the control thread
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
