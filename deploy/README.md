# Running on an existing OS

For a robot that keeps its own Linux (for example the v1 robot's Jetson Nano with L4T R32)
instead of the image built in `os/`. CI packages these files with statically linked
programs as the artifact `obot-rt-aarch64-static`.

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
