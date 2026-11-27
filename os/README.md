# Robot OS image

A Yocto image for the Jetson Orin family: NVIDIA's 6.8 kernel with PREEMPT_RT, one CPU
isolated for the control thread, CAN, `obot-rtd` as a service, and Docker with the NVIDIA
runtime for the Python / ML side.

| | |
|---|---|
| Yocto | 6.0 "wrynose" (LTS to 2030) |
| BSP | [OE4T meta-tegra](https://github.com/OE4T/meta-tegra) `wrynose`: Jetson Linux R39.2.1 (JetPack 7.2.1) |
| Kernel | `linux-noble-nvidia-tegra-rt` 6.8 + `meta-obot/recipes-kernel/linux/files/*.cfg` |
| Build tool | [kas](https://kas.readthedocs.io); every layer pinned in `kas/obot.yml` |

Machines: `kas/orin-nano.yml` (Orin Nano dev kit incl. Super, NVMe), `kas/orin-nano-sd.yml`,
`kas/agx-orin.yml`.

## Build

On Ubuntu 24.04 or 26.04, on a native Linux filesystem (not `/mnt/c` under WSL), with
about 100 GB free:

```sh
sudo apt install gawk wget git diffstat unzip texinfo gcc build-essential chrpath socat cpio \
    python3 python3-pip python3-pexpect xz-utils debianutils iputils-ping python3-git \
    python3-jinja2 python3-subunit zstd lz4 file locales libacl1
pipx install kas
git submodule update --init --recursive

export KAS_WORK_DIR=~/obot-os        # layers, downloads and build output go here
kas build os/kas/obot.yml:os/kas/orin-nano.yml
```

The image is `$KAS_WORK_DIR/build/tmp/deploy/images/<machine>/obot-image-<machine>.tegraflash-tar.zst`.
`obot-rt` is built from the checkout the kas file is in, so the image always carries the
daemon of the same commit.

## Flash

On a native Linux host (WSL has no USB access without usbipd), with the Jetson in recovery
mode on USB:

```sh
mkdir flash && cd flash && tar -xf .../obot-image-jetson-orin-nano-devkit-nvme.tegraflash-tar.zst
sudo ./initrd-flash
```

See meta-tegra's `docs/Flashing.md`.

## What the image sets up

- Kernel command line (`meta-obot/conf/distro/obot.conf`):
  `isolcpus=managed_irq,domain,<cpu> nohz_full=<cpu> rcu_nocbs=<cpu> rcu_nocb_poll
  irqaffinity=<others> kthread_cpus=<others>`. The CPU is `OBOT_RT_CPU`: 5 on the Orin
  Nano, 11 on the AGX Orin.
- `obot-can.service`: brings up the CAN interfaces with the bit rates from
  `/etc/obot/robot.toml`.
- `obot-rt-tuning.service`: performance governor, CAN interrupts and their threads moved
  to the control CPU at priority 70.
- `obot-rtd.service`: the daemon as user `obot` with only `CAP_SYS_NICE` and
  `CAP_IPC_LOCK`. Members of group `obot` can attach to `/dev/shm/obot`.
- `efi=runtime` is kept on the command line although NVIDIA's realtime guide removes it:
  the A/B boot slot logic needs EFI variables.

The image is a development image: root logs in without a password on the serial console
and over ssh. Remove `EXTRA_IMAGE_FEATURES` in `kas/obot.yml` before the robot leaves the
bench.

## Not verified on hardware

The image has been built but not booted. In particular: the CAN pins of the Orin Nano
carrier (header J17 needs soldering and an external transceiver; the AGX Orin has two
controllers on the 40 pin header), which add-on CAN adapters the 1 kHz loop tolerates, and
the latency of the isolated CPU. NVIDIA labels its PREEMPT_RT kernel a developer preview.
Run `obot-rt-latency` and `cyclictest` (both in the image) before trusting it.

## The Jetson Nano (v1 robot)

NVIDIA's support for the original Nano ended with L4T R32.7 (kernel 4.9, Ubuntu 18.04), so
it cannot use this image. Two ways to run the same software there:

1. Keep the existing L4T image and copy the statically linked programs that CI builds
   (artifact `obot-rt-aarch64-static`): they need nothing from the OS but the `usb_rt`
   driver. Run the Python tools in a container (`docker run --ipc=host ...`), since the
   OS's Python is too old.
2. A Yocto image from meta-tegra's `kirkstone-l4t-r32.7.x` branch (Yocto 4.0, end of life,
   needs an Ubuntu 22.04 build host or container). Not set up here.
