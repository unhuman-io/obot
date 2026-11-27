SUMMARY = "obot robot image: realtime actuator daemon, CAN, container runtime for the ML stack"
LICENSE = "MIT"

inherit core-image

IMAGE_FEATURES += "ssh-server-openssh"

# Realtime control
IMAGE_INSTALL:append = " \
    obot-rt \
    obot-system \
    can-utils \
    iproute2 \
    rt-tests \
    stress-ng \
    kernel-modules \
"
IMAGE_INSTALL:append:tegra = " nvidia-kernel-oot-canbus tegra-tools-tegrastats"

# Containers with GPU access. The container images expect CUDA, cuDNN and TensorRT from
# the host (see meta-tegra docs/NVIDIA-Container-Runtime-support.md).
IMAGE_INSTALL:append = " docker"
IMAGE_INSTALL:append:tegra = " nvidia-container-toolkit cuda-libraries tensorrt cudnn"

# Getting around on the robot
IMAGE_INSTALL:append = " \
    bash \
    chrony \
    htop \
    less \
    procps \
    python3 \
    python3-numpy \
    strace \
    systemd-analyze \
    usbutils \
    util-linux \
"

# Room for container images and logs.
IMAGE_ROOTFS_EXTRA_SPACE = "4194304"
