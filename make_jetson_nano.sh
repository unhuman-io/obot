#!/bin/bash

version=R32.5.0
release=5.0

sudo apt-get update 
sudo apt-get install -y libncurses5-dev wget
sudo apt-get install -y build-essential bc 
sudo apt-get install -y lbzip2
sudo apt-get install -y qemu-user-static systemd-container

# keep sudo alive
while true; do
  sleep 300
  sudo -n true
  kill -0 "$$" 2>/dev/null || exit
done &

# Create build folder
# manually download from here:
# https://developer.nvidia.com/embedded/downloads
# L4T sample filesystem https://developer.nvidia.com/embedded/L4T/r32_Release_v5.0/T210/Tegra_Linux_Sample-Root-Filesystem_R32.5.0_aarch64.tbz2
# L4T Jetson driver package https://developer.nvidia.com/embedded/L4T/r32_Release_v5.0/T210/Tegra210_Linux_R32.5.0_aarch64.tbz2
# L4T Sources Jetson https://developer.nvidia.com/embedded/L4T/r32_Release_v5.0/sources/T210/public_sources.tbz2
# 
# https://releases.linaro.org/components/toolchain/binaries/7.3-2018.05/aarch64-linux-gnu/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu.tar.xz

sudo rm -rf $HOME/jetson_nano
mkdir $HOME/jetson_nano 
cd $HOME/jetson_nano 
wget https://developer.nvidia.com/embedded/L4T/r32_Release_v${release}/T210/Tegra_Linux_Sample-Root-Filesystem_${version}_aarch64.tbz2
wget https://developer.nvidia.com/embedded/L4T/r32_Release_v${release}/T210/Tegra210_Linux_${version}_aarch64.tbz2
wget https://developer.nvidia.com/embedded/L4T/r32_Release_v${release}/sources/T210/public_sources.tbz2
wget https://releases.linaro.org/components/toolchain/binaries/7.3-2018.05/aarch64-linux-gnu/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu.tar.xz

sudo tar xpf Tegra210_Linux_${version}_aarch64.tbz2 
cd Linux_for_Tegra/rootfs/ 
sudo tar xpf ../../Tegra_Linux_Sample-Root-Filesystem_${version}_aarch64.tbz2 
cd ../../ 
tar -xvf gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu.tar.xz
sudo tar -xjf public_sources.tbz2
tar -xjf Linux_for_Tegra/source/public/kernel_src.tbz2

cd kernel/kernel-4.9/ 
./scripts/rt-patch.sh apply-patches 
#some sort of bug
sed -i 's/YYLTYPE yylloc;//' scripts/dtc/dtc-lexer.lex.c_shipped

# Compile kernel
TEGRA_KERNEL_OUT=jetson_nano_kernel 
mkdir $TEGRA_KERNEL_OUT 
export CROSS_COMPILE=$HOME/jetson_nano/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
make ARCH=arm64 O=$TEGRA_KERNEL_OUT tegra_defconfig 
make ARCH=arm64 O=$TEGRA_KERNEL_OUT -j10

# Copy results
sudo cp jetson_nano_kernel/arch/arm64/boot/Image $HOME/jetson_nano/Linux_for_Tegra/kernel/Image 
sudo cp -r jetson_nano_kernel/arch/arm64/boot/dts/* $HOME/jetson_nano/Linux_for_Tegra/kernel/dtb/ 
sudo make ARCH=arm64 O=$TEGRA_KERNEL_OUT modules_install INSTALL_MOD_PATH=$HOME/jetson_nano/Linux_for_Tegra/rootfs/ 

# lib modules has links to the original source
cd $HOME/jetson_nano/
mkdir module_headers
cd module_headers
tar -xjf ../Linux_for_Tegra/source/public/kernel_src.tbz2
cd kernel/kernel-4.9
cp $HOME/jetson_nano/kernel/kernel-4.9/${TEGRA_KERNEL_OUT}/.config .
cp $HOME/jetson_nano/kernel/kernel-4.9/${TEGRA_KERNEL_OUT}/Module.symvers .
./scripts/rt-patch.sh apply-patches
make ARCH=arm64 modules_prepare
kernel_ver=$(cat include/config/kernel.release)
sudo cp -r .. $HOME/jetson_nano/Linux_for_Tegra/rootfs/usr/src/linux-headers-${kernel_ver}
cd $HOME/jetson_nano/Linux_for_Tegra/rootfs/lib/modules/${kernel_ver}
sudo rm build
sudo ln -s /usr/src/linux-headers-${kernel_ver}/kernel-4.9 build
sudo rm source
sudo ln -s /usr/src/linux-headers-${kernel_ver}/kernel-4.9 source

cd $HOME/jetson_nano/Linux_for_Tegra/rootfs/ 
sudo tar --owner root --group root -cjf kernel_supplements.tbz2 lib/modules 
sudo mv kernel_supplements.tbz2  ../kernel/ 
sudo tar --owner root --group root -cjf kernel_headers.tbz2 usr/src
sudo mv kernel_headers.tbz2  ../kernel/ 

# Apply binaries
cd .. 
sudo ./apply_binaries.sh

cd $HOME/jetson_nano/Linux_for_Tegra
# Add freebot stuff
cd rootfs
sudo cp /usr/bin/qemu-aarch64-static usr/bin
sudo cp -L /etc/resolv.conf etc/
sudo systemd-nspawn -D . -M tmpjetson --resolv-conf off --pipe /bin/bash << EOF
apt update
apt install -y curl
cd /usr/src/linux-headers-${kernel_ver}/kernel-4.9
make scripts -j10   # fix scripts that were not compiled correctly
curl https://raw.githubusercontent.com/unhuman-io/freebot/main/install-freebot.sh > install-freebot.sh
chmod +x install-freebot.sh
./install-freebot.sh
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
apt update
apt install -y ros-melodic-ros-base
EOF
sudo rm usr/bin/qemu-aarch64-static
#sudo mv etc/resolv.conf.tmp etc/resolv.conf
sudo rm etc/resolv.conf
pushd usr/local/src
sudo sh -c 'curl https://storage.googleapis.com/git-repo-downloads/repo > repo'
sudo chmod a+rx repo
sudo git config --global user.name "Root"
sudo git config --global user.email "root@root.com"
sudo ./repo init -b main -u https://github.com/unhuman-io/freebot
sudo ./repo sync
popd
cd ..

# Generate Jetson Nano image
cd tools
sudo ./jetson-disk-image-creator.sh -o jetson_nano_2gb.img -b jetson-nano-2gb-devkit
sudo ./jetson-disk-image-creator.sh -o jetson_nano_4gb.img -b jetson-nano -r 300
sudo ./jetson-disk-image-creator.sh -o jetson_nano_4agb.img -b jetson-nano -r 200
