#!/bin/bash

version=R32.4.4

sudo apt-get update 
sudo apt-get install libncurses5-dev 
sudo apt-get install build-essential bc 
sudo apt-get install lbzip2
sudo apt-get install qemu-user-static

# Create build folder
mkdir $HOME/jetson_nano 
cd $HOME/jetson_nano 

sudo tar xpf Tegra210_Linux_${version}_aarch64.tbz2 
cd Linux_for_Tegra/rootfs/ 
sudo tar xpf ../../Tegra_Linux_Sample-Root-Filesystem_${version}_aarch64.tbz2 
cd ../../ 
#tar -xvf gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu.tar.xz
sudo tar -xjf public_sources.tbz2
tar -xjf Linux_for_Tegra/source/public/kernel_src.tbz2

cd kernel/kernel-4.9/ 
./scripts/rt-patch.sh apply-patches 

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
./scripts/rt-patch.sh apply-patches
make ARCH=arm64 modules_prepare
kernel_ver=$(cat include/config/kernel.release)
sudo cp -r kernel $HOME/jetson_nano/Linux_for_Tegra/rootfs/usr/src/linux-headers-${kernel_ver}
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

# Generate Jetson Nano image
cd tools
sudo ./jetson-disk-image-creator.sh -o jetson_nano.img -b jetson-nano-2gb-devkit