#!/bin/bash
set -eo pipefail

version=R32.5.0
release=5.0
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
build_dir=$HOME/jetson_nano
l4t_dir=${build_dir}/Linux_for_Tegra

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

sudo rm -rf ${build_dir}
mkdir ${build_dir}
cd ${build_dir}
wget https://developer.nvidia.com/embedded/L4T/r32_Release_v${release}/T210/Tegra_Linux_Sample-Root-Filesystem_${version}_aarch64.tbz2
wget https://developer.nvidia.com/embedded/L4T/r32_Release_v${release}/T210/Tegra210_Linux_${version}_aarch64.tbz2
wget https://developer.nvidia.com/embedded/L4T/r32_Release_v${release}/sources/T210/public_sources.tbz2
wget https://releases.linaro.org/components/toolchain/binaries/7.3-2018.05/aarch64-linux-gnu/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu.tar.xz

sudo tar xpf Tegra210_Linux_${version}_aarch64.tbz2 
cd ${l4t_dir}/rootfs/ 
sudo tar xpf ../../Tegra_Linux_Sample-Root-Filesystem_${version}_aarch64.tbz2 
cd ${build_dir} 
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
sudo cp jetson_nano_kernel/arch/arm64/boot/Image ${l4t_dir}/kernel/Image 
sudo cp -r jetson_nano_kernel/arch/arm64/boot/dts/* ${l4t_dir}/kernel/dtb/ 
sudo make ARCH=arm64 O=$TEGRA_KERNEL_OUT modules_install INSTALL_MOD_PATH=${l4t_dir}/rootfs/ 

# lib modules has links to the original source
cd ${build_dir}
mkdir module_headers

cd ${build_dir}/module_headers
tar -xjf ${l4t_dir}/source/public/kernel_src.tbz2

cd ${build_dir}/module_headers/kernel/kernel-4.9
cp ${build_dir}/kernel/kernel-4.9/${TEGRA_KERNEL_OUT}/.config .
cp ${build_dir}/kernel/kernel-4.9/${TEGRA_KERNEL_OUT}/Module.symvers .

${build_dir}/module_headers/kernel/kernel-4.9/scripts/rt-patch.sh apply-patches
make ARCH=arm64 modules_prepare

kernel_ver=$(cat ${build_dir}/module_headers/kernel/kernel-4.9/include/config/kernel.release)

sudo cp -r .. ${l4t_dir}/rootfs/usr/src/linux-headers-${kernel_ver}
cd ${l4t_dir}/rootfs/lib/modules/${kernel_ver}
sudo rm build
sudo ln -s /usr/src/linux-headers-${kernel_ver}/kernel-4.9 build
sudo rm source
sudo ln -s /usr/src/linux-headers-${kernel_ver}/kernel-4.9 source

cd ${l4t_dir}/rootfs/ 
sudo tar --owner root --group root -cjf kernel_supplements.tbz2 lib/modules 
sudo mv kernel_supplements.tbz2  ../kernel/ 
sudo tar --owner root --group root -cjf kernel_headers.tbz2 usr/src
sudo mv kernel_headers.tbz2  ../kernel/ 

# Apply binaries
cd ${l4t_dir}
sudo ./apply_binaries.sh
# todo add_freebot_rootfs seems to be failing due to apt sources
${script_dir}/add_freebot_rootfs.sh ${kernel_ver}

# Generate Jetson Nano image
cd ${l4t_dir}/tools
sudo ./jetson-disk-image-creator.sh -o jetson_nano_2gb.img -b jetson-nano-2gb-devkit
sudo ./jetson-disk-image-creator.sh -o jetson_nano_4gb.img -b jetson-nano -r 300
sudo ./jetson-disk-image-creator.sh -o jetson_nano_4agb.img -b jetson-nano -r 200
