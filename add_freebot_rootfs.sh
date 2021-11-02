#!/bin/bash
set -exo pipefail

# ./add_freebot_rootfs.sh $(cat $HOME/jetson_nano/module_headers/kernel/kernel-4.9/include/config/kernel.release)

echo "********************** Adding freebot stuff ****************************"

cd rootfs
sudo cp /usr/bin/qemu-aarch64-static usr/bin
sudo cp --remove-destination -L /etc/resolv.conf etc/
sudo systemd-nspawn -D . -M tmpjetson --resolv-conf off --pipe /bin/bash << EOF
set -xeo pipefail
apt update
apt install -y curl
cd /usr/src/linux-headers-${1}/kernel-4.9
make scripts -j10   # fix scripts that were not compiled correctly
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
apt update
apt install -y ros-melodic-ros-base
curl https://raw.githubusercontent.com/unhuman-io/freebot/main/install-freebot.sh > install-freebot.sh
chmod +x install-freebot.sh
./install-freebot.sh
EOF
sudo rm usr/bin/qemu-aarch64-static
sudo rm etc/resolv.conf
pushd usr/local/src
sudo sh -c 'curl https://storage.googleapis.com/git-repo-downloads/repo > repo'
sudo chmod a+rx repo
sudo git config --global user.name "Root"
sudo git config --global user.email "root@root.com"
sudo git config --global color.ui true
sudo ./repo init -b main -u https://github.com/unhuman-io/freebot
sudo ./repo sync
popd